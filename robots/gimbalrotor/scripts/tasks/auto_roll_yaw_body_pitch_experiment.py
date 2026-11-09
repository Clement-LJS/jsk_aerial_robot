#!/usr/bin/env python3
"""Timed finite body-local Y jogs through the roll-yaw navigator (geometric only).

Complete normal perching setup and positioning first; wait for idle CONTACT_LOCKED.
Each publication is one signed increment from the applied body reference, with
coordinated joints owned by the navigator. Positive is down, negative is up.
Private parameters (read once): interval_sec=15.0, step_deg=0.8, num_steps=15,
robot_namespace=/gimbalrotor, debug_prefix=perching/roll_yaw, step_timeout_sec=60.0.
The active diagnostic jog limit must be positive and at most one degree.

From the repository root:
  python3 robots/gimbalrotor/scripts/tasks/auto_roll_yaw_body_pitch_experiment.py \
    _interval_sec:=15.0 _step_deg:=0.8 _num_steps:=15
  python3 robots/gimbalrotor/scripts/tasks/auto_roll_yaw_body_pitch_experiment.py \
    _interval_sec:=5.0 _step_deg:=0.5 _num_steps:=20
  python3 robots/gimbalrotor/scripts/tasks/auto_roll_yaw_body_pitch_experiment.py \
    _interval_sec:=5.0 _step_deg:=-0.5 _num_steps:=10
  rosrun gimbalrotor auto_roll_yaw_body_pitch_experiment.py \
    _interval_sec:=5.0 _step_deg:=0.5 _num_steps:=20

The first step is immediate after readiness checks. Intervals are measured from
actual publication; a slow jog extends the interval without catch-up commands.
Ctrl+C requests braking/contact hold for an unconfirmed owned jog. Normal exit
holds the final reference. The saw remains under separate operator control.
N * step_deg (12 degrees by default) is the requested local reference increment,
not a guaranteed physical or world-Euler angle change. No cutting admittance.

Single operator: run only one scheduler and issue no concurrent keyboard motion,
alignment, preparation or other motion commands. An idle emergency-stop keyboard
is fine. Float64 requests have no session/request ID: an in-flight relock race
cannot be eliminated, and alpha/roll/yaw_target commands have no acknowledgements.
"""

from dataclasses import dataclass
import math
import signal
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Float64, String, UInt8, UInt64


STARTUP_TIMEOUT = 10.0
ADMISSION_TIMEOUT = 3.0
FRESHNESS_TIMEOUT = 1.0
CANCEL_TIMEOUT = 3.0
WAIT_SLICE = 0.05
LIMIT_TOLERANCE = 1e-12
# std::to_string in the navigator rounds admission deltas to six decimal places.
ACK_DELTA_TOLERANCE = 0.500001e-6
FLAGS = ('feedback_waiting', 'stopping', 'prepare_pending',
         'prepared_connection_pending', 'body_pitch_jog')
TERMINALS = ('reference_complete', 'failed', 'cancelled')


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + ' must be a finite number')
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(name + ' must be a finite number') from exc
    if not math.isfinite(number):
        raise ValueError(name + ' must be a finite number')
    return number


def session_integer(value):
    # Never pass nanosecond lock identities through floating point.
    if not isinstance(value, str) or not value.isascii() or not value.isdecimal():
        raise ValueError('invalid decimal session')
    result = int(value)
    if not 0 < result < 2**64:
        raise ValueError('expected a nonzero UInt64 session')
    return result


@dataclass
class Step:
    sent: float
    admitted: bool = False
    rejected: bool = False
    terminal: str = ''
    terminal_time: float = 0.0


class Experiment:
    def __init__(self):
        self.interval = finite_number(rospy.get_param('~interval_sec', 15.0), '~interval_sec')
        self.step_deg = finite_number(rospy.get_param('~step_deg', 0.8), '~step_deg')
        self.timeout = finite_number(rospy.get_param('~step_timeout_sec', 60.0), '~step_timeout_sec')
        self.num_steps = rospy.get_param('~num_steps', 15)
        if self.interval <= 0 or self.timeout <= 0:
            raise ValueError('~interval_sec and ~step_timeout_sec must be strictly positive')
        if self.step_deg == 0 or abs(self.step_deg) > 1.0 + math.degrees(LIMIT_TOLERANCE):
            raise ValueError('~step_deg must be nonzero and at most one degree in magnitude')
        if isinstance(self.num_steps, bool) or not isinstance(self.num_steps, int) or self.num_steps <= 0:
            raise ValueError('~num_steps must be a positive integer (not a boolean or fraction)')
        self.delta = math.radians(self.step_deg)
        if self.delta == 0:
            raise ValueError('~step_deg is too small to represent in radians')
        ns = rospy.get_param('~robot_namespace', '/gimbalrotor')
        prefix = rospy.get_param('~debug_prefix', 'perching/roll_yaw')
        if not isinstance(ns, str) or not isinstance(prefix, str) or not prefix.strip('/'):
            raise ValueError('namespace and debug_prefix must be strings; prefix must be nonempty')
        ns, prefix = ns.rstrip('/'), prefix.rstrip('/')
        self.prefix = prefix if prefix.startswith('/') else ns + '/' + prefix
        self.condition = threading.Condition(threading.RLock())
        self.received = {}
        self.diagnostic_error = ''
        self.session = None
        self.step = None
        self.failure = ''
        self.relinquished = False
        self.stop_time = None
        self.cleaned = False
        self.interrupted = threading.Event()
        self.requested = self.admitted = self.completed = 0
        self.subs = [
            rospy.Subscriber(self.prefix + '/session', UInt64, self.receive,
                             callback_args='session', queue_size=10),
            rospy.Subscriber(self.prefix + '/state', String, self.receive,
                             callback_args='state', queue_size=10),
            rospy.Subscriber(ns + '/flight_state', UInt8, self.receive,
                             callback_args='flight', queue_size=10),
            rospy.Subscriber(self.prefix + '/diagnostics_status', DiagnosticArray,
                             self.diagnostics, queue_size=10),
        ]
        self.ack_sub = rospy.Subscriber(self.prefix + '/command_ack', String, self.ack,
                                        queue_size=100)
        self.motion_pub = rospy.Publisher(self.prefix + '/body_pitch_delta', Float64, queue_size=1)
        self.cancel_pub = rospy.Publisher(self.prefix + '/cancel', UInt64, queue_size=1)

    def fail(self, reason, relinquish=False):
        if not self.failure:
            self.failure = reason
        self.relinquished |= relinquish
        self.condition.notify_all()

    def receive(self, msg, key):
        with self.condition:
            self.received[key] = (msg.data, time.monotonic())
            # Latch even transient unsafe changes between main-loop wakeups.
            if self.session is not None:
                if key == 'session' and msg.data != self.session:
                    self.fail('Contact session changed/lost', relinquish=True)
                elif key == 'flight' and msg.data != 5:
                    self.fail('Flight state is no longer HOVER')
                elif key == 'state' and msg.data.partition(': ')[0] != 'CONTACT_LOCKED':
                    stage = msg.data.partition(': ')[0]
                    self.fail('Incompatible navigator state: ' + msg.data,
                              relinquish=stage not in ('FAULT_HOLD', 'LOCK_FAILED', 'FREE_FLIGHT'))
            self.condition.notify_all()

    def diagnostics(self, msg):
        entries = [entry for entry in msg.status if entry.name == 'gimbalrotor/roll_yaw_perching']
        if not entries:
            return  # Other diagnostic entries do not refresh this navigator.
        parsed = None
        try:
            if len(entries) != 1:
                raise ValueError('ambiguous navigator diagnostic entries')
            entry = entries[0]
            values = {item.key: item.value for item in entry.values}
            if len(values) != len(entry.values):
                raise ValueError('duplicate diagnostic fields')
            session = session_integer(values['session_id'])
            if entry.hardware_id != 'perching_lock_' + str(session):
                raise ValueError('diagnostic hardware/session mismatch')
            flags = {}
            for key in FLAGS:
                if values[key] not in ('0', '1'):
                    raise ValueError('invalid diagnostic boolean ' + key)
                flags[key] = values[key] == '1'
            phase = values['body_pitch_jog_phase']
            if phase not in ('idle', 'moving') + TERMINALS:
                raise ValueError('invalid diagnostic jog phase')
            maximum = float(values['body_pitch_jog_max_delta_rad'])
            if not math.isfinite(maximum) or not 0 < maximum <= math.radians(1) + LIMIT_TOLERANCE:
                raise ValueError('invalid active jog limit; expected positive and <= one degree')
            parsed = dict(flags, session=session, phase=phase, maximum=maximum, level=entry.level)
            error = ''
        except (KeyError, ValueError, OverflowError) as exc:
            error = 'Readiness unavailable: ' + str(exc)
        with self.condition:
            self.received['diagnostics'] = (parsed, time.monotonic())
            self.diagnostic_error = error
            if self.session is not None:
                if parsed is None:
                    self.fail(error)
                elif parsed['session'] != self.session:
                    self.fail('Diagnostic contact session changed', relinquish=True)
                elif parsed['prepare_pending'] or parsed['prepared_connection_pending']:
                    self.fail('Another preparation/connection owns the navigator', relinquish=True)
                elif parsed['feedback_waiting'] or parsed['stopping'] or parsed['level'] >= 2:
                    self.fail('Navigator feedback unavailable, stopping, or faulted')
                elif parsed['body_pitch_jog'] and self.step is None:
                    self.fail('Another body pitch jog owns the navigator', relinquish=True)
            self.condition.notify_all()

    def ack(self, msg):
        now = time.monotonic()
        fields = dict(token.split('=', 1) for token in msg.data.partition(' reason=')[0].split()
                      if '=' in token)
        try:
            session = session_integer(fields.get('session', ''))
        except ValueError:
            return
        with self.condition:
            if session != self.session:
                return
            accepted = fields.get('accepted') == 'true'
            action, phase = fields.get('action'), fields.get('phase')
            if action in ('stop', 'cancel') and accepted:
                if self.stop_time is None:
                    self.stop_time = now
                self.fail('Navigator stop accepted: ' + msg.data)
                return
            if action != 'body_pitch_delta':
                if accepted:
                    self.fail('Another command accepted: ' + msg.data, relinquish=True)
                return
            step = self.step
            if step is None or step.terminal or step.rejected or now < step.sent:
                if accepted and phase == 'moving':
                    self.fail('Unsolicited jog admission: ' + msg.data, relinquish=True)
                return
            if fields.get('accepted') == 'false' or phase in ('rejected', 'noop'):
                # A rejection after our admission can belong to a second sender;
                # do not erase ownership of an already admitted jog.
                step.rejected = not step.admitted
                rospy.logwarn('Navigator: %s', msg.data)
                self.fail(msg.data)
            elif accepted and phase == 'moving':
                try:
                    delta = float(fields.get('delta_rad', 'nan'))
                except ValueError:
                    delta = float('nan')
                if step.admitted or not math.isfinite(delta) or abs(delta - self.delta) > ACK_DELTA_TOLERANCE:
                    self.fail('Ambiguous/competing jog admission: ' + msg.data, relinquish=True)
                elif now > min(step.sent + ADMISSION_TIMEOUT, step.sent + self.timeout):
                    self.fail('Late jog admission: ' + msg.data)
                else:
                    step.admitted = True
                    self.admitted += 1
            elif accepted and phase in TERMINALS:
                if phase != 'reference_complete':
                    rospy.logwarn('Navigator: %s', msg.data)
                if step.admitted:
                    step.terminal, step.terminal_time = phase, now
                    if phase != 'reference_complete':
                        self.fail(msg.data)
                    elif now > step.sent + self.timeout:
                        self.fail('Late reference completion: ' + msg.data)
                    else:
                        self.completed += 1
                        measured = []
                        for key in ('applied_pitch_rad', 'measured_pitch_rad', 'measured_endpoint_error_rad'):
                            try:
                                value = float(fields.get(key, 'nan'))
                            except ValueError:
                                value = float('nan')
                            measured.append(key + '=' + (str(value) if math.isfinite(value) else 'unavailable'))
                        rospy.loginfo('Reference step %d/%d; cumulative requested=%+.6g deg; %s. '
                                      'Reference completion does not prove physical angle accuracy.',
                                      self.completed, self.num_steps, self.completed * self.step_deg,
                                      ' '.join(measured))
                elif phase != 'reference_complete':
                    step.terminal, step.terminal_time = phase, now
                    self.fail(msg.data)  # Missing admission still cannot count success.
            self.condition.notify_all()

    def fresh(self, key, now):
        return key in self.received and now - self.received[key][1] <= FRESHNESS_TIMEOUT

    def readiness(self, now, idle=False):
        for key in ('session', 'state', 'flight', 'diagnostics'):
            if not self.fresh(key, now):
                return 'Missing/stale ' + key
        session = self.received['session'][0]
        diag = self.received['diagnostics'][0]
        if diag is None:
            return self.diagnostic_error or 'Readiness diagnostics unavailable'
        if not session or diag['session'] != session:
            return 'Missing/malformed/inconsistent session diagnostics'
        if self.session is not None and session != self.session:
            return 'Contact session changed'
        if self.received['flight'][0] != 5:
            return 'Require HOVER'
        if self.received['state'][0].partition(': ')[0] != 'CONTACT_LOCKED':
            return 'Require idle CONTACT_LOCKED: ' + self.received['state'][0]
        if diag['level'] >= 2 or any(diag[key] for key in FLAGS[:-1]):
            return 'Navigator feedback/fault/stop/preparation flags are not ready'
        if idle and diag['body_pitch_jog']:
            return 'A body pitch jog is already active'
        if abs(self.delta) > diag['maximum'] + LIMIT_TOLERANCE:
            raise ValueError('~step_deg=%g exceeds active navigator maximum %.12g degrees' %
                             (self.step_deg, math.degrees(diag['maximum'])))
        return ''

    def check_running(self):
        if self.interrupted.is_set() or rospy.is_shutdown():
            raise RuntimeError('Interrupted/shutdown requested')
        if self.failure:
            raise RuntimeError(self.failure)

    def connections(self):
        return (self.motion_pub.get_num_connections() > 0 and
                self.cancel_pub.get_num_connections() > 0 and self.ack_sub.get_num_connections() > 0)

    def run(self):
        with self.condition:
            deadline = time.monotonic() + STARTUP_TIMEOUT
            while True:
                self.check_running()
                now = time.monotonic()
                issue = self.readiness(now, idle=True)
                if not issue and self.connections():
                    self.session = self.received['session'][0]
                    break
                if now >= deadline:
                    raise RuntimeError('Startup: ' + (issue or 'movement/cancel/ack connection missing') +
                                       '; finish or stop other actions before running this script')
                self.condition.wait(min(WAIT_SLICE, deadline - now))
            rospy.loginfo('Topic=%s/body_pitch_delta session=%d step=%+.6g deg interval=%g s '
                          'steps=%d maximum=%.12g deg; single motion operator required',
                          self.prefix, self.session, self.step_deg, self.interval, self.num_steps,
                          math.degrees(self.received['diagnostics'][0]['maximum']))
            for _ in range(self.num_steps):
                due = self.step.sent + self.interval if self.step else time.monotonic()
                while True:
                    self.check_running()
                    now = time.monotonic()
                    issue = self.readiness(now)
                    if issue:
                        raise RuntimeError(issue)
                    diag, receipt = self.received['diagnostics']
                    # Cross-topic delivery order is unspecified. After a terminal
                    # ack, wait for a subsequent inactive diagnostic before reuse.
                    idle = not diag['body_pitch_jog'] and (self.step is None or receipt > self.step.terminal_time)
                    if now >= due and idle:
                        break
                    if self.step and now > max(due, self.step.terminal_time) + FRESHNESS_TIMEOUT:
                        raise RuntimeError('No fresh inactive status after reference completion')
                    self.condition.wait(WAIT_SLICE)
                if not self.connections():
                    raise RuntimeError('Movement/cancel/ack connection lost')
                self.check_running()
                # Arm bookkeeping under the same lock used by callbacks, before
                # publish: even immediate admission + completion remains recorded.
                self.step = Step(time.monotonic())
                self.requested += 1
                self.motion_pub.publish(Float64(data=self.delta))
                while True:
                    self.check_running()
                    now = time.monotonic()
                    issue = self.readiness(now)
                    if issue:
                        raise RuntimeError(issue)
                    if self.step.terminal == 'reference_complete':
                        break
                    deadline = self.step.sent + self.timeout
                    if not self.step.admitted:
                        deadline = min(deadline, self.step.sent + ADMISSION_TIMEOUT)
                    if now >= deadline:
                        raise RuntimeError('Jog admission/completion timeout; increment will not be resent')
                    self.condition.wait(min(WAIT_SLICE, deadline - now))
            rospy.loginfo('All reference steps completed; navigator holds the final contact reference.')

    def same_session(self, now):
        diag = self.received.get('diagnostics', (None, 0))[0]
        return (self.session is not None and self.fresh('session', now) and
                self.received['session'][0] == self.session and self.fresh('diagnostics', now) and
                diag is not None and diag['session'] == self.session)

    def hold_confirmed(self, now, after):
        if not self.same_session(now) or not self.fresh('state', now) or not self.fresh('flight', now):
            return False
        diag, receipt = self.received['diagnostics']
        return (receipt > after and self.received['state'][1] > after and
                self.received['state'][0].partition(': ')[0] == 'CONTACT_LOCKED' and
                self.received['flight'][0] == 5 and diag['level'] < 2 and
                not any(diag[key] for key in FLAGS))

    def cleanup(self):
        with self.condition:
            if self.cleaned:
                return
            self.cleaned = True
            step = self.step
            if self.relinquished:
                rospy.logwarn('Ownership/session changed; no cancel sent to other work. Hold not confirmed.')
                return
            if self.stop_time is None and (step is None or step.rejected or step.terminal == 'reference_complete'):
                return
            now = time.monotonic()
            if self.stop_time is None and step.terminal and self.hold_confirmed(now, step.terminal_time):
                rospy.loginfo('Terminal jog and inactive contact hold confirmed.')
                return
            if not self.same_session(now):
                rospy.logwarn('Session validity uncertain; no cancellation sent. Hold not confirmed.')
                return
            if self.stop_time is None:
                if rospy.is_shutdown() or self.cancel_pub.get_num_connections() == 0:
                    rospy.logwarn('Cancel connection unavailable; hold not confirmed.')
                    return
                self.stop_time = time.monotonic()
                self.cancel_pub.publish(UInt64(data=self.session))
                rospy.logwarn('Requested braking/hold once for session %d', self.session)
            deadline = time.monotonic() + CANCEL_TIMEOUT
            while not rospy.is_shutdown():
                now = time.monotonic()
                if self.relinquished or not self.same_session(now):
                    break
                terminal = step is None or step.rejected or bool(step.terminal)
                if terminal and self.hold_confirmed(now, max(self.stop_time, step.terminal_time if step else 0)):
                    rospy.loginfo('Terminal outcome and fresh inactive contact hold confirmed.')
                    return
                if now >= deadline:
                    break
                self.condition.wait(min(WAIT_SLICE, deadline - now))
            rospy.logwarn('Hold not confirmed within stop deadline; stop acceptance alone is not completed braking.')


def main():
    # Keep ROS callbacks alive during our bounded Ctrl+C/SIGTERM cleanup.
    interrupted = threading.Event()
    previous = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, lambda *_: interrupted.set())
    experiment = None
    result = 1
    try:
        rospy.init_node('auto_roll_yaw_body_pitch_experiment', anonymous=True, disable_signals=True)
        experiment = Experiment()
        experiment.interrupted = interrupted
        experiment.run()
        result = 0
    except (ValueError, RuntimeError, rospy.ROSException) as exc:
        rospy.logerr('%s', exc)
    finally:
        try:
            if experiment is not None:
                experiment.cleanup()
                rospy.loginfo('Requested=%d admitted=%d reference-completed=%d; '
                              'completed requested increment=%+.6g deg', experiment.requested,
                              experiment.admitted, experiment.completed, experiment.completed * experiment.step_deg)
        finally:
            rospy.signal_shutdown('experiment finished')
            for signum, handler in previous.items():
                signal.signal(signum, handler)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
