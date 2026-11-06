#!/usr/bin/env python3
"""Absolute mechanism targets and finite body-local pitch jogs; only the navigator drives actuators."""
import math
import os
import re
import select
import sys
import termios
import threading
import time
import tty

import rospy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String, UInt8, UInt64, Float64

HELP = """
ROLL / YAW PERCHING — small mechanism steps (default 0.2 degrees)
  j / l  active yaw joint - / +       a / f  active roll joint - / +
  i / k  passive hinge alpha + / - (lock-relative, not world Euler pitch)
  o / u  one body-local pitch jog down (+) / up (-), default 0.2 degrees
  SPACE  brake along the applied path, then hold contact
  e / d  enable / disable             z / b  pitch rezero / relock
  r / t  arm / take off               g / h  land / HALT MOTORS
  s / ?  status / help               q / Ctrl-C  hold and quit

Setup: physically perch -> e -> z -> wait PITCH_REZERO_HOLD -> b -> navigate.
Rezero is explicit, freezes active joints and waits for measured pitch/rate.
Manual motion is also available immediately after capture when no setup is active.
Relock stays perched and establishes a new measured baseline. The b key follows multilink final-relock checks: fresh ready, not active, not failed.
General /perching/relock uses base enabled/HOVER capture, without a stopped gate.
Roll/yaw are URDF joint coordinates: positive joints can rotate the body negatively.
The body reference/measurement is published separately; no passive actuator exists.
Body alignment uses separate body_yaw_target/body_roll_target Float64 topics.
Cut commands use prepare/start/stop with current session; wait for measured READY before start.
During an automatic motion, a manual key requests braking and is rejected: retry after stop complete.
Mechanism keys update only their own accumulated nominal destination, like multilink.
o/u are discrete coordinated jogs about applied body +Y; wait for reference completion (measured error is reported separately).
Busy jog requests are rejected, never accumulated or used to cancel the active jog.
body_pitch_delta is a relative increment in radians; ~body_pitch_step_deg is separate
from passive ~pitch_step_deg. Navigator rejects increments above 1 degree. Watch command_ack for moving/terminal status.
Feedback loss cancels a finite jog: restore feedback and send a fresh key. Automatic cuts use explicit resume.
Jogs are geometric only: no admittance/tare. Prepare a cut, reach READY, tare, enable,
then start for compliant cutting. Space cancels preparation or motion and holds.
Other destinations survive quick axis changes. Space holds without zeroing joints.
Targets clamp to navigator effective limits; reversal at a limit takes one step.
Quitting preserves contact hold and restores the terminal. No saw command is sent.
"""


class Keyboard:
    def __init__(self):
        self.lock = threading.RLock()
        self.state = ('', '')
        self.state_time = self.flight_time = self.angles_time = 0.0
        self.session = self.flight = self.angles = None
        self.angles_frame = ''
        self.commanded = None
        self.pending_axes = [False] * 3
        self.jog_busy = False
        self.jog_status_time = self.jog_request_time = self.nominal_after_jog = 0.
        self.command_times = [0.] * 3
        self.position_limits = {'lower': None, 'upper': None}  # wire: roll, alpha, yaw
        self.limit_times = {'lower': 0., 'upper': 0.}
        self.applied = None
        self.applied_time = 0.
        self.rezero = {}
        self.rezero_times = {}
        self.rezero_request_time = None
        self.body_rpy = None
        self.body_time = 0.0
        self.quit = False
        self.enable_pending = None
        self.last_display = None
        self.last_display_time = 0.0
        self.relock_pending = None
        self.had_feedback = False
        self.ns = rospy.get_param('~robot_namespace', '/gimbalrotor').rstrip('/')
        debug_prefix = rospy.get_param('~debug_prefix', 'perching/roll_yaw').rstrip('/')
        self.prefix = debug_prefix if debug_prefix.startswith('/') else self.ns + '/' + debug_prefix
        self.steps = {}
        for axis in ('yaw', 'roll', 'pitch', 'body_pitch'):
            step = float(rospy.get_param('~' + axis + '_step_deg', .2))
            if not math.isfinite(step) or not 0 < step <= 1:
                raise ValueError('~{}_step_deg must be positive and <= 1 degree.'.format(axis))
            self.steps[axis] = math.radians(step)
        self.pubs = {
            'roll': rospy.Publisher(self.prefix + '/roll_target', Float64, queue_size=1),
            'pitch': rospy.Publisher(self.prefix + '/alpha_target', Float64, queue_size=1),
            'yaw': rospy.Publisher(self.prefix + '/yaw_target', Float64, queue_size=1),
            'body_pitch': rospy.Publisher(self.prefix + '/body_pitch_delta', Float64, queue_size=1),
            'stop': rospy.Publisher(self.prefix + '/cancel', UInt64, queue_size=1),
            'rezero': rospy.Publisher(self.prefix + '/pitch_rezero', Empty, queue_size=1),
            'relock': rospy.Publisher(self.ns + '/perching/relock', Empty, queue_size=1),
            'enable': rospy.Publisher(self.ns + '/perching/enable', Bool, queue_size=1),
        }
        for action, topic in (('arm', 'start'), ('takeoff', 'takeoff'), ('land', 'land'), ('halt', 'halt')):
            self.pubs[action] = rospy.Publisher(self.ns + '/teleop_command/' + topic, Empty, queue_size=1)
        self.subs = [
            rospy.Subscriber(self.prefix + '/command_ack', String, self.ack_callback, queue_size=10),
            rospy.Subscriber(self.prefix + '/state', String, self.state_callback, queue_size=1),
            rospy.Subscriber(self.prefix + '/session', UInt64, self.session_callback, queue_size=1),
            rospy.Subscriber(self.prefix + '/nominal_angles', Vector3Stamped, self.angles_callback, queue_size=1),
            rospy.Subscriber(self.prefix + '/target_angles', Vector3Stamped, self.applied_callback, queue_size=1),
            rospy.Subscriber(self.prefix + '/body_measured', Odometry, self.body_callback, queue_size=1),
            rospy.Subscriber(self.ns + '/flight_state', UInt8, self.flight_callback, queue_size=1),
        ]
        for flag in ('active', 'ready', 'failed'):
            self.subs.append(rospy.Subscriber(self.prefix + '/pitch_rezero_' + flag, Bool,
                                             self.rezero_callback, callback_args=flag, queue_size=1))
        for bound in ('lower', 'upper'):
            self.subs.append(rospy.Subscriber(self.prefix + '/position_' + bound, Vector3Stamped,
                                             self.limits_callback, callback_args=bound, queue_size=1))

    def ack_callback(self, msg):
        session = re.search(r'\bsession=(\d+)\b', msg.data)
        with self.lock:
            if session and int(session[1]) == self.session and msg.data.startswith('action=body_pitch_delta '):
                now = time.monotonic()
                phase = re.search(r'\bphase=(\w+)\b', msg.data)
                if phase and phase[1] == 'moving':
                    self.jog_busy, self.jog_status_time = True, now
                    self.commanded = None
                    self.pending_axes = [False] * 3
                elif phase and phase[1] in ('reference_complete', 'cancelled', 'failed', 'rejected', 'noop'):
                    self.jog_busy = False
                    self.commanded = None
                    self.pending_axes = [False] * 3
                    self.nominal_after_jog = now
                print('\nNavigator: ' + msg.data, flush=True)

    def state_callback(self, msg):
        stage, _, reason = msg.data.partition(': ')
        with self.lock:
            correction = re.search(r'\[axis=(alpha|roll|yaw) lock=(\d+) requested=(\S+) accepted=(\S+)\]', reason)
            if correction and int(correction[2]) == self.session and self.commanded is not None:
                axis = {'roll': 0, 'alpha': 1, 'yaw': 2}[correction[1]]
                try:
                    requested, accepted = float(correction[3]), float(correction[4])
                except ValueError:
                    requested = accepted = float('nan')
                # Ignore an older result after a newer key on this axis. Other
                # pending coordinates must never be reset by this correction.
                if (math.isfinite(requested) and math.isfinite(accepted) and self.limits_valid()
                        and self.pending_axes[axis] and abs(self.commanded[axis] - requested) < 1e-9):
                    self.commanded[axis] = self.clamp_target(axis, accepted)
                    self.pending_axes[axis] = False
            if stage.startswith('PITCH_REZERO_') or stage in ('FREE_FLIGHT', 'FAULT_HOLD', 'LOCK_FAILED'):
                self.commanded = None
                self.pending_axes = [False] * 3
            now = time.monotonic()
            active = re.search(r'\bjog_active=(0|1)\b', reason)
            if active and (active[1] == '1' or now - self.jog_request_time > .5):
                if self.jog_busy and active[1] == '0':
                    self.commanded = None
                    self.pending_axes = [False] * 3
                    self.nominal_after_jog = now
                self.jog_busy, self.jog_status_time = active[1] == '1', now
            self.state, self.state_time = (stage, reason), now

    def session_callback(self, msg):
        with self.lock:
            if self.session != msg.data:
                self.jog_busy = False
                self.jog_status_time = self.jog_request_time = self.nominal_after_jog = 0.
                self.angles = self.commanded = self.applied = None
                self.pending_axes = [False] * 3
                self.position_limits = {'lower': None, 'upper': None}
                self.limit_times = {'lower': 0., 'upper': 0.}
                # Remote stamp is identity only; local receipt/request clocks
                # provide freshness, as in the multilink keyboard.
            self.session = msg.data

    def flight_callback(self, msg):
        with self.lock:
            self.flight, self.flight_time = msg.data, time.monotonic()

    def angles_callback(self, msg):
        values = (msg.vector.x, msg.vector.y, msg.vector.z)  # roll, alpha, yaw NOMINAL
        now = time.monotonic()
        with self.lock:
            if msg.header.frame_id != 'perching_lock_' + str(self.session):
                return
            self.angles = values if all(math.isfinite(x) for x in values) else None
            self.angles_frame, self.angles_time = msg.header.frame_id, now
            if self.angles is None:
                self.commanded = None
                self.pending_axes = [False] * 3
                return
            self.reconcile_nominal(now)

    def limits_callback(self, msg, bound):
        values = (msg.vector.x, msg.vector.y, msg.vector.z)  # roll, alpha, yaw
        with self.lock:
            if msg.header.frame_id != 'perching_lock_' + str(self.session):
                return
            self.position_limits[bound] = values if all(math.isfinite(v) for v in values) else None
            self.limit_times[bound] = time.monotonic()
            if not self.limits_valid():
                self.commanded = None
                self.pending_axes = [False] * 3
            elif self.commanded is None:
                self.reconcile_nominal(time.monotonic())
            else:
                # Repeated bound messages are not new nominal acknowledgements.
                self.commanded = [self.clamp_target(i, v) for i, v in enumerate(self.commanded)]

    def limits_valid(self):
        lower, upper = self.position_limits['lower'], self.position_limits['upper']
        return lower is not None and upper is not None and all(lo < hi for lo, hi in zip(lower, upper))

    def clamp_target(self, axis, value):
        return max(self.position_limits['lower'][axis], min(self.position_limits['upper'][axis], value))

    def reconcile_nominal(self, now):
        # Called under the state lock. Bounds may arrive before or after nominal
        # feedback; neither callback publishes a movement on startup/relock.
        if self.angles is None or not self.limits_valid():
            return
        if self.commanded is None:
            self.commanded = [self.clamp_target(i, v) for i, v in enumerate(self.angles)]
        for axis in range(3):
            self.commanded[axis] = self.clamp_target(axis, self.commanded[axis])
            accepted = self.clamp_target(axis, self.angles[axis])
            acknowledged = abs(accepted - self.commanded[axis]) < 1e-9
            if not self.pending_axes[axis] or acknowledged or now - self.command_times[axis] > .5:
                self.commanded[axis] = accepted
                self.pending_axes[axis] = False

    def applied_callback(self, msg):
        values = (msg.vector.x, msg.vector.y, msg.vector.z)
        with self.lock:
            if msg.header.frame_id == 'perching_lock_' + str(self.session) and all(math.isfinite(v) for v in values):
                self.applied, self.applied_time = values, time.monotonic()

    def rezero_callback(self, msg, flag):
        with self.lock:
            self.rezero[flag], self.rezero_times[flag] = msg.data, time.monotonic()

    def body_callback(self, msg):
        q = msg.pose.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values)-1) > .002:
            return
        rpy = (math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
               math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x)))),
               math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        with self.lock:
            self.body_rpy, self.body_time = rpy, time.monotonic()

    def snapshot(self):
        with self.lock:
            return dict(stage=self.state[0], reason=self.state[1], stamp=self.state_time,
                        flight=self.flight, flight_time=self.flight_time, session=self.session,
                        angles=self.angles, angles_frame=self.angles_frame, angles_time=self.angles_time,
                        limits_valid=self.limits_valid(), limit_times=dict(self.limit_times),
                        body_rpy=self.body_rpy, body_time=self.body_time,
                        applied=self.applied, applied_time=self.applied_time,
                        rezero=dict(self.rezero), rezero_times=dict(self.rezero_times))

    @staticmethod
    def fresh(s):
        return time.monotonic() - s['stamp'] < 1 and time.monotonic() - s['flight_time'] < 1

    @staticmethod
    def has_angles(s):
        return (s['angles'] is not None and time.monotonic() - s['angles_time'] < 1
                and s['angles_frame'] == 'perching_lock_' + str(s['session']))

    @staticmethod
    def has_limits(s):
        return s['limits_valid'] and all(time.monotonic() - t < 1 for t in s['limit_times'].values())

    def publish(self, action, message):
        if not self.pubs[action].get_num_connections():
            raise RuntimeError('No navigator connection for ' + action)
        self.pubs[action].publish(message)

    def movement_rejection(self, s):
        if s['session'] is None or s['stage'] == 'FREE_FLIGHT':
            return 'No confirmed contact session.'
        if self.relock_pending:
            return ('Waiting for relock acknowledgement and new target baseline.')
        if s['stage'] in ('FAULT_HOLD', 'PITCH_REZERO_FAILED'):
            return ('Contact/setup fault: ' + s['reason'])
        if s['stage'].startswith('PITCH_REZERO_'):
            return ('Setup owns target: wait ready-HOLD, then b to relock.')
        if not self.has_limits(s):
            return 'Need valid, fresh effective position bounds for the current lock.'
        if not self.fresh(s) or not self.has_angles(s) or s['flight'] != 5:
            return ('Need fresh local receipts of HOVER and current-session targets.')
        if any(not self.pubs[axis].get_num_connections() for axis in ('pitch', 'roll', 'yaw')):
            return 'No navigator connection for mechanism targets.'
        return ''

    def manual(self, axis=None, sign=0):
        s = self.snapshot()
        if s['session'] is None or s['stage'] == 'FREE_FLIGHT':
            if axis is None:
                return
            raise RuntimeError('No confirmed contact session.')
        if axis is None:
            self.publish('stop', UInt64(s['session']))
            with self.lock:
                self.commanded = ([self.clamp_target(i, v) for i, v in enumerate(s['applied'])]
                                  if s['applied'] is not None and self.limits_valid() else None)
                self.pending_axes = [False] * 3
            return
        rejection = self.movement_rejection(s)
        if rejection:
            raise RuntimeError(rejection)
        index = {'roll': 0, 'pitch': 1, 'yaw': 2}[axis]
        with self.lock:
            if self.session != s['session']:
                raise RuntimeError('Lock changed; waiting for its nominal baseline.')
            if self.angles_time < self.nominal_after_jog:
                raise RuntimeError('Waiting for fresh accepted nominal vector after coordinated jog.')
            if not self.has_limits(self.snapshot()):
                raise RuntimeError('Need valid, fresh effective position bounds for the current lock.')
            if self.commanded is None:
                self.reconcile_nominal(time.monotonic())
            if self.commanded is None:
                raise RuntimeError('Waiting for a valid bounded nominal baseline.')
            value = self.clamp_target(index, self.commanded[index] + sign * self.steps[axis])
            self.publish(axis, Float64(value))
            self.commanded[index] = value
            self.pending_axes[index] = True
            self.command_times[index] = time.monotonic()

    def show_status(self):
        s = self.snapshot()
        print('\nFlight={} | {} | session={} | {}'.format(
            s['flight'], s['stage'] or 'waiting', s['session'], s['reason']), flush=True)
        if self.has_angles(s) and s['stage'] != 'FREE_FLIGHT':
            print('Navigator nominal destinations [deg]: roll joint={:+.3f}, alpha={:+.3f}, yaw joint={:+.3f}'.format(
                *map(math.degrees, s['angles'])), flush=True)
        if s['applied'] is not None and time.monotonic() - s['applied_time'] < 1:
            print('Applied mechanism reference [deg]: roll={:+.3f}, alpha={:+.3f}, yaw={:+.3f}'.format(
                *map(math.degrees, s['applied'])), flush=True)
        if s['body_rpy'] is not None and time.monotonic() - s['body_time'] < 1:
            print('Measured body WORLD RPY [deg]: {:+.3f}, {:+.3f}, {:+.3f}'.format(
                *map(math.degrees, s['body_rpy'])), flush=True)
        print('Input: ' + (self.movement_rejection(s) or 'available; navigator validates limits/contact feedback'), flush=True)
        print('Step [deg]: yaw={:.3f}, roll={:.3f}, alpha={:.3f}, body-local pitch jog={:.3f}{}'.format(
            *(math.degrees(self.steps[a]) for a in ('yaw', 'roll', 'pitch', 'body_pitch')),
            '' if self.fresh(s) else ' | feedback stale'), flush=True)

    def tick(self):
        s = self.snapshot()
        fresh = self.fresh(s)
        if self.had_feedback and not fresh and s['stage'] == 'CONTACT_LOCKED':
            self.manual()  # Manual hold only; automatic paths retain explicit pause/resume.
        self.had_feedback = fresh
        if self.enable_pending:
            old_session, started = self.enable_pending
            if fresh and s['stage'] == 'CONTACT_LOCKED' and self.has_angles(s) and s['session'] != old_session:
                print('\nCONTACT_LOCKED confirmed, session {}.'.format(s['session']), flush=True)
                self.enable_pending = None
            elif time.monotonic() - started > 3:
                print('\nEnable not confirmed: ' + s['reason'], flush=True)
                self.enable_pending = None
        if self.relock_pending:
            previous, started = self.relock_pending
            if s['session'] != previous and self.fresh(s) and self.has_angles(s):
                self.relock_pending = None
                self.rezero_request_time = None
                print('Relock acknowledged; new applied mechanism baseline received.', flush=True)
            elif time.monotonic() - started > 3:
                self.relock_pending = None
                print('Relock not confirmed: ' + s['reason'], flush=True)
        display = (s['flight'], s['stage'], s['reason'], s['session'], fresh)
        if display != self.last_display and time.monotonic() - self.last_display_time > .25:
            self.last_display, self.last_display_time = display, time.monotonic()
            self.show_status()

    def key(self, key):
        if key in ('q', '\x03', ''):
            self.quit = True
            self.manual()
        elif key in ('h', 'g'):
            self.enable_pending = None
            self.publish('halt' if key == 'h' else 'land', Empty())
        elif key == 'd':
            self.enable_pending = None
            self.publish('enable', Bool(False))
            print('\nDisable requested; perching references invalidated.', flush=True)
        elif key == ' ':
            self.manual()
            print('\nStop/hold requested; outstanding input discarded.', flush=True)
        elif key == '?':
            print(HELP, flush=True)
        elif key == 's':
            self.show_status()
        elif key in 'jlafik':
            axis, sign = {'j': ('yaw', -1), 'l': ('yaw', 1), 'a': ('roll', -1),
                          'f': ('roll', 1), 'i': ('pitch', 1), 'k': ('pitch', -1)}[key]
            self.manual(axis, sign)
        elif key in ('o', 'u'):
            rejection = self.movement_rejection(self.snapshot())
            if rejection:
                raise RuntimeError(rejection)
            with self.lock:
                now = time.monotonic()
                # Fresh status renews busy; a missing acknowledgement expires.
                # Navigator admission remains authoritative and never queues keys.
                if self.jog_busy and now - max(self.jog_status_time, self.jog_request_time) < 1:
                    raise RuntimeError('Body-local pitch busy; key suppressed (not queued).')
                self.publish('body_pitch', Float64(self.steps['body_pitch'] * (1 if key == 'o' else -1)))
                self.jog_busy, self.jog_request_time = True, now
                self.commanded = None
                self.pending_axes = [False] * 3
            print('\nBody-local pitch jog sent; awaiting navigator acknowledgement (no input backlog).', flush=True)
        elif key in ('z', 'b'):
            s = self.snapshot()
            if not self.fresh(s) or not self.has_angles(s) or s['flight'] != 5:
                raise RuntimeError('Need fresh confirmed contact feedback for setup.')
            if key == 'z':
                if s['stage'].startswith('PITCH_REZERO_') or s['stage'] == 'FAULT_HOLD':
                    raise RuntimeError('Rezero requires IDLE setup and valid contact.')
                self.rezero_request_time = time.monotonic()
                self.publish('rezero', Empty())
            else:
                now = time.monotonic()
                after = self.rezero_request_time or 0.
                flags_fresh = all(now - s['rezero_times'].get(flag, 0.) < 1 and
                                  s['rezero_times'].get(flag, 0.) >= after
                                  for flag in ('active', 'ready', 'failed'))
                if not flags_fresh or s['rezero'] != {'active': False, 'ready': True, 'failed': False}:
                    raise RuntimeError('Final relock requires fresh rezero ready, not active, not failed (multilink workflow).')
                self.publish('relock', Empty())
                self.relock_pending = (s['session'], time.monotonic())
        elif key in ('r', 't', 'e'):
            s = self.snapshot()
            if not self.fresh(s):
                raise RuntimeError('Waiting for fresh robot state.')
            if key == 'r' and s['flight'] in (0, 6) and s['stage'] == 'FREE_FLIGHT':
                self.publish('arm', Empty())
            elif key == 't' and s['flight'] == 2:
                self.publish('takeoff', Empty())
            elif key == 'e' and s['flight'] == 5 and s['stage'] == 'FREE_FLIGHT' and not self.enable_pending:
                self.publish('enable', Bool(True))
                self.enable_pending = (s['session'], time.monotonic())
                print('\nEnable requested; awaiting navigator confirmation.', flush=True)
            else:
                raise RuntimeError('Command unavailable in current state; press s for reason.')


def main():
    if not sys.stdin.isatty():
        raise RuntimeError('Run this keyboard in a terminal.')
    rospy.init_node('keyboard_roll_yaw_perching', disable_signals=True)
    ui = Keyboard()
    print(HELP, flush=True)
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown() and not ui.quit:
            try:
                ui.tick()
                if select.select([sys.stdin], [], [], .05)[0]:
                    keys = os.read(sys.stdin.fileno(), 4096).decode('ascii', errors='ignore')
                    # Safety keys take priority. Otherwise preserve alternating axis taps.
                    safety = next((k for k in ('h', 'g', '\x03', 'q', 'd', ' ') if k in keys), None)
                    if safety is not None:
                        termios.tcflush(sys.stdin, termios.TCIFLUSH)
                        ui.key(safety)
                    else:
                        for key in keys:
                            ui.key(key)
            except (ValueError, RuntimeError, rospy.ROSException) as exc:
                rospy.logwarn_throttle(1.0, str(exc))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            ui.manual()
        except (RuntimeError, rospy.ROSException):
            pass


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    except (ValueError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
