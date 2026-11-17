#!/usr/bin/env python3
"""Send finite onboard pitch-velocity pulses separated by measured holds.

interval_sec is HOLD time; feed_duration_sec defaults to one second. Ctrl+C
ends the sequence while its accepted pulse expires onboard. Errors cancel it.
Optional use_admittance reuses valid tare or collects it unloaded, then verifies enable.
"""

import math
import os
import signal
import sys
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty
from tf.transformations import quaternion_conjugate, quaternion_multiply
try:
    from roll_yaw_motion import MotionClient, RollYawMotion
except ModuleNotFoundError:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from roll_yaw_motion import MotionClient, RollYawMotion


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


def orientation(message):
    q = message.pose.pose.orientation
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values) - 1) > .002:
        raise ValueError('invalid body orientation')
    return values


def local_pitch(start, current):
    relative = quaternion_multiply(quaternion_conjugate(start), current)
    if math.hypot(relative[1], relative[3]) < 1e-6:
        raise ValueError('body-local pitch is undefined at this orientation')
    angle = 2 * math.atan2(relative[1], relative[3])
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Feedback:
    def __init__(self, prefix):
        self.lock = threading.Lock()
        self.status = None
        self.poses = {}
        self.subscribers = [
            rospy.Subscriber(prefix + '/diagnostics_status', DiagnosticArray, self.diagnostic, queue_size=1),
            rospy.Subscriber(prefix + '/body_reference', Odometry, self.pose, callback_args='reference', queue_size=1),
            rospy.Subscriber(prefix + '/body_measured', Odometry, self.pose, callback_args='measured', queue_size=1),
        ]

    def diagnostic(self, message):
        for status in message.status:
            if status.name.endswith('/roll_yaw_perching'):
                values = {v.key: v.value for v in status.values}
                with self.lock:
                    self.status = (time.monotonic(), message.header.stamp.to_sec(), status.message, values)

    def pose(self, message, kind):
        try:
            value = orientation(message)
        except ValueError:
            return  # Invalid data must not refresh the preceding valid sample.
        with self.lock:
            self.poses[kind] = (time.monotonic(), message.header.stamp.to_sec(), value)

    def snapshot(self):
        with self.lock:
            return self.status, dict(self.poses)


class PulseSequence:
    """One accepted pulse at a time; hold starts after the onboard brake."""
    def __init__(self, count, hold, settle=.3):
        self.count, self.hold, self.settle = count, hold, settle
        self.sent = self.sequence = 0
        self.hold_since = self.ready_since = None

    def submitted(self, sequence):
        self.sent += 1
        self.sequence = sequence
        self.hold_since = self.ready_since = None

    def ready(self, stamp, values, held):
        if self.sequence:
            if (int(values.get('pulse_sequence', '0')) != self.sequence or
                    values.get('pulse_complete') != '1'):
                return False
            if self.hold_since is None:
                self.hold_since = stamp
        if held:
            if self.ready_since is None:
                self.ready_since = stamp
        else:
            self.ready_since = None
        return (self.ready_since is not None and stamp - self.ready_since >= self.settle and
                (self.hold_since is None or stamp - self.hold_since >= self.hold))


class AdmittanceStartup:
    """Collect while disarmed, then verify arming, as in no-joint perching.

    Retry only a processed refusal. Missing acknowledgements must not produce
    duplicate tare requests that could later erase an accepted calibration.
    """
    def __init__(self):
        self.started = self.generation = None
        self.pending = None
        self.retry_at = 0.
        self.baseline_seen = False

    def ready(self, now, values, tare, enable):
        generation = int(values['tare_generation'])
        if self.started is None:
            self.started = now
        # Planner retirement is not a physical model change. The navigator
        # validates contact/model identity; this task pins session and tare.
        if values.get('admittance_configured') != '1':
            raise ValueError('Admittance configuration unavailable: ' + values.get('admittance_status', ''))
        if self.pending:
            action, sequence, requested, previous_generation = self.pending
            processed = int(values[action + '_request_sequence'])
            if processed == sequence:
                if now - requested > 2.:
                    raise ValueError(action + ' acknowledgement missing; request is not repeated')
                return False
            if processed != sequence + 1 or (action == 'enable' and values.get('admittance_enable_requested') != '1'):
                raise ValueError('Admittance startup interrupted by another request')
            self.pending = None
            self.retry_at = now + .5
            if action == 'tare' and generation != previous_generation:
                if generation != previous_generation + 1:
                    raise ValueError('Tare changed unexpectedly during startup')
                self.generation = generation
        if self.generation is not None and generation != self.generation:
            raise ValueError('Admittance baseline changed; task abandoned')
        baseline = values.get('baseline_ready') == '1'
        if self.baseline_seen and not baseline:
            raise ValueError('Admittance baseline lost; automatic re-tare is forbidden')
        self.baseline_seen |= baseline
        if values.get('admittance_enabled') == '1':
            if not baseline:
                raise ValueError('Armed admittance has no baseline; disarm and tare unloaded')
            self.generation = generation
            return values.get('admittance_hold_eligible') == '1' and values.get('task_torque_valid') == '1'
        if now - self.started > 30.:
            raise ValueError('No verified admittance baseline/enable within 30 seconds: ' + values.get('admittance_status', ''))
        collecting = values.get('tare_collecting') == '1'
        if baseline or collecting:
            self.generation = generation
        # Controller admission uses nominal hold/contact/tracking, independent of
        # preparation quietness. Fresh diagnostics are required by the caller.
        if values.get('admittance_hold_eligible') != '1' or now < self.retry_at:
            return False
        if not all(abs(float(values.get(field, 'inf'))) <= 1e-8
                   for field in ('offset_rad', 'offset_rate_rad_s', 'offset_acceleration_rad_s2')):
            return False
        if baseline:
            if values.get('task_torque_valid') != '1' or not enable.get_num_connections():
                return False
            action, publisher, message = 'enable', enable, Bool(data=True)
        else:
            if self.generation is not None or collecting:
                return False  # An accepted collection must never be restarted by retry.
            if not tare.get_num_connections():
                return False
            action, publisher, message = 'tare', tare, Empty()
        self.pending = (action, int(values[action + '_request_sequence']), now, generation)
        publisher.publish(message)
        return False


def main():
    rospy.init_node('auto_roll_yaw_body_pitch_experiment', disable_signals=True)
    rate = math.radians(finite_number(rospy.get_param('~angular_velocity_deg_s'), '~angular_velocity_deg_s'))
    duration = finite_number(rospy.get_param('~feed_duration_sec', 1.), '~feed_duration_sec')
    hold = finite_number(rospy.get_param('~hold_duration_sec', rospy.get_param('~interval_sec', 8.)), '~hold_duration_sec')
    count = rospy.get_param('~num_steps', 8)
    if not 0 < duration <= 8 or hold < 0 or isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('Require 0 < feed_duration_sec <= 8, hold_duration_sec >= 0 and positive integer num_steps')
    if not math.isfinite(rate * duration * count) or abs(rate * duration * count) >= math.pi:
        raise ValueError('Total requested feed must be finite and smaller than 180 degrees')
    ns = rospy.get_param('~robot_namespace', '/gimbalrotor').rstrip('/')
    prefix = rospy.get_param('~debug_prefix', 'perching/roll_yaw').rstrip('/')
    prefix = prefix if prefix.startswith('/') else ns + '/' + prefix
    feedback_timeout = finite_number(rospy.get_param('~feedback_timeout_sec', 1.), '~feedback_timeout_sec')
    timeout = finite_number(rospy.get_param('~timeout_sec', count * (duration + hold + 10) + 30), '~timeout_sec')
    if min(feedback_timeout, timeout) <= 0:
        raise ValueError('Timeouts must be positive')
    use_admittance = rospy.get_param('~use_admittance', False)
    if not isinstance(use_admittance, bool):
        raise ValueError('~use_admittance must be a boolean')
    feedback, motion = Feedback(prefix), MotionClient(prefix)
    tare = rospy.Publisher(prefix + '/admittance_tare', Empty, queue_size=1)
    enable = rospy.Publisher(ns + '/perching/admittance_enable', Bool, queue_size=1)
    task = PulseSequence(count, hold)
    interrupted = threading.Event()
    handlers = {number: signal.signal(number, lambda *_: interrupted.set())
                for number in (signal.SIGINT, signal.SIGTERM)}
    started = time.monotonic()
    previous = rospy.get_time()
    sent_at = None
    connection_deadline = None
    normal_exit = False
    startup = AdmittanceStartup()
    measured_start = None
    nominal_start = None
    rospy.loginfo('Feed %+.6g deg/s for %.3g s, then hold %.3g s; %d pulses. Ctrl+C ends the sequence.',
                  math.degrees(rate), duration, hold, count)
    try:
        while not rospy.is_shutdown() and not interrupted.is_set():
            now, ros_now = time.monotonic(), rospy.get_time()
            if ros_now < previous or now - started > timeout:
                raise ValueError('Clock moved backwards or pulse sequence timed out')
            previous = ros_now
            status, poses = feedback.snapshot()
            with motion.lock:
                ack = motion.ack
                if motion.epoch is not None and motion.epoch != motion.current_epoch:
                    raise ValueError('Motion cancelled by operator or contact transition; task abandoned')
            if connection_deadline is not None and now >= connection_deadline:
                raise RuntimeError('Timed out waiting for motion subscriber on ' + prefix + '/motion')
            if not status:
                time.sleep(.02)
                continue
            received, stamp, state, values = status
            session = int(values.get('session_id', '0'))
            fresh = (0 <= now - received <= feedback_timeout and -.02 <= ros_now - stamp <= feedback_timeout)
            if motion.session is not None and (session != motion.session or not state.startswith('CONTACT_LOCKED')):
                raise ValueError('Navigation left the active contact: ' + state)
            if not fresh:
                if task.sequence:
                    raise ValueError('Navigator feedback lost; cancelling sequence')
                time.sleep(.02)
                continue
            if not session or not state.startswith('CONTACT_LOCKED') or motion.current_epoch is None:
                time.sleep(.02)
                continue
            if motion.session is None:
                motion.rearm(session)  # Pin once, including while waiting for an older owner.
            if rate == 0:
                if not motion.cancel(session):
                    raise ValueError('Operator stop was not confirmed')
                normal_exit = True
                return
            if not motion.claimed:
                if connection_deadline is None:
                    connection_deadline = now + feedback_timeout + 1
                if not motion.publisher.get_num_connections():
                    # Keep checking session, epoch and cancellation in the main loop.
                    # Status/epoch subscribers may connect before this publisher.
                    interrupted.wait(.02)
                    continue
                if interrupted.is_set() or rospy.is_shutdown():
                    break
                if not motion.claim(session, timeout=min(2., timeout - (time.monotonic() - started)),
                                    interrupted=interrupted):
                    break
                connection_deadline = None
                continue  # Recheck cancellation and feedback after the acknowledgement wait.
            if sent_at is not None:
                matched = ack and int(ack[1].get('sequence', '0')) == task.sequence
                if matched and ack[1].get('accepted') != 'true':
                    raise ValueError(ack[2])
                if not matched:
                    if now - sent_at > feedback_timeout + 1:
                        raise ValueError('Pulse acknowledgement missing; pulse is not retried')
                    time.sleep(.02)
                    continue
            quiet = (values.get('feedback_waiting', '1') == '0' and values.get('stopping', '1') == '0' and
                     values.get('body_pitch_velocity_active', '0') == '0' and
                     values.get('accepted_destination_pending', '0') == '0' and
                     float(values.get('current_body_rate_rad_s', 'inf')) < float(values.get('settling_body_limit_rad_s', '.05')) and
                     float(values.get('current_joint_rate_rad_s', 'inf')) < float(values.get('settling_joint_limit_rad_s', '.03')) and
                     float(values.get('effective_body_tracking_error_rad', 'inf')) < float(values.get('effective_body_tracking_limit_rad', '.04')))
            if use_admittance and task.sequence and (values.get('admittance_enabled') != '1' or values.get('baseline_ready') != '1'):
                raise ValueError('Admittance calibration or enable was lost; sequence stopped')
            if use_admittance and startup.generation is not None and int(values['tare_generation']) != startup.generation:
                raise ValueError('Admittance baseline or contact model changed; sequence stopped')
            if use_admittance and not task.sequence:
                if not startup.ready(now, values, tare, enable):
                    time.sleep(.02)
                    continue
            pose = poses.get('measured')
            measured_fresh = pose and 0 <= now - pose[0] <= feedback_timeout and -.02 <= ros_now - pose[1] <= feedback_timeout
            load_ready = values.get('admittance_enabled') != '1' or values.get('task_torque_valid') == '1'
            # Later pulses allow legitimate compliant retreat. The controller
            # checks nominal completion/braking and effective tracking; the
            # sequence still enforces pulse identity and the entire hold time.
            held = values.get('admittance_hold_eligible') == '1' if use_admittance else quiet
            ready = task.ready(stamp, values, held and bool(measured_fresh) and load_ready)
            if ready and task.sequence and measured_start is not None:
                progress = math.degrees(local_pitch(measured_start, pose[2]))
                rospy.loginfo('Pulse %d measured pitch change: %+.4f deg (requested ideal %+.4f deg).',
                              task.sent, progress, math.degrees(rate * duration))
                if nominal_start is not None:
                    nominal_progress = float(values.get('local_pitch_nominal_rad', 'nan')) - nominal_start
                    if not math.isfinite(nominal_progress) or rate * nominal_progress <= 0:
                        raise ValueError('Pulse produced no nominal feed under admittance; holding instead of sending another pulse')
                    # A compliant retreat can exceed this pulse's nominal feed.
                    # Its signed measured travel is reported, never called a tracking failure.
                elif rate * progress <= 0 or abs(progress) < .01 * abs(math.degrees(rate * duration)):
                    raise ValueError('Pulse produced no measured forward progress; holding instead of sending another pulse')
                measured_start = None
            if ready and task.sent == count:
                rospy.loginfo('Completed %d pulses and holds; reported progress is measured, not inferred from time.', count)
                normal_exit = True
                return
            if ready and motion.publisher.get_num_connections():
                # No refresh and no queued successor. Expiry is enforced onboard.
                if interrupted.is_set():
                    break
                nominal_start = float(values.get('local_pitch_nominal_rad', 'nan')) if values.get('admittance_enabled') == '1' else None
                if nominal_start is not None and not math.isfinite(nominal_start):
                    raise ValueError('Admittance nominal pitch reference is unavailable')
                sequence = motion.send(session, RollYawMotion.PULSE, rate, duration=duration)
                task.submitted(sequence)
                measured_start = pose[2]
                sent_at = now
                rospy.loginfo('Pulse %d/%d accepted for dispatch; nominal feed %.3g deg', task.sent, count, math.degrees(rate * duration))
            time.sleep(.02)
        normal_exit = True  # Ctrl+C leaves only the already dispatched finite pulse.
    finally:
        try:
            if normal_exit and not motion.release():
                rospy.logwarn('Ownership release unconfirmed; pulse still expires onboard. Next client must claim while idle.')
            elif not normal_exit and not motion.stop(wait=True):
                rospy.logerr('Stop was not confirmed; inspect navigator hold status.')
        finally:
            for number, handler in handlers.items():
                signal.signal(number, handler)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except (ValueError, RuntimeError) as exc:
        rospy.logerr('%s', exc)
        raise SystemExit(1)
