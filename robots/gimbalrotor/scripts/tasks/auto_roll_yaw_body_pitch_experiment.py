#!/usr/bin/env python3
"""Send cumulative body-local pitch destinations in one nominal command frame.

interval_sec is hold time AFTER nominal completion and bounded braking.
Ctrl+C/errors stop this owner's motion; normal completion releases the hold.
An accepted destination remains onboard if this process dies without cleanup.
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


class AngleSequence:
    """One acknowledged destination at a time, followed by a full nominal hold."""
    def __init__(self, count, hold, settle=.3):
        self.count, self.hold, self.settle = count, hold, settle
        self.sent = self.sequence = 0
        self.command = None
        self.accepted = False
        self.progress = self.previous_progress = 0.
        self.hold_since = self.ready_since = None

    def submitted(self, command, stamp):
        self.sent += 1
        self.command, self.sequence, self.sent_at = command, command.sequence, stamp
        self.accepted = False
        self.previous_progress = self.progress
        self.hold_since = self.ready_since = None

    def acknowledge(self, ack):
        if not MotionClient.matches_ack(ack, self.command) or ack[0] < self.sent_at:
            return False
        if ack[1].get('accepted') != 'true':
            raise ValueError(ack[2])
        value = float(ack[1].get('accepted_value', 'nan'))
        if not math.isfinite(value) or abs(value - self.command.value) > 1e-9:
            raise ValueError('Navigator acknowledged a different body-pitch target')
        self.accepted = True
        return True

    def matches_status(self, values):
        command = self.command
        target = float(values.get('body_pitch_target_rad', 'nan'))
        return (values.get('body_pitch_target_valid') == '1' and
                all(values.get(key) == str(value) for key, value in (
                    ('session_id', command.session), ('motion_source', command.source),
                    ('motion_epoch', command.epoch), ('motion_sequence', command.sequence),
                    ('body_pitch_target_frame', command.frame))) and
                math.isfinite(target) and abs(target - command.value) <= 1e-9)

    def ready(self, stamp, values, held):
        if self.sequence:
            if not self.accepted or not self.matches_status(values):
                self.ready_since = None
                return False
            self.progress = float(values.get('body_pitch_target_progress_rad', 'nan'))
            if not math.isfinite(self.progress):
                raise ValueError('Nominal body-pitch progress is unavailable')
            if (values.get('body_pitch_target_reference_complete') != '1' or
                    abs(self.command.value - self.progress) > 1e-5 or
                    values.get('body_pitch_velocity_active') != '0' or values.get('stopping') != '0'):
                self.hold_since = self.ready_since = None
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

    def describe(self, state, values):
        progress = self.progress
        try:
            if self.sequence and self.matches_status(values):
                reported = float(values.get('body_pitch_target_progress_rad', 'nan'))
                if math.isfinite(reported):
                    progress = reported  # Report motion even if its acknowledgement was lost.
        except (ValueError, TypeError):
            pass  # Malformed feedback must not prevent the scoped-stop cleanup.
        remaining = (self.command.value - progress) if self.sequence else 0.
        return ('step %d/%d, last reported remaining %+.6f deg; %s; hold_reason=%s; %s; %s' %
                (self.sent, self.count, math.degrees(remaining), state,
                 values.get('hold_reason', 'unavailable'), values.get('body_pitch_velocity_limit', ''),
                 values.get('admittance_status', '')))


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
    if any(rospy.has_param(key) for key in ('~angular_velocity_deg_s', '~feed_duration_sec')):
        raise ValueError('Pulse parameters removed: use ~step_deg and optional ~body_pitch_rate_deg_s; ~interval_sec is hold time')
    step = math.radians(finite_number(rospy.get_param('~step_deg', None), '~step_deg'))
    rate = math.radians(finite_number(rospy.get_param('~body_pitch_rate_deg_s', .5), '~body_pitch_rate_deg_s'))
    limit = math.radians(finite_number(rospy.get_param('~body_pitch_limit_deg', 20.), '~body_pitch_limit_deg'))
    hold = finite_number(rospy.get_param('~interval_sec', 8.), '~interval_sec')
    if rospy.has_param('~hold_duration_sec'):
        alias = finite_number(rospy.get_param('~hold_duration_sec'), '~hold_duration_sec')
        if rospy.has_param('~interval_sec') and alias != hold:
            raise ValueError('~hold_duration_sec and ~interval_sec conflict; specify one hold duration')
        hold = alias
    count = rospy.get_param('~num_steps', 8)
    if step == 0 or rate <= 0 or not 0 < limit < math.pi or hold < 0:
        raise ValueError('Require nonzero step_deg, positive body_pitch_rate_deg_s, 0 < body_pitch_limit_deg < 180 and nonnegative interval_sec')
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('~num_steps must be a positive integer')
    try:
        total = finite_number(step * count, 'Total requested angle')
        default_timeout = finite_number(abs(total) / rate + count * (hold + 10.) + 30., 'Default timeout')
    except OverflowError as exc:
        raise ValueError('Total requested travel/duration must be finite') from exc
    if abs(total) > limit:
        raise ValueError('Requested total %+.6g deg exceeds body-pitch command limit %.6g deg' %
                         (math.degrees(total), math.degrees(limit)))
    ns = rospy.get_param('~robot_namespace', '/gimbalrotor').rstrip('/')
    prefix = rospy.get_param('~debug_prefix', 'perching/roll_yaw').rstrip('/')
    prefix = prefix if prefix.startswith('/') else ns + '/' + prefix
    feedback_timeout = finite_number(rospy.get_param('~feedback_timeout_sec', 1.), '~feedback_timeout_sec')
    timeout = finite_number(rospy.get_param('~timeout_sec', default_timeout), '~timeout_sec')
    if min(feedback_timeout, timeout) <= 0:
        raise ValueError('Timeouts must be positive')
    use_admittance = rospy.get_param('~use_admittance', False)
    if not isinstance(use_admittance, bool):
        raise ValueError('~use_admittance must be a boolean')
    feedback, motion = Feedback(prefix), MotionClient(prefix)
    tare = rospy.Publisher(prefix + '/admittance_tare', Empty, queue_size=1)
    enable = rospy.Publisher(ns + '/perching/admittance_enable', Bool, queue_size=1)
    task = AngleSequence(count, hold)
    interrupted = threading.Event()
    handlers = {number: signal.signal(number, lambda *_: interrupted.set())
                for number in (signal.SIGINT, signal.SIGTERM)}
    started = time.monotonic()
    previous = rospy.get_time()
    connection_deadline = None
    normal_exit = False
    startup = AdmittanceStartup()
    measured_start = None
    state, values = 'waiting for navigator feedback', {}
    next_report = started
    rospy.loginfo('Step %+.6g deg, maximum slew %.6g deg/s, hold %.3g s after nominal completion; %d steps. Ctrl+C stops motion.',
                  math.degrees(step), math.degrees(rate), hold, count)
    try:
        while not rospy.is_shutdown() and not interrupted.is_set():
            now, ros_now = time.monotonic(), rospy.get_time()
            if ros_now < previous:
                raise ValueError('Clock moved backwards')
            previous = ros_now
            status, poses = feedback.snapshot()
            with motion.lock:
                ack = motion.ack
                if motion.epoch is not None and motion.epoch != motion.current_epoch:
                    raise ValueError('Motion cancelled by operator or contact transition; task abandoned')
            if status:
                received, stamp, state, values = status
            if now - started > timeout:
                raise ValueError('Angle sequence blocked/timed out')
            if connection_deadline is not None and now >= connection_deadline:
                raise RuntimeError('Timed out waiting for motion subscriber on ' + prefix + '/motion')
            if not status:
                time.sleep(.02)
                continue
            session = int(values.get('session_id', '0'))
            fresh = (0 <= now - received <= feedback_timeout and -.02 <= ros_now - stamp <= feedback_timeout)
            if motion.session is not None and (session != motion.session or not state.startswith('CONTACT_LOCKED')):
                raise ValueError('Navigation left the active contact: ' + state)
            if not fresh:
                if task.sequence:
                    raise ValueError('Navigator feedback lost; stopping sequence')
                time.sleep(.02)
                continue
            if not session or not state.startswith('CONTACT_LOCKED') or motion.current_epoch is None:
                time.sleep(.02)
                continue
            if motion.session is None:
                motion.rearm(session)  # Pin once, including while waiting for an older owner.
            if not motion.claimed:
                if connection_deadline is None:
                    connection_deadline = now + feedback_timeout + 1
                if not motion.publisher.get_num_connections():
                    interrupted.wait(.02)
                    continue
                if interrupted.is_set() or rospy.is_shutdown():
                    break
                if not motion.claim(session, timeout=min(2., timeout - (time.monotonic() - started)),
                                    interrupted=interrupted):
                    break
                connection_deadline = None
                continue  # Recheck cancellation and feedback after the acknowledgement wait.
            if task.sequence:
                if not task.accepted:
                    if not task.acknowledge(ack):
                        if now - task.sent_at > feedback_timeout + 1:
                            raise ValueError('Angle acknowledgement missing; destination is not retried')
                        time.sleep(.02)
                        continue
                    rospy.loginfo('Step %d/%d acknowledged: target %+.4f deg.', task.sent, count,
                                  math.degrees(task.command.value))
                if received < task.sent_at or not task.matches_status(values):
                    if now - task.sent_at > feedback_timeout + 1:
                        raise ValueError('Current angle status identity/frame/target unavailable')
                    time.sleep(.02)
                    continue
            if use_admittance and task.sequence and (values.get('admittance_enabled') != '1' or values.get('baseline_ready') != '1'):
                raise ValueError('Admittance calibration or enable was lost; sequence stopped')
            if use_admittance and startup.generation is not None and int(values['tare_generation']) != startup.generation:
                raise ValueError('Admittance baseline or contact model changed; sequence stopped')
            if interrupted.is_set() or rospy.is_shutdown():
                break
            if use_admittance and not task.sequence:
                if not startup.ready(now, values, tare, enable):
                    time.sleep(.02)
                    continue
            pose = poses.get('measured')
            measured_fresh = pose and 0 <= now - pose[0] <= feedback_timeout and -.02 <= ros_now - pose[1] <= feedback_timeout
            if task.sequence and not measured_fresh:
                raise ValueError('Measured body pose feedback lost; stopping sequence')
            load_ready = values.get('admittance_enabled') != '1' or values.get('task_torque_valid') == '1'
            # This gate checks nominal hold/contact/model and effective tracking,
            # allowing vibration and correction motion even without auto-enable.
            held = values.get('admittance_hold_eligible') == '1'
            previous_hold = task.hold_since
            ready = task.ready(stamp, values, held and bool(measured_fresh) and load_ready)
            completed = previous_hold is None and task.hold_since is not None
            if task.sequence and (completed or ready or now >= next_report):
                offset = float(values.get('effective_offset_rad', 'nan'))
                measured = local_pitch(measured_start, pose[2])
                rospy.loginfo('Step %d/%d: target %+.4f deg; nominal advance %+.4f deg (run %+.4f), remaining %+.6f deg; '
                              'admittance offset %+.4f deg; measured body-local change since run start %+.4f deg; %s.',
                              task.sent, count, math.degrees(task.command.value),
                              math.degrees(task.progress - task.previous_progress), math.degrees(task.progress),
                              math.degrees(task.command.value - task.progress), math.degrees(offset), math.degrees(measured),
                              'nominal complete, holding' if task.hold_since is not None else 'nominal moving/waiting')
                next_report = now + 2.
            if ready and task.sent == count:
                if interrupted.is_set() or rospy.is_shutdown():
                    break
                rospy.loginfo('Completed %d nominal angle destinations and full holds; measured travel may differ under load.', count)
                normal_exit = True
                return
            if ready and motion.publisher.get_num_connections():
                if interrupted.is_set() or rospy.is_shutdown():
                    break
                if measured_start is None:
                    measured_start = pose[2]
                target = step * (task.sent + 1)
                with motion.lock:
                    sequence = motion.send(session, RollYawMotion.BODY_PITCH, target, rate)
                    command = RollYawMotion(session=session, epoch=motion.epoch, source=motion.source,
                                            sequence=sequence, mode=RollYawMotion.BODY_PITCH, value=target,
                                            frame=motion.frame, rate_limit=rate)
                task.submitted(command, now)
                rospy.loginfo('Step %d/%d dispatched: target %+.4f deg; awaiting navigator acknowledgement.',
                              task.sent, count, math.degrees(target))
            time.sleep(.02)
    except (ValueError, RuntimeError) as exc:
        raise type(exc)(str(exc) + '; ' + task.describe(state, values)) from exc
    finally:
        try:
            if normal_exit:
                if not motion.release():
                    rospy.logwarn('Ownership release unconfirmed; final nominal destination remains held onboard.')
            else:
                rospy.loginfo('Stopping this client: %s', task.describe(state, values))
                if not motion.stop(wait=True):
                    rospy.logerr('Scoped stop was not confirmed; accepted destination may remain active onboard.')
                elif not motion.release():
                    rospy.logwarn('Ownership release after stop unconfirmed; inspect navigator hold status.')
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
