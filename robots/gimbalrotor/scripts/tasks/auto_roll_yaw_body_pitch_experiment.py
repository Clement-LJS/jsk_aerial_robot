#!/usr/bin/env python3
"""Execute a measured body-local Y stroke through the existing velocity input.

The signed angle is angular_velocity_deg_s * interval_sec * num_steps. Holds
retain that goal and contact session; elapsed time is not completed motion.
No enable/relock/tare is sent. A fault, changed session, timeout or interruption
cancels this motion owner before ROS shutdown. Completion requires both reference and measured progress.
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


class Stroke:
    """Task progress independent of ROS transport, for bag/regression checks."""
    def __init__(self, rate, duration, tolerance, progress_timeout, settle_time=.3):
        self.rate, self.goal = rate, rate * duration
        self.tolerance = min(tolerance, abs(self.goal) * .1) if self.goal else tolerance
        self.progress_timeout, self.settle_time = progress_timeout, settle_time
        self.reference_tolerance = min(1e-4, abs(self.goal) * .01)
        self.session = None
        self.reference_start = self.measured_start = None
        self.started = self.last_progress = self.settled_since = None
        self.best_progress = 0.
        self.reference_progress = self.measured_progress = 0.
        self.reason = 'waiting for fresh enabled contact'
        self.complete = False

    def step(self, now, session, state, reference, measured, fresh, holding, stopping, horizon, sample_stamp=None):
        self.complete = False
        sample_stamp = now if sample_stamp is None else sample_stamp
        stage = state.partition(':')[0]
        if stage in ('FAULT_HOLD', 'LOCK_FAILED'):
            raise ValueError('navigation left the active contact: ' + state)
        if self.started is not None:
            if session != self.session:
                raise ValueError('contact session changed; remaining stroke was not recaptured')
            if stage in ('FREE_FLIGHT', 'FAULT_HOLD', 'LOCK_FAILED') or stage.startswith('PITCH_REZERO_'):
                raise ValueError('navigation left the active contact: ' + state)
            if now - self.last_progress > self.progress_timeout:
                raise ValueError('no measured progress before timeout: ' + self.reason)
        if not fresh or holding or stage != 'CONTACT_LOCKED':
            self.settled_since = None
            self.reason = 'holding remaining stroke: ' + state
            return 0.
        if self.started is None:
            if not session or stopping:
                return 0.
            self.session = session
            self.reference_start, self.measured_start = reference, measured
            self.started = self.last_progress = now
        self.reference_progress = local_pitch(self.reference_start, reference)
        self.measured_progress = local_pitch(self.measured_start, measured)
        direction = 1 if self.rate > 0 else -1
        forward = direction * self.measured_progress
        if forward > self.best_progress + min(self.tolerance * .1, abs(self.goal) * .01):
            self.best_progress = forward
            self.last_progress = now
        remaining = direction * (self.goal - self.reference_progress)
        if remaining <= self.reference_tolerance:
            self.reason = 'reference complete; waiting for measured endpoint and stop'
            endpoint = quaternion_multiply(self.measured_start,
                                          (0., math.sin(self.goal / 2), 0., math.cos(self.goal / 2)))
            error = quaternion_multiply(quaternion_conjugate(endpoint), measured)
            angular_error = 2 * math.atan2(math.sqrt(sum(x*x for x in error[:3])), abs(error[3]))
            if angular_error <= self.tolerance and not stopping:
                if self.settled_since is None:
                    self.settled_since = sample_stamp
                self.complete = sample_stamp - self.settled_since >= self.settle_time
            else:
                self.settled_since = None
            return 0.
        self.settled_since = None
        self.reason = 'advancing remaining stroke'
        # Slow near the endpoint, leaving time for the onboard bounded brake.
        return direction * min(abs(self.rate), remaining / horizon)


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


def main():
    rospy.init_node('auto_roll_yaw_body_pitch_experiment', disable_signals=True)

    interval_sec = finite_number(rospy.get_param('~interval_sec', 8.0), '~interval_sec')
    if rospy.has_param('~step_deg'):
        raise ValueError('~step_deg is an obsolete angle parameter; supply ~angular_velocity_deg_s explicitly')
    if not rospy.has_param('~angular_velocity_deg_s'):
        raise ValueError('~angular_velocity_deg_s is required (degrees/second; no default speed)')
    angular_velocity_deg_s = finite_number(rospy.get_param('~angular_velocity_deg_s'), '~angular_velocity_deg_s')
    num_steps = rospy.get_param('~num_steps', 8)
    if interval_sec <= 0:
        raise ValueError('~interval_sec must be strictly positive')
    if isinstance(num_steps, bool) or not isinstance(num_steps, int) or num_steps <= 0:
        raise ValueError('~num_steps must be a positive integer (not a boolean or fraction)')
    angular_velocity_rad_s = math.radians(angular_velocity_deg_s)
    if angular_velocity_deg_s != 0 and angular_velocity_rad_s == 0:
        raise ValueError('~angular_velocity_deg_s is too small to represent in radians/second')

    ns = rospy.get_param('~robot_namespace', '/gimbalrotor')
    prefix = rospy.get_param('~debug_prefix', 'perching/roll_yaw')
    if not isinstance(ns, str) or not isinstance(prefix, str) or not prefix.strip('/'):
        raise ValueError('namespace and debug_prefix must be strings; prefix must be nonempty')
    ns, prefix = ns.rstrip('/'), prefix.rstrip('/')
    topic_prefix = prefix if prefix.startswith('/') else ns + '/' + prefix
    duration_param = ns + '/navigation/roll_yaw_perching/body_pitch_velocity_command_duration'
    if not rospy.has_param(duration_param):
        raise ValueError('navigator command duration is missing: ' + duration_param)
    command_duration = finite_number(rospy.get_param(duration_param), duration_param)
    if command_duration <= 0:
        raise ValueError(duration_param + ' must be positive')
    refresh_period = finite_number(rospy.get_param('~refresh_period_sec', command_duration / 4),
                                   '~refresh_period_sec')
    if not 0 < refresh_period <= command_duration / 2:
        raise ValueError('~refresh_period_sec must be positive and at most half the navigator lease')
    experiment_duration = interval_sec * num_steps
    if not math.isfinite(experiment_duration):
        raise ValueError('experiment duration must be finite')
    angle = angular_velocity_rad_s * experiment_duration
    if not math.isfinite(angle) or abs(angle) >= math.pi:
        raise ValueError('one body-local stroke must be finite and smaller than 180 degrees')
    navigation_prefix = ns + '/navigation/roll_yaw_perching/'
    tolerance = finite_number(rospy.get_param('~goal_tolerance_deg',
        math.degrees(rospy.get_param(navigation_prefix + 'goal_tolerance', .0087))), '~goal_tolerance_deg')
    progress_timeout = finite_number(rospy.get_param('~progress_timeout_sec', 30.), '~progress_timeout_sec')
    feedback_timeout = finite_number(rospy.get_param('~feedback_timeout_sec', 1.), '~feedback_timeout_sec')
    timeout = finite_number(rospy.get_param('~timeout_sec', max(3 * experiment_duration,
        experiment_duration + 2 * progress_timeout)), '~timeout_sec')
    if min(tolerance, progress_timeout, feedback_timeout, timeout) <= 0 or tolerance > 5:
        raise ValueError('timeouts must be positive and goal tolerance must be in (0, 5] degrees')
    task = Stroke(angular_velocity_rad_s, experiment_duration, math.radians(tolerance), progress_timeout)
    feedback = Feedback(topic_prefix)
    motion = MotionClient(topic_prefix)
    rospy.loginfo('Body-local Y goal %+.6g deg at up to %+.6g deg/s; same contact session, measured completion',
                  math.degrees(angle), angular_velocity_deg_s)
    started = time.monotonic()
    previous = rospy.get_time()
    last_send, last_rate, first_motion = 0., None, None
    interrupted = threading.Event()
    previous_handlers = {number: signal.signal(number, lambda *_: interrupted.set())
                         for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        while not rospy.is_shutdown() and not interrupted.is_set():
            now, ros_now = time.monotonic(), rospy.get_time()
            if ros_now < previous:
                raise ValueError('ROS time moved backwards; experiment abandoned')
            previous = ros_now
            if now - started > timeout:
                raise ValueError('experiment timeout: ' + task.reason)
            status, poses = feedback.snapshot()
            with motion.lock:
                ack = motion.ack
                if motion.epoch is not None and motion.epoch != motion.current_epoch:
                    raise ValueError('Motion cancelled by operator or contact transition; task abandoned')
            rate = 0.
            fresh = False
            if status and all(k in poses for k in ('reference', 'measured')):
                received, stamp, state, values = status
                session = int(values.get('session_id', '0'))
                if angular_velocity_rad_s == 0 and session and motion.current_epoch is not None:
                    if not motion.cancel(session):
                        raise ValueError('Operator stop was not confirmed before timeout')
                    return
                samples = [(received, stamp)] + [(p[0], p[1]) for p in poses.values()]
                fresh = all(0 <= now - r <= feedback_timeout and -.02 <= ros_now - s <= feedback_timeout
                            for r, s in samples)
                if (fresh and session and state.startswith('CONTACT_LOCKED') and
                        motion.session is None and motion.current_epoch is not None):
                    # Pin even while waiting for the preceding motion. An
                    # operator stop must also cancel this not-yet-started task.
                    motion.rearm(session)
                holding = values.get('feedback_waiting', '1') != '0'
                stopping = values.get('stopping', '1') != '0' or values.get('body_pitch_velocity_pending', '0') != '0'
                moving = stopping or values.get('body_pitch_velocity_active', '0') != '0'
                if task.started is None:
                    holding = (holding or moving or values.get('accepted_destination_pending', '0') != '0'
                               or motion.current_epoch is None)
                rate = task.step(now, session, state, poses['reference'][2], poses['measured'][2],
                                 fresh, holding, moving, max(.5, feedback_timeout), poses['measured'][1])
                if ack and task.session and int(ack[1]['session']) == task.session:
                    if ack[1]['accepted'] != 'true':
                        # Feedback holds can reject one refresh; a competing or
                        # revoked source must never be retried under a new epoch.
                        if 'another source' in ack[2] or 'cancelled motion epoch' in ack[2]:
                            raise ValueError(ack[2])
                        rate = 0.
                        rospy.logwarn_throttle(2., ack[2])
                if first_motion is not None and (not ack or int(ack[1]['session']) != task.session or
                                                  ack[0] < first_motion or now - ack[0] > command_duration):
                    if now - first_motion > command_duration:
                        raise ValueError('velocity acknowledgement missing/stale')
            owns_motion = task.started is not None
            if owns_motion and motion.publisher.get_num_connections() and (last_rate is None or rate != last_rate or now - last_send >= refresh_period):
                motion.send(task.session, RollYawMotion.VELOCITY, rate)
                last_send, last_rate = now, rate
                if rate != 0 and first_motion is None:
                    first_motion = now
            if angular_velocity_rad_s == 0 and last_rate == 0:
                return
            if (fresh and task.complete and last_rate == 0 and ack and ack[0] >= last_send and
                    ack[1]['accepted'] == 'true' and int(ack[1]['session']) == task.session):
                rospy.loginfo('Measured stroke completed: reference %+.6g deg, measured %+.6g deg',
                              math.degrees(task.reference_progress), math.degrees(task.measured_progress))
                return
            rospy.loginfo_throttle(2., '%s; reference %.3f deg, measured %.3f deg', task.reason,
                                  math.degrees(task.reference_progress), math.degrees(task.measured_progress))
            time.sleep(.05)
    finally:
        try:
            if not motion.stop(wait=True):
                rospy.logerr('Stop was not confirmed before timeout; check navigator stop/hold status.')
        except (RuntimeError, rospy.ROSException) as error:
            rospy.logerr('Could not confirm stop: %s', error)
        finally:
            for number, handler in previous_handlers.items():
                signal.signal(number, handler)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except (ValueError, RuntimeError) as exc:
        rospy.logerr('%s', exc)
        raise SystemExit(1)
