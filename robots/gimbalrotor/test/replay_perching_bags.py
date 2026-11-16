#!/usr/bin/env python3
"""Read the four failed 261115 bags and run 5 without a ROS master or actuator publishers.

Export recorded feedback/governor inputs to the compiled regression test and
check that the automatic task cannot report success on the failed timelines.
This is input replay, not a closed-loop prediction of a modified flight.
"""

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import tempfile

import rosbag
from tf.transformations import quaternion_conjugate, quaternion_multiply

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('experiment', ROOT / 'scripts/tasks/auto_roll_yaw_body_pitch_experiment.py')
experiment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(experiment)

FILES = ('1_afterperch_navfailed.bag', '2_aftercodex_keyboardnavfailed.bag',
         '3_navfailed.bag', '4stillfailed.bag')
PREFIX = '/gimbalrotor/perching/roll_yaw/'


local_pitch = experiment.local_pitch


class Stroke:
    """Historical velocity-task monitor for the November bag regressions only."""
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



def inspect(path, index, samples):
    before = path.stat()
    poses = {}
    count = bad_servo = servo_count = 0
    task = Stroke(math.radians(.1), 240., math.radians(.5), 10000.)
    terminal = None
    with rosbag.Bag(str(path), 'r') as bag:
        start = bag.get_start_time()
        topics = [PREFIX + name for name in ('diagnostics_status', 'body_reference', 'body_measured')]
        topics.append('/gimbalrotor/servo/states')
        for topic, wire, stamp in bag.read_messages(topics=topics, raw=True):
            elapsed = stamp.to_sec() - start
            if topic.endswith('/servo/states'):
                # Recorded spinal/ServoStates: time (8), array length (4), then
                # seven bytes per ServoState. Never allocate using a corrupt count.
                if wire[2] != '8ab23ee17ec1578fba9ec2244276b0ef':
                    raise ValueError('unexpected ServoStates schema in ' + str(path))
                servo_count += 1
                payload = wire[1]
                number = struct.unpack_from('<I', payload, 8)[0] if len(payload) >= 12 else -1
                bad_servo += number < 0 or len(payload) != 12 + 7 * number
                continue
            message = wire[4]().deserialize(wire[1])
            if not topic.endswith('/diagnostics_status'):
                poses[topic.rsplit('/', 1)[-1]] = experiment.orientation(message)
                continue
            for status in message.status:
                if not status.name.endswith('/roll_yaw_perching'):
                    continue
                values = {v.key: v.value for v in status.values}
                if status.message.startswith('CONTACT_LOCKED:'):
                    keys = ('body_age_s', 'joint_age_s', 'feedback_skew_s',
                            'effective_body_tracking_error_rad', 'effective_joint_tracking_error_rad', 'tracking_hold')
                    row = [float(values.get(key, '0')) for key in keys]
                    if all(math.isfinite(value) for value in row):
                        samples.write(str(index) + ' ' + ' '.join(format(value, '.17g') for value in row) + '\n')
                        count += 1
                # Runs 3/4 include the automatic pitch experiment. Start there;
                # runs 1/2 exercise the enabled contact and subsequent reset.
                earliest = (0., 0., 151.5, 149.3)[index]
                if elapsed < earliest or terminal or len(poses) != 2:
                    continue
                fresh = all(float(values.get(key, 'inf')) <= .15 for key in ('body_age_s', 'joint_age_s'))
                holding = values.get('feedback_waiting', '1') != '0'
                stopping = values.get('stopping', '1') != '0'
                try:
                    task.step(elapsed, int(values['session_id']), status.message,
                              poses['body_reference'], poses['body_measured'], fresh, holding, stopping, 1.)
                except ValueError as error:
                    terminal = str(error)
                if task.complete:
                    raise AssertionError('failed bag incorrectly reported as completed: ' + path.name)
    if count < 100 or task.started is None or not terminal:
        raise AssertionError('missing regression coverage for ' + path.name)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise AssertionError('recording changed while reading: ' + str(path))
    return dict(bag=path.name, contact_samples=count, servo_messages=servo_count,
                malformed_servo_messages=bad_servo, task_result=terminal, completed=task.complete)


def observer_samples(path, destination):
    """Align recorded body/joint streams without changing the bag or publishing."""
    import numpy as np
    from tf.transformations import quaternion_slerp, quaternion_matrix
    body, joints = [], []
    with rosbag.Bag(str(path), 'r') as bag:
        start = bag.get_start_time()
        for topic, msg, _ in bag.read_messages(topics=['/gimbalrotor/uav/baselink/odom', '/gimbalrotor/joint_states']):
            t = msg.header.stamp.to_sec() - start
            if not 668 <= t <= 694:
                continue
            if topic.endswith('/odom'):
                p, q, w = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.angular
                row = (t, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), (w.x, w.y, w.z))
                if not body or t > body[-1][0]:
                    body.append(row)
            elif 'joint_roll' in msg.name and 'joint_yaw' in msg.name:
                row = (t, msg.position[msg.name.index('joint_roll')], msg.position[msg.name.index('joint_yaw')])
                if not joints or t > joints[-1][0]:
                    joints.append(row)
    index = 1
    with destination.open('w') as output:
        for t, roll, yaw in joints:
            while index < len(body) and body[index][0] < t:
                index += 1
            if index >= len(body) or body[index-1][0] > t:
                continue
            a, b = body[index-1], body[index]
            u = (t-a[0])/(b[0]-a[0])
            q = quaternion_slerp(a[2], b[2], u)
            rotation = quaternion_matrix(q)[:3, :3]
            omega_world = ((1-u)*quaternion_matrix(a[2])[:3, :3].dot(a[3]) +
                           u*quaternion_matrix(b[2])[:3, :3].dot(b[3]))
            omega = rotation.T.dot(omega_world)
            position = (1-u)*np.array(a[1]) + u*np.array(b[1])
            values = [t, *position, *q, *omega, roll, yaw]
            output.write(' '.join(format(x, '.17g') for x in values) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag-root', type=Path, required=True, help='directory containing the four 261115 bags')
    parser.add_argument('--core-test', type=Path, required=True, help='compiled roll_yaw_core_test executable')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='perching-bag-replay-') as directory:
        path = Path(directory) / 'samples.txt'
        with path.open('w') as samples:
            reports = [inspect(args.bag_root / name, index, samples) for index, name in enumerate(FILES)]
        motion_path = Path(directory) / 'motion.txt'
        fifth = args.bag_root / '5cutsuccess_codexchanged.bag'
        if fifth.exists():
            before = fifth.stat()
            angle = 0.
            counts = {'keyboard': 0, 'velocity': 0, 'stop': 0}
            topics = [PREFIX + key for key in ('body_pitch_delta', 'body_pitch_velocity', 'cancel')]
            topics += ['/gimbalrotor/perching/enable', '/gimbalrotor/perching/relock']
            with rosbag.Bag(str(fifth), 'r') as bag, motion_path.open('w') as output:
                start = bag.get_start_time()
                for topic, message, stamp in bag.read_messages(topics=topics):
                    if topic.endswith('/body_pitch_delta'):
                        angle = max(-math.radians(20), min(math.radians(20), angle + message.data))
                        kind, value = 0, angle
                        counts['keyboard'] += 1
                    elif topic.endswith('/body_pitch_velocity'):
                        kind, value = 1, message.data
                        angle = 0.
                        counts['velocity'] += 1
                    elif topic.endswith('/enable') and message.data:
                        continue
                    else:
                        kind, value, angle = 2, 0., 0.
                        counts['stop'] += 1
                    output.write('{:.9f} {} {:.17g}\n'.format(stamp.to_sec() - start, kind, value))
            after = fifth.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                raise AssertionError('recording changed while reading: ' + str(fifth))
            reports.append(dict(bag=fifth.name, motion_inputs=counts,
                                check='recorded inputs converted to replaceable targets; not a flight simulation'))
        observer_path = Path(directory) / 'observer.txt'
        if fifth.exists():
            observer_samples(fifth, observer_path)
        print(json.dumps(reports, indent=2), flush=True)
        env = dict(os.environ, ROLL_YAW_BAG_SAMPLES=str(path))
        if fifth.exists():
            env['ROLL_YAW_MOTION_SAMPLES'] = str(motion_path)
            env['ROLL_YAW_OBSERVER_SAMPLES'] = str(observer_path)
        executable = args.core_test.resolve()
        env['LD_LIBRARY_PATH'] = str(executable.parent.parent) + ':' + env.get('LD_LIBRARY_PATH', '')
        subprocess.run([str(executable), '--gtest_filter=Regression.LatestFourBagFeedbackAndTrackingSamples:Motion.RecordedRunFiveCannotBuildJogBacklog:Observer.RecordedQuietIntervalConditioning'],
                       env=env, check=True)


if __name__ == '__main__':
    main()
