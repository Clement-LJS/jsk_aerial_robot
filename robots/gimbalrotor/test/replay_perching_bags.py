#!/usr/bin/env python3
"""Read the four 261115 bags without a ROS master or actuator publishers.

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

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('experiment', ROOT / 'scripts/tasks/auto_roll_yaw_body_pitch_experiment.py')
experiment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(experiment)

FILES = ('1_afterperch_navfailed.bag', '2_aftercodex_keyboardnavfailed.bag',
         '3_navfailed.bag', '4stillfailed.bag')
PREFIX = '/gimbalrotor/perching/roll_yaw/'


def inspect(path, index, samples):
    before = path.stat()
    poses = {}
    count = bad_servo = servo_count = 0
    task = experiment.Stroke(math.radians(.1), 240., math.radians(.5), 10000.)
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag-root', type=Path, required=True, help='directory containing the four 261115 bags')
    parser.add_argument('--core-test', type=Path, required=True, help='compiled roll_yaw_core_test executable')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='perching-bag-replay-') as directory:
        path = Path(directory) / 'samples.txt'
        with path.open('w') as samples:
            reports = [inspect(args.bag_root / name, index, samples) for index, name in enumerate(FILES)]
        print(json.dumps(reports, indent=2), flush=True)
        env = dict(os.environ, ROLL_YAW_BAG_SAMPLES=str(path))
        executable = args.core_test.resolve()
        env['LD_LIBRARY_PATH'] = str(executable.parent.parent) + ':' + env.get('LD_LIBRARY_PATH', '')
        subprocess.run([str(executable), '--gtest_filter=Regression.LatestFourBagFeedbackAndTrackingSamples'],
                       env=env, check=True)


if __name__ == '__main__':
    main()
