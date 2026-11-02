#!/usr/bin/env python3
"""Passive ROS diagnostic recording alongside ordinary manual robot operation.

No command publishers, command remapping, motor settings or automatic halt.
Errors are evidence: log them and keep recording until Ctrl-C or --duration.
"""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import shutil
import signal
import subprocess
import threading
import time

from passive_monitor import Monitor
from capture_paths import create_run

NS = '/gimbalrotor'
TOPICS = [NS + '/' + name for name in (
    'imu', 'servo/states', 'motor_pwms', 'motor_info', 'flight_state',
    'flight_config_cmd', 'flight_config_ack', 'four_axes/command',
    'torque_allocation_matrix_inv', 'rpy/pid', 'servo/target_states_debug',
    'servo/torque_states', 'gimbals_ctrl', 'joints_ctrl', 'joint_states',
    'uav/baselink/odom', 'battery_voltage_status',
    'teleop_command/start', 'teleop_command/takeoff', 'teleop_command/halt',
    'teleop_command/land', 'teleop_command/force_landing',
    'servo/target_states', 'servo/target_current', 'servo/torque_enable',
    'uav_info', 'rpy/gain', 'gimbal_dof', 'esc_telem', 'mocap/pose',
    'uav/nav', 'debug/pose/pid', 'debug/target_vectoring_force',
    'perching/servo_neutral_mode', 'perching/roll_yaw/state', 'pwm_test',
)] + ['/rosout']
MONITORED = ('imu', 'servo/states', 'motor_pwms', 'motor_info', 'flight_state',
             'teleop_command/start', 'teleop_command/takeoff', 'teleop_command/halt',
             'teleop_command/land', 'teleop_command/force_landing',
             'four_axes/command', 'torque_allocation_matrix_inv', 'servo/target_states',
             'servo/target_states_debug', 'rpy/pid', 'esc_telem', 'pwm_test',
             'flight_config_cmd', 'flight_config_ack', 'mocap/pose',
             'battery_voltage_status')


class Capture:
    def __init__(self, args):
        # Deferred ROS import: --help and the offline tests do not connect to ROS.
        import rospy
        from rosgraph_msgs.msg import Log
        self.ros = rospy
        self.args = args
        self.monitor = Monitor(args.pwm_ceiling)
        self.events = queue.Queue(maxsize=2048)
        self.drop_lock = threading.Lock()
        self.dropped = 0
        self.subscribers = []
        self.connections = {}
        self.recorder = None
        self.closed = False
        self.start_time = time.monotonic()
        self.directory, self.run_lock = create_run(args.output)
        print('Capture directory: ' + str(self.directory), flush=True)
        print('USB recorder: run capture_usb.py in another terminal from the same parent directory; no --output needed.', flush=True)
        self.log_file = (self.directory / 'events.jsonl').open('w', buffering=1)
        self.recorder_log = (self.directory / 'recorder.log').open('w')
        try:
            if not shutil.which('rosbag'):
                raise RuntimeError('rosbag is not on PATH; use your existing ROS environment')
            if shutil.disk_usage(self.directory).free < 256 * 1024 * 1024:
                raise RuntimeError('Less than 256 MiB free for the recording')
            for name in MONITORED:
                self.subscribers.append(rospy.Subscriber(
                    NS + '/' + name, rospy.AnyMsg,
                    lambda msg, kind=name: self.raw_message(kind, msg),
                    queue_size=200, buff_size=262144, tcp_nodelay=True))
            self.subscribers.append(rospy.Subscriber('/rosout', Log, self.rosout, queue_size=100))
            # Anonymous node name avoids terminating another recorder with the same name.
            recorder_name = 'rosserial_capture_' + datetime.now().strftime('%H%M%S%f')
            command = ['rosbag', 'record', '--buffsize=64', '--min-space=128M',
                       '--split', '--duration=5m', '__ns:=/', '__name:=' + recorder_name] + TOPICS
            self.recorder = subprocess.Popen(command, cwd=str(self.directory),
                stdout=self.recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
            self.emit('recording_start', command=command, mode='passive',
                      output=str(self.directory), observation_threshold=args.pwm_ceiling,
                      note='No robot command or motor-limit changes; stopping this program only stops recording')
            from capture_context import snapshot
            self.metadata = snapshot(TOPICS)
            (self.directory / 'metadata_start.json').write_text(json.dumps(self.metadata, indent=2) + '\n')
            limits = self.metadata.get('parameters', {}).get('/gimbalrotor/motor_info', {})
            self.emit('startup_limits_observed', max_pwm=limits.get('max_pwm'),
                      min_thrust=limits.get('min_thrust'),
                      note='Parameter snapshot only; recorder does not enforce limits')
        except BaseException:
            self.close(analyze=False)
            raise

    def drop(self):
        with self.drop_lock:
            self.dropped += 1

    def raw_message(self, kind, msg):
        # Do not copy unexpectedly large payloads into the diagnostic queue.
        if len(msg._buff) > 4096:
            self.drop()
            return
        header = getattr(msg, '_connection_header', {}) or {}
        connection = {k: header[k] for k in ('callerid', 'type', 'md5sum') if k in header}
        with self.drop_lock:
            # Bound publisher provenance even with reconnects or unexpected nodes.
            entries = self.connections.setdefault(kind, [])
            if connection not in entries and len(entries) < 8:
                entries.append(connection)
        self.enqueue(kind, bytes(msg._buff))

    def enqueue(self, kind, payload):
        try:
            self.events.put_nowait((time.monotonic(), kind, payload))
        except queue.Full:
            self.drop()

    def rosout(self, msg):
        text = msg.msg[:2048]
        if any(word in text.lower() for word in (
                'deserializ', 'bad_alloc', 'buffer overrun', 'checksum', 'lost sync')):
            self.enqueue('rosout_error', {'node': msg.name, 'text': text,
                         'severity': msg.level, 'source_stamp_ns': msg.header.stamp.to_nsec()})

    def emit(self, event, **values):
        entry = dict(event=event, monotonic=time.monotonic(),
                     wall_time=datetime.now().astimezone().isoformat(),
                     ros_time_ns=self.ros.Time.now().to_nsec(), **values)
        self.log_file.write(json.dumps(entry, allow_nan=False) + '\n')
        # File output only: even diagnostic events are not published onto ROS.
        print(event + ': ' + json.dumps(values, allow_nan=False), flush=True)

    def drain(self):
        for _ in range(1000):
            try:
                received, kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            for entry in self.monitor.process(received, kind, payload):
                self.emit(**entry)

    def run(self):
        last_summary = 0
        last_disk_check = 0
        seen_streams = False
        while not self.ros.is_shutdown():
            self.drain()
            now = time.monotonic()
            if self.recorder.poll() is not None:
                raise RuntimeError('rosbag recorder exited; check recorder.log')
            if not seen_streams and all(k in self.monitor.last for k in ('imu', 'servo/states')):
                seen_streams = True
                self.emit('streams_received', note='IMU and servo telemetry received. Manual control is independent of this recorder.')
            if now - last_summary >= 5:
                with self.drop_lock:
                    dropped = self.dropped
                self.emit('summary', **self.monitor.summary(now), monitor_dropped=dropped,
                          not_yet_observed_topics=[k for k in MONITORED[:5] if k not in self.monitor.last])
                last_summary = now
            if now - last_disk_check >= 1:
                if shutil.disk_usage(self.directory).free < 128 * 1024 * 1024:
                    raise RuntimeError('Less than 128 MiB free disk; stopping recorder only')
                last_disk_check = now
            if self.args.duration and now - self.start_time >= self.args.duration:
                self.emit('duration_complete', note='Stopping recording only; robot state is unchanged')
                break
            time.sleep(0.02)

    def close(self, analyze=True):
        if self.closed:
            return
        self.closed = True
        # No halt, disarm or other robot command, including error/interrupt cleanup.
        for sub in self.subscribers:
            sub.unregister()
        try:
            self.drain()
            self.emit('recording_stop', **self.monitor.summary(time.monotonic()),
                      note='Robot state is unchanged; use your keyboard to stop/disarm')
        except Exception as exc:
            print('Final diagnostic write failed: ' + str(exc), flush=True)
        finally:
            if self.recorder is not None and self.recorder.poll() is None:
                try:
                    self.recorder.send_signal(signal.SIGINT)
                    self.recorder.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    print('Recorder still finalizing; left running to preserve the bag.', flush=True)
            self.recorder_log.close()
            self.log_file.close()
            if getattr(self, 'run_lock', None) is not None:
                self.run_lock.close()
        if analyze:
            try:
                with self.drop_lock:
                    connections = dict(self.connections)
                (self.directory / 'connections.json').write_text(json.dumps(connections, indent=2) + '\n')
                from capture_context import snapshot
                (self.directory / 'metadata_end.json').write_text(json.dumps(snapshot(TOPICS), indent=2) + '\n')
            except Exception as exc:
                print('Final metadata unavailable: ' + str(exc), flush=True)
        print('Capture directory: ' + str(self.directory), flush=True)
        if analyze and self.recorder is not None and self.recorder.poll() is not None:
            try:
                from summarize_capture import summarize
                bags = sorted(self.directory.glob('*.bag'))
                if bags:
                    result = summarize(bags, pwm_ceiling=self.args.pwm_ceiling)
                    (self.directory / 'analysis.json').write_text(
                        json.dumps(result, indent=2, allow_nan=False) + '\n')
                    print('Offline analysis saved: ' + str(self.directory / 'analysis.json'), flush=True)
            except Exception as exc:
                print('Offline analysis failed; keep bags for inspection: ' + str(exc), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=None,
                        help='Parent for a timestamped run folder (default: current working directory)')
    parser.add_argument('--duration', type=float, default=0,
                        help='Recording duration in wall-clock seconds; default 0 means until Ctrl-C')
    parser.add_argument('--pwm-ceiling', type=float, default=None,
                        help='Optional reporting threshold, e.g. 1100; does NOT set or enforce motor limits')
    parser.add_argument('--props-removed', action='store_true',
                        help='Accepted for compatibility; recorder is passive with or without this flag')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be a finite nonnegative number')
    if args.pwm_ceiling is not None and (not math.isfinite(args.pwm_ceiling) or args.pwm_ceiling <= 0):
        parser.error('--pwm-ceiling must be a finite positive number')
    import rospy
    rospy.init_node('rosserial_capture', anonymous=True, disable_signals=True)
    capture = None
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        capture = Capture(args)
        capture.run()
    except KeyboardInterrupt:
        print('Stopping recording only; use your keyboard to stop/disarm the robot.', flush=True)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if capture is not None:
            capture.close()
        rospy.signal_shutdown('Capture finished')


if __name__ == '__main__':
    main()
