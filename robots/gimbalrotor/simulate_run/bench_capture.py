#!/usr/bin/env python3
"""User-run propeller-off bench capture. Never run automatically by development tools.

Requires bench.launch. Arm/takeoff originate ONLY from the user's keyboard.
The existing controller performs allocation and actuation with startup PWM limits.
This program only relays those two requests and publishes stop requests/events.
"""
import argparse
from collections import Counter, deque
from datetime import datetime
import json
from pathlib import Path
import queue
import shutil
import signal
import subprocess
import time

from payload_checks import inspect_payload, motor_info_is_capped, prefix_overlap

NS = '/gimbalrotor'
BASE_NODE = NS + '/aerial_robot_base_node'
EVENT = NS + '/simulate_run/events'
TOPICS = [NS + '/' + name for name in (
    'imu', 'servo/states', 'motor_pwms', 'motor_info', 'flight_state',
    'flight_config_cmd', 'flight_config_ack', 'four_axes/command',
    'torque_allocation_matrix_inv', 'rpy/pid', 'servo/target_states_debug',
    'servo/torque_states', 'gimbals_ctrl', 'joints_ctrl', 'joint_states',
    'uav/baselink/odom', 'battery_voltage_status',
    'teleop_command/start', 'teleop_command/takeoff', 'teleop_command/halt',
    'simulate_run/gated_start', 'simulate_run/gated_takeoff',
)] + ['/rosout', EVENT]


class Bench:
    def __init__(self, args):
        # Deferred imports keep --help and offline tests independent of ROS.
        import rospy
        import rosgraph
        from std_msgs.msg import Empty, String
        from rosgraph_msgs.msg import Log
        self.ros, self.Empty, self.String = rospy, Empty, String
        self.master = rosgraph.Master(rospy.get_name())
        self.args = args
        self.events = queue.Queue(maxsize=2048)
        self.overflow = False
        self.stopping = False
        self.finished = False
        self.arm_sent = None
        self.takeoff_sent = None
        self.state = None
        self.cap_seen = False
        self.pwm_values = ()
        self.last, self.counts, self.issues = {}, Counter(), Counter()
        self.histories = {key: deque(maxlen=300) for key in ('imu', 'motor_pwms')}
        self.previous_stamps = {}
        self.phase = None
        self.ready = False
        self.graph_ok = False
        self.start_time = time.monotonic()
        self.last_graph_check = 0
        self.last_summary = 0
        self.recorder = None
        self.subscribers = []
        self.directory = args.output.expanduser() / datetime.now().strftime('%Y-%m-%d-%H-%M-%S-%f')
        self.directory.mkdir(parents=True, exist_ok=False)
        self.log_file = (self.directory / 'events.jsonl').open('w', buffering=1)
        self.recorder_log = (self.directory / 'recorder.log').open('w')
        self.publisher = rospy.Publisher(EVENT, String, queue_size=20, latch=True)
        self.start_pub = rospy.Publisher(NS + '/simulate_run/gated_start', Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher(NS + '/simulate_run/gated_takeoff', Empty, queue_size=1)
        self.halt_pub = rospy.Publisher(NS + '/teleop_command/halt', Empty, queue_size=1)
        self.recorder_name = '/rosserial_bench_recorder'
        self.emit('initializing', output=str(self.directory))
        try:
            self.validate_parameters()
            if not shutil.which('rosbag'):
                raise RuntimeError('rosbag is not on PATH; use your existing ROS environment')
            for name in ('imu', 'servo/states', 'motor_pwms', 'motor_info', 'flight_state'):
                self.subscribers.append(rospy.Subscriber(
                    NS + '/' + name, rospy.AnyMsg,
                    lambda msg, kind=name: self.enqueue(kind, bytes(msg._buff)),
                    queue_size=200, buff_size=262144, tcp_nodelay=True))
            for action in ('start', 'takeoff'):
                self.subscribers.append(rospy.Subscriber(
                    NS + '/teleop_command/' + action, Empty,
                    lambda msg, kind=action: self.enqueue('request_' + kind, None), queue_size=1))
            self.subscribers.append(rospy.Subscriber('/rosout', Log, self.rosout, queue_size=100))
            if shutil.disk_usage(self.directory).free < 256 * 1024 * 1024:
                raise RuntimeError('Less than 256 MiB free for the recording')
            command = ['rosbag', 'record', '--buffsize=64', '--min-space=128M',
                       '--split', '--duration=5m', '__ns:=/',
                       '__name:=' + self.recorder_name[1:]] + TOPICS
            self.recorder = subprocess.Popen(command, cwd=str(self.directory),
                stdout=self.recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
            self.emit('recording_start', command=command, cap_pwm_equivalent=1100,
                      firmware_matches_source='assumed by user, not verified')
        except BaseException:
            if self.recorder is not None and self.recorder.poll() is None:
                self.recorder.send_signal(signal.SIGINT)
            self.log_file.close()
            self.recorder_log.close()
            raise

    def validate_parameters(self):
        r = self.ros
        if r.get_param('/use_sim_time', False):
            raise RuntimeError('Hardware bench requires wall-clock ROS time')
        if r.get_param(NS + '/simulate_run/bench_profile', '') != 'prop_off_1100_v1':
            raise RuntimeError('Start simulate_run/bench.launch, not ordinary bringup')
        for key, expected in (('min_pwm', 0.5), ('max_pwm', 0.55), ('min_thrust', 0.0)):
            value = float(r.get_param(NS + '/motor_info/' + key))
            if not abs(value - expected) < 1e-6:
                raise RuntimeError('Unexpected bench parameter motor_info/' + key)

    def enqueue(self, kind, payload):
        # Never trust a corrupt embedded array length or retain arbitrarily large data.
        if isinstance(payload, bytes) and len(payload) > 4096:
            self.overflow = True
            return
        try:
            self.events.put_nowait((time.monotonic(), kind, payload))
        except queue.Full:
            self.overflow = True

    def rosout(self, msg):
        text = msg.msg[:2048]
        if msg.name == self.ros.get_name():
            return
        if any(word in text.lower() for word in (
                'deserializ', 'bad_alloc', 'buffer overrun', 'checksum', 'lost sync')):
            self.enqueue('rosout_error', {'node': msg.name, 'text': text,
                         'severity': msg.level, 'source_stamp_ns': msg.header.stamp.to_nsec()})

    def emit(self, event, **values):
        entry = dict(event=event, monotonic=time.monotonic(),
                     wall_time=datetime.now().astimezone().isoformat(),
                     ros_time_ns=self.ros.Time.now().to_nsec(), **values)
        line = json.dumps(entry, allow_nan=False)
        try:
            self.log_file.write(line + '\n')
        except OSError as exc:
            self.stopping = True
            print('Event log failed: ' + str(exc), flush=True)
        try:
            self.publisher.publish(self.String(data=line))
        except Exception as exc:
            self.stopping = True
            print('Event publication failed: ' + str(exc), flush=True)
        if event != 'summary':
            print(event + ': ' + json.dumps(values), flush=True)

    def check_graph(self):
        self.validate_parameters()
        publishers, subscribers, _ = self.master.getSystemState()
        pubs, subs = dict(publishers), dict(subscribers)
        own = self.ros.get_name()
        for action, gated in (('start', 'gated_start'), ('takeoff', 'gated_takeoff')):
            source = NS + '/teleop_command/' + action
            # Recorder can also subscribe. There must be no direct controller subscriber.
            if set(subs.get(source, [])) - {own, self.recorder_name}:
                raise RuntimeError('Arm/takeoff remapping missing or unexpected subscriber: ' + source)
            target = NS + '/simulate_run/' + gated
            if set(subs.get(target, [])) - {self.recorder_name} != {BASE_NODE}:
                return False
            if set(pubs.get(target, [])) != {own}:
                raise RuntimeError('Unexpected publisher on ' + target)
        for topic in ('motor_info', 'flight_config_cmd'):
            if set(pubs.get(NS + '/' + topic, [])) != {BASE_NODE}:
                return False
        for topic in ('pwm_test', 'simulate_run/disabled_joy'):
            if pubs.get(NS + '/' + topic):
                raise RuntimeError('Conflicting actuator/joystick publisher: ' + topic)
        # rosbag must have connected to the essential streams before manual arming.
        for topic in ('imu', 'servo/states', 'motor_pwms', 'flight_state'):
            if self.recorder_name not in subs.get(NS + '/' + topic, []):
                return False
        return (self.start_pub.get_num_connections() > 0 and
                self.takeoff_pub.get_num_connections() > 0 and
                self.halt_pub.get_num_connections() > 0)

    def fresh(self, now):
        return all(now - self.last.get(key, -1e9) < 1.0
                   for key in ('imu', 'servo/states', 'motor_pwms', 'flight_state'))

    def stop(self, reason):
        if not self.stopping:
            self.stopping = True
            self.ready = False
            self.emit('stop_requested', reason=reason)

    def process(self, received, kind, payload):
        self.counts[kind] += 1
        if kind.startswith('request_'):
            # Discard stale queued actions; never replay or auto-rearm.
            now = time.monotonic()
            common = (not self.stopping and now - received < 0.5 and
                      self.graph_ok and self.fresh(now))
            if kind == 'request_start':
                if common and self.ready and self.state == 0 and self.arm_sent is None:
                    self.arm_sent = now
                    self.cap_seen = False
                    self.emit('manual_arm_forwarded')
                    self.start_pub.publish(self.Empty())
                else:
                    self.emit('manual_arm_rejected', state=self.state, ready=self.ready)
            elif (common and self.arm_sent is not None and self.state == 2 and
                  self.cap_seen and self.takeoff_sent is None and
                  len(self.pwm_values) >= 4 and
                  all(1000 <= value <= 1101 for value in self.pwm_values[:4]) and
                  self.last.get('motor_pwms', 0) > self.arm_sent):
                self.takeoff_sent = now
                self.emit('manual_takeoff_forwarded')
                self.takeoff_pub.publish(self.Empty())
            else:
                self.emit('manual_takeoff_rejected', state=self.state, capped_motor_info_seen=self.cap_seen)
            return
        if kind == 'rosout_error':
            self.emit(kind, **payload)
            self.issues[kind] += 1
            # Keep recording through shutdown; a checksum drop is useful evidence too.
            self.stop('rosserial error captured')
            return
        self.last[kind] = received
        if kind == 'flight_state':
            if len(payload) != 1 or payload[0] > 6:
                self.stop('Malformed flight_state')
                return
            old, self.state = self.state, payload[0]
            if old != self.state:
                self.emit('flight_state', previous=old, value=self.state)
            if self.arm_sent is None and self.state != 0:
                self.stop('Robot must be disarmed when starting this script')
            if self.takeoff_sent is not None and self.state in (0, 4, 6):
                self.stop('Controller left bench control state')
            return
        if kind == 'motor_info':
            if not motor_info_is_capped(payload):
                self.emit('uncapped_motor_info', sample_hex=payload[:32].hex())
                self.stop('Controller published motor limits outside bench profile')
            else:
                if not self.cap_seen:
                    self.emit('capped_motor_info_observed')
                self.cap_seen = True
            return
        parsed = inspect_payload(kind, payload)
        if kind == 'motor_pwms' and not parsed['issues']:
            self.pwm_values = parsed.get('values', ())
            if self.takeoff_sent is not None and len(self.pwm_values) < 4:
                self.stop('Motor count disappeared during the active bench test')
        if 'stamp_ns' in parsed:
            previous = self.previous_stamps.get(kind)
            if previous is not None and parsed['stamp_ns'] < previous:
                self.issues['backwards_' + kind] += 1
                self.emit('backwards_stamp', topic=kind,
                          delta_ns=parsed['stamp_ns'] - previous)
            self.previous_stamps[kind] = parsed['stamp_ns']
        if parsed['issues']:
            self.issues.update(parsed['issues'])
            matches = []
            if kind == 'servo/states':
                for topic, history in self.histories.items():
                    for when, sample in history:
                        if abs(received - when) <= 1:
                            overlap = prefix_overlap(payload, sample)
                            if overlap:
                                matches.append(dict(topic=topic, arrival_delta_s=when-received,
                                                    **overlap))
                matches = sorted(matches, key=lambda x: -x['bytes'])[:8]
            self.emit('payload_anomaly', topic=kind, parsed=parsed,
                      sample_hex=payload[:128].hex(), overlaps=matches)
            self.stop('Payload anomaly captured: ' + kind)
        elif kind in self.histories:
            self.histories[kind].append((received, payload))

    def drain(self):
        # Bound each iteration so timeout and stop checks still run under load.
        for _ in range(1000):
            try:
                item = self.events.get_nowait()
            except queue.Empty:
                break
            self.process(*item)

    def run(self):
        while not self.stopping and not self.ros.is_shutdown():
            self.drain()
            now = time.monotonic()
            if self.overflow:
                self.stop('Monitor queue overflow or oversized payload; capture may be incomplete')
            if self.recorder.poll() is not None:
                self.stop('rosbag recorder exited; check recorder.log')
            if now - self.last_graph_check >= 1:
                self.graph_ok = self.check_graph()
                self.last_graph_check = now
                if shutil.disk_usage(self.directory).free < 128 * 1024 * 1024:
                    self.stop('Less than 128 MiB free disk space')
                if self.arm_sent is not None and not self.graph_ok:
                    self.stop('Required ROS graph connection changed')
            if self.arm_sent is None:
                ready = (self.graph_ok and self.state == 0 and self.fresh(now) and
                         now - self.start_time >= 5 and not self.stopping)
                if ready and not self.ready:
                    self.emit('READY', instructions='Now use keyboard r, then t after ARM_ON and capped_motor_info_observed')
                self.ready = ready
                if now - self.start_time > 180:
                    self.stop('No manual arm within 180 seconds')
            else:
                if not self.fresh(now):
                    self.stop('Essential telemetry stale for over one second')
                if self.takeoff_sent is None and now - self.arm_sent > 30:
                    self.stop('No accepted manual takeoff within 30 seconds of arming')
                if self.takeoff_sent is not None:
                    elapsed = now - self.takeoff_sent
                    # Start the active sequence only when TAKEOFF/HOVER is observed.
                    if elapsed > 3 and self.state not in (3, 5):
                        self.stop('Takeoff request not accepted or control state lost')
                    phase = int(elapsed // 20)
                    if phase != self.phase:
                        self.phase = phase
                        instructions = (
                            'Hold the body steady; baseline with controller active.',
                            'Gently tilt the body a few degrees in roll/pitch; keep clear of moving servos.',
                            'Gently turn the body in yaw; avoid abrupt motions.',
                            'Hold steady again for the final baseline.')
                        if phase < len(instructions):
                            self.emit('phase', index=phase, instruction=instructions[phase])
                    if elapsed >= 80:
                        self.stop('Completed 80-second bench sequence')
            if now - self.last_summary >= 5:
                self.emit('summary', counts=dict(self.counts), issues=dict(self.issues),
                          state=self.state, ready=self.ready)
                self.last_summary = now
            time.sleep(0.02)

    def close(self):
        if self.finished:
            return
        self.finished = True
        self.stopping = True  # Any queued manual requests are rejected during cleanup.
        try:
            self.emit('cleanup', instruction='Requesting halt and retaining five seconds of post-event data')
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                # Through the existing navigator; never inject PWM or raw serial frames.
                try:
                    self.halt_pub.publish(self.Empty())
                    self.drain()
                except Exception as exc:
                    print('Stop/feedback attempt failed: ' + str(exc), flush=True)
                time.sleep(0.1)
            confirmed = self.state == 0 and time.monotonic() - self.last.get('flight_state', -1e9) < 1
            self.emit('stop_result', disarmed_state_observed=confirmed,
                      note='State feedback is not independent physical motor verification')
            if not confirmed:
                print('STOP NOT CONFIRMED: use keyboard h and the physical power disconnect.', flush=True)
        finally:
            if self.recorder is not None and self.recorder.poll() is None:
                self.recorder.send_signal(signal.SIGINT)
                try:
                    self.recorder.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    print('Recorder still finalizing; left running to avoid truncating the bag.', flush=True)
            self.recorder_log.close()
            self.log_file.close()
            print('Capture directory: ' + str(self.directory), flush=True)
            if self.recorder is not None and self.recorder.poll() is not None:
                try:
                    from summarize_capture import summarize
                    bags = sorted(self.directory.glob('*.bag'))
                    if bags:
                        result = summarize(bags)
                        (self.directory / 'analysis.json').write_text(
                            json.dumps(result, indent=2, allow_nan=False) + '\n')
                        print('Offline analysis saved: ' + str(self.directory / 'analysis.json'), flush=True)
                except Exception as exc:
                    print('Offline analysis failed; retain bags for inspection: ' + str(exc), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--props-removed', action='store_true',
                        help='Required acknowledgement that all propellers are removed')
    parser.add_argument('--output', type=Path,
                        default=Path.home() / 'rosbag/year2026/10Oct/261030/inspection')
    args = parser.parse_args()
    if not args.props_removed:
        parser.error('Remove all propellers and pass --props-removed; this launch is not for flight')
    import rospy
    rospy.init_node('rosserial_bench', anonymous=False, disable_signals=True)
    bench = None
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    try:
        bench = Bench(args)
        bench.run()
    except KeyboardInterrupt:
        print('Interrupted; stopping bench.', flush=True)
    except Exception as exc:
        print('Bench stopped: ' + str(exc), flush=True)
        raise
    finally:
        # Ignore repeat Ctrl-C only while requesting stop/finalizing the bag.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if bench is not None:
            bench.close()
        rospy.signal_shutdown('Bench finished')


if __name__ == '__main__':
    main()
