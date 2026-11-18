#!/usr/bin/env python3
"""Preview or replay recorded host-to-Spinal commands with live sensor feedback.

Default is offline preview: no ROS node, publishers, hardware or output files.
Execution requires --execute --props-removed and an existing ROS master.
See REPORT.md for the source recording, setup, deliberate bench modifications,
and the limits of reproducing a moving flight with a stationary robot.
"""
import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime, timezone, timedelta
import hashlib
import io
import json
import math
import os
from pathlib import Path
import shutil
import signal
import struct
import subprocess
import threading
import time
import xmlrpc.client

DEFAULT_BAG = Path('/home/clement/rosbag/year2026/11Nov/261117/5cutattempted_autoscriptfailed_manualcut.bag')
SOURCE_NS = '/gimbalrotor/'
MAX_PWM = 1100  # PWM-equivalent, not a guaranteed motor RPM.
TYPES = {
    'gimbal_dof': 'std_msgs/UInt8', 'uav_info': 'spinal/UavInfo',
    'motor_info': 'spinal/PwmInfo', 'joint_profiles': 'spinal/JointProfiles',
    'rpy/gain': 'spinal/RollPitchYawTerms', 'desire_coordinate': 'spinal/DesireCoord',
    'torque_allocation_matrix_inv': 'spinal/TorqueAllocationMatrixInv',
    'four_axes/command': 'spinal/FourAxisCommand',
    'servo/target_states': 'spinal/ServoControlCmd',
    'servo/torque_enable': 'spinal/ServoTorqueCmd',
    'flight_config_cmd': 'spinal/FlightConfigCmd', 'pwm_test': 'spinal/PwmTest',
    'perching/servo_neutral_mode': 'std_msgs/Bool',
}
BOOTSTRAP = ('gimbal_dof', 'uav_info', 'motor_info', 'joint_profiles', 'rpy/gain')
JST = timezone(timedelta(hours=9))


@dataclass
class Event:
    offset: float
    topic: str
    message: object


def finite_message(msg):
    if isinstance(msg, float):
        return math.isfinite(msg)
    if isinstance(msg, (list, tuple)):
        return all(finite_message(v) for v in msg)
    if hasattr(msg, '__slots__'):
        return all(finite_message(getattr(msg, key)) for key in msg.__slots__)
    return True


def validate_wire(kind, data):
    """Bound array counts BEFORE calling a ROS message deserializer."""
    def count(at, limit=256):
        if at + 4 > len(data):
            raise ValueError('truncated array count')
        value = struct.unpack_from('<I', data, at)[0]
        if value > limit:
            raise ValueError('oversized array count')
        return value
    if len(data) > 4096:
        raise ValueError('oversized command')
    if kind in ('gimbal_dof', 'flight_config_cmd', 'perching/servo_neutral_mode'):
        expected = 1
    elif kind == 'uav_info':
        expected = 3
    elif kind == 'desire_coordinate':
        expected = 12
    elif kind == 'four_axes/command':
        expected = 16 + 4 * count(12, 10)
    elif kind == 'torque_allocation_matrix_inv':
        expected = 4 + 6 * count(0, 10)
    elif kind == 'rpy/gain':
        expected = 4 + 14 * count(0, 10)
    elif kind == 'motor_info':
        expected = 21 + 28 * count(17, 32)
    elif kind == 'joint_profiles':
        expected = 4 + 25 * count(0, 8)
    elif kind in ('servo/target_states', 'servo/torque_enable', 'pwm_test'):
        first = count(0, 8)
        second = count(4 + first, 8)
        if first != second or not first:
            raise ValueError('command index/value lengths differ or are empty')
        width = {'servo/target_states': 2, 'servo/torque_enable': 1, 'pwm_test': 4}[kind]
        expected = 8 + first + width * second
    else:
        raise ValueError('topic is not allowlisted')
    if len(data) != expected:
        raise ValueError(f'command length {len(data)} != expected {expected}')


def validate_command(kind, msg):
    if not finite_message(msg):
        raise ValueError('nonfinite command value')
    if kind == 'gimbal_dof' and msg.data != 1:
        raise ValueError('this replay supports the recorded one-DoF beetle only')
    if kind == 'uav_info' and (msg.motor_num != 4 or msg.uav_model != 0):
        raise ValueError('this replay requires the recorded four-motor DRONE configuration')
    if kind == 'motor_info' and (not msg.motor_info or msg.pwm_conversion_mode not in (0, 1)):
        raise ValueError('missing or unsupported motor conversion configuration')
    if kind == 'flight_config_cmd' and msg.cmd not in (0, 1, 2, 160, 161):
        raise ValueError('unknown flight configuration command')
    if kind == 'four_axes/command':
        if len(msg.base_thrust) != 8 or any(abs(x) > 1.0 for x in msg.angles[:2]):
            raise ValueError('unexpected thrust-vector size or excessive attitude target')
    if kind == 'torque_allocation_matrix_inv' and len(msg.rows) != 8:
        raise ValueError('expected eight allocation rows')
    if kind == 'rpy/gain' and len(msg.motors) not in (1, 8):
        raise ValueError('expected one or eight gain records')
    if kind in ('servo/target_states', 'servo/torque_enable', 'pwm_test'):
        ids = list(msg.motor_index if kind == 'pwm_test' else msg.index)
        if len(set(ids)) != len(ids) or any(i not in range(8) for i in ids):
            raise ValueError('invalid or repeated actuator index')
        if kind == 'servo/target_states' and any(not 0 <= x <= 4095 for x in msg.angles):
            raise ValueError('servo target outside the recorded raw range')
        if kind == 'pwm_test' and ids != [4]:
            raise ValueError('only the recorded external saw channel 4 is supported; always held off')
    if kind == 'joint_profiles':
        ids = [j.servo_id for j in msg.joints]
        if len(set(ids)) != len(ids) or any(i not in range(8) for i in ids):
            raise ValueError('invalid joint profile indices')
        for j in msg.joints:
            if j.angle_scale <= 0 or j.angle_sgn not in (-1, 1):
                raise ValueError('invalid joint conversion')


def bench_message(kind, msg):
    """Clone the recorded message; never mutate the original event."""
    stream = io.BytesIO()
    msg.serialize(stream)
    result = type(msg)().deserialize(stream.getvalue())
    if kind == 'motor_info':
        result.min_pwm = 0.5
        result.max_pwm = min(result.max_pwm, MAX_PWM / 2000.0)
        result.min_thrust = 0.0  # Prevent voltage updates raising min_duty above cap.
        if result.max_pwm <= result.min_pwm:
            raise ValueError('recorded motor PWM range cannot support this bench profile')
    if kind == 'pwm_test':
        result.pwms = [0.5] * len(result.pwms)  # Never energize the saw.
    return result


def load_plan(path, duration):
    import rosbag
    events, first, classes = [], {}, {}
    counts = Counter()
    with rosbag.Bag(str(path), 'r') as bag:
        start, end = bag.get_start_time(), bag.get_end_time()
        for topic, raw, stamp in bag.read_messages(topics=[SOURCE_NS+k for k in TYPES], raw=True):
            kind = topic[len(SOURCE_NS):]
            if raw[0] != TYPES[kind]:
                raise ValueError(f'{topic}: unexpected type {raw[0]}')
            validate_wire(kind, raw[1])
            msg = raw[-1]().deserialize(raw[1])
            validate_command(kind, msg)
            if kind in classes and classes[kind]._md5sum != msg._md5sum:
                raise ValueError('mixed message definitions on '+topic)
            classes[kind] = type(msg)
            first.setdefault(kind, msg)
            counts[kind] += 1
            offset = stamp.to_sec() - start
            if not duration or offset <= duration:
                events.append(Event(offset, kind, bench_message(kind, msg)))
    missing = set(BOOTSTRAP)-set(first)
    if missing:
        raise ValueError('Bag lacks initialization topics: '+', '.join(sorted(missing)))
    if not events or not any(e.topic == 'flight_config_cmd' and e.message.cmd == 0 for e in events):
        raise ValueError('Selected duration contains no recorded ARM_ON')
    if not any(e.topic == 'four_axes/command' for e in events):
        raise ValueError('Selected duration contains no flight commands')
    with path.open('rb') as stream:
        digest = hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else None
        if digest is None:
            h = hashlib.sha256()
            for block in iter(lambda: stream.read(1024*1024), b''):
                h.update(block)
            digest = h.hexdigest()
    summary = dict(bag=str(path.resolve()), sha256=digest,
                   start_jst=datetime.fromtimestamp(start, JST).isoformat(),
                   recorded_duration_seconds=end-start,
                   replay_duration_seconds=min(duration, end-start) if duration else end-start,
                   source_command_counts=dict(counts),
                   scheduled_command_counts=dict(Counter(e.topic for e in events)),
                   flight_configuration=[dict(offset_seconds=e.offset, cmd=e.message.cmd)
                                         for e in events if e.topic == 'flight_config_cmd'],
                   pwm_limit_equivalent=MAX_PWM, saw_output='held at 0.5/off',
                   sensor_replay=False, timing='bag receipt offsets, monotonic wall clock, 1x')
    return events, first, classes, summary


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        c = super().make_connection(host)
        c.timeout = 1.0
        return c


class Replay:
    def __init__(self, args, events, first, classes, summary):
        import rospy
        import roslib.message
        from std_msgs.msg import UInt8
        from rosgraph_msgs.msg import Log
        self.ros, self.args = rospy, args
        self.events, self.first, self.summary = events, first, summary
        self.ns = args.namespace.rstrip('/')
        self.bridge = self.ns+'/rosserial_server'
        self.stop = threading.Event()
        self.interrupted = False
        self.failure = None
        self.armed = False
        self.touched = False
        self.last = {}
        self.ack = None
        self.servo_seen = False
        self.latest_pwm = None
        self.pubs, self.subs = {}, []
        self.child = None
        self.recorder = None
        self.log_lock = threading.Lock()
        self.records = Counter()
        self.out = args.output / datetime.now().strftime('replay-%Y%m%d-%H%M%S-%f')
        self.out.mkdir(parents=True, exist_ok=False)
        self.log = (self.out/'events.jsonl').open('w', buffering=1)
        self.summary['namespace'] = self.ns
        self.summary['run_started'] = datetime.now().astimezone().isoformat()
        self.summary['script_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        self.classes = {}
        for kind, name in TYPES.items():
            cls = roslib.message.get_message_class(name)
            if cls is None or (kind in classes and cls._md5sum != classes[kind]._md5sum):
                raise ValueError('Installed ROS message missing or MD5 differs from bag: '+name)
            self.classes[kind] = cls
        self.UInt8, self.Log = UInt8, Log
        self.master = xmlrpc.client.ServerProxy(
            os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'), transport=TimeoutTransport())

    def emit(self, event, **fields):
        with self.log_lock:
            self.log.write(json.dumps(dict(event=event, monotonic=time.monotonic(), **fields), allow_nan=False)+'\n')

    def fail(self, reason):
        if self.failure is None:
            self.failure = reason
            self.emit('abort', reason=reason)
        self.stop.set()

    def stop_requested(self):
        return self.interrupted or self.stop.is_set()

    def wait(self, seconds):
        end = time.monotonic()+seconds
        while not self.stop_requested() and time.monotonic() < end:
            time.sleep(max(0, min(0.02, end-time.monotonic())))
        return self.stop_requested()

    def graph(self):
        code, message, state = self.master.getSystemState(self.ros.get_name())
        if code != 1:
            raise RuntimeError('Cannot inspect ROS graph: '+message)
        return state

    def check_competitors(self, publishers):
        watched = {self.ns+'/'+k for k in TYPES}
        watched.update(self.ns+'/'+k for k in ('set_sim_voltage', 'p_matrix_pseudo_inverse_inertia'))
        conflicts = {t: [n for n in nodes if n != self.ros.get_name()]
                     for t, nodes in publishers if t in watched and any(n != self.ros.get_name() for n in nodes)}
        if conflicts:
            raise RuntimeError('Other command publishers are running. Use rosserial bridge only; '
                               'stop the normal controller/servo_bridge first: '+str(conflicts))

    def raw(self, kind, msg):
        if (getattr(msg, '_connection_header', {}) or {}).get('callerid') != self.bridge:
            self.fail('Telemetry is being published by a node other than '+self.bridge)
            return
        data = bytes(msg._buff)
        now = time.monotonic()
        valid = False
        if kind == 'imu':
            valid = len(data) == 60 and struct.unpack_from('<I', data, 4)[0] < 1000000000
            if valid:
                valid = all(math.isfinite(x) for x in struct.unpack_from('<13f', data, 8))
        elif kind == 'servo/states':
            valid = (len(data) == 68 and struct.unpack_from('<I', data, 8)[0] == 8
                     and struct.unpack_from('<I', data, 4)[0] < 1000000000
                     and sorted(data[12+7*i] for i in range(8)) == list(range(8)))
            if valid:
                self.servo_seen = True
                errors = {data[12+7*i]: data[18+7*i] for i in range(8) if data[18+7*i]}
                if errors:
                    self.emit('live_servo_error', errors=errors)
        elif kind == 'motor_pwms':
            if len(data) >= 4:
                count = struct.unpack_from('<I', data)[0]
                valid = count in (0, 8) and len(data) == 4+2*count
                if valid and count:
                    values = list(struct.unpack_from('<8H', data, 4))
                    self.latest_pwm = values
                    if self.armed and (any(v > MAX_PWM+5 for v in values[:4]) or values[4] > 1005):
                        self.fail('Live motor/saw output exceeds bench limit: '+str(values))
        if valid:
            self.last[kind] = now
        else:
            # Keep collecting the communication anomaly; never deserialize its count.
            self.emit('malformed_live_payload', topic=kind, length=len(data), hex=data[:128].hex())

    def acknowledge(self, msg):
        if (getattr(msg, '_connection_header', {}) or {}).get('callerid') != self.bridge:
            self.fail('Unexpected publisher of flight acknowledgement')
            return
        self.ack = (msg.data, time.monotonic())
        self.emit('flight_ack', command=msg.data)
        if self.armed and msg.data in (1, 2):
            self.fail('Spinal disarmed or entered force landing during replay')

    def rosout(self, msg):
        if any(s in msg.msg.lower() for s in ('checksum', 'deserializ', 'overrun', 'bad_alloc', 'servo', 'failsafe')):
            self.emit('rosout', node=msg.name, level=msg.level, message=msg.msg[:2048])

    def publish(self, topic, message, phase='replay', offset=None):
        stream = io.BytesIO()
        message.serialize(stream)
        # Convert bag-generated classes into installed classes with matching MD5.
        message = self.classes[topic]().deserialize(stream.getvalue())
        self.pubs[topic].publish(message)
        self.records[topic] += 1
        self.emit('command', topic=topic, phase=phase, source_offset=offset,
                  payload_hex=stream.getvalue().hex())

    def command(self, value):
        return self.classes['flight_config_cmd'](cmd=value)

    def request_ack(self, value):
        if value == 1:
            self.armed = False
        since = time.monotonic()
        self.publish('flight_config_cmd', self.command(value), phase='handshake')
        deadline = since+2.0
        while time.monotonic() < deadline:
            if self.ack and self.ack[0] == value and self.ack[1] >= since:
                self.armed = value == 0
                return
            if self.wait(0.01):
                raise RuntimeError(self.failure or 'Interrupted')
        raise RuntimeError('No fresh Spinal acknowledgement for command '+str(value))

    def health(self):
        now = time.monotonic()
        for name in ('imu', 'servo/states', 'motor_pwms'):
            if now-self.last.get(name, 0) > 1.0:
                raise RuntimeError('Live telemetry stale: '+name)
        if self.stop_requested() or self.ros.is_shutdown():
            raise RuntimeError(self.failure or 'Interrupted')
        if self.recorder is not None and self.recorder.poll() is not None:
            raise RuntimeError('Live bag recorder stopped; see recorder.log')
        if shutil.disk_usage(self.out).free < 128*1024*1024:
            raise RuntimeError('Less than 128 MiB free disk')

    def start_recording(self):
        if not shutil.which('rosbag') or shutil.disk_usage(self.out).free < 256*1024*1024:
            raise RuntimeError('rosbag must be on PATH and at least 256 MiB disk must be free')
        topics = sorted({self.ns+'/'+t for t in self.pubs} | {self.ns+'/'+t for t in (
            'imu', 'servo/states', 'motor_pwms', 'rpy/pid', 'servo/target_states_debug',
            'servo/torque_states', 'flight_config_ack', 'battery_voltage_status')} | {'/rosout'})
        self.recorder_log = (self.out/'recorder.log').open('w')
        self.recorder_name = '/spinal_replay_capture_'+str(os.getpid())
        self.recorder = subprocess.Popen(['rosbag', 'record', '--buffsize=64', '--min-space=128M',
            '--split', '--duration=5m', '-o', str(self.out/'live'), '__ns:=/',
            '__name:='+self.recorder_name.lstrip('/')] + topics,
            stdout=self.recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            if self.stop_requested() or self.recorder.poll() is not None:
                raise RuntimeError('Recording startup interrupted or failed')
            subscriptions = dict(self.graph()[1])
            if all(self.recorder_name in subscriptions.get(t, []) for t in topics):
                time.sleep(0.3)
                self.emit('live_recording_started', topics=topics)
                return
            time.sleep(0.1)
        raise RuntimeError('Live recorder did not subscribe to all requested topics')

    def run(self):
        rospy = self.ros
        if rospy.get_param('/use_sim_time', False):
            raise RuntimeError('/use_sim_time must be false; recorded /clock is never replayed')
        publishers, subscribers, _ = self.graph()
        self.check_competitors(publishers)
        nodes = {n for _, names in publishers+subscribers for n in names}
        if self.args.start_bridge:
            if self.bridge in nodes:
                raise RuntimeError('Bridge already exists; omit --start-bridge')
            if not Path(self.args.port).exists():
                raise RuntimeError('Serial device does not exist: '+self.args.port)
            self.bridge_log = (self.out/'bridge.log').open('w')
            self.child = subprocess.Popen(['roslaunch', 'spinal', 'bridge.launch',
                'mode:=serial', 'serial_port:='+self.args.port, 'serial_baud:=921600',
                '__ns:='+self.ns], stdout=self.bridge_log, stderr=subprocess.STDOUT,
                start_new_session=True)
        topics = set(e.topic for e in self.events) | set(BOOTSTRAP) | {
            'flight_config_cmd', 'servo/torque_enable', 'pwm_test', 'perching/servo_neutral_mode', 'desire_coordinate'}
        for topic in topics:
            self.pubs[topic] = rospy.Publisher(self.ns+'/'+topic, self.classes[topic], queue_size=10, latch=False)
        for name in ('imu', 'servo/states', 'motor_pwms'):
            self.subs.append(rospy.Subscriber(self.ns+'/'+name, rospy.AnyMsg,
                lambda m, k=name: self.raw(k, m), queue_size=100))
        self.subs.append(rospy.Subscriber(self.ns+'/flight_config_ack', self.UInt8, self.acknowledge, queue_size=10))
        self.subs.append(rospy.Subscriber('/rosout', self.Log, self.rosout, queue_size=100))
        deadline = time.monotonic()+30
        while True:
            if self.stop_requested():
                raise RuntimeError(self.failure or 'Interrupted')
            publishers, subscribers, _ = self.graph()
            self.check_competitors(publishers)
            registered = dict(subscribers)
            ready = all(self.bridge in registered.get(self.ns+'/'+t, []) and p.get_num_connections()
                        for t, p in self.pubs.items())
            if ready and self.servo_seen and time.monotonic()-self.last.get('imu', 0) < 1:
                break
            if time.monotonic() > deadline or (self.child and self.child.poll() is not None):
                raise RuntimeError('Bridge subscriptions/live eight-servo telemetry not ready; check bridge/firmware')
            time.sleep(0.1)
        print('Live bridge ready. Motor outputs are capped at 1100; saw channel is held OFF.', flush=True)
        print('Servos WILL move. Execution begins in 5 seconds; Ctrl-C cancels.', flush=True)
        if self.wait(5):
            raise RuntimeError('Interrupted before initialization')
        self.check_competitors(self.graph()[0])
        self.start_recording()
        self.check_competitors(self.graph()[0])
        self.touched = True
        self.request_ack(1)
        for topic in BOOTSTRAP:
            self.publish(topic, bench_message(topic, self.first[topic]), phase='bootstrap')
            time.sleep(0.1)  # gimbal DoF must arrive before UavInfo sets motor count.
        self.publish('perching/servo_neutral_mode', self.classes['perching/servo_neutral_mode'](data=False), phase='bootstrap')
        self.publish('desire_coordinate', self.classes['desire_coordinate'](), phase='bootstrap')
        self.publish('pwm_test', self.classes['pwm_test'](motor_index=[4], pwms=[0.5]), phase='bootstrap')
        self.publish('pwm_test', self.classes['pwm_test'](), phase='bootstrap')
        self.health()
        self.epoch = time.monotonic()
        last_graph = self.epoch
        last_print = self.epoch
        for event in self.events:
            target = self.epoch+event.offset
            while time.monotonic() < target:
                self.health()
                self.wait(max(0, min(0.02, target-time.monotonic())))
            self.health()
            now = time.monotonic()
            if now-last_graph >= 0.5:
                self.check_competitors(self.graph()[0])
                last_graph = time.monotonic()
            if time.monotonic()-target > 0.25:
                raise RuntimeError('Replay fell more than 250 ms behind; refusing a catch-up burst')
            if event.topic == 'flight_config_cmd' and event.message.cmd in (0, 1):
                self.request_ack(event.message.cmd)
            elif event.topic == 'flight_config_cmd' and event.message.cmd == 2:
                raise RuntimeError('Recorded force-landing event reached; stopping bench replay')
            else:
                self.publish(event.topic, event.message, offset=event.offset)
            if now-last_print >= 5:
                print(f'Replay {event.offset:.1f}/{self.summary["replay_duration_seconds"]:.1f} s; '
                      f'published={sum(self.records.values())}; PWM={self.latest_pwm}', flush=True)
                last_print = now
        self.emit('timeline_finished')

    def close(self):
        # Best effort only: lost power/USB or SIGKILL can prevent delivery.
        if self.touched:
            self.armed = False
            off = self.classes['servo/torque_enable'](index=list(range(8)), torque_enable=[0]*8)
            since = time.monotonic()
            for _ in range(5):
                for topic, msg in (
                    ('flight_config_cmd', self.command(1)),
                    ('flight_config_cmd', self.command(161)),
                    ('pwm_test', self.classes['pwm_test'](motor_index=[4], pwms=[0.5])),
                    ('pwm_test', self.classes['pwm_test']()),
                    ('servo/torque_enable', off)):
                    try:
                        self.publish(topic, msg, phase='cleanup')
                    except Exception as exc:
                        self.emit('cleanup_error', topic=topic, error=str(exc))
                time.sleep(0.1)
            confirmed = bool(self.ack and self.ack[0] == 1 and self.ack[1] >= since)
            self.emit('cleanup_disarm', acknowledged=confirmed)
            print('DISARM '+('acknowledged.' if confirmed else 'NOT CONFIRMED: use the hardware power stop.'), flush=True)
        for sub in self.subs:
            sub.unregister()
        if self.recorder is not None:
            if self.recorder.poll() is None:
                os.killpg(self.recorder.pid, signal.SIGINT)
                try:
                    self.recorder.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    print('Recorder is still finalizing; left running to preserve the bag.', flush=True)
            self.recorder_log.close()
        if self.child is not None:
            if self.child.poll() is None:
                os.killpg(self.child.pid, signal.SIGINT)
                try:
                    self.child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(self.child.pid, signal.SIGTERM)
                    self.child.wait(timeout=5)
            self.bridge_log.close()
        self.summary.update(published_counts=dict(self.records), failure=self.failure,
                            run_finished=datetime.now().astimezone().isoformat())
        (self.out/'run.json').write_text(json.dumps(self.summary, indent=2)+'\n')
        self.log.close()
        print('Replay log: '+str(self.out), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=DEFAULT_BAG)
    parser.add_argument('--duration', type=float, default=0, help='Seconds from bag start; 0 = entire bag')
    parser.add_argument('--execute', action='store_true', help='Actually publish commands; default is offline preview')
    parser.add_argument('--props-removed', action='store_true', help='Confirm propellers removed, robot secured and moving parts clear')
    parser.add_argument('--namespace', default='/gimbalrotor')
    parser.add_argument('--start-bridge', action='store_true', help='Start serial bridge only; ROS master must already exist')
    parser.add_argument('--port', default='/dev/flight_controller')
    parser.add_argument('--output', type=Path, default=Path.cwd()/'replay_runs', help='Execution logs parent; preview creates nothing')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if not args.namespace.startswith('/') or args.namespace == '/' or '//' in args.namespace:
        parser.error('--namespace must be an absolute non-root ROS namespace')
    if args.execute and not args.props_removed:
        parser.error('--execute requires --props-removed; motors and servos can move')
    if args.start_bridge and not args.execute:
        parser.error('--start-bridge requires --execute')
    events, first, classes, summary = load_plan(args.bag.expanduser().resolve(strict=True), args.duration)
    print(json.dumps(summary, indent=2), flush=True)
    if not args.execute:
        print('PREVIEW ONLY: no ROS node, publishers, actuator commands or files created.')
        return 0
    master = xmlrpc.client.ServerProxy(os.environ.get('ROS_MASTER_URI', 'http://localhost:11311'),
                                      transport=TimeoutTransport())
    if master.getPid('/replay_preflight')[0] != 1:
        raise RuntimeError('ROS master unavailable; start roscore first')
    import rospy
    rospy.init_node('spinal_recorded_command_replay', anonymous=True, disable_signals=True)
    replay = Replay(args, events, first, classes, summary)
    def stop(signum, frame):
        # Signal handlers must not acquire the log/event locks or publish ROS.
        # The main loop observes this flag and performs ordered cleanup.
        replay.interrupted = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    result = 0
    try:
        replay.run()
    except Exception as exc:
        replay.fail(str(exc))
        print('REPLAY STOPPED: '+str(exc), flush=True)
        result = 1
    finally:
        replay.close()
        rospy.signal_shutdown('Replay finished')
    return result


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        raise SystemExit('Replay error: '+str(exc))
