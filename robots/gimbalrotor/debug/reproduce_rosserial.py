#!/usr/bin/env python3
"""Offline, source-pinned rosserial interleaving and recorded-payload reproduction.

No node, network, device, compiler, ROS environment setup, or command publication.
rosbag is imported only to read existing files. Output is JSON on stdout.
This executes a Python port of serialization/framing, NOT the allocator or RTOS.
"""
import argparse
from collections import Counter
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import struct

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / 'aerial_robot_nerve').is_dir())
DEFAULT_RUN = Path('/home/clement/rosbag/year2026/11Nov/261101/5_pidtuning')
NS = '/gimbalrotor/'
SERVO = NS + 'servo/states'
PWM = NS + 'motor_pwms'
JST = timezone(timedelta(hours=9))


def clock(ns):
    return datetime.fromtimestamp(ns / 1e9, JST).isoformat(timespec='microseconds')


def verify_sources():
    """Fail on source drift: a changed implementation needs a new review."""
    manifest = json.loads((HERE / 'source_manifest.json').read_text())
    for entry in manifest['files']:
        path = REPO / entry['path']
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != entry['sha256']:
            raise ValueError('Source changed; review reproduction assumptions: ' + str(path))
    return manifest


def servo_info(payload):
    if len(payload) < 12:
        return {'valid': False, 'length': len(payload), 'reason': 'short header'}
    count = struct.unpack_from('<I', payload, 8)[0]
    expected = 12 + 7 * count
    return {'valid': expected == len(payload), 'length': len(payload),
            'count': count, 'required_bytes': expected}


def encode_servo(payload):
    """Same scalar order/widths as generated ServoStates/ServoState::serialize."""
    info = servo_info(payload)
    if not info['valid'] or info['count'] != 8:
        raise ValueError('Model requires one valid eight-servo sample')
    sec, nsec, count = struct.unpack_from('<III', payload)
    result = bytearray(struct.pack('<III', sec, nsec, count))
    for offset in range(12, len(payload), 7):
        result.extend(struct.pack('<BhBhB', *struct.unpack_from('<BhBhB', payload, offset)))
    return bytes(result)


def encode_pwm(payload):
    if len(payload) != 20 or struct.unpack_from('<I', payload)[0] != 8:
        raise ValueError('Model requires one valid eight-entry PWM sample')
    return struct.pack('<I8H', *struct.unpack('<I8H', payload))


def frame_valid(frame):
    return (len(frame) >= 8 and frame[:2] == b'\xff\xfe'
            and len(frame) == 8 + struct.unpack_from('<H', frame, 2)[0]
            and sum(frame[2:5]) % 256 == 255 and sum(frame[5:]) % 256 == 255)


class SharedBuffer:
    """Bounded port of NodeHandle_::publish and the TX queue's copying boundary."""
    def __init__(self):
        self.data = bytearray(512)

    def finish(self, topic, size):
        lo, hi = size & 255, size >> 8
        self.data[:7] = bytes((255, 254, lo, hi, 255 - ((lo + hi) & 255),
                               topic & 255, topic >> 8))
        self.data[size + 7] = 255 - (sum(self.data[5:size + 7]) & 255)
        return bytes(self.data[:size + 8])  # hardware.write copies to queue

    def publish(self, topic, payload, interrupt_at=None, interrupt=None):
        if len(payload) + 8 > len(self.data):
            raise ValueError('Bounded model does not implement oversize logging')
        # The generated serializers write successive bytes in this order.
        for index, value in enumerate(payload):
            if index == interrupt_at:
                interrupt()
            self.data[7 + index] = value
        return self.finish(topic, len(payload))


def reproduce(good_servo, pwm, observed=None, split=20):
    servo = encode_servo(good_servo)
    pwm = encode_pwm(pwm)
    if not 0 <= split < len(servo):
        raise ValueError('Interrupt offset must be within servo serialization')
    shared = SharedBuffer()
    frames = []
    mixed = shared.publish(157, servo, split,
                           lambda: frames.append(shared.publish(162, pwm)))
    # Full-operation serialization: a second publication waits until the first
    # has finished copying. This is an expected schedule, NOT a firmware patch.
    protected = SharedBuffer()
    clean = protected.publish(157, servo)
    clean_pwm = protected.publish(162, pwm)
    result = {
        'schedule': 'servo bytes 0..%d; complete PWM publish; remaining servo bytes; servo framing/copy' % (split - 1),
        'interruption_offset': split,
        'topic_ids': {'servo': 157, 'pwm': 162, 'note': 'session mapping checked in saved metadata'},
        'interleaved': {'frame_checksum_valid': frame_valid(mixed),
                        'pwm_frame_checksum_valid': frame_valid(frames[0]),
                        'servo': servo_info(mixed[7:-1]), 'payload_hex': mixed[7:-1].hex()},
        'serialized_schedule': {'frame_checksum_valid': frame_valid(clean),
                                'pwm_frame_checksum_valid': frame_valid(clean_pwm),
                                'servo': servo_info(clean[7:-1]),
                                'original_servo_preserved': clean[7:-1] == servo},
    }
    if observed is not None:
        result['comparison'] = {
            'recorded_prefix_matches_pwm': observed[:20] == pwm,
            'modeled_prefix_matches_recorded': mixed[7:27] == observed[:20],
            'same_invalid_count': servo_info(mixed[7:-1])['count'] == servo_info(observed)['count'],
            'whole_payload_matches': mixed[7:-1] == observed,
            'differing_offsets': [i for i, (a, b) in enumerate(zip(mixed[7:-1], observed)) if a != b],
            'note': 'Model uses the preceding valid servo sample. The interrupted sample before corruption is unknown; differing live servo values are expected.'}
    return result


def inspect_bags(paths):
    import rosbag  # offline reader; never calls init_node/master/services
    counts = Counter()
    last_servo = None
    last_pwm = None
    last_control = {}
    examples = {}
    state = None
    samples = []
    anomaly_count = 0
    logs = []
    topics = [SERVO, PWM, NS + 'flight_state', NS + 'four_axes/command',
              NS + 'torque_allocation_matrix_inv', NS + 'rpy/pid', NS + 'imu',
              NS + 'servo/target_states_debug', NS + 'gimbal_dof', '/rosout']
    for path in paths:
        with rosbag.Bag(str(path), 'r') as bag:
            for topic, raw, t in bag.read_messages(topics=topics, raw=True):
                ns = t.to_nsec()
                payload = raw[1]
                counts[topic] += 1
                if topic == SERVO:
                    info = servo_info(payload)
                    if info['valid']:
                        if info['count'] == 8:
                            last_servo = (ns, payload)
                        continue
                    anomaly_count += 1
                    if len(samples) >= 16:
                        continue
                    event = {'bag': str(path), 'time_jst': clock(ns), 'time_ns': ns,
                             'payload': info, 'payload_hex': payload.hex(),
                             'flight_state': state,
                             'seconds_since_control_messages': {k: (ns-v)/1e9 for k, v in last_control.items()}}
                    # genpy iterates actual available records, unlike C++ vector
                    # resize. Bound input and expected schema before trying it.
                    if len(payload) == 68 and raw[0] == 'spinal/ServoStates':
                        try:
                            raw[-1]().deserialize(payload)
                            event['python_decode'] = 'unexpected success'
                        except Exception as exc:
                            event['python_decode'] = type(exc).__name__ + ': ' + str(exc)
                    if last_servo and last_pwm and ns - last_pwm[0] <= 1_000_000_000:
                        event['preceding_pwm_delta_ms'] = (last_pwm[0] - ns) / 1e6
                        event['preceding_pwm_values'] = list(struct.unpack('<I8H', last_pwm[1])[1:])
                        event['reproduction'] = reproduce(last_servo[1], last_pwm[1], payload)
                    samples.append(event)
                elif topic == PWM:
                    if len(payload) == 20 and struct.unpack_from('<I', payload)[0] == 8:
                        last_pwm = (ns, payload)
                elif topic == NS + 'flight_state':
                    state = payload[0] if len(payload) == 1 else None
                elif topic in (NS + 'four_axes/command', NS + 'torque_allocation_matrix_inv'):
                    last_control[topic] = ns
                    if topic not in examples:
                        msg = raw[-1]().deserialize(payload)
                        examples[topic] = {'time_jst': clock(ns), 'message': str(msg)}
                        if topic.endswith('torque_allocation_matrix_inv'):
                            examples[topic]['matrix_shape'] = [len(msg.rows), 3]
                        else:
                            examples[topic]['force_component_count'] = len(msg.base_thrust)
                elif topic == '/rosout':
                    msg = raw[-1]().deserialize(payload)
                    if any(w in msg.msg.lower() for w in ('deserializ', 'checksum', 'bad_alloc')):
                        if len(logs) < 32:
                            logs.append({'time_jst': clock(ns), 'node': msg.name, 'text': msg.msg})
    return {'paths': [str(p) for p in paths], 'topic_counts': dict(counts),
            'recorded_control_path_examples': examples, 'malformed_servo_count': anomaly_count,
            'samples': samples, 'logs': logs,
            'scope': 'Recorded allocation/control outputs establish path activity. No allocator/physics/RTOS is executed by this Python program.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=DEFAULT_RUN,
                        help='Existing diagnostic folder containing sequential .bag splits')
    parser.add_argument('--model-only', action='store_true', help='No rosbag import or recording access')
    args = parser.parse_args()
    manifest = verify_sources()
    servo = struct.pack('<III', 1793520799, 147578055, 8)
    servo += b''.join(struct.pack('<BhBhB', i, 2047, 25, 0, 0) for i in range(8))
    pwm = struct.pack('<I8H', 8, 1000, 1000, 1000, 1000, 0, 0, 0, 0)
    result = {'source_verification': manifest, 'model': reproduce(servo, pwm),
              'limits': 'Deterministic possible interleaving; not proof of the historical MCU schedule. No full-flight simulation or firmware fix is run.'}
    if not args.model_only:
        paths = sorted(args.run.glob('*.bag'))
        if not paths:
            parser.error('No .bag files in ' + str(args.run))
        result['recording'] = inspect_bags(paths)
        metadata_path = args.run / 'metadata_start.json'
        if metadata_path.is_file():
            metadata = json.loads(metadata_path.read_text())
            params = metadata.get('parameters', {})
            result['recording']['saved_launch_selection'] = {
                k: params.get(k) for k in (
                    NS + 'aerial_robot_control_name', NS + 'flight_navigation_plugin_name')}
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
