"""Offline tests only: no ROS import, master, publishers or device access."""
import json
import struct
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from bench_capture import Bench, NS, BASE_NODE
from payload_checks import inspect_payload, motor_info_is_capped, prefix_overlap


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, value):
        self.messages.append(value)


def fake_bench():
    b = Bench.__new__(Bench)
    b.counts = __import__('collections').Counter()
    b.stopping = False
    b.graph_ok = True
    b.ready = True
    b.state = 0
    b.arm_sent = None
    b.takeoff_sent = None
    b.cap_seen = False
    b.pwm_values = (1000, 1000, 1000, 1000)
    b.last = {'motor_pwms': time.monotonic() + 1}
    b.Empty = object
    b.start_pub = Publisher()
    b.takeoff_pub = Publisher()
    b.fresh = lambda now: True
    b.emit = lambda *args, **kwargs: None
    return b


class PayloadTests(unittest.TestCase):
    def test_bad_count_no_allocation(self):
        for count in (9, 65537000, 0xFFFFFFFF):
            packet = struct.pack('<III', 1, 2, count) + bytes(56)
            self.assertIn('invalid_servo_count_or_length', inspect_payload('servo/states', packet)['issues'])

    def test_valid_servo_and_truncation(self):
        packet = struct.pack('<III', 1, 2, 8) + bytes(56)
        self.assertFalse(inspect_payload('servo/states', packet)['issues'])
        for size in (0, 7, 11, 67):
            self.assertTrue(inspect_payload('servo/states', packet[:size])['issues'])

    def test_imu(self):
        packet = struct.pack('<II13f', 1, 2, 0, 0, 9.81, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1)
        self.assertFalse(inspect_payload('imu', packet)['issues'])
        bad = bytearray(packet)
        struct.pack_into('<f', bad, 8, float('nan'))
        result = inspect_payload('imu', bad)
        self.assertIn('nonfinite_imu', result['issues'])
        json.dumps(result, allow_nan=False)

    def test_pwm_limit_and_bounded_length(self):
        def packet(values):
            return struct.pack('<I' + 'H'*len(values), len(values), *values)
        self.assertFalse(inspect_payload('motor_pwms', packet([1100]*4 + [0]*4))['issues'])
        self.assertFalse(inspect_payload('motor_pwms', packet([]))['issues'])
        self.assertIn('pwm_above_bench_cap', inspect_payload('motor_pwms', packet([1170]*4))['issues'])
        self.assertTrue(inspect_payload('motor_pwms', struct.pack('<I', 0xFFFFFFFF))['issues'])

    def test_motor_info(self):
        def packet(low, high, thrust):
            return struct.pack('<4fBI', low, high, thrust, 6, 0, 0)
        self.assertTrue(motor_info_is_capped(packet(.5, .55, 0)))
        for vals in ((.5, .85, 0), (.5, .55, 2), (.5, float('nan'), 0)):
            self.assertFalse(motor_info_is_capped(packet(*vals)))

    def test_prefix_overlap(self):
        pwm = struct.pack('<I8H', 8, 1000, 1000, 1000, 1000, 0, 0, 0, 0)
        self.assertEqual(prefix_overlap(pwm + bytes(48), pwm)['bytes'], 20)
        self.assertIsNone(prefix_overlap(bytes(68), bytes(60)))
        self.assertIsNone(prefix_overlap(bytes(range(68)), bytes(range(100, 160))))


class GateTests(unittest.TestCase):
    def test_no_automatic_commands(self):
        b = fake_bench()
        self.assertEqual(len(b.start_pub.messages), 0)
        self.assertEqual(len(b.takeoff_pub.messages), 0)

    def test_requires_manual_fresh_ready_request(self):
        for field in ('ready', 'graph_ok'):
            b = fake_bench()
            setattr(b, field, False)
            b.process(time.monotonic(), 'request_start', None)
            self.assertFalse(b.start_pub.messages)
        b = fake_bench()
        b.process(time.monotonic()-1, 'request_start', None)
        self.assertFalse(b.start_pub.messages)
        b.process(time.monotonic(), 'request_start', None)
        self.assertEqual(len(b.start_pub.messages), 1)
        b.process(time.monotonic(), 'request_start', None)
        self.assertEqual(len(b.start_pub.messages), 1)

    def test_takeoff_needs_arm_and_observed_cap(self):
        b = fake_bench()
        b.state = 2
        b.process(time.monotonic(), 'request_takeoff', None)
        self.assertFalse(b.takeoff_pub.messages)
        b.arm_sent = time.monotonic()
        b.process(time.monotonic(), 'request_takeoff', None)
        self.assertFalse(b.takeoff_pub.messages)
        b.cap_seen = True
        b.process(time.monotonic(), 'request_takeoff', None)
        self.assertEqual(len(b.takeoff_pub.messages), 1)
        b.process(time.monotonic(), 'request_takeoff', None)
        self.assertEqual(len(b.takeoff_pub.messages), 1)

    def test_stopping_never_forwards(self):
        b = fake_bench()
        b.stopping = True
        b.process(time.monotonic(), 'request_start', None)
        b.state, b.arm_sent, b.cap_seen = 2, time.monotonic(), True
        b.process(time.monotonic(), 'request_takeoff', None)
        self.assertFalse(b.start_pub.messages or b.takeoff_pub.messages)

    def test_takeoff_requires_initialized_motor_feedback(self):
        b = fake_bench()
        b.state, b.arm_sent, b.cap_seen = 2, time.monotonic(), True
        for values in ((), (0, 0, 0, 0), (1170, 1170, 1170, 1170)):
            b.pwm_values = values
            b.process(time.monotonic(), 'request_takeoff', None)
            self.assertFalse(b.takeoff_pub.messages)

    def test_stale_feedback_blocks_manual_request(self):
        b = fake_bench()
        b.fresh = lambda now: False
        b.process(time.monotonic(), 'request_start', None)
        self.assertFalse(b.start_pub.messages)

    def test_launch_limits_and_remaps(self):
        root = ET.parse(Path(__file__).with_name('bench.launch')).getroot()
        params = {p.get('name'): p.get('value') for p in root.findall('param')}
        self.assertEqual(float(params[NS+'/motor_info/max_pwm']), .55)
        self.assertEqual(float(params[NS+'/motor_info/min_thrust']), 0)
        remaps = {r.get('from'): r.get('to') for r in root.findall('group/remap')}
        self.assertEqual(remaps[NS+'/teleop_command/start'], NS+'/simulate_run/gated_start')


if __name__ == '__main__':
    unittest.main()
