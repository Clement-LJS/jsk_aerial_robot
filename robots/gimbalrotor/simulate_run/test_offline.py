"""Offline tests: no ROS node, master, hardware or real subprocess."""
import ast
import io
import json
import queue
import signal
import struct
import time
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

from bench_capture import Capture, NS, TOPICS
from passive_monitor import Monitor
from payload_checks import inspect_payload, read_motor_limits, prefix_overlap
from capture_paths import create_run, usb_run


class CapturePathTests(unittest.TestCase):
    def test_invocation_directory_and_usb_pairing(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            with unittest.mock.patch('capture_paths.Path.cwd', return_value=parent):
                folder, lock = create_run()
                try:
                    self.assertEqual(folder.parent, parent)
                    self.assertEqual(usb_run(), folder)
                    self.assertEqual(usb_run(cwd=folder), folder)
                finally:
                    lock.close()
            with self.assertRaisesRegex(ValueError, 'No active ROS capture'):
                usb_run(cwd=parent)

    def test_old_folder_not_selected_and_ambiguous_runs_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory)
            old, old_lock = create_run(parent)
            old_lock.close()
            first, lock1 = create_run(parent)
            second, lock2 = create_run(parent)
            try:
                with self.assertRaisesRegex(ValueError, 'Multiple active'):
                    usb_run(cwd=parent)
                self.assertEqual(usb_run(output=first), first)
                lock2.close()
                self.assertEqual(usb_run(cwd=parent), first)
            finally:
                lock1.close()
                lock2.close()

    def test_missing_explicit_output(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileNotFoundError):
                usb_run(output=Path(directory) / 'absent')


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

    def test_pwm_threshold_is_opt_in(self):
        def packet(values):
            return struct.pack('<I' + 'H'*len(values), len(values), *values)
        self.assertFalse(inspect_payload('motor_pwms', packet([1170]*4))['issues'])
        self.assertFalse(inspect_payload('motor_pwms', packet([]))['issues'])
        self.assertIn('pwm_above_observation_threshold',
                      inspect_payload('motor_pwms', packet([1170]*4), 1100)['issues'])
        self.assertTrue(inspect_payload('motor_pwms', struct.pack('<I', 0xFFFFFFFF))['issues'])

    def test_observes_normal_and_bench_motor_limits(self):
        for high, thrust in ((.55, 0), (.85, 2)):
            result = read_motor_limits(struct.pack('<4fBI', .5, high, thrust, 6, 0, 0))
            self.assertAlmostEqual(result['max_pwm_equivalent'], high*2000, places=3)
        self.assertIsNone(read_motor_limits(struct.pack('<4fBI', .5, float('nan'), 0, 6, 0, 0)))
        self.assertIsNone(read_motor_limits(b''))

    def test_prefix_overlap(self):
        pwm = struct.pack('<I8H', 8, 1000, 1000, 1000, 1000, 0, 0, 0, 0)
        self.assertEqual(prefix_overlap(pwm + bytes(48), pwm)['bytes'], 20)
        self.assertIsNone(prefix_overlap(bytes(68), bytes(60)))
        self.assertIsNone(prefix_overlap(bytes(range(68)), bytes(range(100, 160))))


class PassiveTests(unittest.TestCase):
    def test_status_overrides_are_not_corruption(self):
        m = Monitor()
        for value, name in ((16, 'LOW_BATTERY'), (17, 'FORCE_LANDING')):
            event = m.process(1, 'flight_state', bytes([value]))[0]
            self.assertEqual(event['name'], name)
            self.assertEqual(m.issues['malformed_flight_state'], 0)

    def test_error_context_contains_motion_and_manual_timing(self):
        m = Monitor()
        m.process(1, 'teleop_command/start', b'')
        m.process(1.1, 'flight_state', b'\x02')
        imu = struct.pack('<II13f', 1, 0, 0, 0, 9.81, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1)
        m.process(1.2, 'imu', imu)
        context = m.process(1.3, 'rosout_error', {'text':'bad checksum'})[0]['context']
        self.assertEqual(context['flight_state'], 2)
        self.assertEqual(context['gyro_peak_rad_s'], 1)
        self.assertAlmostEqual(context['seconds_since_manual_command']['teleop_command/start'], .3)

    def test_usb_command_filters_one_device_without_opening_tty(self):
        from capture_usb import capture_command
        command = capture_command({'bus':3, 'device':7}, Path('/tmp/example.pcap'))
        self.assertEqual(command[-1], 'link[11] = 7 and link[9] = 3')
        self.assertIn('USB_LINUX_MMAPPED', command)
        self.assertFalse(any('ttyUSB' in part for part in command))

    def test_checksum_then_manual_control_keeps_processing(self):
        m = Monitor()
        error = {'node': '/gimbalrotor/rosserial_server', 'text': 'bad checksum'}
        self.assertEqual(m.process(1, 'rosout_error', error)[0]['event'], 'rosout_error')
        self.assertEqual(m.process(2, 'teleop_command/start', b'')[0]['event'], 'manual_command_observed')
        m.process(3, 'flight_state', b'\x02')
        m.process(4, 'teleop_command/takeoff', b'')
        m.process(5, 'flight_state', b'\x03')
        m.process(6, 'servo/states', struct.pack('<III', 1, 2, 8) + bytes(56))
        self.assertEqual(m.state, 3)
        self.assertEqual(m.counts['servo/states'], 1)
        self.assertEqual(m.issues['rosout_error'], 1)

    def test_malformed_then_valid_packet_continues(self):
        m = Monitor()
        m.process(1, 'servo/states', struct.pack('<III', 1, 2, 0xFFFFFFFF) + bytes(56))
        m.process(2, 'servo/states', struct.pack('<III', 1, 3, 8) + bytes(56))
        self.assertEqual(m.counts['servo/states'], 2)
        self.assertEqual(m.issues['servo/states:invalid_servo_count_or_length'], 1)

    def test_pwm_warning_does_not_change_observed_state(self):
        m = Monitor(1100)
        m.process(1, 'flight_state', b'\x03')
        events = m.process(2, 'motor_pwms', struct.pack('<I4H', 4, *([1700]*4)))
        self.assertEqual(m.state, 3)
        self.assertEqual(events[0]['event'], 'payload_anomaly')
        self.assertEqual(len(m.histories['motor_pwms']), 1)

    def test_error_flood_is_bounded_and_counts_continue(self):
        m = Monitor()
        emitted = []
        for index in range(2000):
            emitted += m.process(index, 'rosout_error', {'text': 'bad checksum'})
        self.assertEqual(len(emitted), 101)
        self.assertEqual(m.issues['rosout_error'], 2000)
        for index in range(1000):
            m.process(index, 'motor_pwms', struct.pack('<I4H', 4, *([1000]*4)))
        self.assertEqual(len(m.histories['motor_pwms']), 300)

    def test_no_ros_command_api_in_capture_path(self):
        forbidden = {'Publisher', 'publish', 'ServiceProxy', 'set_param', 'delete_param'}
        for name in ('bench_capture.py', 'passive_monitor.py'):
            tree = ast.parse(Path(__file__).with_name(name).read_text())
            for node in ast.walk(tree):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    self.assertNotIn(node.func.attr, forbidden, name)
        self.assertFalse(any('gated_' in topic for topic in TOPICS))

    def test_close_only_stops_recorder_even_after_log_failure(self):
        c = Capture.__new__(Capture)
        c.closed = False
        c.subscribers = [Mock()]
        c.monitor = Monitor()
        c.events = queue.Queue()
        c.recorder = Mock()
        c.recorder.poll.return_value = None
        c.log_file, c.recorder_log = io.StringIO(), io.StringIO()
        c.emit = Mock(side_effect=OSError('disk full'))
        c.directory = Path('/tmp/offline-test-no-files-created')
        c.close(analyze=False)
        c.recorder.send_signal.assert_called_once_with(signal.SIGINT)
        c.recorder.wait.assert_called_once_with(timeout=15)
        self.assertTrue(c.log_file.closed)

    def test_launch_has_limits_without_command_remaps(self):
        root = ET.parse(Path(__file__).with_name('bench.launch')).getroot()
        params = {p.get('name'): p.get('value') for p in root.findall('param')}
        self.assertEqual(float(params[NS+'/motor_info/max_pwm']), .55)
        self.assertEqual(float(params[NS+'/motor_info/min_thrust']), 0)
        self.assertEqual(root.findall('.//remap'), [])
        self.assertTrue(root.find('include').get('file').endswith('/launch/bringup.launch'))


if __name__ == '__main__':
    unittest.main()
