"""Offline parser test source; not executed during this investigation."""

import struct
import unittest

from offline_audit import inspect_imu, inspect_servo, prefix_match


def imu():
    return struct.pack("<II13f", 1793028583, 895629698,
                       0.25, -0.5, 9.81, 0.01, 0.02, 0.03,
                       15.0, -20.0, 25.0, 0.0, 0.0, 0.0, 1.0)


def servo(count=8, records=8):
    return struct.pack("<III", 1793028583, 895629698, count) + bytes(7 * records)


class ParsingTests(unittest.TestCase):
    def test_valid(self):
        self.assertEqual(inspect_servo(servo())["issues"], [])
        self.assertEqual(inspect_imu(imu())["issues"], [])

    def test_inflated_counts(self):
        for count in (58356, 0xFFFFFFFF):
            parsed = inspect_servo(servo(count))
            self.assertEqual(parsed["expected_length"], 12 + 7 * count)
            self.assertIn("count_length_mismatch", parsed["issues"])
            self.assertNotIn("servos", parsed)  # No count-derived array.

    def test_structural_edge_cases(self):
        self.assertIn("truncated_header", inspect_servo(b"short")["issues"])
        for data in (servo()[:-1], servo() + b"extra"):
            self.assertIn("count_length_mismatch", inspect_servo(data)["issues"])
        zero = inspect_servo(servo(0, 0))
        self.assertNotIn("count_length_mismatch", zero["issues"])
        self.assertIn("unexpected_count_for_this_robot", zero["issues"])

    def test_imu_anomalies(self):
        self.assertIn("wrong_length", inspect_imu(imu()[:-1])["issues"])
        for value in (float("nan"), float("inf")):
            data = bytearray(imu())
            struct.pack_into("<f", data, 8, value)
            self.assertIn("nonfinite_float", inspect_imu(data)["issues"])
        data = bytearray(imu())
        struct.pack_into("<f", data, 56, 2.0)
        self.assertIn("quaternion_norm_outside_0.9_1.1", inspect_imu(data)["issues"])

    def test_overlap(self):
        data = bytearray(servo())
        data[8:40] = imu()[:32]
        self.assertGreaterEqual(prefix_match(data, imu())["bytes"], 32)
        self.assertEqual(prefix_match(data, imu())["servo_offset"], 8)
        self.assertEqual(prefix_match(imu() + bytes(8), imu())["bytes"], 60)
        self.assertIsNone(prefix_match(bytes(68), bytes(60)))
        self.assertIsNone(prefix_match(servo(), imu()))
        # Matches beyond the sample cap are intentionally not searched.
        self.assertIsNone(prefix_match(bytes(128) + imu(), imu()))


if __name__ == "__main__":
    unittest.main()
