"""Offline invariants for the corruption mechanism; no ROS graph or hardware."""
import struct
import unittest
from reproduce_rosserial import encode_servo, frame_valid, reproduce, servo_info, SharedBuffer, verify_sources


class ReproductionTests(unittest.TestCase):
    def setUp(self):
        self.servo = struct.pack('<III', 1793520799, 147578055, 8)
        self.servo += b''.join(struct.pack('<BhBhB', i, 2100+i, 25, -10, 0) for i in range(8))
        self.pwm = struct.pack('<I8H', 8, 1000, 1000, 1000, 1000, 0, 0, 0, 0)

    def test_observed_prefix_and_count_with_valid_frame(self):
        r = reproduce(self.servo, self.pwm)
        self.assertEqual(bytes.fromhex(r['interleaved']['payload_hex']), self.pwm + self.servo[20:])
        self.assertEqual(r['interleaved']['servo']['count'], 65537000)
        self.assertEqual(r['interleaved']['servo']['required_bytes'], 458759012)
        self.assertTrue(r['interleaved']['frame_checksum_valid'])
        self.assertFalse(r['interleaved']['servo']['valid'])

    def test_complete_publication_schedule_preserves_both_messages(self):
        r = reproduce(self.servo, self.pwm)['serialized_schedule']
        self.assertTrue(r['original_servo_preserved'])
        self.assertTrue(r['servo']['valid'])
        self.assertTrue(r['frame_checksum_valid'])
        self.assertTrue(r['pwm_frame_checksum_valid'])

    def test_schedule_matters(self):
        # Before any servo byte is written, a PWM publication cannot leave a
        # foreign prefix: all of its bytes are subsequently overwritten.
        r = reproduce(self.servo, self.pwm, split=0)
        self.assertEqual(bytes.fromhex(r['interleaved']['payload_hex']), self.servo)
        self.assertTrue(r['interleaved']['servo']['valid'])

    def test_post_checksum_change_rejected(self):
        frame = bytearray(SharedBuffer().publish(157, self.servo))
        frame[8] ^= 1
        self.assertFalse(frame_valid(frame))

    def test_bad_counts_do_not_drive_allocation(self):
        payload = struct.pack('<III', 1, 2, 0xffffffff)
        self.assertFalse(servo_info(payload)['valid'])
        with self.assertRaises(ValueError):
            encode_servo(payload)

    def test_reviewed_source_still_matches(self):
        self.assertEqual(len(verify_sources()['files']), 15)


if __name__ == '__main__':
    unittest.main()
