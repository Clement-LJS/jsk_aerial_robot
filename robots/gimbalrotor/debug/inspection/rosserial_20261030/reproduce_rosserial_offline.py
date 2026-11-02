#!/usr/bin/env python3
"""Bounded offline reproduction of rosserial payload corruption.

Default: deterministic byte-buffer model, no ROS installation needed.
--bag: also reproduce Python ROS decoding failure on recorded ServoStates.
No ROS node, master access, sockets, serial devices, publishers, or file writes.
This does not reproduce an actual MCU scheduling event or C++ bad_alloc.
"""

import argparse
import json
import struct
import sys


SERVO_TOPIC = "/gimbalrotor/servo/states"
SERVO_MD5 = "8ab23ee17ec1578fba9ec2244276b0ef"
MAX_SAMPLES = 16


class BufferOverrun(ValueError):
    """Local bounded parser exception, not the ROS C++ exception class."""


def decode_bounded(payload):
    if len(payload) < 12:
        raise BufferOverrun("Buffer Overrun: incomplete ServoStates header")
    count = struct.unpack_from("<I", payload, 8)[0]
    expected = 12 + 7 * count
    if expected != len(payload):
        raise BufferOverrun(
            "Buffer Overrun: count={}, requires {} bytes, actual {}".format(
                count, expected, len(payload)))
    # No record array or count-derived allocation is ever created here.
    return {"count": count, "length": len(payload)}


def outcome(payload):
    try:
        return {"valid": True, **decode_bounded(payload)}
    except BufferOverrun as error:
        return {"valid": False, "error": str(error)}


class SharedOutputModel:
    """Model of node_handle.h: serialize, header, checksum, enqueue-copy.

    Explicit hooks force a possible interleaving. No timing luck or real
    threads are involved. Topic IDs are synthetic, not the recorded IDs.
    """

    def __init__(self):
        self.buffer = bytearray(1024)
        self.frames = []

    def publish(self, topic_id, payload, after_serialize=None):
        size = len(payload)
        if size + 8 > len(self.buffer):
            raise ValueError("model payload exceeds bounded output buffer")
        self.buffer[7:7 + size] = payload
        if after_serialize:
            after_serialize()
        lo, hi = size & 255, size >> 8
        self.buffer[:7] = bytes((255, 254, lo, hi, 255 - ((lo + hi) & 255),
                                 topic_id & 255, topic_id >> 8))
        self.buffer[7 + size] = 255 - (sum(self.buffer[5:7 + size]) & 255)
        frame = bytes(self.buffer[:size + 8])
        self.frames.append(frame)  # Copy, as the UART queue does.
        return frame


def frame_valid(frame):
    if len(frame) < 8 or frame[:2] != b"\xff\xfe":
        return False
    size = struct.unpack_from("<H", frame, 2)[0]
    return (len(frame) == size + 8 and sum(frame[2:5]) % 256 == 255
            and sum(frame[5:]) % 256 == 255)


def model_cases():
    servo = struct.pack("<III", 1793340146, 200000000, 8)
    servo += b"".join(struct.pack("<BhBhB", i, 2047, 30, 0, 0) for i in range(8))
    imu = struct.pack("<II13f", 1793340146, 205000000,
                      -0.25, 0.5, 9.81, 0.01, 0.02, 0.03,
                      25, -15, -40, 0, 0, 0, 1)
    pwm_telemetry = struct.pack("<I8H", 8, 1000, 1000, 1000, 1000, 0, 0, 0, 0)
    results = []
    for name, other in (("IMU interrupts servo serialization", imu),
                        ("PWM telemetry interrupts servo serialization", pwm_telemetry)):
        shared = SharedOutputModel()
        frame = shared.publish(150, servo,
                               after_serialize=lambda: shared.publish(151, other))
        payload = frame[7:-1]
        results.append({"case": name, "serial_frame_checksum_valid": frame_valid(frame),
                        "servo_payload_hex": payload.hex(), "decode": outcome(payload),
                        "foreign_prefix_bytes": len(other),
                        "foreign_prefix_matches": payload.startswith(other)})
    control = SharedOutputModel()
    control.publish(151, imu)
    valid = control.publish(150, servo)
    results.append({"case": "Noninterleaved complete publish operations",
                    "serial_frame_checksum_valid": frame_valid(valid),
                    "decode": outcome(valid[7:-1]),
                    "note": "Serialized schedule control; not an implemented firmware mutex fix."})
    changed = bytearray(valid)
    changed[15] ^= 1  # Simulate data changing after checksum computation.
    results.append({"case": "One payload byte changes after checksum construction",
                    "serial_frame_checksum_valid": frame_valid(changed),
                    "note": "Demonstrates checksum rejection; does not identify the recorded topicId."})
    return results


def recorded_cases(path):
    try:
        import rosbag
        import genpy
    except ImportError as error:
        raise RuntimeError("Use an existing ROS1 Python environment, or supply "
                           "--ros-python-path /opt/ros/one/lib/python3/dist-packages") from error

    samples = []
    bad_total = 0
    with rosbag.Bag(path, "r") as bag:
        for _, raw, timestamp in bag.read_messages(topics=[SERVO_TOPIC], raw=True):
            datatype, payload, md5, _, message_class = raw
            if datatype != "spinal/ServoStates" or md5 != SERVO_MD5:
                raise ValueError("Unexpected ServoStates schema; refusing typed decode")
            checked = outcome(payload)
            if checked["valid"]:
                continue
            bad_total += 1
            if len(samples) >= MAX_SAMPLES:
                continue
            sample = {"bag_time_ns": timestamp.to_nsec(), "bounded_check": checked,
                      "payload_length": len(payload), "hex": payload[:128].hex()}
            # The inspected ROS1 genpy decoder appends one record at a time,
            # with range() in Python 3. It raises on the first incomplete
            # 7-byte record; it does not preallocate the declared array count.
            # Restrict the input to 68 bytes: at most eight records can decode.
            # Never pass this huge-count payload to a C++ vector decoder.
            if len(payload) == 68:
                try:
                    message_class().deserialize(payload)
                except genpy.DeserializationError as error:
                    sample["actual_python_ros_exception"] = type(error).__name__ + ": " + str(error)
                else:
                    sample["actual_python_ros_exception"] = None
            else:
                sample["typed_decode_skipped"] = "Only 68-byte inputs allowed"
            samples.append(sample)
    return {"bag": path, "malformed_total": bad_total,
            "samples_omitted": bad_total - len(samples), "samples": samples}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", help="Existing bag to read offline; never played")
    parser.add_argument("--ros-python-path", help="Existing ROS1 Python library directory")
    args = parser.parse_args()
    if args.ros_python_path:
        sys.path.insert(0, args.ros_python_path)
    result = {"model": model_cases(),
              "limitations": ["Deterministic software model, not a captured MCU race",
                              "Python underfill is not the C++ std::bad_alloc exception",
                              "No robot communication or control"]}
    if args.bag:
        result["recorded"] = recorded_cases(args.bag)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
