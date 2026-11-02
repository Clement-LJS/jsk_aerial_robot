#!/usr/bin/env python3
"""Read existing ROS1 bags; inspect only IMU, servo-state and error-log records.

No ROS node, network connection, publisher, device access, or file output.
Prints one JSON result per bag to stdout. Requires the existing rosbag module.
This saved script has not been executed; the report uses separate read-only
bag inspection performed during the investigation. No environment is configured.
"""

import argparse
import bisect
from collections import Counter
import json
import math
import struct

IMU = "/gimbalrotor/imu"
SERVO = "/gimbalrotor/servo/states"
TOPICS = (IMU, SERVO, "/rosout", "/rosout_agg")
MAX_ANOMALIES = 256
MAX_ERRORS = 512
MAX_SAMPLE_BYTES = 128
WINDOW_NS = 1_000_000_000


def stamp(data):
    return list(struct.unpack_from("<II", data)) if len(data) >= 8 else None


def inspect_servo(data):
    """Never deserialize or allocate an array using its declared length."""
    result = {"length": len(data), "stamp": stamp(data), "issues": []}
    if len(data) < 12:
        result["issues"].append("truncated_header")
        return result
    count = struct.unpack_from("<I", data, 8)[0]
    expected = 12 + 7 * count  # Python integer arithmetic; no allocation.
    result.update(count=count, expected_length=expected)
    if expected != len(data):
        result["issues"].append("count_length_mismatch")
    if count != 8:
        result["issues"].append("unexpected_count_for_this_robot")
    if result["stamp"][1] >= 1_000_000_000:
        result["issues"].append("invalid_nanoseconds")
    return result


def inspect_imu(data):
    result = {"length": len(data), "stamp": stamp(data), "issues": []}
    if len(data) != 60:
        result["issues"].append("wrong_length")
        return result
    if result["stamp"][1] >= 1_000_000_000:
        result["issues"].append("invalid_nanoseconds")
    values = struct.unpack_from("<13f", data, 8)
    if not all(math.isfinite(value) for value in values):
        result["issues"].append("nonfinite_float")
        return result
    norm = math.sqrt(sum(value * value for value in values[9:13]))
    if abs(norm - 1.0) > 0.1:
        result["issues"].append("quaternion_norm_outside_0.9_1.1")
    # This tolerance is an observation threshold, not proof of corruption.
    # Motion-dependent accelerometer/gyro limits are deliberately not assumed.
    return result


def prefix_match(servo_sample, imu):
    """Exact IMU prefix at any sample offset: >=16 bytes, >=4 byte values.

    Search is bounded by 128 sample bytes and 60 IMU bytes. A match indicates
    shared bytes, not by itself their causal origin. Zero padding is rejected.
    """
    if len(imu) != 60:
        return None
    sample = servo_sample[:MAX_SAMPLE_BYTES]
    best = None
    for offset in range(max(0, len(sample) - 15)):
        length = 0
        while length < min(60, len(sample) - offset):
            if sample[offset + length] != imu[length]:
                break
            length += 1
        if length >= 16 and len(set(imu[:length])) >= 4:
            if best is None or length > best["bytes"]:
                best = {"bytes": length, "servo_offset": offset}
    return best


def add_length(histogram, length):
    # Bound cardinality even for an unexpected variety of packet sizes.
    key = str(length)
    if key not in histogram and len(histogram) >= 16:
        key = "other"
    histogram[key] += 1


def audit(path):
    import rosbag  # File reader only; rospy is not imported.

    counts = Counter()
    lengths = {IMU: Counter(), SERVO: Counter()}
    imu_issues = Counter()
    error_counts = Counter()
    anomalies = []
    errors = []
    anomaly_total = 0
    oversized_logs = 0
    with rosbag.Bag(path, "r") as bag:
        for topic, raw, received in bag.read_messages(topics=TOPICS, raw=True):
            _, data, _, _, message_class = raw
            received_ns = received.to_nsec()
            counts[topic] += 1
            if topic == IMU:
                add_length(lengths[IMU], len(data))
                imu_issues.update(inspect_imu(data)["issues"])
            elif topic == SERVO:
                add_length(lengths[SERVO], len(data))
                parsed = inspect_servo(data)
                if parsed["issues"]:
                    anomaly_total += 1
                    if len(anomalies) < MAX_ANOMALIES:
                        parsed.update(bag_time_ns=received_ns,
                                      sample_hex=data[:MAX_SAMPLE_BYTES].hex(),
                                      sample_truncated=len(data) > MAX_SAMPLE_BYTES,
                                      imu_prefix_match=None)
                        anomalies.append(parsed)
            else:
                if len(data) > 16_384:
                    oversized_logs += 1
                    continue
                log = message_class().deserialize(data)
                if ("deserializ" not in log.msg.lower() or
                        not ("std::bad_alloc" in log.msg or "Buffer Overrun" in log.msg)):
                    continue
                # /rosout_agg duplicates are counted separately, never added
                # to /rosout to claim a larger number of distinct failures.
                kind = "std::bad_alloc" if "std::bad_alloc" in log.msg else "Buffer Overrun"
                error_counts[topic + ":" + kind] += 1
                if topic == "/rosout" and len(errors) < MAX_ERRORS:
                    errors.append({"bag_time_ns": received_ns,
                                   "log_stamp_ns": log.header.stamp.to_nsec(),
                                   "node": log.name[:256], "severity": log.level,
                                   "kind": kind})

    # Second pass avoids retaining all high-rate IMU messages. It also finds
    # IMU records arriving after a malformed servo record in bag-time order.
    anomalies.sort(key=lambda item: item["bag_time_ns"])
    times = [item["bag_time_ns"] for item in anomalies]
    if anomalies:
        with rosbag.Bag(path, "r") as bag:
            for _, raw, received in bag.read_messages(topics=[IMU], raw=True):
                data = raw[1]
                received_ns = received.to_nsec()
                start = bisect.bisect_left(times, received_ns - WINDOW_NS)
                end = bisect.bisect_right(times, received_ns + WINDOW_NS)
                for item in anomalies[start:end]:
                    match = prefix_match(bytes.fromhex(item["sample_hex"]), data)
                    if match is None:
                        continue
                    previous = item["imu_prefix_match"]
                    delta = received_ns - item["bag_time_ns"]
                    if (previous is None or match["bytes"] > previous["bytes"] or
                            (match["bytes"] == previous["bytes"] and
                             abs(delta) < abs(previous["bag_delta_ns"]))):
                        match.update(imu_stamp=stamp(data), bag_delta_ns=delta)
                        item["imu_prefix_match"] = match
    for item in anomalies:
        nearest = min(errors, key=lambda e: abs(e["bag_time_ns"] - item["bag_time_ns"]),
                      default=None)
        if nearest and abs(nearest["bag_time_ns"] - item["bag_time_ns"]) <= WINDOW_NS:
            item["nearest_rosout_error"] = dict(nearest)
            item["nearest_rosout_error"]["bag_delta_ns"] = (
                nearest["bag_time_ns"] - item["bag_time_ns"])
        else:
            item["nearest_rosout_error"] = None
    return {"bag": path, "counts": dict(counts), "lengths": lengths,
            "imu_issues": dict(imu_issues), "servo_anomaly_total": anomaly_total,
            "servo_samples_omitted": anomaly_total - len(anomalies),
            "servo_anomalies": anomalies, "error_counts": dict(error_counts),
            "rosout_errors_retained": len(errors), "oversized_log_records_skipped": oversized_logs,
            "limits": {"servo_samples": MAX_ANOMALIES, "rosout_samples": MAX_ERRORS,
                       "raw_sample_bytes": MAX_SAMPLE_BYTES, "match_window_ns": WINDOW_NS},
            "time_note": "Bag reception time is not MCU time or monotonic arrival time; no latency claim."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+")
    args = parser.parse_args()
    for path in args.bags:
        print(json.dumps(audit(path), sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
