#!/usr/bin/env python3
"""Offline raw-message audit; no ROS node, publishers, master or device access."""
import argparse
from collections import Counter
import json
from pathlib import Path

from payload_checks import inspect_payload, prefix_overlap


def summarize(paths, pwm_ceiling=None):
    import rosbag
    prefix = '/gimbalrotor/'
    raw_topics = [prefix + k for k in ('imu', 'servo/states', 'motor_pwms')]
    counts, issues, backwards = Counter(), Counter(), Counter()
    previous, retained, errors = {}, [], []
    anomaly_total = 0
    for path in paths:
        # Compare stamps within each bag; split boundaries are not tested here.
        previous.clear()
        with rosbag.Bag(str(path), 'r') as bag:
            for topic, raw, timestamp in bag.read_messages(topics=raw_topics + ['/rosout'], raw=True):
                counts[topic] += 1
                data = raw[1]
                if topic == '/rosout':
                    if len(data) > 16384:
                        issues['oversized_rosout_not_decoded'] += 1
                        continue
                    message = raw[-1]().deserialize(data)
                    text = message.msg[:2048]
                    if any(word in text.lower() for word in (
                            'deserializ', 'bad_alloc', 'buffer overrun', 'checksum', 'lost sync')):
                        issues['rosout_error'] += 1
                        if len(errors) < 100:
                            errors.append(dict(bag=str(path), bag_time_ns=timestamp.to_nsec(),
                                               node=message.name, text=text))
                    continue
                kind = topic[len(prefix):]
                parsed = inspect_payload(kind, data, pwm_ceiling=pwm_ceiling)
                stamp = parsed.get('stamp_ns')
                if stamp is not None:
                    if kind in previous and stamp < previous[kind]:
                        backwards[kind] += 1
                    previous[kind] = stamp
                if parsed['issues']:
                    anomaly_total += 1
                    issues.update(kind + ':' + issue for issue in parsed['issues'])
                    if len(retained) < 100:
                        retained.append(dict(bag=str(path), topic=topic, parsed=parsed,
                            bag_time_ns=timestamp.to_nsec(), sample_hex=data[:128].hex(),
                            best_prefix_overlap=None))
    servo_samples = [sample for sample in retained if sample['topic'] == prefix + 'servo/states']
    if servo_samples:
        # Second pass permits matches to packets recorded AFTER the corrupt servo
        # packet, including across consecutive split files in this run.
        for path in paths:
            with rosbag.Bag(str(path), 'r') as bag:
                for topic, raw, timestamp in bag.read_messages(
                        topics=[prefix+'imu', prefix+'motor_pwms'], raw=True):
                    candidate = raw[1]
                    if len(candidate) > 128:
                        continue
                    for sample in servo_samples:
                        delta = timestamp.to_nsec() - sample['bag_time_ns']
                        if abs(delta) > 1_000_000_000:
                            continue
                        match = prefix_overlap(bytes.fromhex(sample['sample_hex']), candidate)
                        if match and (sample['best_prefix_overlap'] is None or
                                      match['bytes'] > sample['best_prefix_overlap']['bytes']):
                            sample['best_prefix_overlap'] = dict(topic=topic,
                                bag_delta_ns=delta, candidate_hex=candidate.hex(), **match)
    for sample in retained:
        nearest = min(errors, key=lambda e: abs(e['bag_time_ns']-sample['bag_time_ns']), default=None)
        if nearest and abs(nearest['bag_time_ns']-sample['bag_time_ns']) <= 1_000_000_000:
            sample['nearby_rosout_error'] = nearest
    from event_context import annotate
    annotate(paths, retained + errors)
    return dict(bags=[str(p) for p in paths], counts=dict(counts), issues=dict(issues),
                pwm_observation_threshold=pwm_ceiling,
                backwards_stamps=dict(backwards), anomaly_total=anomaly_total,
                samples_retained=len(retained), anomalies=retained, rosout_errors=errors,
                interpretation='PWM ceiling violations and IMU norm outliers are observations, '
                'not proof of serialization corruption. Exact prefix matches support mixing '
                'but do not by themselves prove the firmware mechanism. ROS bag bytes are '
                'payloads after the bridge, not raw UART framing. No anomalies does not exclude a rare race.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bags', nargs='+', type=Path)
    parser.add_argument('--pwm-ceiling', type=float, default=None,
                        help='Optional reporting threshold; not an enforced motor limit')
    args = parser.parse_args()
    print(json.dumps(summarize(args.bags, pwm_ceiling=args.pwm_ceiling), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
