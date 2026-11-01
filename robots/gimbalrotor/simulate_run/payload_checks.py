"""Bounded decoding for the specific ROS messages used by this bench capture."""
import math
import struct

def inspect_payload(kind, data, pwm_ceiling=None):
    result = {'length': len(data), 'issues': []}
    issues = result['issues']
    if kind in ('imu', 'servo/states'):
        if len(data) >= 8:
            sec, nsec = struct.unpack_from('<II', data)
            result['stamp_ns'] = sec * 1_000_000_000 + nsec
            if nsec >= 1_000_000_000:
                issues.append('invalid_nanoseconds')
        if kind == 'imu':
            if len(data) != 60:
                issues.append('wrong_imu_length')
            else:
                v = struct.unpack_from('<13f', data, 8)
                if not all(math.isfinite(x) for x in v):
                    issues.append('nonfinite_imu')
                else:
                    result.update(acc_norm=math.sqrt(sum(x*x for x in v[:3])),
                                  gyro_norm=math.sqrt(sum(x*x for x in v[3:6])),
                                  quaternion_norm=math.sqrt(sum(x*x for x in v[9:])))
                    if abs(result['quaternion_norm'] - 1) > 0.1:
                        issues.append('quaternion_norm_outlier')
        elif len(data) < 12:
            issues.append('truncated_servo_header')
        else:
            count = struct.unpack_from('<I', data, 8)[0]
            result.update(count=count, expected_length=12 + 7*count)
            if 12 + 7*count != len(data) or count != 8:
                issues.append('invalid_servo_count_or_length')
    elif kind == 'motor_pwms':
        if len(data) < 4:
            issues.append('truncated_pwm_header')
        else:
            count = struct.unpack_from('<I', data)[0]
            result['count'] = count
            # A freshly booted FC has zero motors until START sends UavInfo.
            if count not in (0, 4, 8) or len(data) != 4 + 2*count:
                issues.append('invalid_pwm_count_or_length')
            else:
                values = struct.unpack_from('<' + str(count) + 'H', data, 4)
                result['values'] = values
                # Current beetle has four motors; zero remaining entries are normal.
                if pwm_ceiling is not None and any(x > pwm_ceiling + 1 for x in values):
                    issues.append('pwm_above_observation_threshold')
    return result


def read_motor_limits(data):
    if len(data) < 21:
        return None
    minimum, maximum, thrust, landing = struct.unpack_from('<4f', data)
    if not all(math.isfinite(x) for x in (minimum, maximum, thrust, landing)):
        return None
    return dict(min_pwm=minimum, max_pwm=maximum, min_thrust=thrust,
                force_landing_thrust=landing, max_pwm_equivalent=maximum * 2000)


def prefix_overlap(sample, candidate):
    """Find a nontrivial exact prefix overlap, bounded to small message samples."""
    best = None
    for offset in range(max(0, len(sample[:128]) - 15)):
        size = 0
        while size < min(len(candidate), 128 - offset, len(sample) - offset):
            if sample[offset + size] != candidate[size]:
                break
            size += 1
        if size >= 16 and len(set(candidate[:size])) >= 4:
            if best is None or size > best['bytes']:
                best = {'offset': offset, 'bytes': size}
    return best
