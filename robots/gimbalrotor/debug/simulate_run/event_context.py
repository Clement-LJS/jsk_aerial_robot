"""Add before/after activity to bounded error samples using raw bag reads."""
import bisect
from collections import Counter
from payload_checks import inspect_payload
from passive_monitor import FLIGHT_STATES


def annotate(paths, events):
    if not events:
        return
    import rosbag
    ordered = sorted(events, key=lambda e: e['bag_time_ns'])
    times = [e['bag_time_ns'] for e in ordered]
    for event in ordered:
        event['context'] = dict(window_seconds_each_side=1, flight_state_at_or_before=None,
                               last_manual_commands={}, last_motor_pwms=None,
                               before=dict(topic_counts=Counter(), gyro_peak_rad_s=None,
                                           acc_norm_min_m_s2=None, acc_norm_max_m_s2=None),
                               after=dict(topic_counts=Counter(), gyro_peak_rad_s=None,
                                          acc_norm_min_m_s2=None, acc_norm_max_m_s2=None))
    state, pwm, commands = None, None, {}
    for path in paths:
        with rosbag.Bag(str(path), 'r') as bag:
            for topic, raw, stamp in bag.read_messages(raw=True):
                at = stamp.to_nsec()
                data = raw[1]
                prefix = '/gimbalrotor/'
                kind = topic[len(prefix):] if topic.startswith(prefix) else topic
                if kind == 'flight_state' and len(data) == 1:
                    state = dict(value=data[0], name=FLIGHT_STATES.get(data[0], 'UNKNOWN_STATUS'), bag_time_ns=at)
                if kind.startswith('teleop_command/'):
                    commands[kind] = at
                parsed = None
                if kind in ('imu', 'motor_pwms'):
                    parsed = inspect_payload(kind, data)
                    if kind == 'motor_pwms' and 'values' in parsed:
                        pwm = dict(values=parsed['values'], bag_time_ns=at)
                for index in range(bisect.bisect_left(times, at-1_000_000_000),
                                   bisect.bisect_right(times, at+1_000_000_000)):
                    event = ordered[index]
                    context = event['context']
                    before = at <= event['bag_time_ns']
                    side = context['before' if before else 'after']
                    side['topic_counts'][topic] += 1
                    if before:
                        context['flight_state_at_or_before'] = state
                        context['last_motor_pwms'] = pwm
                        context['last_manual_commands'] = dict(commands)
                    if kind == 'imu' and 'gyro_norm' in parsed:
                        old = side['gyro_peak_rad_s']
                        side['gyro_peak_rad_s'] = max(parsed['gyro_norm'], old or 0)
                        low = side['acc_norm_min_m_s2']
                        high = side['acc_norm_max_m_s2']
                        side['acc_norm_min_m_s2'] = min(low, parsed['acc_norm']) if low is not None else parsed['acc_norm']
                        side['acc_norm_max_m_s2'] = max(high, parsed['acc_norm']) if high is not None else parsed['acc_norm']
