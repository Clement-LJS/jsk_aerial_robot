"""Bounded, ROS-independent observation state. Produces records, never commands."""
from collections import Counter, deque
from payload_checks import inspect_payload, prefix_overlap, read_motor_limits

FLIGHT_STATES = {0: 'ARM_OFF', 1: 'START', 2: 'ARM_ON', 3: 'TAKEOFF',
                 4: 'LAND', 5: 'HOVER', 6: 'STOP', 16: 'LOW_BATTERY', 17: 'FORCE_LANDING'}


class Monitor:
    def __init__(self, pwm_ceiling=None):
        self.pwm_ceiling = pwm_ceiling
        self.counts, self.issues = Counter(), Counter()
        self.last, self.stamps = {}, {}
        self.histories = {key: deque(maxlen=300) for key in ('imu', 'motor_pwms')}
        self.details = Counter()
        self.state = None
        self.motor_limits = None
        self.windows = {}
        self.motion = deque(maxlen=512)
        self.pwm = None
        self.command_times = {}

    def context(self, now):
        samples = [p for when, p in self.motion if now - when <= 1]
        return dict(flight_state=self.state,
                    seconds_since_manual_command={k: now-v for k, v in self.command_times.items()},
                    messages_in_previous_second={k: sum(now-t <= 1 for t, _ in q)
                                                 for k, q in self.windows.items()},
                    bytes_in_previous_second={k: sum(n for t, n in q if now-t <= 1)
                                              for k, q in self.windows.items()},
                    imu_samples=len(samples),
                    gyro_peak_rad_s=max((p['gyro_norm'] for p in samples), default=None),
                    acc_norm_range_m_s2=[min(p['acc_norm'] for p in samples),
                                         max(p['acc_norm'] for p in samples)] if samples else None,
                    last_motor_pwms=self.pwm,
                    note='Local callback windows, not wire timings; history is bounded to 512 messages/topic')

    def detail(self, key, event, **data):
        # Bounded detailed samples per category; counters continue for every packet.
        self.details[key] += 1
        if self.details[key] <= 100:
            return [dict(event=event, **data)]
        if self.details[key] == 101:
            return [dict(event='detail_limit', category=key,
                         note='Detailed events suppressed; counters and rosbag recording continue')]
        return []

    def process(self, received, kind, payload):
        self.counts[kind] += 1
        self.last[kind] = received
        if isinstance(payload, bytes):
            self.windows.setdefault(kind, deque(maxlen=512)).append((received, len(payload)))
        if kind == 'rosout_error':
            self.issues[kind] += 1
            return self.detail(kind, kind, context=self.context(received), **payload)
        if kind.startswith('teleop_command/'):
            self.command_times[kind] = received
            return self.detail(kind, 'manual_command_observed', topic=kind)
        if kind == 'flight_state':
            if len(payload) != 1:
                self.issues['malformed_flight_state'] += 1
                return self.detail(kind, 'payload_anomaly', topic=kind, sample_hex=payload[:128].hex())
            old, self.state = self.state, payload[0]
            if old != self.state:
                return self.detail(kind, 'flight_state', previous=old, value=self.state,
                                   name=FLIGHT_STATES.get(self.state, 'UNKNOWN_STATUS'),
                                   note='16/17 are status overrides; underlying navigation state is not encoded')
            return []
        if kind == 'motor_info':
            limits = read_motor_limits(payload)
            if limits is None:
                self.issues['invalid_motor_info'] += 1
                return self.detail(kind, 'payload_anomaly', topic=kind, sample_hex=payload[:128].hex())
            if limits != self.motor_limits:
                self.motor_limits = limits
                return self.detail(kind, 'motor_limits_observed', **limits)
            return []
        parsed = inspect_payload(kind, payload, pwm_ceiling=self.pwm_ceiling)
        if kind == 'imu' and 'gyro_norm' in parsed:
            self.motion.append((received, {k: parsed[k] for k in ('gyro_norm', 'acc_norm')}))
        if kind == 'motor_pwms' and 'values' in parsed:
            self.pwm = dict(values=parsed['values'], received_monotonic=received)
        events = []
        if 'stamp_ns' in parsed:
            previous = self.stamps.get(kind)
            if previous is not None and parsed['stamp_ns'] < previous:
                self.issues['backwards_' + kind] += 1
                events += self.detail('backwards_'+kind, 'backwards_stamp', topic=kind,
                                      delta_ns=parsed['stamp_ns'] - previous)
            self.stamps[kind] = parsed['stamp_ns']
        if parsed['issues']:
            self.issues.update(kind + ':' + issue for issue in parsed['issues'])
            # Expensive prefix matching is also bounded to the first 100 anomalies.
            matches = []
            if kind == 'servo/states' and self.details['anomaly_'+kind] < 100:
                for topic, history in self.histories.items():
                    for when, sample in history:
                        if abs(received - when) <= 1:
                            overlap = prefix_overlap(payload, sample)
                            if overlap:
                                matches.append(dict(topic=topic, arrival_delta_s=when-received, **overlap))
            events += self.detail('anomaly_'+kind, 'payload_anomaly', topic=kind, parsed=parsed,
                                  sample_hex=payload[:128].hex(),
                                  context=self.context(received),
                                  overlaps=sorted(matches, key=lambda x: -x['bytes'])[:8])
        if kind in self.histories and (not parsed['issues'] or
                parsed['issues'] == ['pwm_above_observation_threshold']):
            self.histories[kind].append((received, payload))
        return events

    def summary(self, now):
        return dict(counts=dict(self.counts), issues=dict(self.issues), state=self.state,
                    state_name=FLIGHT_STATES.get(self.state, 'UNKNOWN_STATUS'),
                    motor_limits=self.motor_limits,
                    seconds_since_last={k: max(0, now-v) for k, v in self.last.items()})
