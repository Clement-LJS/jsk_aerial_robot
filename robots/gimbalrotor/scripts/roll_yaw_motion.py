#!/usr/bin/env python3
"""Session-bound replaceable motion requests shared by keyboard and experiment."""
import math
import re
import threading
import time
import uuid

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String, UInt64
from gimbalrotor.msg import RollYawMotion


class MotionClient:
    def __init__(self, prefix):
        self.lock = threading.RLock()
        self.prefix = prefix
        self.source = uuid.uuid4().hex
        self.session = self.epoch = self.current_epoch = None
        self.current_session = None
        self.sequence = 0
        self.mode = None
        self.claimed = False
        self.frame = 0
        self.stop_epoch = None
        self.stop_message = None
        self.ack = None
        self.status = None
        self.last_motion_time = 0.
        self.cleanup_started = False
        self.cleanup_confirmed = False
        self.publisher = rospy.Publisher(prefix + '/motion', RollYawMotion, queue_size=10)
        self.subscribers = [
            rospy.Subscriber(prefix + '/motion_epoch', UInt64, self.epoch_callback, queue_size=1),
            rospy.Subscriber(prefix + '/session', UInt64, self.session_callback, queue_size=1),
            rospy.Subscriber(prefix + '/command_ack', String, self.ack_callback, queue_size=10),
            rospy.Subscriber(prefix + '/diagnostics_status', DiagnosticArray, self.status_callback, queue_size=1),
        ]

    def epoch_callback(self, message):
        with self.lock:
            if self.current_epoch is None or message.data > self.current_epoch:
                self.current_epoch = message.data

    def session_callback(self, message):
        with self.lock:
            self.current_session = message.data

    def ack_callback(self, message):
        if not message.data.startswith('action=motion '):
            return
        fields = dict(re.findall(r'\b(\w+)=(\S+)', message.data))
        with self.lock:
            if fields.get('source') == self.source:
                self.ack = (time.monotonic(), fields, message.data)

    def status_callback(self, message):
        for status in message.status:
            if status.name.endswith('/roll_yaw_perching'):
                with self.lock:
                    values = {v.key: v.value for v in status.values}
                    self.status = (time.monotonic(), values)
                    epoch = int(values.get('motion_epoch', '0'))
                    if self.current_epoch is None or epoch > self.current_epoch:
                        self.current_epoch = epoch

    def rearm(self, session):
        """Only an explicit new keyboard action may acquire a replacement epoch."""
        with self.lock:
            if self.current_epoch is None or self.current_epoch == self.stop_epoch:
                raise RuntimeError('Waiting for navigator motion epoch after stop.')
            changed = self.session != session or self.epoch != self.current_epoch
            if changed:
                self.session, self.epoch = session, self.current_epoch
                self.mode = None
                self.claimed = False
                self.ack = None
            return changed

    @staticmethod
    def matches_ack(ack, message):
        return ack and all(ack[1].get(key) == str(getattr(message, key))
                           for key in ('session', 'epoch', 'source', 'sequence', 'mode'))

    def claim(self, session, timeout=2., interrupted=None):
        """Explicit idle handover; never reacquire after a task was cancelled."""
        deadline = time.monotonic() + timeout
        with self.lock:
            if interrupted is not None and interrupted.is_set():
                return False
            if rospy.is_shutdown() or time.monotonic() >= deadline:
                raise RuntimeError('Motion acquisition ended before dispatch; no motion sent.')
            if self.claimed:
                if self.session != session or self.epoch != self.current_epoch:
                    raise RuntimeError('Motion was cancelled; task will not resume.')
                return True
            self.send(session, RollYawMotion.CLAIM)
            message = self.claim_message
        last_send = time.monotonic()
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if interrupted is not None and interrupted.is_set():
                return False
            with self.lock:
                if (self.session != session or self.epoch != message.epoch or self.stop_epoch == message.epoch or
                        self.current_session not in (None, session) or
                        self.current_epoch not in (message.epoch, message.epoch + 1)):
                    raise RuntimeError('Motion cancelled during ownership handover.')
                if self.status:
                    values = self.status[1]
                    if (int(values.get('motion_epoch', '0')) == message.epoch + 1 and
                            values.get('motion_source') != self.source):
                        raise RuntimeError('Motion cancelled during ownership handover.')
                if self.matches_ack(self.ack, message):
                    if self.ack[1].get('accepted') != 'true':
                        raise RuntimeError(self.ack[2])
                    epoch = int(self.ack[1]['motion_epoch'])
                    if epoch != message.epoch + 1:
                        raise RuntimeError('Invalid ownership grant epoch; no motion sent.')
                    if self.current_epoch == epoch:
                        self.epoch, self.claimed = epoch, True
                        return True
                if time.monotonic() - last_send >= .2:
                    # Connection counts include recorders. Only the matching
                    # accepted grant confirms navigator readiness/ownership.
                    self.publisher.publish(message)
                    last_send = time.monotonic()
            if interrupted is None:
                time.sleep(.02)
            else:
                interrupted.wait(.02)
        raise RuntimeError('Motion ownership acknowledgement missing; no motion sent.')

    def cleanup(self, timeout=2.):
        """Keyboard exit: release idle ownership or brake only our own work."""
        with self.lock:
            if self.cleanup_started:
                return self.cleanup_confirmed
            self.cleanup_started = True
            if not self.claimed or self.current_epoch != self.epoch or self.current_session not in (None, self.session):
                self.cleanup_confirmed = True
                return True
            idle = self.mode == RollYawMotion.CLAIM
            if self.status and self.status[0] >= self.last_motion_time and time.monotonic() - self.status[0] < 1.:
                values = self.status[1]
                idle = (values.get('motion_source') == self.source and
                        int(values.get('motion_epoch', '0')) == self.epoch and
                        int(values.get('motion_sequence', '0')) == self.sequence and
                        values.get('admittance_hold_eligible') == '1')
        deadline = time.monotonic() + timeout
        confirmed = self.release(timeout) if idle else self.stop(wait=True, timeout=timeout)
        if confirmed and not idle:
            # STOP normally retires ownership. Never rearm to release it.
            confirmed = self.release(max(0., deadline - time.monotonic()))
        with self.lock:
            self.cleanup_confirmed = confirmed
        return confirmed

    def release(self, timeout=2.):
        """Keep the accepted finite reference, retire this source and its late packets."""
        with self.lock:
            if not self.claimed or self.current_epoch != self.epoch or self.current_session not in (None, self.session):
                return True
            sequence = self.send(self.session, RollYawMotion.RELEASE)
            self.claimed = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if self.ack and int(self.ack[1].get('sequence', '0')) == sequence:
                    return self.ack[1].get('accepted') == 'true'
                if self.current_epoch != self.epoch:
                    return True  # Release or operator cancellation retired this source.
            time.sleep(.02)
        return False  # Idle CLAIM remains possible even if this process disappears.

    def send(self, session, mode, value=0., rate_limit=0., duration=0.):
        with self.lock:
            if self.epoch is None:
                self.rearm(session)
            if session != self.session or self.epoch != self.current_epoch or self.epoch == self.stop_epoch:
                raise RuntimeError('Motion was cancelled or contact changed; task will not resume.')
            if self.current_session not in (None, session):
                raise RuntimeError('Contact session changed; task will not resume.')
            if mode != RollYawMotion.CLAIM and not self.publisher.get_num_connections():
                raise RuntimeError('No navigator motion subscriber.')
            self.sequence += 1
            if mode == RollYawMotion.BODY_PITCH and self.mode != mode:
                self.frame = self.sequence
            message = RollYawMotion(session=session, epoch=self.epoch, source=self.source,
                                    sequence=self.sequence, mode=mode, value=value, frame=self.frame,
                                    rate_limit=rate_limit, duration=duration)
            self.mode = mode
            self.last_motion_time = time.monotonic()
            if mode == RollYawMotion.CLAIM:
                self.claim_message = message
            self.publisher.publish(message)
            if mode == RollYawMotion.STOP:
                self.stop_epoch = self.epoch
                self.stop_message = message
            return self.sequence

    def stop(self, wait=False, timeout=2.):
        with self.lock:
            if not self.claimed:
                return True
            # Another stop/session transition already revoked this task. Never
            # stop a new owner merely because this old process is shutting down.
            if self.current_epoch != self.epoch or self.current_session not in (None, self.session):
                return True
            if self.ack and self.ack[1].get('accepted') == 'false' and 'another source' in self.ack[2]:
                return True  # This client never acquired the occupied motion owner.
            if self.stop_epoch != self.epoch:
                self.send(self.session, RollYawMotion.STOP)
            sequence, epoch = self.sequence, self.epoch
        if not wait:
            return True
        deadline = time.monotonic() + timeout
        last_send = time.monotonic()
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                acknowledged = (self.ack and int(self.ack[1].get('sequence', '0')) == sequence
                                and self.ack[1].get('accepted') == 'true')
                stopped = (self.status and int(self.status[1].get('motion_epoch', '0')) > epoch
                           and self.status[1].get('stopping') == '0'
                           and self.status[1].get('body_pitch_velocity_active') == '0')
                if stopped and (acknowledged or self.current_epoch != epoch):
                    return True
                if self.current_epoch == epoch and time.monotonic() - last_send >= .2:
                    self.publisher.publish(self.stop_message)
                    last_send = time.monotonic()
            time.sleep(.02)
        return False

    def cancel(self, session, timeout=2.):
        """Explicit operator stop, including a zero-speed command from a new process."""
        publisher = rospy.Publisher(self.prefix + '/cancel', UInt64, queue_size=1)
        deadline, last_send = time.monotonic() + timeout, -math.inf
        with self.lock:
            epoch = self.current_epoch
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if (self.status and int(self.status[1].get('motion_epoch', '0')) > epoch
                        and self.status[1].get('stopping') == '0'
                        and self.status[1].get('body_pitch_velocity_active') == '0'):
                    return True
            if self.current_epoch == epoch and publisher.get_num_connections() and time.monotonic() - last_send >= .2:
                publisher.publish(UInt64(data=session))
                last_send = time.monotonic()
            time.sleep(.02)
        return False
