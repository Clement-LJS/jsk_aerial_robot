#!/usr/bin/env python3
"""Identified motion requests; the navigator owns admission and completion."""
import re
import threading
import time
import uuid

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from gimbalrotor.msg import RollYawMotion


class MotionClient:
    def __init__(self, prefix):
        self.lock = threading.RLock()
        self.prefix = prefix
        self.source = uuid.uuid4().hex
        self.session = self.epoch = self.current_epoch = self.current_session = None
        self.sequence = 0
        self.frame = 0
        self.mode = None
        self.stop_epoch = None
        self.ack = self.status = self.request = None
        self.request_seen = False
        self.publisher = rospy.Publisher(prefix + '/motion', RollYawMotion, queue_size=1)
        self.subscribers = [
            rospy.Subscriber(prefix + '/command_ack', String, self.ack_callback, queue_size=10),
            rospy.Subscriber(prefix + '/diagnostics_status', DiagnosticArray, self.status_callback, queue_size=1),
        ]

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
                    self.current_epoch = int(values.get('motion_epoch', '0'))
                    self.current_session = int(values.get('session_id', '0'))

    def rearm(self, session):
        """Only a new operator action starts again after cancellation."""
        with self.lock:
            if self.current_epoch is None or self.current_epoch == self.stop_epoch:
                raise RuntimeError('Waiting for navigator feedback after stop.')
            if session != self.current_session:
                raise RuntimeError('Contact session changed.')
            self.session, self.epoch = session, self.current_epoch

    @staticmethod
    def matches_ack(ack, message):
        return ack and all(ack[1].get(key) == str(getattr(message, key))
                           for key in ('session', 'epoch', 'source', 'sequence', 'mode'))

    def send(self, session, mode, value=0., rate_limit=0., duration=0.):
        with self.lock:
            if (session != self.session or session != self.current_session or self.epoch is None
                    or self.epoch != self.current_epoch or self.epoch == self.stop_epoch):
                raise RuntimeError('Motion cancelled or contact changed; task will not resume.')
            if not self.publisher.get_num_connections():
                raise RuntimeError('No navigator connection for motion requests.')
            self.sequence += 1
            if mode == RollYawMotion.BODY_PITCH and self.mode != mode:
                self.frame = self.sequence
            self.request = RollYawMotion(session=session, epoch=self.epoch, source=self.source,
                                         sequence=self.sequence, mode=mode, value=value, frame=self.frame,
                                         rate_limit=rate_limit, duration=duration)
            self.mode = mode
            self.request_seen = False
            self.publisher.publish(self.request)
            return self.sequence

    def completed(self, feedback_timeout=1.):
        with self.lock:
            if not self.status or time.monotonic() - self.status[0] > feedback_timeout:
                raise RuntimeError('Navigator feedback lost.')
            values = self.status[1]
            if int(values['session_id']) != self.session or int(values['motion_epoch']) != self.epoch:
                raise RuntimeError('Motion cancelled or contact changed.')
            if self.request is None or not self.matches_ack(self.ack, self.request):
                return False
            if self.ack[1].get('accepted') != 'true':
                raise RuntimeError(self.ack[2])
            if self.status[0] < self.ack[0]:
                return False
            if values.get('motion_source') != self.source or int(values.get('motion_sequence', '0')) != self.sequence:
                # Diagnostics and acknowledgements use separate ROS connections.
                if not self.request_seen:
                    return False
                raise RuntimeError('Motion replaced by another request.')
            self.request_seen = True
            ready = 'jog_ready' if self.request.mode == RollYawMotion.JOG else 'admittance_hold_eligible'
            return values.get(ready) == '1'

    def stop(self, wait=False, timeout=2.):
        with self.lock:
            if self.request is None or self.session != self.current_session or self.epoch != self.current_epoch:
                return True
            if self.mode != RollYawMotion.STOP:
                self.send(self.session, RollYawMotion.STOP)
            request = self.request
            self.stop_epoch = self.epoch
        if not wait:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if self.matches_ack(self.ack, request):
                    return self.ack[1].get('accepted') == 'true'
                if self.current_epoch != request.epoch:
                    return True
            time.sleep(.02)
        return False

    def cleanup(self, timeout=2.):
        with self.lock:
            if self.request is None:
                return True
            if (self.status and time.monotonic() - self.status[0] < 1.
                    and self.matches_ack(self.ack, self.request) and self.ack[1].get('accepted') == 'true'
                    and self.status[0] >= self.ack[0]):
                values = self.status[1]
                if (values.get('motion_source') == self.source
                        and int(values.get('motion_sequence', '0')) == self.sequence
                        and values.get('admittance_hold_eligible') == '1'):
                    return True
        return self.stop(wait=True, timeout=timeout)
