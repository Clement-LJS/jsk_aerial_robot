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
        self.sequence = 0
        self.mode = None
        self.frame = 0
        self.stop_epoch = None
        self.stop_message = None
        self.ack = None
        self.status = None
        self.publisher = rospy.Publisher(prefix + '/motion', RollYawMotion, queue_size=10)
        self.subscribers = [
            rospy.Subscriber(prefix + '/motion_epoch', UInt64, self.epoch_callback, queue_size=1),
            rospy.Subscriber(prefix + '/command_ack', String, self.ack_callback, queue_size=10),
            rospy.Subscriber(prefix + '/diagnostics_status', DiagnosticArray, self.status_callback, queue_size=1),
        ]

    def epoch_callback(self, message):
        with self.lock:
            self.current_epoch = message.data

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
                    self.status = (time.monotonic(), {v.key: v.value for v in status.values})

    def rearm(self, session):
        """Only an explicit new keyboard action may acquire a replacement epoch."""
        with self.lock:
            if self.current_epoch is None or self.current_epoch == self.stop_epoch:
                raise RuntimeError('Waiting for navigator motion epoch after stop.')
            changed = self.session != session or self.epoch != self.current_epoch
            if changed:
                self.session, self.epoch = session, self.current_epoch
                self.mode = None
                self.ack = None
            return changed

    def send(self, session, mode, value=0., rate_limit=0.):
        with self.lock:
            if self.epoch is None:
                self.rearm(session)
            if session != self.session or self.epoch != self.current_epoch or self.epoch == self.stop_epoch:
                raise RuntimeError('Motion was cancelled or contact changed; task will not resume.')
            if not self.publisher.get_num_connections():
                raise RuntimeError('No navigator motion subscriber.')
            self.sequence += 1
            if mode == RollYawMotion.BODY_PITCH and self.mode != mode:
                self.frame = self.sequence
            message = RollYawMotion(session=session, epoch=self.epoch, source=self.source,
                                    sequence=self.sequence, mode=mode, value=value, frame=self.frame, rate_limit=rate_limit)
            self.mode = mode
            self.publisher.publish(message)
            if mode == RollYawMotion.STOP:
                self.stop_epoch = self.epoch
                self.stop_message = message
            return self.sequence

    def stop(self, wait=False, timeout=2.):
        with self.lock:
            if self.session is None or self.sequence == 0:
                return True
            # Another stop/session transition already revoked this task. Never
            # stop a new owner merely because this old process is shutting down.
            if self.current_epoch != self.epoch:
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
