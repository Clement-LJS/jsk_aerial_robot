#!/usr/bin/env python3
"""Keyboard navigation requests; actuator commands remain in the navigator."""
import math
import select
import sys
import termios
import threading
import time
import tty

import rospy
from std_msgs.msg import Bool, Empty, Float64, String

HELP = """
Pitch-only navigation (status angles in radians; default key step 0.2 degrees)
e: enable/reselect perching in HOVER; capture the measured pitch joint and contact
z: body pitch rezero around passive pivot, active servo frozen; r: relock after READY
j/l: active servo -/+ in SERVO_ADJUSTMENT; f: freeze settled servo
i/k: passive angle +/- in PASSIVE_CUTTING; space: return to post-rezero passive zero
v: unfreeze/rebase at current passive angle, then use j/l again
d: disable perching and return to normal navigation (same as multilink)
a: optional detached zero hold, only if the measured joint is already near zero
p: hold current settled pose for manual release; y: acknowledge release
x: recover a feedback/rezero fault after correcting it
s/h: status/help; b: show the latest state (compatibility key)
Ctrl+C: close keyboard; navigator continues holding its current target
No keyboard heartbeat, calibration flags, external safety or saw topics required.
This UI does not actuate the saw or mechanically attach/release the perch.
"""


class Keyboard:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = None
        self.received = 0.0
        ns = rospy.get_param('~robot_namespace', '/gimbalrotor').rstrip('/')
        self.prefix = rospy.get_param('~topic_prefix', ns + '/perching/pitch_only')
        self.pubs = {}
        for key in ('confirm_detached_zero', 'pitch_rezero', 'freeze',
                    'unfreeze', 'prepare_unperch', 'confirm_released', 'recover'):
            self.pubs[key] = rospy.Publisher(self.prefix + '/' + key, Empty, queue_size=1)
        self.pubs['servo_command'] = rospy.Publisher(self.prefix + '/servo_command', Float64, queue_size=1)
        self.pubs['alpha'] = rospy.Publisher(ns + '/perching/manual_pitch_delta', Float64, queue_size=1)
        self.pubs['enable'] = rospy.Publisher(ns + '/perching/enable', Bool, queue_size=1)
        self.pubs['relock'] = rospy.Publisher(ns + '/perching/relock', Empty, queue_size=1)
        self.sub = rospy.Subscriber(self.prefix + '/state', String, self.callback, queue_size=1)
        sign = float(rospy.get_param(ns + '/navigation/pitch_only_perching/pitch_command_sign', 1.0))
        self.pitch_command_sign = -1.0 if math.isfinite(sign) and sign < 0 else 1.0
        self.step = float(rospy.get_param('~step', math.radians(.2)))
        if not math.isfinite(self.step) or self.step <= 0:
            raise ValueError('step must be finite and positive [rad]')

    def callback(self, msg):
        fields = msg.data.split('|', 14)
        if len(fields) != 15:
            return
        try:
            state = {'stage': fields[0], 'generation': int(fields[1]), 'stamp': float(fields[2]),
                     'q': float(fields[3]), 'q_target': float(fields[4]), 'pitch': float(fields[5]),
                     'alpha': float(fields[6]), 'measured_alpha': float(fields[7]),
                     'min': float(fields[8]), 'max': float(fields[9]), 'valid': fields[10] == '1',
                     'pivot_error': float(fields[12]),
                     'armed': fields[13] == '1', 'status': fields[14]}
        except ValueError:
            return
        with self.lock:
            self.state, self.received = state, time.monotonic()

    def snapshot(self):
        with self.lock:
            if self.state is None or time.monotonic() - self.received > .5:
                return None
            s = dict(self.state)
        age = rospy.Time.now().to_sec() - s['stamp']
        if not math.isfinite(age) or not 0 <= age <= .5:
            return None
        return s

    def transition(self, action, message, expected, generation=False, timeout=3.0):
        before = self.snapshot()
        if before is None:
            print('Blocked: stale state')
            return
        stamp = rospy.Time.now().to_sec()
        if not self.pubs[action].get_num_connections():
            print('Blocked: navigator not connected')
            return
        self.pubs[action].publish(message)  # exactly once; never retry enable
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            s = self.snapshot()
            if s and s['stamp'] > stamp and s['stage'] in (expected if isinstance(expected, tuple) else (expected,)):
                if (not generation or s['generation'] > before['generation']) and (action != 'confirm_detached_zero' or s['armed']):
                    print('Navigator acknowledged:', s)
                    return
            if s and s['stamp'] > stamp:
                if not s['valid'] or s['stage'] == 'FAULT_HOLD' or (
                        s['stage'] == before['stage'] and s['generation'] == before['generation']
                        and s['status'] != before['status']):
                    print('Request not accepted:', s['status'], '(stage:', s['stage'] + '). You can retry.')
                    return
            time.sleep(.03)
        s = self.snapshot()
        if s:
            print('No acceptance observed. Latest navigator status:', s['status'],
                  '(stage:', s['stage'] + '). Request was not resent. You can retry.')
        else:
            print('No fresh navigator response. Request was not resent; check s before retrying.')

    def key(self, key):
        if key == 'h':
            print(HELP)
            return
        s = self.snapshot()
        if key == 's':
            print(s or 'STALE / no navigator snapshot')
            return
        # Disable is always available, including while joint feedback is invalid.
        if key == 'd':
            self.pubs['enable'].publish(Bool(False))
            print('Perching disable requested')
            return
        if s is None or not s['valid']:
            print('Blocked:', s['status'] if s else 'no fresh navigator state; check plugin/namespace/ROS master')
            return
        if key == 'b':
            print('Fresh navigator state:', s)
            return
        stage = s['stage']
        transitions = {
            'a': ('FREE_FLIGHT', 'confirm_detached_zero', 'FREE_FLIGHT', False),
            'z': ('CONTACT_LOCKED_PRE_REZERO', 'pitch_rezero', ('PITCH_REZERO_ACTIVE', 'PITCH_REZERO_READY_HOLD'), False),
            'r': ('PITCH_REZERO_READY_HOLD', 'relock', 'SERVO_ADJUSTMENT', True),
            'f': ('SERVO_ADJUSTMENT', 'freeze', 'PASSIVE_CUTTING', True),
            'v': ('PASSIVE_CUTTING', 'unfreeze', 'SERVO_ADJUSTMENT', True),
            'p': ('SERVO_ADJUSTMENT', 'prepare_unperch', 'UNPERCH_PREP', False),
            'y': ('UNPERCH_PREP', 'confirm_released', 'DISENGAGED', True),
            'x': ('FAULT_HOLD', 'recover', None, False),
        }
        if key == 'e':
            self.transition('enable', Bool(True), 'CONTACT_LOCKED_PRE_REZERO', True)
        elif key in transitions and stage == transitions[key][0]:
            _, action, expected, generation = transitions[key]
            if action == 'recover':
                # Recovery can lead to provisional contact or servo adjustment.
                expected = ('SERVO_ADJUSTMENT', 'CONTACT_LOCKED_PRE_REZERO')
            self.transition(action, Empty(), expected, generation)
        elif key in ('j', 'l') and stage == 'SERVO_ADJUSTMENT':
            self.pubs['servo_command'].publish(Float64(s['q_target'] / self.pitch_command_sign + (-self.step if key == 'j' else self.step)))
        elif key in ('i', 'k', ' ') and stage == 'PASSIVE_CUTTING':
            value = 0.0 if key == ' ' else s['alpha'] + (self.step if key == 'i' else -self.step)
            self.pubs['alpha'].publish(Float64(max(s['min'], min(s['max'], value))))
        else:
            print('Blocked in stage', stage + ':', s['status'])


def main():
    rospy.init_node('keyboard_pitch_only_perching')
    ui = Keyboard()
    print(HELP)
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], .1)[0]:
                key = sys.stdin.read(1)
                if key == '\x03' or not key:
                    break
                ui.key(key)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        # No zero, disable, torque-off, saw or detach command on exit.


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
