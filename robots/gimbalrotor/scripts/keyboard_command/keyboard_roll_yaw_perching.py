#!/usr/bin/env python3
"""Interactive body-motion requests; the navigator owns all actuator commands."""
import collections
import math
import os
import select
import sys
import termios
import threading
import time
import tty

import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Empty, String, UInt8, UInt64

HELP = """
ROLL / YAW PERCHING — angles entered in DEGREES
  r  arm                  t  take off          e  enable perching
  y  set body YAW          o  set body ROLL
  u  raise local pitch by an amount            c  cut downward by an amount
  v  set pitch speed (degrees/second)
  SPACE  pause / resume   x  cancel and hold   s  show status
  d  disable perching     l  land              h  HALT MOTORS
  ?  help                 q / Ctrl+C  quit (cancel this keyboard's move)

Example: r, t, wait for HOVER, e, y 30 Enter, wait, o 30 Enter,
         wait, u 10 Enter, wait, c 20 Enter.
Yaw/roll are relative to the contact lock. Up decreases local pitch;
down increases it. All moves coordinate body, CoG and mechanism joints.
Wait for "Done" before the next move. No saw motor command is sent.
On hardware, establish physical contact before pressing e.
"""


class Keyboard:
    def __init__(self):
        self.lock = threading.RLock()
        self.state = ('', '')
        self.state_time = self.flight_time = self.angles_time = 0.0
        self.session = None
        self.flight = None
        self.angles = None
        self.angles_frame = ''
        self.counter = 0
        self.events = collections.deque(maxlen=1000)
        self.pending = None
        self.entry = None
        self.quit = False
        self.last_display = None
        self.ns = rospy.get_param('~robot_namespace', '/gimbalrotor').rstrip('/')
        self.prefix = self.ns + '/perching/roll_yaw'
        self.request_path = self.ns + '/navigation/roll_yaw_perching/request'
        self.rate = .03
        self.acceleration = .02
        self.pubs = {a: rospy.Publisher(self.prefix + '/' + a, UInt64, queue_size=1)
                     for a in ('prepare', 'align', 'start', 'pause', 'resume', 'cancel')}
        self.pubs['enable'] = rospy.Publisher(self.ns + '/perching/enable', Bool, queue_size=1)
        for name in ('arm', 'takeoff', 'land', 'halt'):
            topic = 'start' if name == 'arm' else name
            self.pubs[name] = rospy.Publisher(self.ns + '/teleop_command/' + topic, Empty, queue_size=1)
        self.subs = [
            rospy.Subscriber(self.prefix + '/state', String, self.state_callback, queue_size=100),
            rospy.Subscriber(self.prefix + '/session', UInt64, self.session_callback, queue_size=10),
            rospy.Subscriber(self.prefix + '/target_angles', Vector3Stamped, self.angles_callback, queue_size=10),
            rospy.Subscriber(self.ns + '/flight_state', UInt8, self.flight_callback, queue_size=10),
        ]

    def state_callback(self, msg):
        stage, _, reason = msg.data.partition(': ')
        with self.lock:
            self.state = (stage, reason)
            self.state_time = time.monotonic()
            self.counter += 1
            self.events.append((self.counter, stage, reason))

    def session_callback(self, msg):
        with self.lock:
            self.session = msg.data

    def flight_callback(self, msg):
        with self.lock:
            self.flight, self.flight_time = msg.data, time.monotonic()

    def angles_callback(self, msg):
        values = (msg.vector.z, msg.vector.x, msg.vector.y)  # yaw, roll, local pitch
        with self.lock:
            self.angles = values if all(math.isfinite(x) for x in values) else None
            self.angles_frame = msg.header.frame_id
            self.angles_time = time.monotonic()

    def snapshot(self):
        with self.lock:
            return dict(stage=self.state[0], reason=self.state[1], stamp=self.state_time,
                        flight=self.flight, flight_time=self.flight_time, session=self.session,
                        angles=self.angles, angles_frame=self.angles_frame,
                        angles_time=self.angles_time, counter=self.counter)

    @staticmethod
    def fresh(s):
        return time.monotonic() - s['stamp'] < 1.0 and time.monotonic() - s['flight_time'] < 1.0

    @staticmethod
    def has_angles(s):
        return (s['angles'] is not None and time.monotonic() - s['angles_time'] < 1.0
                and s['angles_frame'] == 'perching_lock_' + str(s['session']))

    def publish(self, action, value=None):
        pub = self.pubs[action]
        if not pub.get_num_connections():
            raise RuntimeError('Robot is not connected. Start the roll-yaw bringup first.')
        if action == 'enable':
            message = Bool(value)
        elif action in ('arm', 'takeoff', 'land', 'halt'):
            message = Empty()
        else:
            if value is None:
                raise RuntimeError('No current contact lock.')
            message = UInt64(value)
        pub.publish(message)

    def show_status(self):
        s = self.snapshot()
        flight = {0: 'disarmed', 1: 'arming', 2: 'armed', 3: 'taking off',
                  4: 'landing', 5: 'HOVER', 6: 'stopped'}.get(s['flight'], 'not connected')
        motion = {'FREE_FLIGHT': 'Perching off', 'CONTACT_LOCKED': 'Contact locked',
                  'REZERO': 'Planning', 'PRE_ALIGN': 'Aligning body', 'READY': 'Aligned and holding',
                  'EXECUTING': 'Moving', 'PAUSED': 'Paused', 'COMPLETED_HOLD': 'Holding contact',
                  'FAULT_HOLD': 'Fault: ' + s['reason']}.get(s['stage'], 'Waiting for robot')
        print('\nFlight: {} | {}'.format(flight, motion), flush=True)
        if self.has_angles(s) and s['stage'] != 'FREE_FLIGHT':
            print('Body target: yaw {:+.1f}°, roll {:+.1f}°, local pitch {:+.1f}°'.format(
                *map(math.degrees, s['angles'])), flush=True)
        print('Pitch speed: {:.2f}°/s{}'.format(math.degrees(self.rate),
              ' | waiting for feedback' if not self.fresh(s) else ''), flush=True)

    def cancel(self):
        s = self.snapshot()
        self.pending = None  # Never continue an automatic sequence after cancellation.
        self.entry = None
        if s['stage'] in ('REZERO', 'PRE_ALIGN', 'READY', 'EXECUTING', 'PAUSED', 'CONTACT_LOCKED'):
            self.publish('cancel', s['session'])
            print('\nCancelling; the robot will hold contact.', flush=True)
        else:
            print('\nNo active move to cancel.', flush=True)

    def begin_move(self, key, degrees):
        s = self.snapshot()
        if self.pending or not self.fresh(s) or not self.has_angles(s) or s['flight'] != 5:
            raise RuntimeError('Wait for connected, settled HOVER and the previous move to finish.')
        if s['stage'] not in ('CONTACT_LOCKED', 'COMPLETED_HOLD', 'READY'):
            raise RuntimeError('Enable perching first, or cancel the current move with x.')
        yaw, roll, pitch = s['angles']
        end = pitch
        if key == 'y':
            if abs(degrees) > 180:
                raise ValueError('Yaw must be between -180 and 180 degrees.')
            yaw = math.radians(degrees)
        elif key == 'o':
            if abs(degrees) >= 90:
                raise ValueError('Roll must be between -90 and 90 degrees, excluding endpoints.')
            roll = math.radians(degrees)
        else:
            if degrees <= 0:
                raise ValueError('Enter a positive amount of pitch movement.')
            end += math.radians(degrees) * (-1 if key == 'u' else 1)
            if abs(end) >= math.pi:
                raise ValueError('The resulting local pitch must be between -180 and 180 degrees.')
        request = dict(body_yaw=yaw, body_roll=roll, pitch_start=pitch, pitch_end=end,
                       pitch_rate=self.rate, pitch_acceleration=self.acceleration,
                       align_only=key in ('y', 'o'))
        label = {'y': 'yaw', 'o': 'roll', 'u': 'pitch up', 'c': 'downward pitch'}[key]
        self.pending = dict(session=s['session'], phase='cancel_old' if s['stage'] == 'READY' else 'prepare',
                            request=request, label=label, started=time.monotonic(),
                            phase_started=time.monotonic(), after=s['counter'], seen=False)
        print('\nRequested {}. Planning the coordinated motion...'.format(label), flush=True)
        try:
            if s['stage'] == 'READY':
                self.publish('cancel', s['session'])
            else:
                self.prepare()
        except Exception:
            self.pending = None
            raise

    def prepare(self):
        p = self.pending
        rospy.set_param(self.request_path, p['request'])
        before = self.snapshot()
        p.update(phase='prepare', after=before['counter'], seen=False,
                 before_reason=before['reason'], phase_started=time.monotonic())
        self.publish('prepare', p['session'])

    def tick(self):
        s = self.snapshot()
        display = (s['flight'], s['stage'])
        if display != self.last_display:
            self.last_display = display
            if self.entry is None:
                self.show_status()
        p = self.pending
        with self.lock:
            events = list(self.events)
            self.events.clear()
        if p is None:
            return
        if not self.fresh(s):
            self.abort('Feedback stopped. Automatic movement sequence cancelled.')
            return
        if p['phase'] == 'enable':
            if s['stage'] == 'CONTACT_LOCKED' and self.has_angles(s) and s['session'] != p['session']:
                self.pending = None
                print('\nPerching enabled. Use y for yaw, then o for roll.', flush=True)
            elif time.monotonic() - p['started'] > 5:
                self.pending = None
                print('\nPerching was not enabled: ' + s['reason'], flush=True)
            return
        if s['session'] != p['session'] or s['flight'] != 5 or s['stage'] in ('FREE_FLIGHT', 'FAULT_HOLD'):
            self.abort('Motion stopped: ' + s['reason'])
            return
        if time.monotonic() - p['started'] > 600:
            self.abort('Motion timed out. Check robot status before trying again.')
            return
        if not p['seen'] and time.monotonic() - p['phase_started'] > 5:
            self.abort('Robot did not accept the next action: ' + s['reason'])
            return
        for count, stage, reason in events:
            if self.pending is not p or count <= p['after']:
                continue
            phase = p['phase']
            if phase == 'cancel_old' and stage == 'COMPLETED_HOLD':
                self.prepare()
            elif phase == 'prepare':
                if stage == 'REZERO':
                    p['seen'] = True
                elif stage == 'CONTACT_LOCKED' and (p['seen'] or
                        (reason.startswith('invalid/incomplete request') and reason != p['before_reason'])):
                    if 'plan ready' not in reason:
                        self.pending = None
                        print('\nMove rejected: ' + reason, flush=True)
                        break
                    p.update(phase='align', after=self.snapshot()['counter'], seen=False,
                             phase_started=time.monotonic())
                    self.publish('align', p['session'])
                    print('Moving into alignment...', flush=True)
            elif phase == 'align':
                if stage == 'PRE_ALIGN':
                    p['seen'] = True
                elif stage == 'READY' and p['seen']:
                    p.update(phase='finish' if p['request']['align_only'] else 'stroke',
                             after=self.snapshot()['counter'], seen=False, phase_started=time.monotonic())
                    self.publish('cancel' if p['request']['align_only'] else 'start', p['session'])
                    if not p['request']['align_only']:
                        print('Executing coordinated {}...'.format(p['label']), flush=True)
            elif phase == 'stroke' and stage == 'EXECUTING':
                p['seen'] = True
            elif stage == 'COMPLETED_HOLD' and (phase == 'finish' or (phase == 'stroke' and p['seen'])):
                self.pending = None
                print('\nDone: {}. Holding contact; ready for your next key.'.format(p['label']), flush=True)
                self.show_status()
                break

    def abort(self, message):
        p, self.pending = self.pending, None
        if p and p['phase'] != 'enable':
            try:
                self.publish('cancel', p['session'])
            except RuntimeError:
                pass
        print('\n' + message, flush=True)

    def key(self, key):
        if key in ('q', '\x03', ''):
            self.quit = True
            if self.pending:
                self.cancel()
            return
        if key == 'h':
            self.pending = self.entry = None
            self.publish('halt')
            print('\nHALT requested.', flush=True)
            return
        if key == 'x':
            self.cancel()
            return
        if key == ' ':
            s = self.snapshot()
            if not self.fresh(s):
                raise RuntimeError('No fresh robot state.')
            if s['stage'] in ('PRE_ALIGN', 'EXECUTING'):
                self.publish('pause', s['session'])
                print('\nPausing; wait for PAUSED.', flush=True)
            elif s['stage'] == 'PAUSED':
                self.publish('resume', s['session'])
                print('\nResuming.', flush=True)
            else:
                print('\nNothing is moving.', flush=True)
            return
        if self.entry is not None:
            command, value = self.entry
            if key == '\x1b':
                self.entry = None
                print('\nEntry cancelled.', flush=True)
            elif key in ('\r', '\n'):
                self.entry = None
                print('', flush=True)
                try:
                    degrees = float(value)
                except ValueError:
                    raise ValueError('Enter a number, e.g. 30. Press the motion key to try again.')
                if not math.isfinite(degrees):
                    raise ValueError('Enter a finite number of degrees.')
                if command == 'v':
                    if degrees <= 0:
                        raise ValueError('Speed must be positive.')
                    self.rate = math.radians(degrees)
                    print('Pitch speed set to {:.2f}°/s for the next move.'.format(degrees), flush=True)
                else:
                    self.begin_move(command, degrees)
            elif key in ('\x7f', '\b'):
                self.entry = (command, value[:-1])
                if value:
                    print('\b \b', end='', flush=True)
            elif key in '+-0123456789.':
                self.entry = (command, value + key)
                print(key, end='', flush=True)
            return
        if key == '?':
            print(HELP, flush=True)
        elif key == 's':
            self.show_status()
        elif key in ('y', 'o', 'u', 'c', 'v'):
            if self.pending:
                print('A move is in progress. Wait for Done, SPACE to pause, or x to cancel.', flush=True)
                return
            prompts = {'y': 'Body yaw target (degrees)', 'o': 'Body roll target (degrees)',
                       'u': 'Raise local pitch by (positive degrees)',
                       'c': 'Cut downward by (positive degrees)', 'v': 'Pitch speed (degrees/second)'}
            self.entry = (key, '')
            print('\n' + prompts[key] + ': ', end='', flush=True)
        else:
            s = self.snapshot()
            if not self.fresh(s):
                raise RuntimeError('Waiting for the robot. Start roll_yaw_perching.launch first.')
            if key == 'r' and s['flight'] in (0, 6) and s['stage'] == 'FREE_FLIGHT':
                self.publish('arm')
                print('Arming requested. Wait for Flight: armed before pressing t.', flush=True)
            elif key == 't' and s['flight'] == 2:
                self.publish('takeoff')
                print('Takeoff requested. Wait for HOVER before pressing e.', flush=True)
            elif key == 'e' and not self.pending and s['flight'] == 5 and s['stage'] == 'FREE_FLIGHT':
                self.publish('enable', True)
                self.pending = dict(phase='enable', session=s['session'], started=time.monotonic())
                print('Capturing settled contact...', flush=True)
            elif key == 'd' and not self.pending and s['stage'] in ('CONTACT_LOCKED', 'COMPLETED_HOLD', 'READY', 'FAULT_HOLD'):
                self.publish('enable', False)
                print('Perching disable requested. The Gazebo fixture releases contact.', flush=True)
            elif key == 'l' and s['stage'] == 'FREE_FLIGHT' and s['flight'] in (3, 5):
                self.publish('land')
            elif key.strip():
                print('Key unavailable in the current state. Press s for status or ? for help.', flush=True)


def main():
    if not sys.stdin.isatty():
        raise RuntimeError('Run this keyboard in a terminal.')
    rospy.init_node('keyboard_roll_yaw_perching', disable_signals=True)
    ui = Keyboard()
    print(HELP, flush=True)
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown() and not ui.quit:
            try:
                ui.tick()
                if select.select([sys.stdin], [], [], .05)[0]:
                    ui.key(os.read(sys.stdin.fileno(), 1).decode('ascii', errors='ignore'))
            except (ValueError, RuntimeError, rospy.ROSException) as exc:
                if ui.pending:
                    ui.abort(str(exc))
                else:
                    print('\n' + str(exc), flush=True)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if ui.pending:
            try:
                ui.cancel()
                time.sleep(.1)
            except (RuntimeError, rospy.ROSException):
                pass


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
