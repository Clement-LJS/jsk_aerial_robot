#!/usr/bin/env python3
"""Small absolute mechanism targets with local receipt-time freshness; only the navigator drives actuators."""
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
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String, UInt8, UInt64

HELP = """
ROLL / YAW PERCHING — small mechanism steps (default 0.2 degrees)
  j / l  active yaw joint - / +       a / f  active roll joint - / +
  i / k  passive hinge alpha + / - (lock-relative, not world Euler pitch)
  SPACE  stop advancement and hold the applied reference
  e / d  enable / disable             z / b  pitch rezero / relock
  r / t  arm / take off               g / h  land / HALT MOTORS
  s / ?  status / help               q / Ctrl-C  hold and quit

Setup: physically perch -> e -> z -> wait PITCH_REZERO_HOLD -> b -> navigate.
Rezero is explicit, freezes active joints and waits for measured pitch/rate.
Manual motion is also available immediately after capture when no setup is active.
Relock stays perched and establishes a new measured baseline. Failed setup needs
cause resolution and disable/retry; relock cannot bypass a fault.
Roll/yaw are URDF joint coordinates: positive joints can rotate the body negatively.
The body reference/measurement is published separately; no passive actuator exists.
Keys have the same meanings with qp_planning=false (default) and true.
Optional QP body requests use prepare/align/start topics, never ordinary keys.
Each absolute target uses current applied feedback. Repeated keys cannot queue
more than the navigator's bounded lookahead. Space never zeros the joints.
Quitting preserves contact hold and restores the terminal. No saw command is sent.
"""


class Keyboard:
    def __init__(self):
        self.lock = threading.RLock()
        self.state = ('', '')
        self.state_time = self.flight_time = self.angles_time = 0.0
        self.session = self.flight = self.angles = None
        self.angles_frame = ''
        self.body_rpy = None
        self.body_time = 0.0
        self.quit = False
        self.enable_pending = None
        self.last_display = None
        self.last_display_time = 0.0
        self.relock_pending = None
        self.had_feedback = False
        self.ns = rospy.get_param('~robot_namespace', '/gimbalrotor').rstrip('/')
        self.prefix = self.ns + '/perching/roll_yaw'
        maximum = float(rospy.get_param(self.ns + '/navigation/roll_yaw_perching/manual_max_step',
                                        math.radians(.5)))
        if not math.isfinite(maximum) or not 0 < maximum <= math.radians(1):
            raise ValueError('Navigator manual_max_step must be positive and at most 1 degree.')
        self.steps = {}
        for axis in ('yaw', 'roll', 'pitch'):
            step = float(rospy.get_param('~' + axis + '_step_deg', .2))
            if not math.isfinite(step) or not 0 < math.radians(step) <= maximum:
                raise ValueError('~{}_step_deg must be positive and <= {:.3f}.'.format(
                    axis, math.degrees(maximum)))
            self.steps[axis] = math.radians(step)
        self.pubs = {
            'manual': rospy.Publisher(self.prefix + '/target', Vector3Stamped, queue_size=1),
            'stop': rospy.Publisher(self.prefix + '/cancel', UInt64, queue_size=1),
            'rezero': rospy.Publisher(self.prefix + '/pitch_rezero', Empty, queue_size=1),
            'relock': rospy.Publisher(self.ns + '/perching/relock', Empty, queue_size=1),
            'enable': rospy.Publisher(self.ns + '/perching/enable', Bool, queue_size=1),
        }
        for action, topic in (('arm', 'start'), ('takeoff', 'takeoff'), ('land', 'land'), ('halt', 'halt')):
            self.pubs[action] = rospy.Publisher(self.ns + '/teleop_command/' + topic, Empty, queue_size=1)
        self.subs = [
            rospy.Subscriber(self.prefix + '/state', String, self.state_callback, queue_size=1),
            rospy.Subscriber(self.prefix + '/session', UInt64, self.session_callback, queue_size=1),
            rospy.Subscriber(self.prefix + '/target_angles', Vector3Stamped, self.angles_callback, queue_size=1),
            rospy.Subscriber(self.prefix + '/body_measured', Odometry, self.body_callback, queue_size=1),
            rospy.Subscriber(self.ns + '/flight_state', UInt8, self.flight_callback, queue_size=1),
        ]

    def state_callback(self, msg):
        stage, _, reason = msg.data.partition(': ')
        with self.lock:
            self.state, self.state_time = (stage, reason), time.monotonic()

    def session_callback(self, msg):
        with self.lock:
            if self.session != msg.data:
                self.angles = None  # wait for feedback acknowledging this base lock
            self.session = msg.data

    def flight_callback(self, msg):
        with self.lock:
            self.flight, self.flight_time = msg.data, time.monotonic()

    def angles_callback(self, msg):
        values = (msg.vector.x, msg.vector.y, msg.vector.z)  # active roll, passive alpha, active yaw
        with self.lock:
            self.angles = values if all(math.isfinite(x) for x in values) else None
            self.angles_frame, self.angles_time = msg.header.frame_id, time.monotonic()

    def body_callback(self, msg):
        q = msg.pose.pose.orientation
        values = (q.x, q.y, q.z, q.w)
        if not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values)-1) > .002:
            return
        rpy = (math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)),
               math.asin(max(-1., min(1., 2*(q.w*q.y-q.z*q.x)))),
               math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        with self.lock:
            self.body_rpy, self.body_time = rpy, time.monotonic()

    def snapshot(self):
        with self.lock:
            return dict(stage=self.state[0], reason=self.state[1], stamp=self.state_time,
                        flight=self.flight, flight_time=self.flight_time, session=self.session,
                        angles=self.angles, angles_frame=self.angles_frame, angles_time=self.angles_time,
                        body_rpy=self.body_rpy, body_time=self.body_time)

    @staticmethod
    def fresh(s):
        return time.monotonic() - s['stamp'] < 1 and time.monotonic() - s['flight_time'] < 1

    @staticmethod
    def has_angles(s):
        return (s['angles'] is not None and time.monotonic() - s['angles_time'] < 1
                and s['angles_frame'] == 'perching_lock_' + str(s['session']))

    def publish(self, action, message):
        if not self.pubs[action].get_num_connections():
            raise RuntimeError('No navigator connection for ' + action)
        self.pubs[action].publish(message)

    def movement_rejection(self, s):
        if s['session'] is None or s['stage'] == 'FREE_FLIGHT':
            return 'No confirmed contact session.'
        if self.relock_pending:
            return ('Waiting for relock acknowledgement and new target baseline.')
        if s['stage'] in ('FAULT_HOLD', 'PITCH_REZERO_FAILED'):
            return ('Contact/setup fault: ' + s['reason'])
        if s['stage'].startswith('PITCH_REZERO_'):
            return ('Setup owns target: wait ready-HOLD, then b to relock.')
        if not self.fresh(s) or not self.has_angles(s) or s['flight'] != 5:
            return ('Need fresh local receipts of HOVER and current-session targets.')
        if not self.pubs['manual'].get_num_connections():
            return 'No navigator connection for manual targets.'
        return ''

    def manual(self, axis=None, sign=0):
        s = self.snapshot()
        if s['session'] is None or s['stage'] == 'FREE_FLIGHT':
            if axis is None:
                return
            raise RuntimeError('No confirmed contact session.')
        if axis is None:
            self.publish('stop', UInt64(s['session']))
            return
        rejection = self.movement_rejection(s)
        if rejection:
            raise RuntimeError(rejection)
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()  # informational only, not a freshness clock on robot
        msg.header.frame_id = 'perching_lock_' + str(s['session'])
        msg.vector.x, msg.vector.y, msg.vector.z = s['angles']
        field = {'roll': 'x', 'pitch': 'y', 'yaw': 'z'}[axis]
        setattr(msg.vector, field, getattr(msg.vector, field) + sign * self.steps[axis])
        self.publish('manual', msg)

    def show_status(self):
        s = self.snapshot()
        print('\nFlight={} | {} | session={} | {}'.format(
            s['flight'], s['stage'] or 'waiting', s['session'], s['reason']), flush=True)
        if self.has_angles(s) and s['stage'] != 'FREE_FLIGHT':
            print('Applied mechanism reference [deg]: roll joint={:+.3f}, alpha={:+.3f}, yaw joint={:+.3f}'.format(
                *map(math.degrees, s['angles'])), flush=True)
        if s['body_rpy'] is not None and time.monotonic() - s['body_time'] < 1:
            print('Measured body WORLD RPY [deg]: {:+.3f}, {:+.3f}, {:+.3f}'.format(
                *map(math.degrees, s['body_rpy'])), flush=True)
        print('Input: ' + (self.movement_rejection(s) or 'available; navigator validates limits/tracking'), flush=True)
        print('Step [deg]: yaw={:.3f}, roll={:.3f}, pitch={:.3f}{}'.format(
            *(math.degrees(self.steps[a]) for a in ('yaw', 'roll', 'pitch')),
            '' if self.fresh(s) else ' | feedback stale'), flush=True)

    def tick(self):
        s = self.snapshot()
        fresh = self.fresh(s)
        if self.had_feedback and not fresh:
            self.manual()  # best effort; navigator independently faults on feedback loss
        self.had_feedback = fresh
        if self.enable_pending:
            old_session, started = self.enable_pending
            if fresh and s['stage'] == 'CONTACT_LOCKED' and self.has_angles(s) and s['session'] != old_session:
                print('\nCONTACT_LOCKED confirmed, session {}.'.format(s['session']), flush=True)
                self.enable_pending = None
            elif time.monotonic() - started > 3:
                print('\nEnable not confirmed: ' + s['reason'], flush=True)
                self.enable_pending = None
        if self.relock_pending:
            previous, started = self.relock_pending
            if s['session'] != previous and self.fresh(s) and self.has_angles(s):
                self.relock_pending = None
                print('Relock acknowledged; new applied mechanism baseline received.', flush=True)
            elif time.monotonic() - started > 3:
                self.relock_pending = None
                print('Relock not confirmed: ' + s['reason'], flush=True)
        display = (s['flight'], s['stage'], s['reason'], s['session'], fresh)
        if display != self.last_display and time.monotonic() - self.last_display_time > .25:
            self.last_display, self.last_display_time = display, time.monotonic()
            self.show_status()

    def key(self, key):
        if key in ('q', '\x03', ''):
            self.quit = True
            self.manual()
        elif key in ('h', 'g'):
            self.enable_pending = None
            self.publish('halt' if key == 'h' else 'land', Empty())
        elif key == 'd':
            self.enable_pending = None
            self.publish('enable', Bool(False))
            print('\nDisable requested; perching references invalidated.', flush=True)
        elif key == ' ':
            self.manual()
            print('\nStop/hold requested; outstanding input discarded.', flush=True)
        elif key == '?':
            print(HELP, flush=True)
        elif key == 's':
            self.show_status()
        elif key in 'jlafik':
            axis, sign = {'j': ('yaw', -1), 'l': ('yaw', 1), 'a': ('roll', -1),
                          'f': ('roll', 1), 'i': ('pitch', 1), 'k': ('pitch', -1)}[key]
            self.manual(axis, sign)
        elif key in ('z', 'b'):
            s = self.snapshot()
            if not self.fresh(s) or not self.has_angles(s) or s['flight'] != 5:
                raise RuntimeError('Need fresh confirmed contact feedback for setup.')
            if key == 'z':
                self.publish('rezero', Empty())
            else:
                self.publish('relock', Empty())
                self.relock_pending = (s['session'], time.monotonic())
        elif key in ('r', 't', 'e'):
            s = self.snapshot()
            if not self.fresh(s):
                raise RuntimeError('Waiting for fresh robot state.')
            if key == 'r' and s['flight'] in (0, 6) and s['stage'] == 'FREE_FLIGHT':
                self.publish('arm', Empty())
            elif key == 't' and s['flight'] == 2:
                self.publish('takeoff', Empty())
            elif key == 'e' and s['flight'] == 5 and s['stage'] == 'FREE_FLIGHT' and not self.enable_pending:
                self.publish('enable', Bool(True))
                self.enable_pending = (s['session'], time.monotonic())
                print('\nEnable requested; awaiting navigator confirmation.', flush=True)
            else:
                raise RuntimeError('Command unavailable in current state; press s for reason.')


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
                    keys = os.read(sys.stdin.fileno(), 4096).decode('ascii', errors='ignore')
                    # Coalesce terminal backlog; safety keys take priority over movement.
                    key = next((k for k in ('h', 'g', '\x03', 'q', 'd', ' ') if k in keys), keys[-1:] or '')
                    if key in ('h', 'g', '\x03', 'q', 'd', ' '):
                        termios.tcflush(sys.stdin, termios.TCIFLUSH)
                    ui.key(key)
            except (ValueError, RuntimeError, rospy.ROSException) as exc:
                rospy.logwarn_throttle(1.0, str(exc))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            ui.manual()
        except (RuntimeError, rospy.ROSException):
            pass


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
    except (ValueError, RuntimeError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
