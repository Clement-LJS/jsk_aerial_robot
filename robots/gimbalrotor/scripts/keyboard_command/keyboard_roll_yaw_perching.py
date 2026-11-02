#!/usr/bin/env python3
"""Discrete contact-relative BODY increments; only the navigator drives actuators."""
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
ROLL / YAW PERCHING — one small increment per tap (default 0.2 degrees)
  j / l  body yaw - / +       a / f  body roll - / +
  i / k  local-plane pitch + / - (right-hand local Y, not world up/down)
  SPACE  discard manual work, decelerate and hold current contact
  e / d  request enable / disable       s / ?  status / help
  r / t  arm / take off                 h  HALT MOTORS
  q / Ctrl-C  stop advancement, keep contact hold, restore terminal and quit

Body orientation: N * Rz(yaw) * Rx(roll) * Ry(local pitch), with N frozen at lock.
Steps: ~yaw_step_deg, ~roll_step_deg, ~pitch_step_deg (positive, default 0.2).
The navigator validates every increment; busy/lagging input is discarded.
Terminals provide characters, not reliable key-release events. Autorepeat may
request further steps; without new input at most one accepted increment finishes.
Space holds without zeroing, disabling perching, resuming, or starting a stroke.
Enable only captures contact; wait for navigator-confirmed CONTACT_LOCKED.
Establish physical contact before enable. No saw motor command is sent.
Automatic prepare/align/start topics remain separate from this keyboard.
"""


class Keyboard:
    def __init__(self):
        self.lock = threading.RLock()
        self.state = ('', '')
        self.state_time = self.flight_time = self.angles_time = 0.0
        self.session = self.flight = self.angles = None
        self.angles_frame = ''
        self.quit = False
        self.enable_pending = None
        self.last_display = None
        self.last_display_time = 0.0
        self.last_stamp = rospy.Time(0)
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
            'manual': rospy.Publisher(self.prefix + '/manual_delta', Vector3Stamped, queue_size=1),
            'enable': rospy.Publisher(self.ns + '/perching/enable', Bool, queue_size=1),
        }
        for action, topic in (('arm', 'start'), ('takeoff', 'takeoff'), ('halt', 'halt')):
            self.pubs[action] = rospy.Publisher(self.ns + '/teleop_command/' + topic, Empty, queue_size=1)
        self.subs = [
            rospy.Subscriber(self.prefix + '/state', String, self.state_callback, queue_size=1),
            rospy.Subscriber(self.prefix + '/session', UInt64, self.session_callback, queue_size=1),
            rospy.Subscriber(self.prefix + '/target_angles', Vector3Stamped, self.angles_callback, queue_size=1),
            rospy.Subscriber(self.ns + '/flight_state', UInt8, self.flight_callback, queue_size=1),
        ]

    def state_callback(self, msg):
        stage, _, reason = msg.data.partition(': ')
        with self.lock:
            self.state, self.state_time = (stage, reason), time.monotonic()

    def session_callback(self, msg):
        with self.lock:
            self.session = msg.data

    def flight_callback(self, msg):
        with self.lock:
            self.flight, self.flight_time = msg.data, time.monotonic()

    def angles_callback(self, msg):
        values = (msg.vector.x, msg.vector.y, msg.vector.z)  # roll, local pitch, yaw
        age = (rospy.Time.now() - msg.header.stamp).to_sec()
        with self.lock:
            self.angles = values if all(math.isfinite(x) for x in values) and 0 <= age < 1 else None
            self.angles_frame, self.angles_time = msg.header.frame_id, time.monotonic()

    def snapshot(self):
        with self.lock:
            return dict(stage=self.state[0], reason=self.state[1], stamp=self.state_time,
                        flight=self.flight, flight_time=self.flight_time, session=self.session,
                        angles=self.angles, angles_frame=self.angles_frame, angles_time=self.angles_time)

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

    def manual(self, axis=None, sign=0):
        s = self.snapshot()
        if s['session'] is None or s['stage'] == 'FREE_FLIGHT':
            if axis is None:
                return
            raise RuntimeError('No confirmed contact session.')
        if axis is not None:
            if s['stage'] == 'FAULT_HOLD':
                raise RuntimeError('Contact fault: {}. Resolve the cause, then d/e to recapture.'.format(s['reason']))
            if not self.fresh(s) or not self.has_angles(s) or s['flight'] != 5:
                raise RuntimeError('Need fresh HOVER and current-session target feedback.')
            if s['stage'] not in ('CONTACT_LOCKED', 'COMPLETED_HOLD'):
                return  # discard autorepeat while busy; never queue a local destination
        stamp = rospy.Time.now()
        if stamp <= self.last_stamp:
            return  # stopped/backwards ROS clock; never invent a future timestamp
        msg = Vector3Stamped()
        msg.header.stamp, msg.header.frame_id = stamp, 'perching_lock_' + str(s['session'])
        if axis is not None:
            setattr(msg.vector, {'roll': 'x', 'pitch': 'y', 'yaw': 'z'}[axis], sign * self.steps[axis])
        self.publish('manual', msg)
        self.last_stamp = stamp

    def show_status(self):
        s = self.snapshot()
        print('\nFlight={} | {} | session={} | {}'.format(
            s['flight'], s['stage'] or 'waiting', s['session'], s['reason']), flush=True)
        if self.has_angles(s) and s['stage'] != 'FREE_FLIGHT':
            print('Navigator held body target [deg]: roll={:+.3f}, local pitch={:+.3f}, yaw={:+.3f}'.format(
                *map(math.degrees, s['angles'])), flush=True)
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
        display = (s['flight'], s['stage'], s['reason'], s['session'], fresh)
        if display != self.last_display and time.monotonic() - self.last_display_time > .25:
            self.last_display, self.last_display_time = display, time.monotonic()
            self.show_status()

    def key(self, key):
        if key in ('q', '\x03', ''):
            self.quit = True
            self.manual()
        elif key == 'h':
            self.enable_pending = None
            self.publish('halt', Empty())
        elif key == 'd':
            self.enable_pending = None
            self.publish('enable', Bool(False))
            print('\nDisable requested (moving references decelerate first).', flush=True)
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
                    key = next((k for k in ('h', '\x03', 'q', 'd', ' ') if k in keys), keys[-1:] or '')
                    if key in ('h', '\x03', 'q', 'd', ' '):
                        termios.tcflush(sys.stdin, termios.TCIFLUSH)
                    ui.key(key)
            except (ValueError, RuntimeError, rospy.ROSException) as exc:
                print('\n' + str(exc), flush=True)
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
