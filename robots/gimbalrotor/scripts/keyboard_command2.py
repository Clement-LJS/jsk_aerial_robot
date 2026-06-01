#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph




msg = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)


Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""

STATE_DISARMED = 0
STATE_ARMED = 1
STATE_FLYING = 2

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

def printMsg(msg, msg_len = 50):
        print(msg.ljust(msg_len) + "\r", end="")

if __name__=="__main__":
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node("keyboard_command")
        robot_ns = rospy.get_param("~robot_ns", "");
        print(msg)

        if not robot_ns:
                master = rosgraph.Master('/rostopic')
                try:
                        _, subs, _ = master.getSystemState()

                except socket.error:
                        raise ROSTopicIOException("Unable to communicate with master!")

                teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
                if len(teleop_topics) == 1:
                        robot_ns = teleop_topics[0].split('/teleop')[0]

        ns = robot_ns + "/teleop_command"
        land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
        halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
        start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
        takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
        force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
        nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=1)

        xy_vel   = rospy.get_param("xy_vel", 0.2)
        z_vel    = rospy.get_param("z_vel", 0.2)
        yaw_vel  = rospy.get_param("yaw_vel", 0.2)

        motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)

        flight_state = STATE_DISARMED

        try:
                while(True):
                        nav_msg = FlightNav()
                        nav_msg.control_frame = FlightNav.WORLD_FRAME
                        nav_msg.target = FlightNav.COG

                        key = getKey()

                        msg = ""

                        if key == 'l':
                                if flight_state == STATE_FLYING:
                                        land_pub.publish(Empty())
                                        flight_state = STATE_DISARMED
                                        msg = "send land command"
                                else:
                                        msg = "blocked land command: robot is not flying"

                        if key == 'r':
                                if flight_state == STATE_DISARMED:
                                        start_pub.publish(Empty())
                                        flight_state = STATE_ARMED
                                        msg = "send motor-arming command"
                                else:
                                        msg = "blocked motor-arming command: already armed or flying"

                        if key == 'h':
                                halt_pub.publish(Empty())
                                flight_state = STATE_DISARMED
                                msg = "send motor-disarming (halt) command"

                        if key == 'f':
                                force_landing_pub.publish(Empty())
                                flight_state = STATE_DISARMED
                                msg = "send force landing command"

                        if key == 't':
                                if flight_state == STATE_ARMED:
                                        takeoff_pub.publish(Empty())
                                        flight_state = STATE_FLYING
                                        msg = "send takeoff command"
                                elif flight_state == STATE_FLYING:
                                        msg = "blocked takeoff command: robot is already flying"
                                else:
                                        msg = "blocked takeoff command: please arm motor first"

                        if key == 'x':
                                if flight_state == STATE_FLYING:
                                        motion_start_pub.publish()
                                        msg = "send task-start command"
                                else:
                                        msg = "blocked task-start command: robot is not flying"

                        if key == 'w':
                                if flight_state == STATE_FLYING:
                                        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                        nav_msg.target_vel_x = xy_vel
                                        nav_pub.publish(nav_msg)
                                        msg = "send +x vel command"
                                else:
                                        msg = "blocked +x vel command: robot is not flying"

                        if key == 's':
                                if flight_state == STATE_FLYING:
                                        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                        nav_msg.target_vel_x = -xy_vel
                                        nav_pub.publish(nav_msg)
                                        msg = "send -x vel command"
                                else:
                                        msg = "blocked -x vel command: robot is not flying"

                        if key == 'a':
                                if flight_state == STATE_FLYING:
                                        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                        nav_msg.target_vel_y = xy_vel
                                        nav_pub.publish(nav_msg)
                                        msg = "send +y vel command"
                                else:
                                        msg = "blocked +y vel command: robot is not flying"

                        if key == 'd':
                                if flight_state == STATE_FLYING:
                                        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                                        nav_msg.target_vel_y = -xy_vel
                                        nav_pub.publish(nav_msg)
                                        msg = "send -y vel command"
                                else:
                                        msg = "blocked -y vel command: robot is not flying"

                        if key == 'q':
                                if flight_state == STATE_FLYING:
                                        nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                        nav_msg.target_omega_z = yaw_vel
                                        nav_pub.publish(nav_msg)
                                        msg = "send +yaw vel command"
                                else:
                                        msg = "blocked +yaw vel command: robot is not flying"

                        if key == 'e':
                                if flight_state == STATE_FLYING:
                                        nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                                        nav_msg.target_omega_z = -yaw_vel
                                        msg = "send -yaw vel command"
                                        nav_pub.publish(nav_msg)
                                else:
                                        msg = "blocked -yaw vel command: robot is not flying"

                        if key == '[':
                                if flight_state == STATE_FLYING:
                                        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                        nav_msg.target_vel_z = z_vel
                                        nav_pub.publish(nav_msg)
                                        msg = "send +z vel command"
                                else:
                                        msg = "blocked +z vel command: robot is not flying"

                        if key == ']':
                                if flight_state == STATE_FLYING:
                                        nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                                        nav_msg.target_vel_z = -z_vel
                                        nav_pub.publish(nav_msg)
                                        msg = "send -z vel command"
                                else:
                                        msg = "blocked -z vel command: robot is not flying"

                        if key == '\x03':
                                break

                        printMsg(msg)
                        rospy.sleep(0.001)

        except Exception as e:
                print(repr(e))
        finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
