#!/usr/bin/env python3
"""Publish Cartesian position/rotation increments; this script contains no kinematics."""

import math
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import TwistStamped


KEYS = {
    "w": (0, 1), "s": (0, -1),
    "a": (1, 1), "d": (1, -1),
    "r": (2, 1), "f": (2, -1),
    "u": (3, 1), "j": (3, -1),
    "i": (4, 1), "k": (4, -1),
    "o": (5, 1), "l": (5, -1),
}


def main():
    rospy.init_node("keyboard_motion")
    topic = rospy.get_param("~command_topic", "/aerial_robot/motion/command/increment")
    frame = rospy.get_param("~frame", "tool")
    translation = float(rospy.get_param("~translation_step", 0.005))
    rotation = math.radians(float(rospy.get_param("~rotation_step_deg", 0.5)))
    if not all(math.isfinite(step) and step > 0 for step in (translation, rotation)):
        raise ValueError("Keyboard step sizes must be finite and positive")
    if not sys.stdin.isatty():
        raise RuntimeError("keyboard_motion.py requires an interactive terminal")
    publisher = rospy.Publisher(topic, TwistStamped, queue_size=1)
    print("Cartesian tool increments: w/s X, a/d Y, r/f Z")
    print("u/j roll, i/k pitch, o/l yaw (+/-); q or Ctrl-C exits")
    print("Translation {:.4f} m; rotation {:.3f} deg; frame '{}'".format(
        translation, math.degrees(rotation), frame))
    print("Six 'k' presses request -3 degrees with the default 0.5 degree step.")
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        while not rospy.is_shutdown():
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue
            key = sys.stdin.read(1)
            if key in ("q", "\x03", "\x04"):
                break
            if key not in KEYS:
                continue
            if publisher.get_num_connections() == 0:
                rospy.logwarn_throttle(1.0, "No motion command subscriber; increment discarded")
                continue
            axis, sign = KEYS[key]
            values = [0.0] * 6
            values[axis] = sign * (translation if axis < 3 else rotation)
            message = TwistStamped()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = frame
            message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = values[:3]
            message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = values[3:]
            publisher.publish(message)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print()


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
