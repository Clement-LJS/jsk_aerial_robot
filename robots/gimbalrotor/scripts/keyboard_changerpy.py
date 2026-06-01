#!/usr/bin/env python

from __future__ import print_function

import sys
import select
import termios
import tty
import math

import rospy
from geometry_msgs.msg import Vector3Stamped


msg = """
Instruction:

Publish target baselink RPY to:
/gimbalrotor/final_target_baselink_rpy

Keys:

p : increase roll  (+0.02 rad)
o : decrease roll  (-0.02 rad)

i : increase pitch (+0.02 rad)
u : decrease pitch (-0.02 rad)

y : increase yaw   (+0.02 rad)
t : decrease yaw   (-0.02 rad)

space : reset roll, pitch, yaw to 0

CTRL+c to quit

Limit:
roll, pitch, yaw are limited to +/-20 deg
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def clamp(value, lower, upper):
    if value > upper:
        return upper
    if value < lower:
        return lower
    return value


def publish_rpy(pub, roll, pitch, yaw):
    rpy_msg = Vector3Stamped()

    rpy_msg.header.stamp = rospy.Time.now()
    rpy_msg.header.frame_id = ""

    rpy_msg.vector.x = roll
    rpy_msg.vector.y = pitch
    rpy_msg.vector.z = yaw

    pub.publish(rpy_msg)


def print_state(roll, pitch, yaw):
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    text = "roll: {:+.3f} rad ({:+.1f} deg), pitch: {:+.3f} rad ({:+.1f} deg), yaw: {:+.3f} rad ({:+.1f} deg)".format(
        roll, roll_deg,
        pitch, pitch_deg,
        yaw, yaw_deg
    )

    print(text.ljust(120) + "\r", end="")


if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("rpy_keyboard_command")

    topic_name = rospy.get_param(
        "~topic_name",
        "/gimbalrotor/final_target_baselink_rpy"
    )

    step = rospy.get_param("~step", 0.02)

    limit_deg = rospy.get_param("~limit_deg", 20.0)
    limit_rad = math.radians(limit_deg)

    rpy_pub = rospy.Publisher(topic_name, Vector3Stamped, queue_size=1)

    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    print(msg)
    print("Publishing to: {}".format(topic_name))
    print("Step size: {} rad".format(step))
    print("Limit: +/-{} deg = +/-{:.3f} rad".format(limit_deg, limit_rad))

    rospy.sleep(0.5)

    try:
        while not rospy.is_shutdown():

            key = getKey()

            if key == 'p':
                roll += step

            elif key == 'o':
                roll -= step

            elif key == 'i':
                pitch += step

            elif key == 'u':
                pitch -= step

            elif key == 'y':
                yaw += step

            elif key == 't':
                yaw -= step

            elif key == ' ':
                roll = 0.0
                pitch = 0.0
                yaw = 0.0

            elif key == '\x03':
                break

            else:
                print("Unknown key: {}".format(key).ljust(120) + "\r", end="")
                continue

            roll = clamp(roll, -limit_rad, limit_rad)
            pitch = clamp(pitch, -limit_rad, limit_rad)
            yaw = clamp(yaw, -limit_rad, limit_rad)

            publish_rpy(rpy_pub, roll, pitch, yaw)
            print_state(roll, pitch, yaw)

            rospy.sleep(0.001)

    except Exception as e:
        print(repr(e))

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("")
