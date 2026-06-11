#!/usr/bin/env python

from __future__ import print_function

import sys
import select
import termios
import tty
import math

import rospy
from std_msgs.msg import Float64


MSG = """
Instruction:

Manual perching pitch delta keyboard command.

Publish to:
/gimbalrotor/perching/manual_pitch_delta

Keys:

i : increase pitch delta (+step)
u : decrease pitch delta (-step)

space : reset pitch delta to 0

CTRL+c : quit

Default:
step  = 0.02 rad
limit = +/-20 deg

Meaning:
This command is relative to the locked perching pitch.

Example:
  +0.10 rad means locked_pitch + 0.10 rad
  -0.10 rad means locked_pitch - 0.10 rad
   0.00 rad means return to locked_pitch

This does NOT publish /gimbalrotor/uav/nav.
Therefore it will not fight /perching_cutting_mission.
"""


def get_key(settings):
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


def publish_delta(pub, pitch_delta, repeat_count):
    msg = Float64()
    msg.data = pitch_delta

    for _ in range(repeat_count):
        pub.publish(msg)
        rospy.sleep(0.02)


def print_state(pitch_delta):
    pitch_delta_deg = math.degrees(pitch_delta)
    text = "manual pitch delta: {:+.3f} rad ({:+.1f} deg)".format(
        pitch_delta,
        pitch_delta_deg
    )
    print(text.ljust(120) + "\r", end="")


if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("perching_pitch_delta_keyboard_command")

    topic_name = rospy.get_param(
        "~topic_name",
        "/gimbalrotor/perching/manual_pitch_delta"
    )

    step = rospy.get_param("~step", 0.02)

    limit_deg = rospy.get_param("~limit_deg", 20.0)
    limit_rad = math.radians(limit_deg)

    repeat_count = rospy.get_param("~repeat_count", 5)

    pub = rospy.Publisher(topic_name, Float64, queue_size=1)

    pitch_delta = 0.0

    print(MSG)
    print("Publishing to: {}".format(topic_name))
    print("Step size: {} rad = {:.2f} deg".format(step, math.degrees(step)))
    print("Limit: +/-{} deg = +/-{:.3f} rad".format(limit_deg, limit_rad))
    print("Repeat count per key press: {}".format(repeat_count))
    print("")

    rospy.sleep(0.5)

    try:
        while not rospy.is_shutdown():

            key = get_key(settings)

            if key == 'i':
                pitch_delta += step

            elif key == 'u':
                pitch_delta -= step

            elif key == ' ':
                pitch_delta = 0.0

            elif key == '\x03':
                break

            else:
                print("Unknown key: {}".format(key).ljust(120) + "\r", end="")
                continue

            pitch_delta = clamp(pitch_delta, -limit_rad, limit_rad)

            publish_delta(pub, pitch_delta, repeat_count)
            print_state(pitch_delta)

    except Exception as e:
        print(repr(e))

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print("")