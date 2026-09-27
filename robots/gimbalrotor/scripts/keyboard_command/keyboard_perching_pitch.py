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
step  = 0.0017 rad (approximately 0.1 deg)
limit = +/-20 deg
start_from_zero = false (read the navigator's current command)

Meaning:
This command is a rotation delta about the locked robot/navigation local +Y
passive revolute axis, in radians, not a world Euler-pitch increment.
The navigator applies perching_command_pitch_sign to this input, then
perching_arc_pitch_sign to the bounded logical delta for both position and attitude.
Zero returns to the locked pose, subject to the configured axial deadband.

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


def current_input_delta(state_topic_name):
    # The navigator publishes the bounded logical angle after command_pitch_sign.
    state = rospy.wait_for_message(state_topic_name, Float64, timeout=5.0)
    if math.isnan(state.data) or math.isinf(state.data):
        raise ValueError("Invalid commanded pitch delta")

    command_sign = float(rospy.get_param(
        "/gimbalrotor/navigation/perching_command_pitch_sign", 1.0))
    if math.isnan(command_sign) or math.isinf(command_sign) or command_sign == 0.0:
        command_sign = 1.0
    command_sign = 1.0 if command_sign > 0.0 else -1.0
    return state.data / command_sign


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

    step = rospy.get_param("~step", 0.0017) # approximately 0.1 deg
    state_topic_name = rospy.get_param(
        "~state_topic_name",
        "/gimbalrotor/perching/commanded_pitch_delta"
    )
    start_from_zero = rospy.get_param("~start_from_zero", False)

    limit_deg = rospy.get_param("~limit_deg", 20.0)
    limit_rad = math.radians(limit_deg)

    repeat_count = rospy.get_param("~repeat_count", 5)

    pub = rospy.Publisher(topic_name, Float64, queue_size=1)

    try:
        pitch_delta = clamp(current_input_delta(state_topic_name),
                            -limit_rad, limit_rad)
    except (rospy.ROSException, ValueError) as error:
        rospy.logerr("Cannot read current perching pitch delta from %s: %s",
                     state_topic_name, error)
        sys.exit(1)

    if start_from_zero:
        pitch_delta = 0.0
        publish_delta(pub, pitch_delta, repeat_count)

    print(MSG)
    print("Publishing to: {}".format(topic_name))
    print("Step size: {} rad = {:.2f} deg".format(step, math.degrees(step)))
    print("Limit: +/-{} deg = +/-{:.3f} rad".format(limit_deg, limit_rad))
    print("Repeat count per key press: {}".format(repeat_count))
    print("Starting from: {} ({:+.2f} deg)".format(
        "zero" if start_from_zero else "current command",
        math.degrees(pitch_delta)))
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