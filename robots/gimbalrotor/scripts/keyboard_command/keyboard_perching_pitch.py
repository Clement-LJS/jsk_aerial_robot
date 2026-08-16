#!/usr/bin/env python

from __future__ import print_function

import math
import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty, Float64
from tf.transformations import euler_from_quaternion


MSG = """
Instruction:

Manual perching pitch delta keyboard command.

Publish pitch command to:
/gimbalrotor/perching/manual_pitch_delta

Manual override topic:
/gimbalrotor/perching/manual_pitch_override

Keys:

i : increase pitch delta (+step)
u : decrease pitch delta (-step)

space : reset pitch delta to 0

CTRL+c : quit

Default:
step  = 0.0017 rad, approximately 0.1 deg
limit = +/-20 deg

Meaning:
This command is relative to the locked perching pitch.

When i, u, or space is pressed, the automatic pitch
experiment is stopped before the keyboard command is sent.

This does NOT publish /gimbalrotor/uav/nav.
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)

    key = sys.stdin.read(1)

    termios.tcsetattr(
        sys.stdin,
        termios.TCSADRAIN,
        settings
    )

    return key


def clamp(value, lower, upper):
    if value > upper:
        return upper

    if value < lower:
        return lower

    return value


def normalize_angle(angle):
    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )


def get_pitch_from_pose(pose):
    quaternion = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ]

    _, pitch, _ = euler_from_quaternion(quaternion)

    return pitch


def read_current_pitch_delta(
        locked_pose_topic,
        commanded_pose_topic,
        timeout):
    """
    Read the navigator's current commanded pitch delta.

    This prevents the keyboard from starting again at zero
    after the automatic script has already moved the robot.
    """

    locked_pose_msg = rospy.wait_for_message(
        locked_pose_topic,
        PoseStamped,
        timeout=timeout
    )

    commanded_pose_msg = rospy.wait_for_message(
        commanded_pose_topic,
        PoseStamped,
        timeout=timeout
    )

    locked_pitch = get_pitch_from_pose(
        locked_pose_msg.pose
    )

    commanded_pitch = get_pitch_from_pose(
        commanded_pose_msg.pose
    )

    return normalize_angle(
        commanded_pitch - locked_pitch
    )


def publish_manual_override(
        publisher,
        repeat_count,
        publish_interval):
    """
    Notify the automatic script before publishing a keyboard command.
    """

    message = Empty()

    for _ in range(repeat_count):
        if rospy.is_shutdown():
            return

        publisher.publish(message)
        rospy.sleep(publish_interval)


def publish_delta(
        publisher,
        pitch_delta,
        repeat_count,
        publish_interval):
    message = Float64()
    message.data = pitch_delta

    for _ in range(repeat_count):
        if rospy.is_shutdown():
            return

        publisher.publish(message)
        rospy.sleep(publish_interval)


def print_state(pitch_delta):
    pitch_delta_deg = math.degrees(pitch_delta)

    text = (
        "manual pitch delta: {:+.3f} rad "
        "({:+.1f} deg)"
    ).format(
        pitch_delta,
        pitch_delta_deg
    )

    print(text.ljust(120) + "\r", end="")


if __name__ == "__main__":

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node(
        "perching_pitch_delta_keyboard_command"
    )

    topic_name = rospy.get_param(
        "~topic_name",
        "/gimbalrotor/perching/manual_pitch_delta"
    )

    manual_override_topic = rospy.get_param(
        "~manual_override_topic",
        "/gimbalrotor/perching/manual_pitch_override"
    )

    locked_pose_topic = rospy.get_param(
        "~locked_pose_topic",
        "/gimbalrotor/perching/locked_pose"
    )

    commanded_pose_topic = rospy.get_param(
        "~commanded_pose_topic",
        "/gimbalrotor/perching/commanded_pose"
    )

    step = rospy.get_param(
        "~step",
        0.0017
    )

    limit_deg = rospy.get_param(
        "~limit_deg",
        20.0
    )

    repeat_count = rospy.get_param(
        "~repeat_count",
        5
    )

    publish_interval = rospy.get_param(
        "~publish_interval",
        0.02
    )

    pose_wait_timeout = rospy.get_param(
        "~pose_wait_timeout",
        2.0
    )

    limit_rad = math.radians(limit_deg)

    pitch_publisher = rospy.Publisher(
        topic_name,
        Float64,
        queue_size=1
    )

    manual_override_publisher = rospy.Publisher(
        manual_override_topic,
        Empty,
        queue_size=1
    )

    pitch_delta = 0.0

    print(MSG)
    print("Publishing pitch to: {}".format(topic_name))
    print(
        "Publishing override to: {}".format(
            manual_override_topic
        )
    )
    print(
        "Step size: {} rad = {:.2f} deg".format(
            step,
            math.degrees(step)
        )
    )
    print(
        "Limit: +/-{} deg = +/-{:.3f} rad".format(
            limit_deg,
            limit_rad
        )
    )
    print(
        "Repeat count per key press: {}".format(
            repeat_count
        )
    )
    print("")

    rospy.sleep(0.5)

    try:
        while not rospy.is_shutdown():

            key = get_key(settings)

            if key == '\x03':
                break

            if key not in ('i', 'u', ' '):
                print(
                    "Unknown key: {}".format(key).ljust(120)
                    + "\r",
                    end=""
                )
                continue

            # Manual control has priority.
            # Notify the automatic script before publishing pitch.
            publish_manual_override(
                manual_override_publisher,
                repeat_count,
                publish_interval
            )

            if key == ' ':
                pitch_delta = 0.0

            else:
                try:
                    # Synchronize with the current navigator command.
                    # This prevents a jump back toward zero.
                    pitch_delta = read_current_pitch_delta(
                        locked_pose_topic,
                        commanded_pose_topic,
                        pose_wait_timeout
                    )

                except rospy.ROSException as exception:
                    rospy.logerr(
                        "Cannot read current perching pitch: %s",
                        str(exception)
                    )

                    # Do not send a potentially unsafe stale target.
                    continue

                if key == 'i':
                    pitch_delta += step

                elif key == 'u':
                    pitch_delta -= step

            pitch_delta = clamp(
                pitch_delta,
                -limit_rad,
                limit_rad
            )

            publish_delta(
                pitch_publisher,
                pitch_delta,
                repeat_count,
                publish_interval
            )

            print_state(pitch_delta)

    except Exception as exception:
        print(repr(exception))

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings
        )

        print("")