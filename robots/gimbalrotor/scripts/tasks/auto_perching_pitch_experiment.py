#!/usr/bin/env python

from __future__ import print_function

import math

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion


# ============================================================
# Experiment settings
# ============================================================

STEP_DEG = 0.6 #0.8
HOLD_TIME = 8.0 #15.0
NUMBER_OF_STEPS = 5 #15

PUBLISH_REPEAT = 5
PUBLISH_INTERVAL = 0.02


# ============================================================
# Functions
# ============================================================

def get_pitch_from_pose(pose):
    quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

    _, pitch, _ = euler_from_quaternion(quaternion)

    return pitch


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def publish_pitch_delta(publisher, pitch_delta_rad):
    message = Float64()
    message.data = pitch_delta_rad

    for _ in range(PUBLISH_REPEAT):
        if rospy.is_shutdown():
            return

        publisher.publish(message)
        rospy.sleep(PUBLISH_INTERVAL)


# ============================================================
# Main
# ============================================================

def main():
    rospy.init_node("auto_perching_pitch_command")

    publisher = rospy.Publisher("/gimbalrotor/perching/manual_pitch_delta", Float64, queue_size=1)

    rospy.sleep(1.0)

    # Read the locked perching orientation.
    locked_pose_msg = rospy.wait_for_message("/gimbalrotor/perching/locked_pose", PoseStamped)

    # Read the current nominal commanded orientation.
    commanded_pose_msg = rospy.wait_for_message("/gimbalrotor/perching/commanded_pose", PoseStamped)

    locked_pitch = get_pitch_from_pose(locked_pose_msg.pose)

    commanded_pitch = get_pitch_from_pose(commanded_pose_msg.pose)

    # manual_pitch_delta is relative to the locked pitch.
    pitch_delta_rad = normalize_angle(commanded_pitch - locked_pitch)

    rospy.loginfo("Starting from current pitch delta: %.2f deg", math.degrees(pitch_delta_rad))

    step_rad = math.radians(STEP_DEG)

    for step_number in range(1, NUMBER_OF_STEPS + 1):
        if rospy.is_shutdown():
            break

        # Positive pitch delta moves downward.
        pitch_delta_rad += step_rad

        publish_pitch_delta(publisher, pitch_delta_rad)

        rospy.loginfo("Step %d/%d: pitch delta = %.2f deg. Hold %.1f s.", step_number, NUMBER_OF_STEPS, math.degrees(pitch_delta_rad), HOLD_TIME)

        rospy.sleep(HOLD_TIME)

    rospy.loginfo("Finished at pitch delta: %.2f deg", math.degrees(pitch_delta_rad))


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
