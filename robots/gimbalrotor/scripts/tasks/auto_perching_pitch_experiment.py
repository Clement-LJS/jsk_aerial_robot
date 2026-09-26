#!/usr/bin/env python

from __future__ import print_function

import math

import rospy
from std_msgs.msg import Float64


# ============================================================
# Experiment settings
# ============================================================

STEP_DEG = 0.8
HOLD_TIME = 15.0
NUMBER_OF_STEPS = 15

PUBLISH_REPEAT = 5
PUBLISH_INTERVAL = 0.02


# ============================================================
# Functions
# ============================================================

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

    # The state is the bounded logical rotation about locked local +Y.
    delta_msg = rospy.wait_for_message("/gimbalrotor/perching/commanded_pitch_delta", Float64)
    command_sign = float(rospy.get_param("/gimbalrotor/navigation/perching_command_pitch_sign", 1.0))
    if math.isnan(command_sign) or math.isinf(command_sign) or command_sign == 0.0:
        command_sign = 1.0
    command_sign = 1.0 if command_sign > 0.0 else -1.0
    if math.isnan(delta_msg.data) or math.isinf(delta_msg.data):
        rospy.logerr("Invalid commanded local-axis pitch delta")
        return
    # Convert state back to manual input units; the navigator applies this sign.
    pitch_delta_rad = delta_msg.data / command_sign

    rospy.loginfo("Starting from current pitch delta: %.2f deg", math.degrees(pitch_delta_rad))

    step_rad = math.radians(STEP_DEG)

    for step_number in range(1, NUMBER_OF_STEPS + 1):
        if rospy.is_shutdown():
            break

        # Positive input retains the configured cutting-direction sign convention.
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
