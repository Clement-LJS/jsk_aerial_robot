#!/usr/bin/env python

import rospy
from spinal.msg import PwmTest

def main():
    # Initialize the ROS node
    rospy.init_node('pwm_sender', anonymous=True)

    # Create a publisher to /pwm_test topic
    pub = rospy.Publisher('/pwm_test', PwmTest, queue_size=10)

    # Wait a moment to ensure publisher is registered
    rospy.sleep(1)

    # Create the message
    msg = PwmTest()
    msg.motor_index = [0]       # motor ID 0
    msg.pwms = [0.55]           # fixed PWM value

    # Publish once
    pub.publish(msg)
    rospy.loginfo("Sent PWM 0.55 to motor 0")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

