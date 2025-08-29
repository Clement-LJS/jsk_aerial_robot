#!/usr/bin/env python

import sys
import time
import rospy
import math
import tf2_ros
import numpy as np
from std_msgs.msg import UInt8
from geometry_msgs.msg import Point
from aerial_robot_msgs.msg import FlightNav
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs

def contact_callback(msg):
    global contact_state
    contact_state = msg


if __name__ == "__main__":

    rospy.init_node("filter_contact_force")
    contact_state = ContactsState()
    end_wrench = WrenchStamped()

    wrench_pub = rospy.Publisher("/hydrus_xi/end_wrench", WrenchStamped, queue_size=1)
    contact_sub = rospy.Subscriber("/hydrus_xi/bumper_states", ContactsState, contact_callback)

    time.sleep(1)
    alpha = 0.05  # Smoothing factor

    filtered_force = np.zeros(3)
    measured_force = np.zeros(3)
  
    Q = 0.02*np.eye(3)
    R = 0.2*np.eye(3)
    P = np.zeros((3,3))
    gamma = 0.3

    while not rospy.is_shutdown():
        if not contact_state.states:
            measured_force = np.zeros(3)
        else:
            measured_force[0] = contact_state.states[0].total_wrench.force.x
            measured_force[1] = contact_state.states[0].total_wrench.force.y
            measured_force[2] = contact_state.states[0].total_wrench.force.z

        predicted_force = filtered_force
        P_pred = P + Q

        r = measured_force - filtered_force
        Q = 0.02*np.eye(3)
        # if abs(r[0]) > gamma:
        #     Q = 0.02*np.eye(3)
        # else:
        #     Q = 0.001*np.eye(3)
        
        K =  P_pred*np.linalg.inv(P_pred + R)
        filtered_force = predicted_force + K @ r

        P = (np.eye(3) - K) * P_pred
       
        end_wrench.header.stamp = rospy.Time.now()
        end_wrench.wrench.force.x = filtered_force[0]
        end_wrench.wrench.force.y = filtered_force[1]
        end_wrench.wrench.force.z = filtered_force[2]
        # end_wrench.wrench.torque.x = filtered_torque[0]
        # end_wrench.wrench.torque.y = filtered_torque[1]
        # end_wrench.wrench.torque.z = filtered_torque[2]
        print("Q", Q)
        print(end_wrench)
        wrench_pub.publish(end_wrench)
 
        time.sleep(0.05)


