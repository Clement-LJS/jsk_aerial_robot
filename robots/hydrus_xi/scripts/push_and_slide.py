#!/usr/bin/env python

import sys
import time
import rospy
import math
import tf2_ros
from std_msgs.msg import UInt8
from geometry_msgs.msg import Point
from aerial_robot_msgs.msg import FlightNav
from sensor_msgs.msg import JointState

from tf.transformations import euler_from_quaternion

if __name__ == "__main__":

    rospy.init_node("push_and_slide")

    link_num = rospy.get_param("~link_num", 4)
    duration = rospy.get_param("~duration", 0.2)
    joint_pub = rospy.Publisher("/hydrus_xi/joints_ctrl", JointState, queue_size=1)
    mode_pub = rospy.Publisher("/hydrus_xi/imp_mode", UInt8, queue_size=1)
    pos_pub = rospy.Publisher("/hydrus_xi/pos_cmds", Point, queue_size=1)
    nav_pub = rospy.Publisher("/hydrus_xi/uav/nav", FlightNav, queue_size=1)
    time.sleep(1)


    mode = UInt8()
    mode.data = 1
    joints = JointState()
    pos = Point()
    pos.x = 1.0
    pos.y = 0.2
    pos.z = 0.0
    joints.position = [1.215, -0.265, 1.048]
    nav_msg = FlightNav()
    nav_msg.pos_xy_nav_mode = 4 # pos_vel mode
    nav_msg.target_pos_x = -0.25
    nav_msg.target_vel_x = -0.2
    nav_msg.target_pos_y = 0.0
    nav_msg.target_vel_y = -0.2
    nav_msg.pos_z_nav_mode = 4 
    nav_msg.target_pos_z = 0.98
    nav_msg.target_vel_z = 0.2
    nav_msg.yaw_nav_mode = 4 
    nav_msg.target_omega_z = 2.0
    nav_msg.target_yaw = 1.57

    pos_pub.publish(pos)
    joint_pub.publish(joints)
    nav_pub.publish(nav_msg) # go to the origin point
    tf_buffer = tf2_ros.Buffer() 
    tf_listener = tf2_ros.TransformListener(tf_buffer) 
    
 

    time.sleep(8)
    print("preparation 1")
 
    transform = tf_buffer.lookup_transform("hydrus_xi/end_effector", "hydrus_xi/cog", rospy.Time(0), rospy.Duration(1.0)) 
    quatenion = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
    roll, pitch, yaw = euler_from_quaternion(quatenion)
    nav_msg.target_yaw = 3.14 + yaw
    nav_msg.target_omega_z = 0.3
    nav_msg.target_pos_x = -1.2
    nav_msg.target_pos_y = -0.5
    nav_pub.publish(nav_msg)
    time.sleep(5)
    round = 0

    while not rospy.is_shutdown():
        nav_msg = FlightNav()
        round += 1
        nav_msg.pos_xy_nav_mode = 1
        if round % 80 < 40: 
            #nav_msg.target_pos_y = 1.00   # to right
            nav_msg.target_vel_y = 0.2
            # print(nav_msg)
        else: 
            #nav_msg.target_pos_y = -0.75  # to left
            nav_msg.target_vel_y = -0.2
            # print(nav_msg)
        nav_pub.publish(nav_msg)
        time.sleep(duration)


