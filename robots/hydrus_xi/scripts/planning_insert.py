#!/usr/bin/env python

import sys
import time
import rospy
import math
import tf2_ros
import tf
import numpy as np
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped

from aerial_robot_msgs.msg import FlightNav
from sensor_msgs.msg import JointState

from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs



def wrench_callback(msg):
    global external_wrench
    external_wrench = msg

   
def odam_callback(msg):
    global cog2world
    cog2world.transform.translation = msg.pose.pose.position
    cog2world.transform.rotation = msg.pose.pose.orientation

if __name__ == "__main__":

    rospy.init_node("planning_insert")

    rate = rospy.Rate(50)
    joint_pub = rospy.Publisher("/hydrus_xi/joints_ctrl", JointState, queue_size=1)
    mode_pub = rospy.Publisher("/hydrus_xi/imp_mode", UInt8, queue_size=1)
    pos_pub = rospy.Publisher("/hydrus_xi/pos_cmds", Point, queue_size=1)
    wrench_sub = rospy.Subscriber("/hydrus_xi/estimated_external_wrench", WrenchStamped, wrench_callback)
    cog_sub = rospy.Subscriber("/hydrus_xi/uav/cog/odom", Odometry, odam_callback)
    nav_pub = rospy.Publisher("/hydrus_xi/uav/nav", FlightNav, queue_size=1)
    plan_pub = rospy.Publisher("/hydrus_xi/plan_start", Empty, queue_size=1)
    move_pub = rospy.Publisher("/hydrus_xi/move_start", Empty, queue_size=1)
    cog2world = TransformStamped()
    external_wrench = WrenchStamped()

    time.sleep(1)
    pre_pose_flag = False
  

    plan =  Empty()
    move = Empty()


    plan_pub.publish(plan)
    print("planning")
    time.sleep(5)
    move_pub.publish(move)
    print("moving")

    nav_msg = FlightNav()
    nav_msg.pos_xy_nav_mode = 4 # pos_vel mode
    nav_msg.pos_z_nav_mode = 4 
    nav_msg.yaw_nav_mode = 4 


    # # #mode_pub.publish(mode)
    # # pos_pub.publish(pos)
    # # joint_pub.publish(joints)
    # # nav_pub.publish(nav_msg) # go to the origin point
    tf_buffer = tf2_ros.Buffer() 
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    tf_listener = tf2_ros.TransformListener(tf_buffer) 
 

 
    # # time.sleep(20)
    # # print("preparation 2")


    while not rospy.is_shutdown():
        end2cog = tf_buffer.lookup_transform("hydrus_xi/cog", "hydrus_xi/end_frame", rospy.Time(0), rospy.Duration(1.0)) 
        quatenion = [cog2world.transform.rotation.x, cog2world.transform.rotation.y, cog2world.transform.rotation.z, cog2world.transform.rotation.w]
        roll, pitch, yaw = euler_from_quaternion(quatenion)

        end2cog_pose = PointStamped()
        end2cog_pose.point.x = end2cog.transform.translation.x
        end2cog_pose.point.y = end2cog.transform.translation.y
        end2cog_pose.point.z = end2cog.transform.translation.z


        end2world = tf2_geometry_msgs.do_transform_point(end2cog_pose, cog2world) 
        print(end2world)

    
        if cog2world.transform.translation.x > 0.86 and (not pre_pose_flag):
            nav_msg.pos_xy_nav_mode = 1 
            nav_msg.pos_z_nav_mode = 1 
            nav_msg.yaw_nav_mode = 1
      
            nav_msg.target_vel_x = 0.05 * (2.45 - end2world.point.x)
            nav_msg.target_vel_y = 0.2 * (0.00 - end2world.point.y)
            nav_msg.target_vel_z = 0.4 * (1.00 - end2world.point.z)
       
            nav_msg.target_omega_z = 0.00
            nav_pub.publish(nav_msg)
            print((end2world.point.z - cog2world.transform.translation.z))
            print("navigating")
            if (abs(2.40 - end2world.point.x) < 0.02 and abs(0.00 - end2world.point.y) < 0.02 and abs(1.00 - end2world.point.z) < 0.02) :
                pre_pose_flag = True
        
        if pre_pose_flag:
            # nav_msg.target_vel_y = 0.3 * (0.00 - end2world.point.y)
            # nav_msg.target_vel_z = 0.5 * (1.00 - end2world.point.z)
            if external_wrench.wrench.force.x < -0.25 and 0< 2.45 - end2world.point.x < 0.01:
                nav_msg.target_vel_x = -0.02 * (-0.25 - external_wrench.wrench.force.x)
                nav_msg.target_vel_y = 1.0 * (0.00 - end2world.point.y)
                nav_msg.target_vel_z = 1.0 * (1.00 - end2world.point.z)
                print("a")
            else:
                nav_msg.target_vel_x = 0.04 * (2.60 - end2world.point.x)
                nav_msg.target_vel_y = 0.2 * (0.00 - end2world.point.y)
                nav_msg.target_vel_z = 0.4 * (1.00 - end2world.point.z)
                print("b")
            nav_pub.publish(nav_msg)
             

        

        #     nav_msg.target_pos_y = 1.25   # to right
        #     nav_msg.target_vel_y = 0.08 
        #     print(nav_msg)
        # else: 
        #     nav_msg.target_pos_y = -1.25  # to left
        #     nav_msg.target_vel_y = -0.08
        #     print(nav_msg)
        # nav_pub.publish(nav_msg)
        rate.sleep()


