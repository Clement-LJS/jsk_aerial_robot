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
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs

def odam_callback(msg):
    global cog2world
    cog2world.transform.translation = msg.pose.pose.position
    cog2world.transform.rotation = msg.pose.pose.orientation

def wrench_callback(msg):
    global external_wrench
    external_wrench = msg

if __name__ == "__main__":

    rospy.init_node("push_and_slide")
    cog2world = TransformStamped()
    external_wrench = WrenchStamped()

    link_num = rospy.get_param("~link_num", 4)
    duration = rospy.get_param("~duration", 0.05)
    joint_pub = rospy.Publisher("/hydrus_xi/joints_ctrl", JointState, queue_size=1)
    mode_pub = rospy.Publisher("/hydrus_xi/imp_mode", UInt8, queue_size=1)
    cog_sub = rospy.Subscriber("/hydrus_xi/uav/cog/odom", Odometry, odam_callback)
    wrench_sub = rospy.Subscriber("/hydrus_xi/estimated_external_wrench", WrenchStamped, wrench_callback)
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
    nav_msg.target_pos_x = -2.00
    nav_msg.target_vel_x = -0.2
    nav_msg.target_pos_y = 0.0
    nav_msg.target_vel_y = -0.2
    nav_msg.pos_z_nav_mode = 4 
    nav_msg.target_pos_z = 0.6
    nav_msg.target_vel_z = 0.2
    nav_msg.yaw_nav_mode = 4 
    nav_msg.target_omega_z = 2.0
    nav_msg.target_yaw = -0.85

    pos_pub.publish(pos)
    joint_pub.publish(joints)
    nav_pub.publish(nav_msg) # go to the origin point
    tf_buffer = tf2_ros.Buffer() 
    tf_listener = tf2_ros.TransformListener(tf_buffer) 
    
 

    time.sleep(5)
    print("preparation 1")

    nav_msg.pos_xy_nav_mode = 4
    nav_msg.yaw_nav_mode = 4 
    nav_msg.target_yaw = -0.85
    # nav_msg.target_yaw = yaw
    # nav_msg.target_omega_z = 0.3
    nav_msg.target_pos_x = -1.5
    nav_msg.target_pos_y = -0.6

    nav_pub.publish(nav_msg)
 
    # # time.sleep(20)
    # print("preparation 2")
    round = 0

    while not rospy.is_shutdown():
        end2cog = tf_buffer.lookup_transform("hydrus_xi/cog", "hydrus_xi/end_effector", rospy.Time(0), rospy.Duration(1.0)) 
        quatenion = [cog2world.transform.rotation.x, cog2world.transform.rotation.y, cog2world.transform.rotation.z, cog2world.transform.rotation.w]
        roll, pitch, yaw = euler_from_quaternion(quatenion)

        end2cog_pose = PointStamped()
        end2cog_pose.point.x = end2cog.transform.translation.x
        end2cog_pose.point.y = end2cog.transform.translation.y
        end2cog_pose.point.z = end2cog.transform.translation.z


        end2world = tf2_geometry_msgs.do_transform_point(end2cog_pose, cog2world) 
        print(yaw)
        print(end2world.point)
       
        nav_msg = FlightNav()

        round += 1
        nav_msg.pos_xy_nav_mode = 1
        if round % 200 < 100: 
            if abs(-0.2 - end2world.point.x) > 0.02:
                nav_msg.target_vel_x = 0.2 * (-0.2 - end2world.point.x)
                nav_msg.target_vel_y = 0.1
            else:
                nav_msg.target_vel_x = -0.5 * (-0.6 - external_wrench.wrench.force.x)
                nav_msg.target_vel_y = 0.15
            nav_msg.target_vel_z = 0.2 * (0.6 - end2world.point.z)

        else: 

            if abs(-0.2 - end2world.point.x) > 0.02:
                nav_msg.target_vel_x = 0.2 * (-0.2 - end2world.point.x)
                nav_msg.target_vel_y = -0.1
            else:
                nav_msg.target_vel_x = -0.5 * (-0.6 - external_wrench.wrench.force.x)
                nav_msg.target_vel_y = -0.15
           
            nav_msg.target_vel_z = 0.2 * (0.6 - end2world.point.z)
        nav_pub.publish(nav_msg)
        time.sleep(duration)


