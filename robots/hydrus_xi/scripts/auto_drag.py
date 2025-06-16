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
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench, Point, Vector3

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs


if __name__ == "__main__":

    rospy.init_node("auto_drag")

    link_num = rospy.get_param("~link_num", 4)
    duration = rospy.get_param("~duration", 0.05)
    nav_pub = rospy.Publisher("/hydrus_xi/uav/nav", FlightNav, queue_size=1)
    joint_pub = rospy.Publisher("/hydrus_xi/joints_ctrl", JointState, queue_size=1)
    applywerench_srv = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    
    time.sleep(1)

#[-0.07, 0.79, -0.39]
#[1.0, 1.5, -0.59]

    joints = JointState()
    nav_msg = FlightNav()
    wrench = ApplyBodyWrench()
    joints.position = [1.0, 1.5, -0.59]

    joint_pub.publish(joints)
    
    time.sleep(5)
    print("preparation 1")
    nav_msg.pos_xy_nav_mode = 4 # pos_vel mode
    nav_msg.target_pos_x = -3.0
    nav_msg.target_vel_x = -0.1
    nav_msg.target_pos_y = 0.3
    nav_msg.target_vel_y = -0.1
    nav_msg.pos_z_nav_mode = 4 
    nav_msg.target_pos_z = 0.6
    nav_msg.target_vel_z = 0.1
    nav_msg.yaw_nav_mode = 4 
    nav_msg.target_yaw = -0.85

    nav_pub.publish(nav_msg)
    time.sleep(8)
    print("preparation 2")

    force = Vector3(x=5.0, y=0.0, z=0.0)
    torque = Vector3(x=0.0, y=0.0, z=0.0)
    wrench = Wrench(force=force, torque=torque)

    ref_point = Point(x=0.0, y=0.0, z=0.0)

    start_time = rospy.Time(0) 
    
    round = 0
    

    while not rospy.is_shutdown():
        round += 1
        print("round: ",round)
        if round % 8 == 1 or round % 8 == 2:
            duration = rospy.Duration(3.0)  
            force = Vector3(x=0.0, y=3.0, z=0.0)
            wrench = Wrench(force=force, torque=torque)
        elif round % 8 == 3 or round % 8 == 4:
            duration = rospy.Duration(1.0)  
            force = Vector3(x=0.0, y=1.0, z=0.0)
            wrench = Wrench(force=force, torque=torque)
        elif round % 8 == 5 or round % 8 == 6:
            duration = rospy.Duration(3.0)  
            force = Vector3(x=0.0, y=-3.0, z=0.0)
            wrench = Wrench(force=force, torque=torque)
        elif round % 8 == 7 or round % 8 == 0:
            duration = rospy.Duration(1.0)  
            force = Vector3(x=0.0, y=-1.0, z=0.0)
            wrench = Wrench(force=force, torque=torque)
        response = applywerench_srv(
        body_name="hydrus_xi::link4", 
        wrench=wrench,
        start_time=start_time,
        duration=duration)

        if round % 4 == 1 or round % 4 == 2:
            time.sleep(10)
        else:
            time.sleep(7)