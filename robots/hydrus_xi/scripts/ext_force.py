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

    rospy.init_node("ext_force")

    applywerench_srv = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
    ext_signal_pub = rospy.Publisher("/hydrus_xi/ext_force", Wrench, queue_size=10)
    time.sleep(0.5)
    force = Vector3(x=1.0, y=0.0, z=0.0)
    torque = Vector3(x=0.0, y=0.0, z=0.0)
    wrench = Wrench(force=force, torque=torque)
    start_time = rospy.Time(0) 
    duration = rospy.Duration(10.0)  
    ext_signal_pub.publish(wrench)
    response = applywerench_srv(
        body_name="hydrus_xi::link4", 
        wrench=wrench,
        start_time=start_time,
        duration=duration)

    time.sleep(10.0)
    force = Vector3(x=0.0, y=0.0, z=0.0)
    torque = Vector3(x=0.0, y=0.0, z=0.0)
    wrench = Wrench(force=force, torque=torque)
    ext_signal_pub.publish(wrench)

