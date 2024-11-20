#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist, PoseStamped
from aerial_robot_msgs.msg import FlightNav
import numpy as np
from diagnostic_msgs.msg import KeyValue
import math

class aprilPIDcontroller():
    def __init__(self):
        rospy.init_node("aprilpidVel")

        #distance
        self.currentDistance = np.array([0.0, 0.0, 0.0])
        self.lastDistance = np.array([0.0, 0.0, 0.0])

        #ros
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback1)

        #others
        self.rate = rospy.Rate(40)
        self.nav_msg = FlightNav()
        
    def callback1(self,data):
        if data.detections:
            #set values to current pose
            position = data.detections[0].pose.pose.pose.position
            self.currentDistance = [position.z, -position.x, -position.y]
            
            #set distance log
            self.lastDistance = self.currentDistance
            
            self.tag_lost_flag = False
            
        else:
            self.tag_lost_flag = True

            #load log
            self.currentDistance = self.lastDistance
            
    def main(self):
        # self.docking_msg.data = True 
        while not rospy.is_shutdown():
            currentEucdistance = math.sqrt(self.currentDistance[0] ** 2 + self.currentDistance[1] ** 2 + self.currentDistance[2] ** 2)
            rospy.loginfo(currentEucdistance)

            self.rate.sleep()
            
if __name__ == "__main__":
    try:
        aprilpid = aprilPIDcontroller()
        aprilpid.main()
    except rospy.ROSInterruptException: pass
