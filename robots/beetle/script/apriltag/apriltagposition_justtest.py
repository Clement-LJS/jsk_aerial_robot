#!/usr/bin/env python
import rospy, tf, time, math
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64, Float32, Bool
from geometry_msgs.msg import Twist, PoseStamped
from aerial_robot_msgs.msg import FlightNav

class AprilPIDController:
    def __init__(self):
        rospy.init_node("aprilpidVel")

        # Positions
        self.currentPosition = np.array([0.00001, 0.00001, 0.00001])

        # Angles
        self.currentEuler = np.array([0.0, 0.0, 0.0])
        self.target_yaw = np.array([0.0])

        # Target Positions
        self.targetPosition = np.array([0.6, 0.0, -0.3])
    
        # Flags and states
        self.id0_detected_start_time = None
        self.tag_lost_flag = False

        # ROS Subscribers and Publishers
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback1)
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=1)

        # ROS rate
        self.rate = rospy.Rate(40)

        # Flight navigation message
        self.nav_msg = FlightNav()

    def calculateError(self):
        self.err = self.targetPosition - self.currentPosition
 
    def set_target_position(self, detected_positions, detected_orientations, tag_id):
        position = detected_positions[tag_id]
        orientation = detected_orientations[tag_id]
        self.currentPosition = [position.z, -position.x, -position.y]
        temp_currentEuler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

        if temp_currentEuler[0] >= 0.0:
            self.currentEuler = np.array([temp_currentEuler[2], np.pi - temp_currentEuler[0], -temp_currentEuler[1]])
        else:
            self.currentEuler = np.array([temp_currentEuler[2], -np.pi - temp_currentEuler[0], -temp_currentEuler[1]])

        print(f"{math.degrees(self.currentEuler[2])}")

    def callback1(self, data):
        if data.detections:
            detected_ids = [detection.id[0] for detection in data.detections]
            detected_positions = {detection.id[0]: detection.pose.pose.pose.position for detection in data.detections}
            detected_orientations = {detection.id[0]: detection.pose.pose.pose.orientation for detection in data.detections}

            if 0 in detected_ids:
                if self.id0_detected_start_time is None:
                    self.id0_detected_start_time = time.time()
                elif time.time() - self.id0_detected_start_time >= 3.0:
                    # rospy.loginfo("\033[1;32m[Msg] Using id[0] for positioning.\033[0m")
                    self.set_target_position(detected_positions, detected_orientations, 0)
                    self.targetPosition = self.targetPosition
                    return
            if 1 in detected_ids:
                # rospy.loginfo("\033[1;32m[Msg] Using id[1] for positioning.\033[0m")
                self.set_target_position(detected_positions, detected_orientations, 1)
                self.targetPosition = self.targetPosition
            if 0 not in detected_ids:
                self.id0_detected_start_time = None
            
            self.tag_lost_flag = False

        else:
            self.tag_lost_flag = True
            rospy.loginfo("\033[1;91m[Warn] Tag lost.\033[0m")

    def main(self):
        while not rospy.is_shutdown():

            # Error calculating
            self.calculateError()
            self.nav_msg.target_yaw = self.target_yaw
            self.pub.publish(self.nav_msg)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        april_pid = AprilPIDController()
        april_pid.main()
    except rospy.ROSInterruptException:
        pass
