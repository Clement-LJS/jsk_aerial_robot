#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist, PoseStamped
from aerial_robot_msgs.msg import FlightNav
import numpy as np
import tf
import time


class AprilPIDController:
    def __init__(self):
        rospy.init_node("aprilpidVel")

        # PID control parameters
        self.kp = -0.2
        self.ki = -0.0002
        self.kd = -0.01
        self.dt = 0.025  # time interval

        # Error terms
        self.p_term = np.array([0.0, 0.0, 0.0])
        self.i_term = np.array([0.0, 0.0, 0.0])
        self.d_term = np.array([0.0, 0.0, 0.0])
        self.err_i = np.array([0.0, 0.0, 0.0])
        self.err = np.array([0.0, 0.0, 0.0])
        self.pre_err = np.array([0.0, 0.0, 0.0])
        self.TermCorrector = np.array([0.5, 1, 1])

        # Positions and distances
        self.targetDistance = np.array([0.0, 0.0, 0.0])
        self.currentDistance = np.array([0.0, 0.0, 0.0])
        self.lastDistance = np.array([0.0, 0.0, 0.0])
        self.currentEuler = np.array([0.0, 0.0, 0.0])
        self.lastEuler = np.array([0.0, 0.0, 0.0])

        # Target positions
        self.bigtag_target_distance = np.array([0.5, 0, 0])
        self.smalltag_target_distance = np.array([0.1, 0, 0])

        # Flags and states
        self.reached_target1 = False
        self.id0_detected_start_time = None
        self.tag_lost_flag = False
        self.mocap_lost_flag = False

        # ROS Subscribers and Publishers
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback1)
        self.yaw_sub = rospy.Subscriber('/beetle1/mocap/pose', PoseStamped, self.callback2)
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=1)

        # ROS rate
        self.rate = rospy.Rate(40)

        # Flight navigation message
        self.nav_msg = FlightNav()

    def calculateError(self):
        self.err = self.targetDistance - self.currentDistance
        self.err_i += self.err
        if np.all(self.pre_err == 0.0):
            self.pre_err = self.err

    def defineYaw(self):
        self.yaw = self.lastEuler[2] - self.currentEuler[2]

    def set_target_distance(self, detected_positions, detected_orientations, tag_id):
        position = detected_positions[tag_id]
        orientation = detected_orientations[tag_id]
        self.currentDistance = [position.z, -position.x, -position.y]
        self.currentEuler = tf.transformations.euler_from_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        )

    def is_target_reached(self, target_distance, threshold=0.05):
        distance = np.linalg.norm(np.array(self.currentDistance) - np.array(target_distance))
        return distance < threshold

    def callback1(self, data):
        if data.detections:
            detected_ids = [detection.id[0] for detection in data.detections]
            detected_positions = {detection.id[0]: detection.pose.pose.pose.position for detection in data.detections}
            detected_orientations = {detection.id[0]: detection.pose.pose.pose.orientation for detection in data.detections}

            if not self.reached_target1 and 1 in detected_ids:
                self.set_target_distance(detected_positions, detected_orientations, 1)
                rospy.loginfo("\033[1;32m[Msg] Using id[1] for positioning.\033[0m")
                self.targetDistance = self.bigtag_target_distance

                if self.is_target_reached(self.bigtag_target_distance):
                    rospy.loginfo("\033[1;32m[Msg]Reached target position 1.\033[0m")
                    self.reached_target1 = True

            elif self.reached_target1 and 0 in detected_ids:
                self.set_target_distance(detected_positions, detected_orientations, 0)
                if self.id0_detected_start_time is None:
                    self.id0_detected_start_time = time.time()
                elif time.time() - self.id0_detected_start_time >= 1.0:
                    rospy.loginfo("\033[1;32m[Msg] Using id[0] for positioning.\033[0m")
                    self.targetDistance = self.smalltag_target_distance
            else:
                self.id0_detected_start_time = None

            self.lastDistance = self.currentDistance
            self.lastEuler = self.currentEuler
            self.tag_lost_flag = False
        else:
            self.tag_lost_flag = True
            rospy.logwarn("\033[1;91m[Warn] Tag lost.\033[0m")
            self.currentDistance = self.lastDistance
            self.currentEuler = self.lastEuler

    def callback2(self, data):
        if data.pose:
            moc = data.pose.orientation
            self.currentEuler = tf.transformations.euler_from_quaternion((moc.x, moc.y, moc.z, moc.w))
            self.lastEuler = self.currentEuler
            self.mocap_lost_flag = False
        else:
            self.mocap_lost_flag = True
            rospy.logwarn(("\033[1;91m[Warn] Mocap lost.\033[0m"))

    def main(self):
        while not rospy.is_shutdown():
            self.calculateError()

            # PID terms
            self.p_term = self.kp * self.err
            self.i_term = self.ki * self.err_i
            self.d_term = self.kd * (self.err - self.pre_err) / self.dt

            if np.linalg.norm(self.currentDistance) <= 0.3:  # Weaken PID if close
                self.p_term *= self.TermCorrector
                self.i_term *= self.TermCorrector
                self.d_term *= self.TermCorrector

            # Compute velocity
            self.vel = self.p_term + self.i_term + self.d_term

            # Publish navigation command
            self.nav_msg.control_frame = 1
            self.nav_msg.target = 1
            self.nav_msg.pos_xy_nav_mode = 1
            self.nav_msg.target_vel_x = self.vel[0]
            self.nav_msg.target_vel_y = self.vel[1]
            self.nav_msg.pos_z_nav_mode = 1
            self.nav_msg.target_vel_z = self.vel[2]
            self.pub.publish(self.nav_msg)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        april_pid = AprilPIDController()
        april_pid.main()
    except rospy.ROSInterruptException:
        pass
