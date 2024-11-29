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

        # PID control parameters
        self.kp = -0.3
        self.ki = -0.0002
        self.kd = -0.01
        self.dt = 0.025  # time interval

        # Error terms
        self.p_term = np.array([0.0, 0.0, 0.0])
        self.i_term = np.array([0.0, 0.0, 0.0])
        self.d_term = np.array([0.0, 0.0, 0.0])

        self.yaw_p_term = np.array([0.0])
        self.yaw_i_term = np.array([0.0])
        self.yaw_d_term = np.array([0.0])

        self.err = np.array([0.0, 0.0, 0.0])
        self.err_i = np.array([0.0, 0.0, 0.0])
        self.pre_err = np.array([0.0, 0.0, 0.0])

        self.yaw_err = np.array([0.0])
        self.yaw_err_i = np.array([0.0])
        self.yaw_pre_err = np.array([0.0])

        self.TermCorrector = np.array([0.5, 1, 1])

        # Distances
        self.targetPosition = np.array([0.0, 0.0, 0.0])
        self.currentDistance = np.array([0.00001, 0.00001, 0.00001])
        self.lastDistance = np.array([0.0, 0.0, 0.0])

        # Angles
        self.currentEuler = np.array([0.0, 0.0, 0.0])
        self.lastEuler = np.array([0.0, 0.0, 0.0])
        
        # Mocap Positions
        self.currentMocapPosition = np.array([0.0, 0.0, 0.0])
        self.lastMocapPosition = np.array([0.0, 0.0, 0.0])
        self.currentMocapEuler = np.array([0.0, 0.0, 0.0])
        self.lastMocapEuler = np.array([0.0, 0.0, 0.0])

        # Target positions
        self.bigtag_target_distance = np.array([0.6, 0, -0.3])
        self.smalltag_target_distance = np.array([0.2, 0, -0.3])
               
        # Flags and states
        self.reached_target1 = False
        self.id0_detected_start_time = None
        self.tag_lost_flag = False

        # ROS Subscribers and Publishers
        self.sub = rospy.Subscriber('/beetle1/tag_detections', AprilTagDetectionArray, self.callback1)
        self.yaw_sub = rospy.Subscriber('/beetle1/mocap/pose', PoseStamped, self.callback2)
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=1)

        # ROS rate
        self.rate = rospy.Rate(40)

        # Flight navigation message
        self.nav_msg = FlightNav()

    def calculateError(self):
        self.err = self.targetPosition - self.currentDistance
        self.err_i += self.err
        self.pre_err = self.err

        # if self.currentDistance[1] != 0:
        #     self.yawchecker = self.currentDistance[0] / self.currentDistance[1]
        # else:
        #     self.yawchecker = 0.01
            
        # if self.yawchecker >= 0:
        #     self.yaw_err = self.currentMocapEuler[2] + (math.pi/2 - math.atan(self.yawchecker))
        # else:
        #     self.yaw_err = self.currentMocapEuler[2] + (-math.pi/2 - math.atan(self.yawchecker))
        self.yaw_err = -self.currentEuler[2]
        self.yaw_err_i += self.yaw_err
        self.yaw_pre_err = self.yaw_err

    def set_target_distance(self, detected_positions, detected_orientations, tag_id):
        position = detected_positions[tag_id]
        orientation = detected_orientations[tag_id]
        self.currentDistance = [position.z, -position.x, -position.y]
        temp_currentEuler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

        if temp_currentEuler[0] >= 0.0:
            self.currentEuler = np.array([temp_currentEuler[2], np.pi - temp_currentEuler[0], -temp_currentEuler[1]])
        else:
            self.currentEuler = np.array([temp_currentEuler[2], -np.pi - temp_currentEuler[0], -temp_currentEuler[1]])

        if twist_currentEuler[0] >= 0.0:
            self.currentEuler = np.array([temp_currentEuler[2], np.pi - temp.currentEuler[0], -temp.currentEuler[1]])
        else:
            self.currentEuler = np.array([temp_currentEuler[2], -np.pi - temp.currentEuler[0], -temp.currentEuler[1]])

    def callback1(self, data):
        if data.detections:
            detected_ids = [detection.id[0] for detection in data.detections]
            detected_positions = {detection.id[0]: detection.pose.pose.pose.position for detection in data.detections}
            detected_orientations = {detection.id[0]: detection.pose.pose.pose.orientation for detection in data.detections}

            print(f"{detected_ids}")

            if 0 in detected_ids:
                if self.id0_detected_start_time is None:
                    self.id0_detected_start_time = time.time()
                elif time.time() - self.id0_detected_start_time >= 3.0:
                    # rospy.loginfo("\033[1;32m[Msg] Using id[0] for positioning.\033[0m")
                    self.set_target_distance(detected_positions, detected_orientations, 0)
                    self.targetPosition = self.smalltag_target_distance
                    return

            if 1 in detected_ids:
                # rospy.loginfo("\033[1;32m[Msg] Using id[1] for positioning.\033[0m")
                self.set_target_distance(detected_positions, detected_orientations, 1)
                self.targetPosition = self.bigtag_target_distance

            if 0 not in detected_ids:
                self.id0_detected_start_time = None
            
            self.lastDistance = self.currentDistance
            self.lastEuler = self.currentEuler
            self.tag_lost_flag = False

        else:
            self.tag_lost_flag = True
            rospy.loginfo("\033[1;91m[Warn] Tag lost.\033[0m")
            self.currentDistance = self.lastDistance
            self.currentEuler = self.lastEuler

    def callback2(self, data):
        if data.pose:
            moc = data.pose.orientation
            self.currentMocapEuler = tf.transformations.euler_from_quaternion((moc.x, moc.y, moc.z, moc.w))
            self.lastMocapEuler = self.currentMocapEuler
            self.mocap_lost_flag = False

        else:
            self.mocap_lost_flag = True
            self.currentMocapEuler = self.lastMocapEuler

    def main(self):
        while not rospy.is_shutdown():

            # Error calculating
            self.calculateError()

            # print(f"{self.targetPosition}")
            
            # PID terms
            self.p_term = self.kp * self.err
            self.i_term = self.ki * self.err_i
            self.d_term = self.kd * (self.err - self.pre_err) / self.dt

            self.yaw_p_term = self.kp * self.yaw_err
            self.yaw_i_term = self.ki * self.yaw_err_i
            self.yaw_d_term = self.kd * (self.yaw_err - self.yaw_pre_err) / self.dt

            # if np.linalg.norm(self.currentDistance) <= 0.3:  # Weaken PID if close
            #     self.p_term *= self.TermCorrector
            #     self.i_term *= self.TermCorrector
            #     self.d_term *= self.TermCorrector

            # Compute velocity
            self.vel = self.p_term + self.i_term + self.d_term
            self.yaw = self.yaw_p_term + self.yaw_i_term + self.yaw_d_term

            # Publish navigation command
            self.nav_msg.control_frame = 1
            self.nav_msg.target = 1
            self.nav_msg.pos_xy_nav_mode = 1
            self.nav_msg.target_vel_x = self.vel[0]
            self.nav_msg.target_vel_y = self.vel[1]
            self.nav_msg.pos_z_nav_mode = 1
            self.nav_msg.target_vel_z = self.vel[2]
            self.nav_msg.yaw_nav_mode = 2
            self.nav_msg.target_yaw = self.yaw
            self.pub.publish(self.nav_msg)
            self.rate.sleep()

            # print(f"{math.degrees(self.currentMocapEuler[2])}")
            # if self.yawchecker >= 0:
            #     print(f"{math.degrees(math.pi/2 - math.atan(self.yawchecker))}")
            # else:
            #     print(f"{math.degrees(-math.pi/2 - math.atan(self.yawchecker))}")
            # print(f"{math.degrees(self.yaw)}")

if __name__ == "__main__":
    try:
        april_pid = AprilPIDController()
        april_pid.main()
    except rospy.ROSInterruptException:
        pass
