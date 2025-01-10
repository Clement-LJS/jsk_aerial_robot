#!/usr/bin/env python
import rospy, tf, time, math
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64, Float32, Bool
from geometry_msgs.msg import Twist, PoseStamped
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord

class AprilPIDController:
    def __init__(self):
        rospy.init_node("aprilpidVel")

        # PID control parameters
        self.kp = -0.3
        self.ki = -0.0002
        self.kd = -0.01
        self.gaincontroller = 2

        # self.dt = 0.05  # time interval
        self.dt = 0.025

        # Terms and Errors
        self.p_term = np.array([0.0, 0.0, 0.0])
        self.i_term = np.array([0.0, 0.0, 0.0])
        self.d_term = np.array([0.0, 0.0, 0.0])

        self.pitch_p_term = np.array([0.0])
        self.pitch_i_term = np.array([0.0])
        self.pitch_d_term = np.array([0.0])
        
        self.yaw_p_term = np.array([0.0])
        self.yaw_i_term = np.array([0.0])
        self.yaw_d_term = np.array([0.0])

        self.err = np.array([0.0, 0.0, 0.0])
        self.err_i = np.array([0.0, 0.0, 0.0])
        self.pre_err = np.array([0.0, 0.0, 0.0])

        self.pitch_err = np.array([0.0])
        self.pitch_err_i = np.array([0.0])
        self.pitch_pre_err = np.array([0.0])
        
        self.yaw_err = np.array([0.0])
        self.yaw_err_i = np.array([0.0])
        self.yaw_pre_err = np.array([0.0])
        
        # Positions
        self.currentPosition = np.array([0.0, 0.0, 0.0])
        self.lastPosition = np.array([0.0, 0.0, 0.0])
        # Mocap Positions
        self.currentMocapPosition = np.array([0.0, 0.0, 0.0])
        self.lastMocapPosition = np.array([0.0, 0.0, 0.0])

        # Angles
        self.currentEuler = np.array([0.0, 0.0, 0.0])
        self.lastEuler = np.array([0.0, 0.0, 0.0])
        # Mocap Angles
        self.currentMocapEuler = np.array([0.0, 0.0, 0.0])
        self.lastMocapEuler = np.array([0.0, 0.0, 0.0])

        # Targets
        self.targetPosition = np.array([0.0, 0.0, 0.0])
        
        # self.bigtagTargetPosition = np.array([0.4, 0, 0.1])
        # self.smalltagTargetPosition = np.array([0.2, 0, 0.1])
        self.bigtagTargetPosition = np.array([0.4, 0.0, 0.0])
        self.smalltagTargetPosition = np.array([0.18, 0.0, 0.03])

        self.vel = np.array([0.0, 0.0, 0.0])
        self.targetPitch = np.array([0.0])
        self.targetYaw = np.array([0.0])
        self.target_pos_z = np.array([0.0])
               
        # Flags and states
        self.id0DetectedStartTime = None
        self.tagLostFlag = False
        self.mocapLostFlag = False
        self.targetPosChangedFlag = False

        # sub and pub
        self.sub = rospy.Subscriber('/beetle1/tag_detections', AprilTagDetectionArray, self.callback1)
        self.yaw_sub = rospy.Subscriber('/beetle1/mocap/pose', PoseStamped, self.callback2)
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=10)
        self.rot_pub = rospy.Publisher('/beetle1/final_target_baselink_rot', DesireCoord, queue_size=10)
        # self.rate = rospy.Rate(20)
        self.rate = rospy.Rate(40)

        # Flight navigation message
        self.nav_msg = FlightNav()
        self.rot_msg = DesireCoord()

    def calculateError(self):
        self.pre_err = self.err
        self.err = self.targetPosition - self.currentPosition
        self.err_i += self.err

        self.p_term = self.kp * self.err
        self.i_term = self.ki * self.err_i
        self.d_term = self.kd * (self.err - self.pre_err) / self.dt

        self.vel = self.p_term + self.i_term + self.d_term

        max_velocity = 0.3
        self.vel = np.clip(self.vel, -max_velocity, max_velocity)

    def setTargetPitch(self):
        self.pre_pitch_err = self.pitch_err
        self.pitch_err = -self.currentEuler[1]
        self.pitch_err_i += self.pitch_err

        self.pitch_p_term = self.kp * self.gaincontroller * self.pitch_err
        self.pitch_i_term = self.ki * self.gaincontroller * self.pitch_err_i
        self.pitch_d_term = self.kd * self.gaincontroller * (self.pitch_err - self.pitch_pre_err) / self.dt
    
        self.targetPitch = self.currentMocapEuler[1] + self.pitch_p_term + self.pitch_i_term + self.pitch_d_term
        self.targetPitchPlus = self.pitch_p_term + self.pitch_i_term + self.pitch_d_term

        # print(f"{self.targetPitch}")
        
    def setTargetYaw(self):
        self.pre_yaw_err = self.yaw_err
        self.yaw_err = -self.currentEuler[2]
        self.yaw_err_i += self.yaw_err

        self.yaw_p_term = self.kp * self.gaincontroller * self.yaw_err
        self.yaw_i_term = self.ki * self.gaincontroller * self.yaw_err_i
        self.yaw_d_term = self.kd * self.gaincontroller * (self.yaw_err - self.yaw_pre_err) / self.dt
    
        self.targetYaw = self.currentMocapEuler[2] + self.yaw_p_term + self.yaw_i_term + self.yaw_d_term

    def setTargetPosition(self, detected_positions, detected_orientations, tag_id):
        position = detected_positions[tag_id]
        orientation = detected_orientations[tag_id]
        self.currentPosition = [position.z, -position.x, -position.y]
        temp_currentEuler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

        if temp_currentEuler[0] >= 0.0:
            self.currentEuler = np.array([temp_currentEuler[2], np.pi - temp_currentEuler[0], -temp_currentEuler[1]])
        else:
            self.currentEuler = np.array([temp_currentEuler[2], -np.pi - temp_currentEuler[0], -temp_currentEuler[1]])

    def callback1(self, data):
        if data.detections:
            detected_ids = [detection.id[0] for detection in data.detections]
            detected_positions = {detection.id[0]: detection.pose.pose.pose.position for detection in data.detections}
            detected_orientations = {detection.id[0]: detection.pose.pose.pose.orientation for detection in data.detections}

            print(f"{detected_ids}")

            if 0 in detected_ids:
                if self.id0DetectedStartTime is None:
                    self.id0DetectedStartTime = time.time()
                elif time.time() - self.id0DetectedStartTime >= 3.0:
                    self.targetPosChangedFlag = True
                    self.setTargetPosition(detected_positions, detected_orientations, 0)
                    self.targetPosition = self.smalltagTargetPosition
                    # rospy.loginfo("\033[1;32m[Msg] Tag [0] is detected.\033[0m")
                    return
            if 1 in detected_ids:
                self.setTargetPosition(detected_positions, detected_orientations, 1)
                self.targetPosition = self.bigtagTargetPosition
                # rospy.loginfo("\033[1;32m[Msg] Tag [1] is detected.\033[0m")
            if 0 not in detected_ids:
                self.id0DetectedStartTime = None
                self.targetPosChangedFlag = False

            self.setTargetYaw()
            self.setTargetPitch()
                
            self.lastPosition = self.currentPosition
            self.lastEuler = self.currentEuler
            self.tagLostFlag = False

        else:
            self.tagLostFlag = True
            rospy.loginfo("\033[1;91m[Warn] Tag lost.\033[0m")
            self.currentPosition = self.lastPosition
            self.currentEuler = self.lastEuler

    def callback2(self, data):
        if data.pose:
            moc_pos = data.pose.position
            moc_ang = data.pose.orientation

            self.currentMocapPosition = (moc_pos.x, moc_pos.y, moc_pos.z)
            self.currentMocapEuler = tf.transformations.euler_from_quaternion((moc_ang.x, moc_ang.y, moc_ang.z, moc_ang.w))
            self.lastMocapEuler = self.currentMocapPosition
            self.lastMocapEuler = self.currentMocapEuler
            self.mocapLostFlag = False

        else:
            self.mocapLostFlag = True
            self.currentMocapPosition = self.lastMocapPosition
            self.currentMocapEuler = self.lastMocapEuler

    def main(self):
        tag_lost_yaw = None
        tag_lost_z = None
        yaw_toggle = False
        while not rospy.is_shutdown():
            if not self.tagLostFlag:
                if not self.targetPosChangedFlag:
                    tag_lost_yaw = None
                    tag_lost_z = None
                    self.calculateError()
                    # Publish navigation command
                    self.nav_msg.control_frame = 1
                    self.nav_msg.target = 1
                    self.nav_msg.pos_xy_nav_mode = 1
                    self.nav_msg.target_vel_x = self.vel[0]
                    self.nav_msg.target_vel_y = self.vel[1]
                    self.nav_msg.pos_z_nav_mode = 2
                    self.nav_msg.target_pos_z = self.currentMocapPosition[2] + self.currentPosition[2] + self.targetPosition[2]
                    self.nav_msg.yaw_nav_mode = 2
                    self.nav_msg.target_yaw = self.targetYaw

                    self.pub.publish(self.nav_msg)

                    self.rot_msg.pitch = self.targetPitch

                    self.rot_pub.publish(self.rot_msg)
                    
                else:
                    tag_lost_yaw = None
                    tag_lost_z = None
                    self.calculateError()
                    # Publish navigation command
                    self.nav_msg.control_frame = 1
                    self.nav_msg.target = 1
                    self.nav_msg.pos_xy_nav_mode = 1
                    self.nav_msg.target_vel_x = self.vel[0]
                    self.nav_msg.target_vel_y = self.vel[1]
                    self.nav_msg.pos_z_nav_mode = 2
                    self.nav_msg.target_pos_z = self.currentMocapPosition[2] + self.currentPosition[2] + self.targetPosition[2]
                    self.nav_msg.yaw_nav_mode = 2
                    self.nav_msg.target_yaw = self.targetYaw

                    self.pub.publish(self.nav_msg)

                    self.rot_msg.pitch = self.targetPitch

                    self.rot_pub.publish(self.rot_msg)
                    
            else:
                if tag_lost_yaw is None:
                    tag_lost_yaw = self.currentMocapEuler[2]
                    tag_lost_z = self.currentMocapPosition[2]

                self.nav_msg.control_frame = 1
                self.nav_msg.target = 1
                self.nav_msg.pos_xy_nav_mode = 1
                self.nav_msg.target_vel_x = 0.0
                self.nav_msg.target_vel_y = 0.0
                self.nav_msg.pos_z_nav_mode = 2
                # self.nav_msg.target_pos_z = tag_lost_z
                self.nav_msg.yaw_nav_mode = 2

                self.pub.publish(self.nav_msg)

                self.rot_msg.pitch = 0.0

                self.rot_pub.publish(self.rot_msg)
            
            self.rate.sleep()

if __name__ == "__main__":
    try:        
        april_pid = AprilPIDController()
        april_pid.main()
        rate.sleep()
    except rospy.ROSInterruptException:
        pass
