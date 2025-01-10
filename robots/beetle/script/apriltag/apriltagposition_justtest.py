#!/usr/bin/env python
import rospy, tf, time, math, smach, smach_ros
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64, Float32, Bool
from geometry_msgs.msg import Twist, PoseStamped
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord

#### state classes ####

"""
Aproachbigtag -> Rotate -> Approachsmalltag
              ^         |
              |_ _ _ _ _|      
"""

class AprilTagCallback:
    def __init__(self, tag_id):
        self.tag_id = tag_id
        self.currentPosition = np.array([0.0, 0.0, 0.0])
        self.lastPosition = np.array([0.0, 0.0, 0.0])
        self.currentEuler = np.array([0.0, 0.0, 0.0])
        self.lastEuler = np.array([0.0, 0.0, 0.0])
        self.sub = rospy.Subscriber('/beetle1/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.rate = rospy.Rate(40)
        self.tagLostFlag = False

    def setPosition(self, detected_positions, detected_orientations, self.tag_id):
        position = detected_positions[self.tag_id]
        orientation = detected_orientations[self.tag_id]
        self.currentPosition = [position.z, -position.x, -position.y]
        temp_currentEuler = tf.transformations.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

        if temp_currentEuler[0] >= 0.0:
            self.currentEuler = np.array([temp_currentEuler[2], np.pi - temp_currentEuler[0], -temp_currentEuler[1]])
        else:
            self.currentEuler = np.array([temp_currentEuler[2], -np.pi - temp_currentEuler[0], -temp_currentEuler[1]])
        
    def tag_callback(self, data):
        if data.detections:
            detected_ids = [detection.id[0] for detection in data.detections]
            detected_positions = {detection.id[0]: detection.pose.pose.pose.position for detection in data.detections}
            detected_orientations = {detection.id[0]: detection.pose.pose.pose.orientation for detection in data.detections}
            if self.tag_id in detected_ids:
                self.setPosition(detected_positions, detected_orientations, self.tag_id)
            self.lastPosition = self.currentPosition
            self.lastEuler = self.currentEuler
            self.tagLostFlag = False

        else:
            self.tagLostFlag = True
            rospy.loginfo("\033[1;91m[Warn] Tag lost.\033[0m")
            self.currentPosition = self.lastPosition
            self.currentEuler = self.lastEuler

class MocapTagCallback:
    def __init__(self, tag_id):
        self.currentMocapPosition = np.array([0.0, 0.0, 0.0])
        self.lastMocapPosition = np.array([0.0, 0.0, 0.0])
        self.currentMocapEuler = np.array([0.0, 0.0, 0.0])
        self.lastMocapEuler = np.array([0.0, 0.0, 0.0])
        
    def mocap_callback(self, data):
        if data.pose:
            moc_pos = data.pose.position
            moc_ang = data.pose.orientation
            self.currentMocapPosition = (moc_pos.x, moc_pos.y, moc_pos.z)
            self.lastMocapEuler = self.currentMocapPosition
            self.currentMocapEuler = tf.transformations.euler_from_quaternion((moc_ang.x, moc_ang.y, moc_ang.z, moc_ang.w))
            self.lastMocapEuler = self.currentMocapEuler
            self.mocapLostFlag = False

        else:
            self.mocapLostFlag = True
            self.currentMocapPosition = self.lastMocapPosition
            self.currentMocapEuler = self.lastMocapEuler
            
class ApproachbigtagState(smach.state):
    def __init__(self, tag_callback):
        smach.State.__init__(self, outcomes=['done','in_process'])
        self.tag_callback = tag_callback

        # PID control parameters
        self.kp = -0.3
        self.ki = -0.0002
        self.kd = -0.01
        self.dt = 0.025
        self.p_term = np.array([0.0, 0.0, 0.0])
        self.i_term = np.array([0.0, 0.0, 0.0])
        self.d_term = np.array([0.0, 0.0, 0.0])
        self.err = np.array([0.0, 0.0, 0.0])
        self.err_i = np.array([0.0, 0.0, 0.0])
        self.pre_err = np.array([0.0, 0.0, 0.0])
        self.pos_error_tol = np.array([0.05, 0.05, 0.01])

        # Other args and ROS
        self.currentPosition = np.array([0.0, 0.0, 0.0])
        self.lastPosition = np.array([0.0, 0.0, 0.0])
        self.targetPosition = np.array([0.0, 0.0, 0.0])
        self.bigtagTargetPosition = np.array([0.4, 0.0, 0.0])
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=1)
        self.rate = rospy.Rate(40)
        self.nav_msg = FlightNav()

        self.tag_callback = tag_callback

    def execute(self, userdata):
        self.pre_err = self.err
        self.err = self.bigtagTargetPosition - self.tag_callback.currentPosition
        self.err_i += self.err        
        self.p_term = self.kp * self.err
        self.i_term = self.ki * self.err_i
        self.d_term = self.kd * (self.err - self.pre_err) / self.dt

        self.vel = self.p_term + self.i_term + self.d_term

        if np.less(np.abs(self.err), self.pos_error_tol):
            userdata.position = self.currentPosition
            userdata.euler = self.currentEuler
            return 'done'
        else:
            self.nav_msg.control_frame = 1
            self.nav_msg.target = 1
            self.nav_msg.pos_xy_nav_mode = 1
            self.nav_msg.target_vel_x = self.vel[0]
            self.nav_msg.target_vel_y = self.vel[1]
            self.nav_msg.pos_z_nav_mode = 2
            self.nav_msg.target_pos_z = self.currentMocapPosition[2] + self.currentPosition[2] + self.targetPosition[2]
            return 'in_process'

class RotateState(smach.state):
    def __init__(self, tag_callback):
        smach.State.__init__(self, outcomes=['done','in_process'])

        # Parameters
        self.rot_error_tol = np.array([0.05, 0.05, 0.05])

        # Other args and ROS
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=1)
        self.rate = rospy.Rate(40)
        self.nav_msg = FlightNav()

        self.mocap_callback = mocap_callback

    def execute(self, userdata):
        currentPosition = userdata.position
        currentEuler = userdata.euler
        self.mocap_callback.

        if np.less(np.abs(self.err), self.pos_error_tol):
            return 'done'
        else:
            self.nav_msg.control_frame = 1
            self.nav_msg.target = 1
            self.nav_msg.pos_xy_nav_mode = 0
            self.nav_msg.target_pos_x = 
            self.nav_msg.target_pos_y = 
            self.nav_msg.pos_z_nav_mode = 2
            self.nav_msg.target_pos_z = self.currentMocapPosition[2] + self.currentPosition[2] + self.targetPosition[2]























            

            
    def main(self):
        while not rospy.is_shutdown():
            if not self.tagLostFlag:
                if not self.closeToBigTag:
                    self.targetPosition = self.bigtagTargetPosition
                elif not self.orientationReached:
                    self.targetYaw = self.targetYaw  # Continue adjusting orientation
                elif self.id0DetectedStartTime is not None and time.time() - self.id0DetectedStartTime >= 3.0:
                    self.targetPosition = self.smalltagTargetPosition

                self.calculateError()
                self.nav_msg.control_frame = 1
                self.nav_msg.target = 1
                self.nav_msg.pos_xy_nav_mode = 1
                self.nav_msg.target_vel_x = self.vel[0]
                self.nav_msg.target_vel_y = self.vel[1]
                self.nav_msg.pos_z_nav_mode = 2
                self.nav_msg.target_pos_z = self.targetPosition[2]
                self.nav_msg.yaw_nav_mode = 2
                self.nav_msg.target_yaw = self.targetYaw
            else:
                self.nav_msg.target_vel_x = 0.0
                self.nav_msg.target_vel_y = 0.0

            self.pub.publish(self.nav_msg)
            self.rate.sleep()

if __name__ == "__main__":
    try:        
        april_pid = AprilPIDController()
        april_pid.main()
    except rospy.ROSInterruptException:
        pass
