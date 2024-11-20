#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float32, Float64, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
import numpy as np
import tf.transformations as tf
from diagnostic_msgs.msg import KeyValue

class aprilPIDcontroller():
    def __init__(self):
        rospy.init_node("aprilpidVel")
        # parameters for PID control

        #gains
        self.kp = -2.0 #-0.2
        self.ki = -0.005 #-0.0002
        self.kd = -0.05 #-0.01
        self.recovery_gain = 3
        self.yaw_kp = -0.4
        self.yaw_ki = -0.001
        self.yaw_kd = -0.02

        #duration
        self.dt = 0.025

        #terms
        self.p_term = np.array([0.0, 0.0, 0.0])
        self.i_term = np.array([0.0, 0.0, 0.0])
        self.d_term = np.array([0.0, 0.0, 0.0])
        self.yaw_p_term = 0.0
        self.yaw_i_term = 0.0
        self.yaw_d_term = 0.0

        #term corrector
        self.TermCorrector = np.array([0.5, 1, 1])
        
        #position and angular error
        self.err_i = np.array([0.0, 0.0, 0.0])
        self.err = np.array([0.0, 0.0, 0.0])
        self.pre_err = np.array([0.0, 0.0, 0.0])
        self.yaw_err_i = 0.0
        self.yaw_err = 0.0
        self.yaw_pre_err = 0.0

        #vel, yaw
        self.vel = np.array([0.0, 0.0, 0.0])
        self.vel_recovery = np.array([0.0, 0.0, 0.0])
        self.currentVelocity = np.array([0.0, 0.0, 0.0])
        self.velocitylog_queue = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        self.yaw = 0.0
        self.yaw_pre = 0.0
        self.target_yaw = 0.0
        self.pre_yaw_diff = 0.0

        #angular
        self.currentmocapQuarter = np.array([0.0, 0.0, 0.0, 0.0])
        self.currentmocapEuler = np.array([0.0, 0.0, 0.0])
        self.lastmocapEuler = np.array([0.0, 0.0, 0.0])
        self.currentQuarter = np.array([0.0, 0.0, 0.0, 0.0])
        self.currentEuler = np.array([0.0, 0.0, 0.0])
        self.lastEuler = np.array([0.0, 0.0, 0.0])

        #distance
        self.targetDistance = np.array([0.38171, 0.009239, 0.01017]) #x offset = 1.58171
        self.currentDistance = np.array([0.0, 0.0, 0.0])
        self.lastDistance = np.array([0.0, 0.0, 0.0])
        self.distancelog_queue = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        self.currentEucDistance = 0.0

        #ros
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=1)
        self.sub = rospy.Subscriber('/beetle1/tag_detections', AprilTagDetectionArray, self.callback1)
        self.sub = rospy.Subscriber('/beetle1/uav/cog/odom', Odometry, self.callback3)
        self.yaw_sub = rospy.Subscriber('/beetle1/mocap/pose', PoseStamped, self.callback2)

        # #boolean for docking
        # self.docking_msg = Bool()
        # self.docking_pub = rospy.Publisher('/beetle1/docking_cmd', Bool, queue_size=1)
        
        # #flags for docking
        # self.flag_msg = KeyValue()
        # self.flag_pub_1 = rospy.Publisher('/beetle1/assembly_flag', KeyValue, queue_size=1)
        # self.flag_pub_2 = rospy.Publisher('/beetle2/assembly_flag', KeyValue, queue_size=1)

        #others
        self.rate = rospy.Rate(40)
        self.nav_msg = FlightNav()

        #flags
        self.tag_lost_flag = False
        self.mocap_lost_flag = False

    def calculateError(self):
        #calculate errors for position and yaw control
        self.pre_err = self.err
        self.err = self.targetDistance - self.currentDistance
        self.err_i += self.err

        self.yaw_err = self.yaw - self.currentmocapEuler[2]
        self.yaw_err_i += self.yaw_err
        self.yaw_pre_err = self.yaw_err

        
    # def dockingVerifier(self):
    #     #set docking message
    #     if self.currentEucDistance <= 0.01:
    #         if self.currentEuler[1] <=2:
    #             self.flag_msg.key = '1'
    #             self.flag_msg.value = '1'
    #     else:
    #         self.flag_msg.key = '0'
    #         self.flag_msg.value = '0'

    def defineYaw(self):
        self.nav_msg.yaw_nav_mode = 2
        self.nav_msg.target_yaw = self.yaw
        
        # if abs(self.currentEuler[1]) >= 0.1:
        #     self.nav_msg.yaw_nav_mode = 2
        #     self.nav_msg.target_yaw = self.target_yaw
            
        #     if self.currentEuler[1] >= 0.1:
        #         self.nav_msg.target_yaw = self.yaw - 0.05
        #         rospy.loginfo("+")
        #     elif self.currentEuler[1] <= 0.1:
        #         self.nav_msg.target_yaw = self.yaw + 0.05
        #         rospy.loginfo("-")
        #     else:
        #         self.nav_msg.target_yaw = self.yaw
        # else:
        #     self.nav_msg.yaw_nav_mode = 0

    def callback1(self,data):
        if data.detections:
            #set values to current pose
            position = data.detections[0].pose.pose.pose.position
            self.currentDistance = np.array([position.z, -position.x, -position.y])
            currentDistance_log = np.array([[position.z, -position.x, -position.y]])
            
            #calculate current euclid distance
            self.currentEucDistance = ((position.z-self.targetDistance[0])**2 + (-position.x-self.targetDistance[1])**2 + (-position.y-self.targetDistance[2])**2)**0.5

            #set distance log
            self.lastDistance = self.currentDistance
            self.distancelog_queue = np.append(self.distancelog_queue, currentDistance_log, axis = 0)
            if len(self.distancelog_queue) > 10:
                self.distancelog_queue = np.delete(self.distancelog_queue, 0, axis = 0)
            #rospy.loginfo(self.distancelog_queue)
            # self.double_lastDistance = self.lastDistance
            # self.lastDistance = self.currentDistance

            #set values to current quarter angle
            orientation = data.detections[0].pose.pose.pose.orientation
            self.currentQuarter = [orientation.x, orientation.y, orientation.z, orientation.w]

            #calculate current Euler angle
            self.currentEuler = tf.euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))

            #set angular log
            self.lastEuler = self.currentEuler

            #calculate target yaw
            yaw_change_diff = abs(self.currentEuler[1] - self.pre_yaw_diff)
            if yaw_change_diff > 0.2:
                rospy.loginfo("ignore unreliable data!")
                self.pre_yaw_diff = self.currentEuler[1]
                return
            # rospy.loginfo("stable!")
            self.pre_yaw_diff = self.currentEuler[1]
            self.yaw = self.currentmocapEuler[2] + self.currentEuler[1]
            # rospy.loginfo(self.currentmocapEuler[2])
            # rospy.loginfo(self.currentEuler[1])
            # rospy.loginfo(self.yaw)
            
            self.tag_lost_flag = False

        else:
            self.tag_lost_flag = True

            #load log
            average_position = np.mean(self.distancelog_queue, axis = 0)
            self.currentDistance = self.lastDistance +  self.recovery_gain * (self.lastDistance - average_position)
            self.currentEuler = self.lastEuler

        #calculate target yaw
        yaw_change_diff = abs(self.currentEuler[1] - self.pre_yaw_diff)
        if yaw_change_diff > 0.2:
            rospy.loginfo("ignore unreliable data!")
            self.pre_yaw_diff = self.currentEuler[1]
            return
        self.pre_yaw_diff = self.currentEuler[1]
        self.yaw = self.currentmocapEuler[2] + self.currentEuler[1]

    def callback2(self, data):
        if data.pose:
            moc = data.pose.orientation
            self.currentmocapQuarter = [moc.x, moc.y, moc.z, moc.w]
            #calculate current mocap Euler angular
            self.currentmocapEuler = tf.euler_from_quaternion((moc.x, moc.y, moc.z, moc.w))
            self.lastmocapEuler = self.currentmocapEuler
            self.mocap_lost_flag = False

        else:
            self.mocap_lost_flag = True
            self.currentmocapEuler = self.lastmocapEuler

    def callback3(self, data):
        #set values to current velocity
        odom = data.twist.twist.linear
        self.currentVelocity = np.array([[odom.x, odom.y, odom.z]])
        # rospy.loginfo(self.currentVelocity)
        if not self.tag_lost_flag:
            self.velocitylog_queue = np.append(self.velocitylog_queue, self.currentVelocity, axis = 0)
            if len(self.velocitylog_queue) > 10:
                self.velocitylog_queue = np.delete(self.velocitylog_queue, 0, axis = 0)
            average_velocity = np.mean(self.velocitylog_queue, axis = 0)
            self.vel_recovery = -1.0 * average_velocity
        
    def main(self):
        # self.docking_msg.data = True 
        while not rospy.is_shutdown():
            #callback1, 2

            #calcurate error
            self.calculateError()

            #calculate term
            self.p_term = self.kp * self.err
            self.i_term = self.ki * self.err_i
            self.d_term = self.kd * (self.err - self.pre_err) / self.dt

            self.yaw_p_term = -self.yaw_kp * self.yaw_err
            self.yaw_i_term = -self.yaw_ki * self.yaw_err_i
            self.yaw_d_term = -self.yaw_kd * (self.yaw_err - self.yaw_pre_err) / self.dt
            # rospy.loginfo(self.yaw_p_term)
            
            # calcurate terms
            if self.currentEucDistance <= 0.5:
                #weaken x PID
                self.p_term *= self.TermCorrector
                self.i_term *= self.TermCorrector
                self.d_term *= self.TermCorrector
        
            # calculate main process
            if not self.tag_lost_flag:
                self.vel = self.p_term + self.i_term + self.d_term            
                self.target_yaw = self.yaw_p_term + self.yaw_i_term + self.yaw_d_term
            else:
                self.err = 0
                self.err_i = 0
                self.pre_err = 0
                self.yaw_nav_mode = 0
                self.vel = self.vel_recovery
        
            #publish message
            self.nav_msg.control_frame = 1
            self.nav_msg.target = 1
            self.nav_msg.pos_xy_nav_mode = 1
            
            self.nav_msg.target_vel_x = self.vel[0]
            self.nav_msg.target_vel_y = self.vel[1]

            self.defineYaw()
            self.nav_msg.pos_z_nav_mode = 1
            self.nav_msg.target_vel_z = self.vel[2]
            self.pub.publish(self.nav_msg)

            # #beetle1 mode verifier
            # self.dockingVerifier()

            # #switch beetle1 into assembled mode
            #  # true ->activated false->disactivate
            # self.docking_pub.publish(self.docking_msg)
            
            #  #flag_msg = KeyValue()
            #  #flag_msg.key = '1'
            #  #flag_msg.value = '1' # 1 -> assembly, 2->separate
            # self.flag_pub_1.publish(self.flag_msg)
            # self.flag_pub_2.publish(self.flag_msg)

            #rospy.loginfo(self.yaw)
            # rospy.loginfo(self.currentmocapEuler[2])
            # rospy.loginfo(self.currenotEuler[1])
            # rospy.loginfo(self.targetDistance[0])
            # rospy.loginfo(self.currentDistance[2])
            # rospy.loginfo(self.err[0])
            # rospy.loginfo(self.p_term)
            # rospy.loginfo(self.i_term)
            #rospy.loginfo(self.target_yaw)
            rospy.loginfo(self.yaw_err)
            rospy.loginfo(self.target_yaw)
            rospy.loginfo("")

            self.rate.sleep()
            
if __name__ == "__main__":
    try:
        aprilpid = aprilPIDcontroller()
        aprilpid.main()
    except rospy.ROSInterruptException: pass
