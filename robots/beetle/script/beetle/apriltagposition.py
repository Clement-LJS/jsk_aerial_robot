#!/usr/bin/env python
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist, PoseStamped
from aerial_robot_msgs.msg import FlightNav
import numpy as np
from diagnostic_msgs.msg import KeyValue

class aprilPIDcontroller():
    def __init__(self):
        rospy.init_node("aprilpidVel")
        # parameter's for PID control

        #position gains
        self.kp = -0.2
        self.ki = -0.0002
        self.kd = -0.01

        #duration
        self.dt = 0.025

        #position terms
        self.p_term = np.array([0.0, 0.0, 0.0])
        self.i_term = np.array([0.0, 0.0, 0.0])
        self.d_term = np.array([0.0, 0.0, 0.0])

        #term corrector
        self.TermCorrector = np.array([0.5, 1, 1])
        
        #position error
        self.err_i = np.array([0.0, 0.0, 0.0])
        self.err = np.array([0.0, 0.0, 0.0])
        self.pre_err = np.array([0.0, 0.0, 0.0])

        #vel, yaw
        self.vel = np.array([0.0, 0.0, 0.0])
        self.yaw = 0
        self.yaw_pre = 0

        #angular        
        self.currentmocapQuarter = np.array([0.0, 0.0, 0.0, 0.0])
        self.currentmocapEuler = np.array([0.0, 0.0, 0.0])
        self.lastmocapEuler = np.array([0.0, 0.0, 0.0])
        self.currentQuarter = np.array([0.0, 0.0, 0.0, 0.0])
        self.currentEuler = np.array([0.0, 0.0, 0.0])
        self.lastEuler = np.array([0.0, 0.0, 0.0])

        #distance
        self.targetDistance = np.array([1.08171, -0.01239, 0.01017]) #x offset = 1.58171
        self.currentDistance = np.array([0.0, 0.0, 0.0])
        self.lastDistance = np.array([0.0, 0.0, 0.0])
        self.currentEucDistance = 0

        #ros
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=1)
        self.sub = rospy.Subscriber('/beetle1/tag_detections', AprilTagDetectionArray, self.callback1)
        self.yaw_sub = rospy.Subscriber('/beetle1/mocap/pose', PoseStamped, self.callback2)

        #others
        self.rate = rospy.Rate(40)
        self.nav_msg = FlightNav()

        #apriltag lost flag
        self.tag_lost_flag = False

        #mocap lost flag
        self.mocap_lost_flag = False

    def calculateError(self):
        #calculate errors for PID 
        self.err = self.targetDistance - self.currentDistance
        self.err_i += self.err
        self.pre_err = self.err

    def defineYaw(self):
        self.yaw = self.currentmocapEuler[2] - self.currentEuler[2]

    def callback1(self,data):
        if data.detections:
            #set values to current pose
            position = data.detections[0].pose.pose.pose.position
            self.currentDistance = [position.z, -position.x, -position.y]
            
            #calculate current euclid distance
            self.currentEucDistance = ((position.z-self.targetDistance[0])**2 + (-position.x-self.targetDistance[1])**2 + (-position.y-self.targetDistance[2])**2)**0.5

            #set distance log
            self.lastDistance = self.currentDistance

            #calculate current Euler angle
            orientation = data.detections[0].pose.pose.pose.orientation
            self.currentQuarter = [orientation.x, orientation.y, orientation.z, orientation.w]
            self.currentEuler = np.array([np.arctan2(2 * (orientation.x * orientation.y + orientation.w * orientation.z), 1 - 2 * (orientation.x**2 + orientation.y**2)),
                             np.arcsin(2 * (orientation.w * orientation.y - orientation.x * orientation.z)),
                             np.arctan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y), 1 - 2 * (orientation.y**2 + orientation.z**2))])

            #set angular log
            self.lastEuler = self.currentEuler
            
            self.tag_lost_flag = False
            
        else:
            self.tag_lost_flag = True

            #load log
            self.currentDistance = self.lastDistance
            self.currentEuler = self.lastEuler

    def callback2(self, data):
        if data.pose:
            moc = data.pose.orientation
            self.currentmocapQuarter = [moc.x, moc.y, moc.z, moc.w]
            #calculate current mocap Euler angular
            self.currentmocapEuler = np.degrees(np.array([np.arctan2(2 * (moc.w * moc.x + moc.y * moc.z), 1 - 2 * (moc.x**2 + moc.y**2)),
                                 np.arcsin(2 * (moc.w * moc.y - moc.z * moc.x)),
                                 np.arctan2(2 * (moc.w * moc.z + moc.x * moc.y), 1 - 2 * (moc.y**2 + moc.z**2))]))
            self.lastmocapEuler = self.currentmocapEuler
            self.mocap_lost_flag = False

        else:
            self.mocap_lost_flag = True
            self.currentmocapEuler = self.lastmocapEuler
            
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
            
            # calcurate terms
            if self.currentEucDistance <= 0.3:
                #weaken x PID
                self.p_term *= self.TermCorrector
                self.i_term *= self.TermCorrector
                self.d_term *= self.TermCorrector
        
            # calculate main process
            self.vel = self.p_term + self.i_term + self.d_term
        
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

            rospy.loginfo(self.currentDistance)

            self.rate.sleep()
            
if __name__ == "__main__":
    try:
        aprilpid = aprilPIDcontroller()
        aprilpid.main()
    except rospy.ROSInterruptException: pass
