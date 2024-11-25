#!/usr/bin/env python
import os, tf, math, rospy, time
import numpy as np
import xml.etree.ElementTree as ET
from apriltag_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Transform, Quaternion
from aerial_robot_msgs.msg import FlightNav
from gazebo_msgs.srv import SpawnModel

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

        # Distances
        self.targetPosition = np.array([0.0, 0.0, 0.0])
        self.currentDistance = np.array([0.0, 0.0, 0.0])
        self.lastDistance = np.array([0.0, 0.0, 0.0])
        self.currentEuler = np.array([0.0, 0.0, 0.0])
        self.lastEuler = np.array([0.0, 0.0, 0.0])

        # Mocap Positions
        self.currentMocapPosition = np.array([0.0, 0.0, 0.0])
        self.lastMocapPosition = np.array([0.0, 0.0, 0.0])
        self.currentMocapEuler = np.array([0.0, 0.0, 0.0])
        self.lastMocapEuler = np.array([0.0, 0.0, 0.0])

        # Target positions
        self.bigtag_target_distance = np.array([0.5, 0, 0])
        self.smalltag_target_distance = np.array([0.1, 0, 0])

        # Flags and states
        self.reached_target1 = False
        self.id0_detected_start_time = None
        self.tag_lost_flag = False

        # ROS Subscribers and Publishers
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback1)
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=1)

        # ROS rate
        self.rate = rospy.Rate(40)

        # Flight navigation message
        self.nav_msg = FlightNav()

    def euler_to_quaternion(self, roll, pitch, yaw):
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
        
    def calculateError(self):
        self.err = self.targetPosition - self.currentDistance
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

    def spawnModel(self):
        model_path = f'/home/iida/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/models/sim/doubletag/doubletag_texture.sdf'

        try:
            with open(model_path, 'r') as model_file:
                model_sdf = model_file.read()
            tree = ET.parse(model_path)
            root = tree.getroot()
            pose_element = root.find('.//pose')
        
            if pose_element is not None:
                pose_values = list(map(float, pose_element.text.split()))
                x, y, z = pose_values[0:3]
                roll, pitch, yaw = pose_values[3:6]
    
                pose = Point(x, y, z)
                orientation = self.euler_to_quaternion(roll, pitch, yaw)
                initial_pose = Pose(pose, orientation)

            else:
                print("\033[1;91m[Warn] No <pose> tag found in the model file.\033[0m")
                return None, None, None

            inertia_element = root.find('.//inertia')
            if inertia_element is not None:
                inertia = {}
                inertia['m'] = float(inertia_element.find('./mass').text)
                inertia['ixx'] = float(inertia_element.find('./ixx').text)
                inertia['ixy'] = float(inertia_element.find('./ixy').text)
                inertia['ixz'] = float(inertia_element.find('./ixz').text)
                inertia['iyy'] = float(inertia_element.find('./iyy').text)
                inertia['iyz'] = float(inertia_element.find('./iyz').text)
                inertia['izz'] = float(inertia_element.find('./izz').text)

            else:
                print("\033[1;91m[Warn] No <inertia> tag found in the model file.\033[0m")
                return None, None, None

            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_sdf(model_name="doubletag", model_xml=model_sdf, robot_namespace="beetle", initial_pose=initial_pose, reference_frame="world")
            rospy.loginfo("\033[1;32m[Msg] Model spawned successfully.\033[0m")

        except rospy.ServiceException as e:
            rospy.logerr("\033[1;91m[Warn] Spawn service call failed: %s\033[0m" % e)
            return None, None, None
        
    def callback1(self, data):
        if data.detections:
            detected_ids = [detection.id[0] for detection in data.detections]
            detected_positions = {detection.id[0]: detection.pose.pose.pose.position for detection in data.detections}
            detected_orientations = {detection.id[0]: detection.pose.pose.pose.orientation for detection in data.detections}

            if 0 in detected_ids:
                if self.id0_detected_start_time is None:
                    self.id0_detected_start_time = time.time()
                elif time.time() - self.id0_detected_start_time >= 3.0:
                    rospy.loginfo("\033[1;32m[Msg] Using id[0] for positioning.\033[0m")
                    self.set_target_distance(detected_positions, detected_orientations, 0)
                    self.targetPosition = self.smalltag_target_distance
                    return 

            if 1 in detected_ids:
                rospy.loginfo("\033[1;32m[Msg] Using id[1] for positioning.\033[0m")
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

    def main(self):
        rospy.sleep(1)

        self.spawnModel()
        
        while not rospy.is_shutdown():

            # Error calculating
            self.calculateError()
            # rospy.loginfo(f"Current distance: {self.currentDistance}")

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
