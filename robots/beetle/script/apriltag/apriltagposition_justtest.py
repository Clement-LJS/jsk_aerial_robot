#!/usr/bin/env python
import rospy, tf, math
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose

class AprilPIDController:
    def __init__(self):
        rospy.init_node("aprilpid_distance_angle_printer")

        # Variables to store the current position and orientation
        self.current_distance = np.array([0.0, 0.0, 0.0])
        self.twist_current_euler = np.array([0.0, 0.0, 0.0])

        # Subscriber to the /beetle1/tag_detections_image topic
        self.sub = rospy.Subscriber('/beetle1/tag_detections', AprilTagDetectionArray, self.callback)

        # ROS rate
        self.rate = rospy.Rate(10)  # 10 Hz

    def euler_from_quaternion(self, orientation):
        """Convert quaternion to Euler angles."""
        return tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )

    def callback(self, data):
        if data.detections:
            # Extract detected tag IDs
            detected_ids = [detection.id[0] for detection in data.detections]
            detected_positions = {detection.id[0]: detection.pose.pose.pose.position for detection in data.detections}
            detected_orientations = {detection.id[0]: detection.pose.pose.pose.orientation for detection in data.detections}

            if 1 in detected_ids:  # Check if id[1] is detected
                # Get the position and orientation of id[1]
                position = detected_positions[1]
                orientation = detected_orientations[1]

                # Update current distance and orientation
                self.current_distance = np.array([position.z, -position.x, -position.y])
                current_euler = self.euler_from_quaternion(orientation)
                
                if current_euler[0] >= 0.0:
                    self.twist_current_euler = np.array([current_euler[2], np.pi - current_euler[0], -current_euler[1]])
                else:
                    self.twist_current_euler = np.array([current_euler[2], -np.pi - current_euler[0], -current_euler[1]])

                # Print current distance and orientation
                yawchecker = self.current_distance[0] / self.current_distance[1]

                if yawchecker >= 0:
                    self.yaw = math.degrees(math.pi/2 - math.atan(yawchecker))
                else:
                    self.yaw = math.degrees(-math.pi/2 - math.atan(yawchecker))
                    
                print(f"{self.current_distance[1]}")
                print(f"{self.yaw}")
                
            else:
                rospy.loginfo("Tag ID 1 not detected.")
        else:
            rospy.loginfo("No tags detected.")

    def main(self):
        rospy.loginfo("Starting AprilPIDController node...")
        rospy.spin()  # Keep the node running

if __name__ == "__main__":
    try:
        april_pid = AprilPIDController()
        april_pid.main()
    except rospy.ROSInterruptException:
        pass
