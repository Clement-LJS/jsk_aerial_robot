#!/usr/bin/env python3

import math
import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

from tf.transformations import (
    euler_from_quaternion,
    quaternion_inverse,
    quaternion_multiply,
)


class PerchBranch(object):
    def __init__(self):
        rospy.init_node("perch_branch", anonymous=False)

        # Branch / pipe mocap pose.
        # This comes from mocap.launch robot_id:=2.
        self.branch_pose_topic = rospy.get_param(
            "~branch_pose_topic",
            "/mocap/pose"
        )

        # Gimbalrotor current odometry.
        self.gimbal_odom_topic = rospy.get_param(
            "~gimbal_odom_topic",
            "/gimbalrotor/uav/baselink/odom"
        )

        self.print_rate = rospy.get_param("~print_rate", 5.0)

        self.branch_pose_msg = None
        self.gimbal_odom_msg = None
        self.last_print_time = rospy.Time.now()

        # Subscribers
        self.branch_pose_sub = rospy.Subscriber(
            self.branch_pose_topic,
            PoseStamped,
            self.branchPoseCallback,
            queue_size=1
        )

        self.gimbal_odom_sub = rospy.Subscriber(
            self.gimbal_odom_topic,
            Odometry,
            self.gimbalOdomCallback,
            queue_size=1
        )

        # Publishers
        self.branch_position_world_pub = rospy.Publisher(
            "~branch_position_world",
            Vector3Stamped,
            queue_size=1
        )

        self.gimbal_position_world_pub = rospy.Publisher(
            "~gimbal_position_world",
            Vector3Stamped,
            queue_size=1
        )

        self.position_difference_world_pub = rospy.Publisher(
            "~position_difference_world",
            Vector3Stamped,
            queue_size=1
        )

        self.branch_rpy_world_pub = rospy.Publisher(
            "~branch_rpy_world",
            Vector3Stamped,
            queue_size=1
        )

        self.gimbal_rpy_world_pub = rospy.Publisher(
            "~gimbal_rpy_world",
            Vector3Stamped,
            queue_size=1
        )

        self.attitude_difference_rpy_pub = rospy.Publisher(
            "~attitude_difference_rpy",
            Vector3Stamped,
            queue_size=1
        )

        rospy.logwarn("[perch_branch] initialized")
        rospy.logwarn("[perch_branch] branch pose topic: %s", self.branch_pose_topic)
        rospy.logwarn("[perch_branch] gimbal odom topic: %s", self.gimbal_odom_topic)

    def branchPoseCallback(self, msg):
        self.branch_pose_msg = msg
        self.calculate()

    def gimbalOdomCallback(self, msg):
        self.gimbal_odom_msg = msg
        self.calculate()

    def calculate(self):
        if self.branch_pose_msg is None:
            return

        if self.gimbal_odom_msg is None:
            return

        now = rospy.Time.now()

        # ============================================================
        # 1. Branch pose wrt world
        # ============================================================

        branch_pose = self.branch_pose_msg.pose

        branch_x = branch_pose.position.x
        branch_y = branch_pose.position.y
        branch_z = branch_pose.position.z

        q_branch = [
            branch_pose.orientation.x,
            branch_pose.orientation.y,
            branch_pose.orientation.z,
            branch_pose.orientation.w
        ]

        branch_roll, branch_pitch, branch_yaw = euler_from_quaternion(q_branch)

        # Distance from world origin to branch
        branch_distance_from_world_origin = math.sqrt(
            branch_x * branch_x +
            branch_y * branch_y +
            branch_z * branch_z
        )

        # ============================================================
        # 2. Gimbalrotor pose wrt world
        # ============================================================

        gimbal_pose = self.gimbal_odom_msg.pose.pose

        gimbal_x = gimbal_pose.position.x
        gimbal_y = gimbal_pose.position.y
        gimbal_z = gimbal_pose.position.z

        q_gimbal = [
            gimbal_pose.orientation.x,
            gimbal_pose.orientation.y,
            gimbal_pose.orientation.z,
            gimbal_pose.orientation.w
        ]

        gimbal_roll, gimbal_pitch, gimbal_yaw = euler_from_quaternion(q_gimbal)

        # Distance from world origin to gimbalrotor
        gimbal_distance_from_world_origin = math.sqrt(
            gimbal_x * gimbal_x +
            gimbal_y * gimbal_y +
            gimbal_z * gimbal_z
        )

        # ============================================================
        # 3. Position difference in world frame
        # ============================================================

        diff_x = branch_x - gimbal_x
        diff_y = branch_y - gimbal_y
        diff_z = branch_z - gimbal_z

        distance_gimbal_to_branch = math.sqrt(
            diff_x * diff_x +
            diff_y * diff_y +
            diff_z * diff_z
        )

        # Meaning:
        #
        #   [diff_x, diff_y, diff_z]
        #
        # is the vector from gimbalrotor to branch,
        # expressed in world frame.

        # ============================================================
        # 4. Attitude difference
        # ============================================================

        # Quaternion relative attitude:
        #
        #   q_relative = inverse(q_gimbal_world) * q_branch_world
        #
        # Meaning:
        #   how much the gimbal attitude must rotate to match branch attitude.
        q_relative = quaternion_multiply(
            quaternion_inverse(q_gimbal),
            q_branch
        )

        rel_roll, rel_pitch, rel_yaw = euler_from_quaternion(q_relative)

        # Simple Euler subtraction.
        #
        # This is easier to understand for debugging, but less mathematically
        # clean than quaternion relative attitude.
        simple_diff_roll = branch_roll - gimbal_roll
        simple_diff_pitch = branch_pitch - gimbal_pitch
        simple_diff_yaw = branch_yaw - gimbal_yaw

        # ============================================================
        # 5. Publish branch position wrt world
        # ============================================================

        branch_pos_msg = Vector3Stamped()
        branch_pos_msg.header.stamp = now
        branch_pos_msg.header.frame_id = "world"
        branch_pos_msg.vector.x = branch_x
        branch_pos_msg.vector.y = branch_y
        branch_pos_msg.vector.z = branch_z
        self.branch_position_world_pub.publish(branch_pos_msg)

        # ============================================================
        # 6. Publish gimbal position wrt world
        # ============================================================

        gimbal_pos_msg = Vector3Stamped()
        gimbal_pos_msg.header.stamp = now
        gimbal_pos_msg.header.frame_id = "world"
        gimbal_pos_msg.vector.x = gimbal_x
        gimbal_pos_msg.vector.y = gimbal_y
        gimbal_pos_msg.vector.z = gimbal_z
        self.gimbal_position_world_pub.publish(gimbal_pos_msg)

        # ============================================================
        # 7. Publish position difference wrt world
        # ============================================================

        diff_pos_msg = Vector3Stamped()
        diff_pos_msg.header.stamp = now
        diff_pos_msg.header.frame_id = "world"
        diff_pos_msg.vector.x = diff_x
        diff_pos_msg.vector.y = diff_y
        diff_pos_msg.vector.z = diff_z
        self.position_difference_world_pub.publish(diff_pos_msg)

        # ============================================================
        # 8. Publish branch attitude wrt world
        # ============================================================

        branch_rpy_msg = Vector3Stamped()
        branch_rpy_msg.header.stamp = now
        branch_rpy_msg.header.frame_id = "world"
        branch_rpy_msg.vector.x = branch_roll
        branch_rpy_msg.vector.y = branch_pitch
        branch_rpy_msg.vector.z = branch_yaw
        self.branch_rpy_world_pub.publish(branch_rpy_msg)

        # ============================================================
        # 9. Publish gimbal attitude wrt world
        # ============================================================

        gimbal_rpy_msg = Vector3Stamped()
        gimbal_rpy_msg.header.stamp = now
        gimbal_rpy_msg.header.frame_id = "world"
        gimbal_rpy_msg.vector.x = gimbal_roll
        gimbal_rpy_msg.vector.y = gimbal_pitch
        gimbal_rpy_msg.vector.z = gimbal_yaw
        self.gimbal_rpy_world_pub.publish(gimbal_rpy_msg)

        # ============================================================
        # 10. Publish attitude difference
        # ============================================================

        attitude_diff_msg = Vector3Stamped()
        attitude_diff_msg.header.stamp = now
        attitude_diff_msg.header.frame_id = "gimbal_to_branch"

        # Use quaternion-based relative RPY.
        attitude_diff_msg.vector.x = rel_roll
        attitude_diff_msg.vector.y = rel_pitch
        attitude_diff_msg.vector.z = rel_yaw

        self.attitude_difference_rpy_pub.publish(attitude_diff_msg)

        # ============================================================
        # 11. Print debug
        # ============================================================

        if (now - self.last_print_time).to_sec() > 1.0 / self.print_rate:
            self.last_print_time = now

            rospy.loginfo(
                "\n[perch_branch]"
                "\n"
                "\n  ========== branch pose wrt world =========="
                "\n  branch position:"
                "\n    x: %.4f"
                "\n    y: %.4f"
                "\n    z: %.4f"
                "\n  branch distance from world origin:"
                "\n    %.4f m"
                "\n  branch attitude wrt world:"
                "\n    roll : %.4f rad / %.2f deg"
                "\n    pitch: %.4f rad / %.2f deg"
                "\n    yaw  : %.4f rad / %.2f deg"
                "\n"
                "\n  ========== gimbalrotor pose wrt world =========="
                "\n  gimbal position:"
                "\n    x: %.4f"
                "\n    y: %.4f"
                "\n    z: %.4f"
                "\n  gimbal distance from world origin:"
                "\n    %.4f m"
                "\n  gimbal attitude wrt world:"
                "\n    roll : %.4f rad / %.2f deg"
                "\n    pitch: %.4f rad / %.2f deg"
                "\n    yaw  : %.4f rad / %.2f deg"
                "\n"
                "\n  ========== gimbalrotor to branch difference =========="
                "\n  position difference, branch - gimbal, wrt world:"
                "\n    x: %.4f"
                "\n    y: %.4f"
                "\n    z: %.4f"
                "\n  distance from gimbalrotor to branch:"
                "\n    %.4f m"
                "\n"
                "\n  attitude difference, quaternion relative:"
                "\n    roll : %.4f rad / %.2f deg"
                "\n    pitch: %.4f rad / %.2f deg"
                "\n    yaw  : %.4f rad / %.2f deg"
                "\n"
                "\n  attitude difference, simple branch_rpy - gimbal_rpy:"
                "\n    roll : %.4f rad / %.2f deg"
                "\n    pitch: %.4f rad / %.2f deg"
                "\n    yaw  : %.4f rad / %.2f deg",
                branch_x,
                branch_y,
                branch_z,
                branch_distance_from_world_origin,
                branch_roll,
                math.degrees(branch_roll),
                branch_pitch,
                math.degrees(branch_pitch),
                branch_yaw,
                math.degrees(branch_yaw),
                gimbal_x,
                gimbal_y,
                gimbal_z,
                gimbal_distance_from_world_origin,
                gimbal_roll,
                math.degrees(gimbal_roll),
                gimbal_pitch,
                math.degrees(gimbal_pitch),
                gimbal_yaw,
                math.degrees(gimbal_yaw),
                diff_x,
                diff_y,
                diff_z,
                distance_gimbal_to_branch,
                rel_roll,
                math.degrees(rel_roll),
                rel_pitch,
                math.degrees(rel_pitch),
                rel_yaw,
                math.degrees(rel_yaw),
                simple_diff_roll,
                math.degrees(simple_diff_roll),
                simple_diff_pitch,
                math.degrees(simple_diff_pitch),
                simple_diff_yaw,
                math.degrees(simple_diff_yaw),
            )


if __name__ == "__main__":
    try:
        node = PerchBranch()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
