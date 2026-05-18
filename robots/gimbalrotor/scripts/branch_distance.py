#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

from tf.transformations import (
    quaternion_inverse,
    quaternion_multiply,
    euler_from_quaternion,
)


class BranchDistance(object):
    def __init__(self):
        rospy.init_node("perch_branch", anonymous=False)

        # ===============================
        # Parameters
        # ===============================
        self.branch_pose_topic = rospy.get_param(
            "~branch_pose_topic",
            "/mocap/pose"
        )

        self.gimbal_odom_topic = rospy.get_param(
            "~gimbal_odom_topic",
            "/gimbalrotor/uav/baselink/odom"
        )

        self.print_rate = rospy.get_param("~print_rate", 5.0)

        # ===============================
        # Internal storage
        # ===============================
        self.branch_pose_msg = None
        self.gimbal_odom_msg = None

        self.last_print_time = rospy.Time.now()

        # ===============================
        # Subscribers
        # ===============================
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

        # ===============================
        # Publishers
        # ===============================
        self.diff_world_pub = rospy.Publisher(
            "~diff_position_world",
            Vector3Stamped,
            queue_size=1
        )

        self.diff_gimbal_frame_pub = rospy.Publisher(
            "~diff_position_gimbal_frame",
            Vector3Stamped,
            queue_size=1
        )

        self.relative_pose_pub = rospy.Publisher(
            "~relative_pose_gimbal_to_branch",
            PoseStamped,
            queue_size=1
        )

        rospy.logwarn("[perch_branch] initialized")
        rospy.logwarn("[perch_branch] branch mocap topic: %s", self.branch_pose_topic)
        rospy.logwarn("[perch_branch] gimbal odom topic: %s", self.gimbal_odom_topic)

    # ============================================================
    # Branch mocap callback
    # This is the branch rigid body pose from OptiTrack.
    # If mocap.launch uses robot_id:=2, then /mocap/pose is branch ID 2.
    # ============================================================
    def branchPoseCallback(self, msg):
        self.branch_pose_msg = msg
        self.calculateRelativePose()

    # ============================================================
    # Gimbal odom callback
    # This is the current gimbalrotor position/orientation.
    # ============================================================
    def gimbalOdomCallback(self, msg):
        self.gimbal_odom_msg = msg
        self.calculateRelativePose()

    # ============================================================
    # Main calculation
    # ============================================================
    def calculateRelativePose(self):
        if self.branch_pose_msg is None:
            return

        if self.gimbal_odom_msg is None:
            return

        branch = self.branch_pose_msg.pose
        gimbal = self.gimbal_odom_msg.pose.pose

        # ------------------------------------------------------------
        # 1. Position of branch in world frame
        # ------------------------------------------------------------
        bx = branch.position.x
        by = branch.position.y
        bz = branch.position.z

        # ------------------------------------------------------------
        # 2. Position of gimbalrotor in world frame
        # ------------------------------------------------------------
        gx = gimbal.position.x
        gy = gimbal.position.y
        gz = gimbal.position.z

        # ------------------------------------------------------------
        # 3. Position difference in world frame
        #
        # Meaning:
        #   diff_world = branch_position - gimbal_position
        #
        # If diff_world.x is positive:
        #   branch is in front of gimbal in world x direction
        # ------------------------------------------------------------
        dx_w = bx - gx
        dy_w = by - gy
        dz_w = bz - gz

        distance = math.sqrt(dx_w * dx_w + dy_w * dy_w + dz_w * dz_w)

        # ------------------------------------------------------------
        # 4. Orientation of branch and gimbal
        # ------------------------------------------------------------
        q_branch = [
            branch.orientation.x,
            branch.orientation.y,
            branch.orientation.z,
            branch.orientation.w
        ]

        q_gimbal = [
            gimbal.orientation.x,
            gimbal.orientation.y,
            gimbal.orientation.z,
            gimbal.orientation.w
        ]

        # ------------------------------------------------------------
        # 5. Relative orientation from gimbal to branch
        #
        # q_relative = inverse(q_gimbal) * q_branch
        #
        # Meaning:
        #   orientation of branch seen from gimbal orientation
        # ------------------------------------------------------------
        q_gimbal_inv = quaternion_inverse(q_gimbal)
        q_relative = quaternion_multiply(q_gimbal_inv, q_branch)

        rel_roll, rel_pitch, rel_yaw = euler_from_quaternion(q_relative)

        # ------------------------------------------------------------
        # 6. Position difference expressed in gimbal frame
        #
        # First we have diff in world frame:
        #   p_branch - p_gimbal
        #
        # To express this vector in gimbal body frame:
        #   p_diff_gimbal = inverse(q_gimbal) rotate p_diff_world
        # ------------------------------------------------------------
        diff_quat_world = [dx_w, dy_w, dz_w, 0.0]

        temp = quaternion_multiply(q_gimbal_inv, diff_quat_world)
        diff_quat_gimbal = quaternion_multiply(temp, q_gimbal)

        dx_g = diff_quat_gimbal[0]
        dy_g = diff_quat_gimbal[1]
        dz_g = diff_quat_gimbal[2]

        # ------------------------------------------------------------
        # 7. Publish world-frame position difference
        # ------------------------------------------------------------
        diff_world_msg = Vector3Stamped()
        diff_world_msg.header.stamp = rospy.Time.now()
        diff_world_msg.header.frame_id = "world"
        diff_world_msg.vector.x = dx_w
        diff_world_msg.vector.y = dy_w
        diff_world_msg.vector.z = dz_w
        self.diff_world_pub.publish(diff_world_msg)

        # ------------------------------------------------------------
        # 8. Publish gimbal-frame position difference
        # ------------------------------------------------------------
        diff_gimbal_msg = Vector3Stamped()
        diff_gimbal_msg.header.stamp = rospy.Time.now()
        diff_gimbal_msg.header.frame_id = "baselink"
        diff_gimbal_msg.vector.x = dx_g
        diff_gimbal_msg.vector.y = dy_g
        diff_gimbal_msg.vector.z = dz_g
        self.diff_gimbal_frame_pub.publish(diff_gimbal_msg)

        # ------------------------------------------------------------
        # 9. Publish relative pose from gimbal to branch
        # ------------------------------------------------------------
        relative_pose_msg = PoseStamped()
        relative_pose_msg.header.stamp = rospy.Time.now()
        relative_pose_msg.header.frame_id = "gimbal_to_branch"

        # Position difference in gimbal frame is usually more useful
        # for control/planning, so put gimbal-frame difference here.
        relative_pose_msg.pose.position.x = dx_g
        relative_pose_msg.pose.position.y = dy_g
        relative_pose_msg.pose.position.z = dz_g

        relative_pose_msg.pose.orientation.x = q_relative[0]
        relative_pose_msg.pose.orientation.y = q_relative[1]
        relative_pose_msg.pose.orientation.z = q_relative[2]
        relative_pose_msg.pose.orientation.w = q_relative[3]

        self.relative_pose_pub.publish(relative_pose_msg)

        # ------------------------------------------------------------
        # 10. Print debug info
        # ------------------------------------------------------------
        now = rospy.Time.now()

        if (now - self.last_print_time).to_sec() > 1.0 / self.print_rate:
            self.last_print_time = now

            rospy.loginfo(
                "\n[perch_branch] gimbal to branch"
                "\n  branch position world:"
                "\n    x: %.4f, y: %.4f, z: %.4f"
                "\n  gimbal position world:"
                "\n    x: %.4f, y: %.4f, z: %.4f"
                "\n  diff position world: branch - gimbal"
                "\n    x: %.4f, y: %.4f, z: %.4f"
                "\n  diff position gimbal frame:"
                "\n    x: %.4f, y: %.4f, z: %.4f"
                "\n  distance:"
                "\n    %.4f m"
                "\n  relative orientation gimbal -> branch:"
                "\n    roll : %.3f rad / %.2f deg"
                "\n    pitch: %.3f rad / %.2f deg"
                "\n    yaw  : %.3f rad / %.2f deg",
                bx, by, bz,
                gx, gy, gz,
                dx_w, dy_w, dz_w,
                dx_g, dy_g, dz_g,
                distance,
                rel_roll, math.degrees(rel_roll),
                rel_pitch, math.degrees(rel_pitch),
                rel_yaw, math.degrees(rel_yaw)
            )


if __name__ == "__main__":
    try:
        node = BranchDistance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
