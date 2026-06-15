#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy

from std_msgs.msg import Bool, Empty, Float64
from geometry_msgs.msg import PoseStamped, PointStamped
from aerial_robot_msgs.msg import FlightNav


NAV_MODE_NONE = 0
NAV_MODE_VEL = 1
NAV_MODE_POS = 2
NAV_MODE_POS_VEL = 3


class PerchingCuttingMission:
    def __init__(self):
        self.ns = rospy.get_param("~robot_namespace", "/gimbalrotor").rstrip("/")

        self.branch_pose_topic = rospy.get_param(
            "~branch_pose_topic",
            "/mocap/pose"
        )

        self.use_branch_pose_as_perching_point = rospy.get_param(
            "~use_branch_pose_as_perching_point",
            False
        )

        self.perching_point_offset_x = rospy.get_param(
            "~perching_point_offset_x",
            0.0
        )
        self.perching_point_offset_y = rospy.get_param(
            "~perching_point_offset_y",
            0.0
        )
        self.perching_point_offset_z = rospy.get_param(
            "~perching_point_offset_z",
            0.0
        )

        self.command_rate = rospy.get_param("~command_rate", 20.0)

        self.default_pitch_deg = rospy.get_param("~default_pitch_deg", 0.0)
        self.pitch_limit_deg = rospy.get_param("~pitch_limit_deg", 30.0)

        self.current_target_pitch_rad = math.radians(self.default_pitch_deg)

        self.enabled = False
        self.latest_branch_pose = None

        self.branch_pose_sub = rospy.Subscriber(
            self.branch_pose_topic,
            PoseStamped,
            self.branch_pose_callback,
            queue_size=1
        )

        self.enable_sub = rospy.Subscriber(
            self.ns + "/perching_mission/enable",
            Bool,
            self.enable_callback,
            queue_size=1
        )

        self.pitch_deg_sub = rospy.Subscriber(
            self.ns + "/perching_mission/target_pitch_deg",
            Float64,
            self.target_pitch_deg_callback,
            queue_size=1
        )

        self.pitch_delta_deg_sub = rospy.Subscriber(
            self.ns + "/perching_mission/add_pitch_deg",
            Float64,
            self.add_pitch_deg_callback,
            queue_size=1
        )

        self.relock_sub = rospy.Subscriber(
            self.ns + "/perching_mission/relock",
            Empty,
            self.relock_callback,
            queue_size=1
        )

        self.reset_sub = rospy.Subscriber(
            self.ns + "/perching_mission/reset",
            Empty,
            self.reset_callback,
            queue_size=1
        )

        self.perching_enable_pub = rospy.Publisher(
            self.ns + "/perching/enable",
            Bool,
            queue_size=1,
            latch=True
        )

        self.branch_pose_pub = rospy.Publisher(
            self.ns + "/perching/branch_pose",
            PoseStamped,
            queue_size=1,
            latch=True
        )

        self.perching_point_pub = rospy.Publisher(
            self.ns + "/perching/point",
            PointStamped,
            queue_size=1,
            latch=True
        )

        self.perching_relock_pub = rospy.Publisher(
            self.ns + "/perching/relock",
            Empty,
            queue_size=1
        )

        self.perching_reset_pub = rospy.Publisher(
            self.ns + "/perching/reset",
            Empty,
            queue_size=1
        )

        self.nav_pub = rospy.Publisher(
            self.ns + "/uav/nav",
            FlightNav,
            queue_size=1
        )

        rospy.Timer(
            rospy.Duration(1.0 / self.command_rate),
            self.timer_callback
        )

        rospy.logwarn("[PerchingCuttingMission] initialized")
        rospy.logwarn("[PerchingCuttingMission] branch_pose_topic: %s", self.branch_pose_topic)
        rospy.logwarn("[PerchingCuttingMission] use_branch_pose_as_perching_point: %s",
                      str(self.use_branch_pose_as_perching_point))

    def branch_pose_callback(self, msg):
        self.latest_branch_pose = msg

        # Always republish branch mocap as reference information.
        # The perching navigator can use this as existence/reference,
        # but the hand-center pivot calculation does not use this origin.
        self.branch_pose_pub.publish(msg)

        # Legacy mode only.
        # Keep this option for debugging old experiments, but default is false.
        if not self.use_branch_pose_as_perching_point:
            return

        point_msg = PointStamped()
        point_msg.header = msg.header
        point_msg.point.x = msg.pose.position.x + self.perching_point_offset_x
        point_msg.point.y = msg.pose.position.y + self.perching_point_offset_y
        point_msg.point.z = msg.pose.position.z + self.perching_point_offset_z

        self.perching_point_pub.publish(point_msg)

    def enable_callback(self, msg):
        self.enabled = msg.data

        self.perching_enable_pub.publish(Bool(data=self.enabled))

        if self.enabled:
            rospy.logwarn("[PerchingCuttingMission] ENABLED")
            rospy.logwarn("[PerchingCuttingMission] target pitch: %.2f deg",
                          math.degrees(self.current_target_pitch_rad))
        else:
            rospy.logwarn("[PerchingCuttingMission] DISABLED")

    def target_pitch_deg_callback(self, msg):
        pitch_deg = self.clamp(
            msg.data,
            -self.pitch_limit_deg,
            self.pitch_limit_deg
        )

        self.current_target_pitch_rad = math.radians(pitch_deg)

        rospy.logwarn("[PerchingCuttingMission] target pitch set: %.2f deg", pitch_deg)

    def add_pitch_deg_callback(self, msg):
        current_deg = math.degrees(self.current_target_pitch_rad)
        target_deg = current_deg + msg.data

        target_deg = self.clamp(
            target_deg,
            -self.pitch_limit_deg,
            self.pitch_limit_deg
        )

        self.current_target_pitch_rad = math.radians(target_deg)

        rospy.logwarn("[PerchingCuttingMission] target pitch changed to: %.2f deg", target_deg)

    def relock_callback(self, _msg):
        rospy.logwarn("[PerchingCuttingMission] relock requested")
        self.perching_relock_pub.publish(Empty())

    def reset_callback(self, _msg):
        rospy.logwarn("[PerchingCuttingMission] reset requested")
        self.enabled = False
        self.perching_enable_pub.publish(Bool(data=False))
        self.perching_reset_pub.publish(Empty())

    def timer_callback(self, _event):
        if not self.enabled:
            return

        nav_msg = FlightNav()

        nav_msg.pos_xy_nav_mode = NAV_MODE_NONE
        nav_msg.pos_z_nav_mode = NAV_MODE_NONE

        nav_msg.target_vel_x = 0.0
        nav_msg.target_vel_y = 0.0
        nav_msg.target_vel_z = 0.0

        nav_msg.roll_nav_mode = NAV_MODE_NONE
        nav_msg.pitch_nav_mode = NAV_MODE_POS
        nav_msg.yaw_nav_mode = NAV_MODE_NONE

        nav_msg.target_pitch = self.current_target_pitch_rad

        self.nav_pub.publish(nav_msg)

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(max_value, value))


def main():
    rospy.init_node("perching_cutting_mission")
    PerchingCuttingMission()
    rospy.spin()


if __name__ == "__main__":
    main()