#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


def get_param_first(keys):
    for key in keys:
        if rospy.has_param(key):
            return rospy.get_param(key)
    raise rospy.ROSInitException(
        "Missing perching geometry parameter. Load NavigationConfig.yaml first "
        "or set one of: %s" % ", ".join(keys)
    )


def make_point(x, y, z):
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def resolve_topic(robot_namespace, topic_name):
    """
    If topic_name starts with '/', use it directly.
    Otherwise, attach robot namespace.

    Example:
      robot_namespace = /gimbalrotor
      topic_name      = perching/locked_pivot
      result          = /gimbalrotor/perching/locked_pivot
    """
    if topic_name.startswith("/"):
        return topic_name

    return robot_namespace.rstrip("/") + "/" + topic_name.lstrip("/")


class HandPerchingCenterVisualizer:
    def __init__(self):
        self.world_frame = rospy.get_param("~world_frame", "world")

        # If this is empty, the node tries common base_link frame names.
        self.base_frame = rospy.get_param("~base_frame", "")

        self.child_frame = rospy.get_param(
            "~child_frame",
            "hand_perching_center_live"
        )

        self.locked_child_frame = rospy.get_param(
            "~locked_child_frame",
            "hand_perching_center_locked"
        )

        self.robot_namespace = rospy.get_param(
            "~robot_namespace",
            "/gimbalrotor"
        ).rstrip("/")

        self.rate_hz = float(rospy.get_param("~rate", 20.0))

        # ------------------------------------------------------------------
        # Read the SAME parameter names used by the updated C++ navigator.
        #
        # YAML:
        #   navigation:
        #     hand_perching_center_offset_baselink_x
        #     hand_perching_center_offset_baselink_y
        #     hand_perching_center_offset_baselink_z
        #
        # Meaning:
        #   base_link origin -> physical hand perching center
        #
        # Frame:
        #   base_link
        #
        # Unit:
        #   meter
        # ------------------------------------------------------------------
        self.offset_x = float(get_param_first(
            [
                "~hand_perching_center_offset_baselink_x",
                self.robot_namespace + "/navigation/hand_perching_center_offset_baselink_x",
            ]
        ))

        self.offset_y = float(get_param_first(
            [
                "~hand_perching_center_offset_baselink_y",
                self.robot_namespace + "/navigation/hand_perching_center_offset_baselink_y",
            ]
        ))

        self.offset_z = float(get_param_first(
            [
                "~hand_perching_center_offset_baselink_z",
                self.robot_namespace + "/navigation/hand_perching_center_offset_baselink_z",
            ]
        ))

        self.offset_baselink = np.array(
            [self.offset_x, self.offset_y, self.offset_z],
            dtype=float
        )

        # ------------------------------------------------------------------
        # Correct locked pivot topic.
        #
        # C++ publishes this after perching lock:
        #   locked_pivot_pub_ = nh_.advertise<PointStamped>(locked_pivot_topic_, ...)
        #
        # Default resolved topic:
        #   /gimbalrotor/perching/locked_pivot
        # ------------------------------------------------------------------
        locked_pivot_topic_param = rospy.get_param(
            "~locked_pivot_topic",
            rospy.get_param(
                self.robot_namespace + "/navigation/perching_locked_pivot_topic",
                "perching/locked_pivot"
            )
        )

        self.locked_pivot_topic = resolve_topic(
            self.robot_namespace,
            str(locked_pivot_topic_param)
        )

        self.marker_topic = rospy.get_param(
            "~marker_topic",
            self.robot_namespace + "/perching/hand_center_markers"
        )

        self.live_point_topic = rospy.get_param(
            "~live_point_topic",
            self.robot_namespace + "/perching/hand_center_live_point"
        )

        self.live_pose_topic = rospy.get_param(
            "~live_pose_topic",
            self.robot_namespace + "/perching/hand_center_live_pose"
        )

        self.locked_point_topic = rospy.get_param(
            "~locked_point_topic",
            self.robot_namespace + "/perching/hand_center_locked_point"
        )

        self.marker_pub = rospy.Publisher(
            self.marker_topic,
            MarkerArray,
            queue_size=1
        )

        self.live_point_pub = rospy.Publisher(
            self.live_point_topic,
            PointStamped,
            queue_size=1
        )

        self.live_pose_pub = rospy.Publisher(
            self.live_pose_topic,
            PoseStamped,
            queue_size=1
        )

        self.locked_point_pub = rospy.Publisher(
            self.locked_point_topic,
            PointStamped,
            queue_size=1
        )

        self.locked_pivot_sub = rospy.Subscriber(
            self.locked_pivot_topic,
            PointStamped,
            self.lockedPivotCallback,
            queue_size=1
        )

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.latest_locked_pivot = None

        self.base_frame_candidates = [
            self.base_frame,
            "base_link",
            "baselink",
            "gimbalrotor/base_link",
            "gimbalrotor/baselink",
            self.robot_namespace.strip("/") + "/base_link",
            self.robot_namespace.strip("/") + "/baselink",
        ]

        # Remove empty candidates and duplicates while preserving order.
        cleaned = []
        for frame in self.base_frame_candidates:
            if frame and frame not in cleaned:
                cleaned.append(frame)
        self.base_frame_candidates = cleaned

        self.selected_base_frame = None

        rospy.logwarn("[HandPerchingCenterVisualizer] initialized")
        rospy.logwarn("[HandPerchingCenterVisualizer] world_frame: %s", self.world_frame)
        rospy.logwarn("[HandPerchingCenterVisualizer] base_frame candidates: %s",
                      str(self.base_frame_candidates))
        rospy.logwarn(
            "[HandPerchingCenterVisualizer] offset base_link -> hand center [m]: "
            "x %.5f, y %.5f, z %.5f",
            self.offset_x,
            self.offset_y,
            self.offset_z
        )
        rospy.logwarn("[HandPerchingCenterVisualizer] subscribed locked pivot topic: %s",
                      self.locked_pivot_topic)
        rospy.logwarn("[HandPerchingCenterVisualizer] marker topic: %s",
                      self.marker_topic)
        rospy.logwarn("[HandPerchingCenterVisualizer] live point topic: %s",
                      self.live_point_topic)
        rospy.logwarn("[HandPerchingCenterVisualizer] locked point topic: %s",
                      self.locked_point_topic)
        rospy.logwarn("[HandPerchingCenterVisualizer] live TF child frame: %s",
                      self.child_frame)
        rospy.logwarn("[HandPerchingCenterVisualizer] locked TF child frame: %s",
                      self.locked_child_frame)

    def lockedPivotCallback(self, msg):
        self.latest_locked_pivot = msg
        self.locked_point_pub.publish(msg)

    def find_base_frame(self):
        if self.selected_base_frame:
            return self.selected_base_frame

        for candidate in self.base_frame_candidates:
            try:
                self.tf_listener.lookupTransform(
                    self.world_frame,
                    candidate,
                    rospy.Time(0)
                )
                self.selected_base_frame = candidate
                rospy.logwarn(
                    "[HandPerchingCenterVisualizer] selected base frame: %s",
                    self.selected_base_frame
                )
                return self.selected_base_frame
            except Exception:
                pass

        rospy.logwarn_throttle(
            2.0,
            "[HandPerchingCenterVisualizer] waiting for base_link TF. "
            "Tried: %s. If this keeps happening, pass _base_frame:=YOUR_BASE_FRAME"
            % str(self.base_frame_candidates)
        )

        return None

    def compute_live_hand_center(self):
        base_frame = self.find_base_frame()
        if base_frame is None:
            return None

        try:
            trans, quat = self.tf_listener.lookupTransform(
                self.world_frame,
                base_frame,
                rospy.Time(0)
            )
        except Exception as e:
            rospy.logwarn_throttle(
                2.0,
                "[HandPerchingCenterVisualizer] TF lookup failed: %s",
                str(e)
            )
            return None

        base_pos_world = np.array(trans, dtype=float)

        rot_mat_4x4 = tf.transformations.quaternion_matrix(quat)
        rot_mat_3x3 = rot_mat_4x4[0:3, 0:3]

        hand_center_world = base_pos_world + rot_mat_3x3.dot(self.offset_baselink)

        return {
            "base_frame": base_frame,
            "base_pos_world": base_pos_world,
            "base_quat_world": quat,
            "base_rot_world": rot_mat_3x3,
            "hand_center_world": hand_center_world,
        }

    def publish_live_tf(self, data, stamp):
        p = data["hand_center_world"]
        q = data["base_quat_world"]

        self.tf_broadcaster.sendTransform(
            (float(p[0]), float(p[1]), float(p[2])),
            (float(q[0]), float(q[1]), float(q[2]), float(q[3])),
            stamp,
            self.child_frame,
            self.world_frame
        )

    def publish_locked_tf(self, stamp):
        if self.latest_locked_pivot is None:
            return

        p = self.latest_locked_pivot.point

        self.tf_broadcaster.sendTransform(
            (float(p.x), float(p.y), float(p.z)),
            (0.0, 0.0, 0.0, 1.0),
            stamp,
            self.locked_child_frame,
            self.latest_locked_pivot.header.frame_id
        )

    def publish_live_point_and_pose(self, data, stamp):
        p = data["hand_center_world"]
        q = data["base_quat_world"]

        point_msg = PointStamped()
        point_msg.header.stamp = stamp
        point_msg.header.frame_id = self.world_frame
        point_msg.point.x = float(p[0])
        point_msg.point.y = float(p[1])
        point_msg.point.z = float(p[2])
        self.live_point_pub.publish(point_msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose.position.x = float(p[0])
        pose_msg.pose.position.y = float(p[1])
        pose_msg.pose.position.z = float(p[2])
        pose_msg.pose.orientation.x = float(q[0])
        pose_msg.pose.orientation.y = float(q[1])
        pose_msg.pose.orientation.z = float(q[2])
        pose_msg.pose.orientation.w = float(q[3])
        self.live_pose_pub.publish(pose_msg)

    def make_live_sphere_marker(self, data, stamp):
        p = data["hand_center_world"]

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.world_frame
        m.ns = "hand_perching_center_live"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = float(p[0])
        m.pose.position.y = float(p[1])
        m.pose.position.z = float(p[2])
        m.pose.orientation.w = 1.0

        m.scale.x = 0.08
        m.scale.y = 0.08
        m.scale.z = 0.08

        # Green sphere = live hand center computed from base_link + offset.
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.95

        return m

    def make_locked_sphere_marker(self, stamp):
        if self.latest_locked_pivot is None:
            return None

        p = self.latest_locked_pivot.point

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.latest_locked_pivot.header.frame_id
        m.ns = "hand_perching_center_locked"
        m.id = 100
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = float(p.x)
        m.pose.position.y = float(p.y)
        m.pose.position.z = float(p.z)
        m.pose.orientation.w = 1.0

        m.scale.x = 0.11
        m.scale.y = 0.11
        m.scale.z = 0.11

        # Magenta sphere = most recently published lock snapshot.
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 0.95

        return m

    def make_base_to_hand_arrow_marker(self, data, stamp):
        base = data["base_pos_world"]
        hand = data["hand_center_world"]

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.world_frame
        m.ns = "hand_perching_center_live"
        m.id = 1
        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.points.append(make_point(base[0], base[1], base[2]))
        m.points.append(make_point(hand[0], hand[1], hand[2]))

        m.scale.x = 0.015  # shaft diameter
        m.scale.y = 0.040  # head diameter
        m.scale.z = 0.070  # head length

        # Yellow arrow = base_link to live hand center.
        m.color.r = 1.0
        m.color.g = 0.8
        m.color.b = 0.0
        m.color.a = 0.9

        return m

    def make_axis_arrow_marker(self, data, stamp, axis_index, marker_id, color):
        hand = data["hand_center_world"]
        rot = data["base_rot_world"]

        axis_length = 0.18
        axis_vec = rot[:, axis_index] * axis_length
        end = hand + axis_vec

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.world_frame
        m.ns = "hand_perching_center_live_axes"
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD

        m.points.append(make_point(hand[0], hand[1], hand[2]))
        m.points.append(make_point(end[0], end[1], end[2]))

        m.scale.x = 0.010
        m.scale.y = 0.030
        m.scale.z = 0.050

        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        m.color.a = 0.9

        return m

    def make_live_text_marker(self, data, stamp):
        p = data["hand_center_world"]
        base_frame = data["base_frame"]

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.world_frame
        m.ns = "hand_perching_center_live"
        m.id = 10
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = float(p[0])
        m.pose.position.y = float(p[1])
        m.pose.position.z = float(p[2] + 0.14)
        m.pose.orientation.w = 1.0

        m.scale.z = 0.055

        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0

        m.text = (
            "LIVE hand_perching_center\n"
            "base: %s\n"
            "world: x %.3f, y %.3f, z %.3f\n"
            "offset_baselink: x %.3f, y %.3f, z %.3f"
            % (
                base_frame,
                p[0], p[1], p[2],
                self.offset_x, self.offset_y, self.offset_z
            )
        )

        return m

    def make_locked_text_marker(self, stamp):
        if self.latest_locked_pivot is None:
            return None

        p = self.latest_locked_pivot.point

        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = self.latest_locked_pivot.header.frame_id
        m.ns = "hand_perching_center_locked"
        m.id = 110
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD

        m.pose.position.x = float(p.x)
        m.pose.position.y = float(p.y)
        m.pose.position.z = float(p.z + 0.18)
        m.pose.orientation.w = 1.0

        m.scale.z = 0.055

        m.color.r = 1.0
        m.color.g = 0.4
        m.color.b = 1.0
        m.color.a = 1.0

        m.text = (
            "LAST LOCK pivot (navigator)\n"
            "%s\n"
            "x %.3f, y %.3f, z %.3f"
            % (self.locked_pivot_topic, p.x, p.y, p.z)
        )

        return m

    def publish_markers(self, data, stamp):
        arr = MarkerArray()

        arr.markers.append(self.make_live_sphere_marker(data, stamp))
        arr.markers.append(self.make_base_to_hand_arrow_marker(data, stamp))

        # Live hand-center frame axes, using base_link orientation:
        # X = red, Y = green, Z = blue.
        arr.markers.append(self.make_axis_arrow_marker(data, stamp, 0, 2, (1.0, 0.0, 0.0)))
        arr.markers.append(self.make_axis_arrow_marker(data, stamp, 1, 3, (0.0, 1.0, 0.0)))
        arr.markers.append(self.make_axis_arrow_marker(data, stamp, 2, 4, (0.0, 0.3, 1.0)))

        arr.markers.append(self.make_live_text_marker(data, stamp))

        locked_sphere = self.make_locked_sphere_marker(stamp)
        if locked_sphere is not None:
            arr.markers.append(locked_sphere)

        locked_text = self.make_locked_text_marker(stamp)
        if locked_text is not None:
            arr.markers.append(locked_text)

        self.marker_pub.publish(arr)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)

        while not rospy.is_shutdown():
            stamp = rospy.Time.now()

            data = self.compute_live_hand_center()

            if data is not None:
                self.publish_live_tf(data, stamp)
                self.publish_locked_tf(stamp)
                self.publish_live_point_and_pose(data, stamp)
                self.publish_markers(data, stamp)

            rate.sleep()


def main():
    rospy.init_node("visualize_hand_perching_center")

    node = HandPerchingCenterVisualizer()
    node.spin()


if __name__ == "__main__":
    main()
