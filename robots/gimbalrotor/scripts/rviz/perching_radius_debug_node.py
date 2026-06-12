#!/usr/bin/env python3

import math
import rospy
import tf.transformations as tft

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Empty
from visualization_msgs.msg import Marker, MarkerArray


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_mul(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_norm(a):
    return math.sqrt(vec_dot(a, a))


def vec_normalize(a, fallback=None):
    n = vec_norm(a)
    if n < 1e-9:
        if fallback is None:
            return [1.0, 0.0, 0.0]
        return fallback
    return [a[0] / n, a[1] / n, a[2] / n]


def vec_cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def point_msg(v):
    p = Point()
    p.x = v[0]
    p.y = v[1]
    p.z = v[2]
    return p


def quat_to_rot_matrix(q):
    """
    q: geometry_msgs/Quaternion
    returns 3x3 rotation matrix.
    """
    mat = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    return [
        [mat[0][0], mat[0][1], mat[0][2]],
        [mat[1][0], mat[1][1], mat[1][2]],
        [mat[2][0], mat[2][1], mat[2][2]],
    ]


def mat_vec_mul(R, v):
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


class PerchingRadiusDebugNode:
    def __init__(self):
        self.branch_pose_topic = rospy.get_param("~branch_pose_topic", "/mocap/pose")
        self.robot_odom_topic = rospy.get_param("~robot_odom_topic", "/gimbalrotor/uav/baselink/odom")

        self.fixed_frame = rospy.get_param("~fixed_frame", "world")
        self.debug_ns = rospy.get_param("~debug_ns", "/gimbalrotor/perching_debug").rstrip("/")

        # Hand/perching point offset from robot base_link.
        # Default is your previously discussed hand pivot:
        # x = 0.19178 + 0.3875 = 0.57928 m
        self.hand_offset_base = [
            rospy.get_param("~hand_offset_x", 0.57928),
            rospy.get_param("~hand_offset_y", 0.0),
            rospy.get_param("~hand_offset_z", 0.0),
        ]

        # If target_radius <= 0 and auto_set_target_radius is true,
        # the node will use the current measured radius as target.
        self.target_radius = rospy.get_param("~target_radius", 0.0)
        self.auto_set_target_radius = rospy.get_param("~auto_set_target_radius", True)
        self.target_radius_locked = self.target_radius > 1e-6

        # Visualization parameters.
        self.branch_axis_length = rospy.get_param("~branch_axis_length", 2.0)
        self.circle_points = rospy.get_param("~circle_points", 96)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.20)
        self.publish_rate = rospy.get_param("~publish_rate", 30.0)

        self.branch_pose = None
        self.robot_odom = None
        self.last_measured_radius = None

        self.warned_frame_mismatch = False

        rospy.Subscriber(self.branch_pose_topic, PoseStamped, self.branch_pose_cb, queue_size=1)
        rospy.Subscriber(self.robot_odom_topic, Odometry, self.robot_odom_cb, queue_size=1)

        # Publish this once when you want to set the current radius as the target radius:
        # rostopic pub -1 /gimbalrotor/perching_debug/set_target_from_current std_msgs/Empty "{}"
        rospy.Subscriber(self.debug_ns + "/set_target_from_current", Empty, self.set_target_from_current_cb, queue_size=1)

        self.marker_pub = rospy.Publisher(self.debug_ns + "/markers", MarkerArray, queue_size=1)

        self.measured_radius_pub = rospy.Publisher(self.debug_ns + "/measured_radius", Float64, queue_size=1)
        self.target_radius_pub = rospy.Publisher(self.debug_ns + "/target_radius", Float64, queue_size=1)
        self.radius_error_pub = rospy.Publisher(self.debug_ns + "/radius_error", Float64, queue_size=1)
        self.along_branch_y_pub = rospy.Publisher(self.debug_ns + "/along_branch_y", Float64, queue_size=1)

        self.hand_x_pub = rospy.Publisher(self.debug_ns + "/hand_x", Float64, queue_size=1)
        self.hand_y_pub = rospy.Publisher(self.debug_ns + "/hand_y", Float64, queue_size=1)
        self.hand_z_pub = rospy.Publisher(self.debug_ns + "/hand_z", Float64, queue_size=1)

        self.closest_x_pub = rospy.Publisher(self.debug_ns + "/closest_axis_x", Float64, queue_size=1)
        self.closest_y_pub = rospy.Publisher(self.debug_ns + "/closest_axis_y", Float64, queue_size=1)
        self.closest_z_pub = rospy.Publisher(self.debug_ns + "/closest_axis_z", Float64, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_cb)

        rospy.loginfo("Perching radius debug node started.")
        rospy.loginfo("Branch pose topic: %s", self.branch_pose_topic)
        rospy.loginfo("Robot odom topic: %s", self.robot_odom_topic)
        rospy.loginfo("Debug namespace: %s", self.debug_ns)
        rospy.loginfo("Hand offset in base_link: x=%.5f y=%.5f z=%.5f",
                      self.hand_offset_base[0],
                      self.hand_offset_base[1],
                      self.hand_offset_base[2])

    def branch_pose_cb(self, msg):
        self.branch_pose = msg

    def robot_odom_cb(self, msg):
        self.robot_odom = msg

    def set_target_from_current_cb(self, _msg):
        if self.last_measured_radius is None:
            rospy.logwarn("Cannot set target radius yet: no measured radius available.")
            return

        self.target_radius = self.last_measured_radius
        self.target_radius_locked = True

        rospy.logwarn("Target radius locked from current measurement: %.5f m", self.target_radius)

    def make_marker(self, marker_id, marker_type, name, now):
        m = Marker()
        m.header.frame_id = self.fixed_frame
        m.header.stamp = now
        m.ns = name
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.lifetime = rospy.Duration(self.marker_lifetime)
        m.pose.orientation.w = 1.0
        return m

    def set_color(self, marker, r, g, b, a=1.0):
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

    def create_branch_axis_marker(self, marker_id, center, axis, now):
        m = self.make_marker(marker_id, Marker.LINE_STRIP, "branch_axis", now)
        m.scale.x = 0.025
        self.set_color(m, 0.1, 0.8, 1.0, 1.0)

        half = self.branch_axis_length * 0.5
        p1 = vec_sub(center, vec_mul(axis, half))
        p2 = vec_add(center, vec_mul(axis, half))

        m.points.append(point_msg(p1))
        m.points.append(point_msg(p2))
        return m

    def create_hand_marker(self, marker_id, hand_pos, now):
        m = self.make_marker(marker_id, Marker.SPHERE, "hand_point", now)
        m.pose.position = point_msg(hand_pos)
        m.scale.x = 0.08
        m.scale.y = 0.08
        m.scale.z = 0.08
        self.set_color(m, 1.0, 0.2, 0.2, 1.0)
        return m

    def create_closest_marker(self, marker_id, closest_pos, now):
        m = self.make_marker(marker_id, Marker.SPHERE, "closest_axis_point", now)
        m.pose.position = point_msg(closest_pos)
        m.scale.x = 0.06
        m.scale.y = 0.06
        m.scale.z = 0.06
        self.set_color(m, 0.2, 1.0, 0.2, 1.0)
        return m

    def create_measured_radius_line_marker(self, marker_id, hand_pos, closest_pos, now):
        m = self.make_marker(marker_id, Marker.LINE_STRIP, "measured_radius_line", now)
        m.scale.x = 0.018
        self.set_color(m, 1.0, 1.0, 0.1, 1.0)
        m.points.append(point_msg(closest_pos))
        m.points.append(point_msg(hand_pos))
        return m

    def create_error_line_marker(self, marker_id, hand_pos, target_pos, now):
        m = self.make_marker(marker_id, Marker.LINE_STRIP, "radius_error_line", now)
        m.scale.x = 0.025
        self.set_color(m, 1.0, 0.1, 1.0, 1.0)
        m.points.append(point_msg(hand_pos))
        m.points.append(point_msg(target_pos))
        return m

    def create_target_circle_marker(self, marker_id, center, axis, radius, now):
        m = self.make_marker(marker_id, Marker.LINE_STRIP, "target_radius_circle", now)
        m.scale.x = 0.015
        self.set_color(m, 0.2, 0.6, 1.0, 0.9)

        if radius <= 1e-6:
            return m

        # Build two perpendicular unit vectors around branch axis.
        world_z = [0.0, 0.0, 1.0]
        world_x = [1.0, 0.0, 0.0]

        u = vec_cross(axis, world_z)
        if vec_norm(u) < 1e-6:
            u = vec_cross(axis, world_x)
        u = vec_normalize(u)

        v = vec_normalize(vec_cross(axis, u))

        for i in range(self.circle_points + 1):
            theta = 2.0 * math.pi * float(i) / float(self.circle_points)
            p = vec_add(
                center,
                vec_add(
                    vec_mul(u, radius * math.cos(theta)),
                    vec_mul(v, radius * math.sin(theta)),
                )
            )
            m.points.append(point_msg(p))

        return m

    def create_text_marker(self, marker_id, hand_pos, measured_radius, target_radius, radius_error, along_axis, now):
        m = self.make_marker(marker_id, Marker.TEXT_VIEW_FACING, "radius_text", now)
        m.pose.position = point_msg(vec_add(hand_pos, [0.0, 0.0, 0.18]))
        m.scale.z = 0.08
        self.set_color(m, 1.0, 1.0, 1.0, 1.0)

        if self.target_radius_locked:
            m.text = (
                "radius = %.4f m\n"
                "target = %.4f m\n"
                "error = %.4f m\n"
                "branch_y = %.4f m"
            ) % (measured_radius, target_radius, radius_error, along_axis)
        else:
            m.text = (
                "radius = %.4f m\n"
                "target not locked\n"
                "publish Empty to:\n"
                "%s/set_target_from_current"
            ) % (measured_radius, self.debug_ns)

        return m

    def timer_cb(self, _event):
        if self.branch_pose is None or self.robot_odom is None:
            return

        now = rospy.Time.now()

        branch_frame = self.branch_pose.header.frame_id
        robot_frame = self.robot_odom.header.frame_id

        if not self.warned_frame_mismatch:
            if branch_frame and robot_frame and branch_frame != robot_frame:
                rospy.logwarn(
                    "Branch pose frame [%s] and robot odom frame [%s] are different. "
                    "This node assumes both are already expressed in the same world frame. "
                    "If they are not, add TF transformation support.",
                    branch_frame,
                    robot_frame
                )
                self.warned_frame_mismatch = True

        branch_pos = [
            self.branch_pose.pose.position.x,
            self.branch_pose.pose.position.y,
            self.branch_pose.pose.position.z,
        ]

        robot_pos = [
            self.robot_odom.pose.pose.position.x,
            self.robot_odom.pose.pose.position.y,
            self.robot_odom.pose.pose.position.z,
        ]

        R_branch = quat_to_rot_matrix(self.branch_pose.pose.orientation)
        R_robot = quat_to_rot_matrix(self.robot_odom.pose.pose.orientation)

        # Branch local Y-axis expressed in world frame.
        # This is the important part:
        # radius is distance to the branch centerline axis, not just y difference.
        branch_axis_y_world = vec_normalize(mat_vec_mul(R_branch, [0.0, 1.0, 0.0]))

        # Hand/perching point expressed in world frame.
        hand_offset_world = mat_vec_mul(R_robot, self.hand_offset_base)
        hand_pos = vec_add(robot_pos, hand_offset_world)

        # Project hand point onto branch axis.
        hand_from_branch_origin = vec_sub(hand_pos, branch_pos)
        along_axis = vec_dot(hand_from_branch_origin, branch_axis_y_world)

        closest_axis_point = vec_add(
            branch_pos,
            vec_mul(branch_axis_y_world, along_axis)
        )

        radius_vec = vec_sub(hand_pos, closest_axis_point)
        measured_radius = vec_norm(radius_vec)

        self.last_measured_radius = measured_radius

        if (not self.target_radius_locked) and self.auto_set_target_radius:
            self.target_radius = measured_radius
            self.target_radius_locked = True
            rospy.logwarn("Auto target radius locked: %.5f m", self.target_radius)

        if self.target_radius_locked:
            radius_error = measured_radius - self.target_radius
        else:
            radius_error = 0.0

        # Target point on the target radius circle in same radial direction.
        radial_dir = vec_normalize(radius_vec, fallback=[1.0, 0.0, 0.0])
        target_point = vec_add(closest_axis_point, vec_mul(radial_dir, self.target_radius))

        marker_array = MarkerArray()

        marker_array.markers.append(
            self.create_branch_axis_marker(
                0,
                closest_axis_point,
                branch_axis_y_world,
                now
            )
        )

        marker_array.markers.append(
            self.create_target_circle_marker(
                1,
                closest_axis_point,
                branch_axis_y_world,
                self.target_radius,
                now
            )
        )

        marker_array.markers.append(
            self.create_hand_marker(
                2,
                hand_pos,
                now
            )
        )

        marker_array.markers.append(
            self.create_closest_marker(
                3,
                closest_axis_point,
                now
            )
        )

        marker_array.markers.append(
            self.create_measured_radius_line_marker(
                4,
                hand_pos,
                closest_axis_point,
                now
            )
        )

        if self.target_radius_locked:
            marker_array.markers.append(
                self.create_error_line_marker(
                    5,
                    hand_pos,
                    target_point,
                    now
                )
            )

        marker_array.markers.append(
            self.create_text_marker(
                6,
                hand_pos,
                measured_radius,
                self.target_radius,
                radius_error,
                along_axis,
                now
            )
        )

        self.marker_pub.publish(marker_array)

        self.measured_radius_pub.publish(Float64(measured_radius))
        self.target_radius_pub.publish(Float64(self.target_radius))
        self.radius_error_pub.publish(Float64(radius_error))
        self.along_branch_y_pub.publish(Float64(along_axis))

        self.hand_x_pub.publish(Float64(hand_pos[0]))
        self.hand_y_pub.publish(Float64(hand_pos[1]))
        self.hand_z_pub.publish(Float64(hand_pos[2]))

        self.closest_x_pub.publish(Float64(closest_axis_point[0]))
        self.closest_y_pub.publish(Float64(closest_axis_point[1]))
        self.closest_z_pub.publish(Float64(closest_axis_point[2]))


if __name__ == "__main__":
    rospy.init_node("perching_radius_debug_node")
    node = PerchingRadiusDebugNode()
    rospy.spin()