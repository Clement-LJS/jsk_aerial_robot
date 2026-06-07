#!/usr/bin/env python3

import math
import rospy

from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Empty, UInt8
from aerial_robot_msgs.msg import FlightNav

import tf2_ros

from tf.transformations import (
    quaternion_inverse,
    quaternion_multiply,
    quaternion_matrix,
    quaternion_from_euler,
    euler_from_quaternion,
)


NAV_MODE_NONE = 0
NAV_MODE_VEL = 1
NAV_MODE_POS = 2
NAV_MODE_POS_VEL = 3

ARM_OFF_STATE = 0
START_STATE = 1
ARM_ON_STATE = 2
TAKEOFF_STATE = 3
LAND_STATE = 4
HOVER_STATE = 5
STOP_STATE = 6

LOW_BATTERY_STATE = 16
FORCE_LANDING_STATE = 17


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap_to_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def shortest_angle_step(current, target, max_step):
    err = wrap_to_pi(target - current)

    if abs(err) <= max_step:
        return target

    return current + math.copysign(max_step, err)


def quat_rotate_vector(q, v):
    vq = [v[0], v[1], v[2], 0.0]
    q_inv = quaternion_inverse(q)

    out = quaternion_multiply(
        quaternion_multiply(q, vq),
        q_inv
    )

    return [out[0], out[1], out[2]]


class PerchBranchArcTakeoff(object):
    """
    Safer branch-constrained attitude takeoff node.

    Main change:

      Previous version:
        After HOVER, it commanded position x/y/z from the branch arc.
        That made target_z jump from about 0.48 m to 0.65 m,
        causing aggressive oscillation.

      This version:
        During HOVER, it commands attitude only.
        It does NOT command x/y/z position from this node.
        Normal JSK hover keeps position.
        This node slowly changes pitch:
            takeoff_hold_pitch_deg -> final_pitch_deg

    Goal:
      1. Take off successfully.
      2. Reach HOVER_STATE.
      3. Hold safe pitch for a few seconds.
      4. Slowly ramp pitch to 0 deg.
      5. Avoid branch-arc z target jump.
    """

    def __init__(self):
        rospy.init_node("perch_branch_arc_takeoff", anonymous=False)

        # ============================================================
        # Input topics
        # ============================================================

        self.branch_pose_topic = rospy.get_param(
            "~branch_pose_topic",
            "/mocap/pose"
        )

        self.gimbal_odom_topic = rospy.get_param(
            "~gimbal_odom_topic",
            "/gimbalrotor/uav/baselink/odom"
        )

        self.flight_state_topic = rospy.get_param(
            "~flight_state_topic",
            "/gimbalrotor/flight_state"
        )

        # ============================================================
        # Output topics
        # ============================================================

        self.nav_out_topic = rospy.get_param(
            "~nav_out_topic",
            "/gimbalrotor/uav/nav"
        )

        self.final_target_baselink_rot_topic = rospy.get_param(
            "~final_target_baselink_rot_topic",
            "/gimbalrotor/final_target_baselink_rot"
        )

        self.robot_takeoff_topic = rospy.get_param(
            "~robot_takeoff_topic",
            "/gimbalrotor/teleop_command/takeoff"
        )

        self.robot_land_topic = rospy.get_param(
            "~robot_land_topic",
            "/gimbalrotor/teleop_command/land"
        )

        # ============================================================
        # Frame settings
        # ============================================================

        self.target_world_frame = rospy.get_param(
            "~target_world_frame",
            "world"
        )

        self.branch_default_frame = rospy.get_param(
            "~branch_default_frame",
            self.target_world_frame
        )

        self.robot_default_frame = rospy.get_param(
            "~robot_default_frame",
            self.target_world_frame
        )

        # ============================================================
        # Pitch settings
        # ============================================================

        self.final_pitch_deg = rospy.get_param(
            "~final_pitch_deg",
            0.0
        )

        self.takeoff_hold_pitch_deg = rospy.get_param(
            "~takeoff_hold_pitch_deg",
            20.0
        )

        # Kept same parameter name as your launch.
        # Now this is pitch ramp rate in deg/s.
        self.arc_rate_deg_per_sec = rospy.get_param(
            "~arc_rate_deg_per_sec",
            0.1
        )

        self.max_commanded_pitch_deg = rospy.get_param(
            "~max_commanded_pitch_deg",
            80.0
        )

        self.hover_settle_time_sec = rospy.get_param(
            "~hover_settle_time_sec",
            4.0
        )

        # ============================================================
        # Attitude command mode
        # ============================================================

        # Critical safety setting:
        # False = do not publish position x/y/z from this node.
        # This avoids the target_z jump that caused oscillation.
        self.publish_position_command = rospy.get_param(
            "~publish_position_command",
            False
        )

        # If position command is disabled, these modes remain NO_NAVIGATION.
        # Normal JSK hover target stays in control of position.
        self.hold_yaw_nav_mode = rospy.get_param(
            "~hold_yaw_nav_mode",
            False
        )

        # ============================================================
        # Roll/yaw adaptation settings
        # ============================================================

        self.adapt_roll_yaw_after_hover = rospy.get_param(
            "~adapt_roll_yaw_after_hover",
            False
        )

        self.roll_yaw_adapt_alpha = rospy.get_param(
            "~roll_yaw_adapt_alpha",
            0.001
        )

        self.max_roll_adapt_deg = rospy.get_param(
            "~max_roll_adapt_deg",
            2.0
        )

        self.max_yaw_adapt_deg = rospy.get_param(
            "~max_yaw_adapt_deg",
            2.0
        )

        self.adapt_distance_gate_m = rospy.get_param(
            "~adapt_distance_gate_m",
            0.03
        )

        # ============================================================
        # Branch geometry safety / debug
        # ============================================================

        self.min_valid_radius = rospy.get_param(
            "~min_valid_radius",
            0.05
        )

        self.max_valid_radius = rospy.get_param(
            "~max_valid_radius",
            2.0
        )

        self.branch_clearance_deadband_m = rospy.get_param(
            "~branch_clearance_deadband_m",
            0.01
        )

        # ============================================================
        # State-machine settings
        # ============================================================

        self.print_rate = rospy.get_param(
            "~print_rate",
            5.0
        )

        self.publish_hold_when_enabled = rospy.get_param(
            "~publish_hold_when_enabled",
            True
        )

        self.publish_robot_takeoff_land = rospy.get_param(
            "~publish_robot_takeoff_land",
            True
        )

        self.arc_return_before_land = rospy.get_param(
            "~arc_return_before_land",
            True
        )

        self.pitch_reached_thresh_deg = rospy.get_param(
            "~pitch_reached_thresh_deg",
            1.0
        )

        self.stop_command_on_autoland = rospy.get_param(
            "~stop_command_on_autoland",
            True
        )

        # ============================================================
        # TF
        # ============================================================

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ============================================================
        # Internal state
        # ============================================================

        self.branch_pose_msg = None
        self.odom_msg = None

        self.flight_state = ARM_OFF_STATE
        self.prev_flight_state = ARM_OFF_STATE

        self.enabled = False
        self.locked = False

        self.takeoff_requested = False
        self.pitch_ramp_active = False

        self.land_requested = False
        self.land_after_pitch_return = False

        self.hover_enter_time = None

        self.send_takeoff_until = rospy.Time(0)
        self.send_land_until = rospy.Time(0)

        self.pivot = [0.0, 0.0, 0.0]
        self.initial_robot = [0.0, 0.0, 0.0]
        self.initial_quat = [0.0, 0.0, 0.0, 1.0]

        self.initial_roll = 0.0
        self.initial_pitch = 0.0
        self.initial_yaw = 0.0

        self.current_pitch_cmd = 0.0

        self.roll_offset_cmd = 0.0
        self.yaw_offset_cmd = 0.0

        self.radius_3d = 0.0
        self.radius_xz = 0.0

        self.initial_y_offset = 0.0
        self.initial_diff_x = 0.0
        self.initial_diff_y = 0.0
        self.initial_diff_z = 0.0
        self.initial_theta = 0.0

        self.last_time = rospy.Time.now()
        self.last_print_time = rospy.Time.now()

        # ============================================================
        # Subscribers
        # ============================================================

        self.branch_sub = rospy.Subscriber(
            self.branch_pose_topic,
            PoseStamped,
            self.branchPoseCallback,
            queue_size=1
        )

        self.odom_sub = rospy.Subscriber(
            self.gimbal_odom_topic,
            Odometry,
            self.gimbalOdomCallback,
            queue_size=1
        )

        self.flight_state_sub = rospy.Subscriber(
            self.flight_state_topic,
            UInt8,
            self.flightStateCallback,
            queue_size=1
        )

        self.enable_sub = rospy.Subscriber(
            "~enable",
            Bool,
            self.enableCallback,
            queue_size=1
        )

        self.takeoff_sub = rospy.Subscriber(
            "~takeoff",
            Bool,
            self.takeoffCallback,
            queue_size=1
        )

        self.land_sub = rospy.Subscriber(
            "~land",
            Bool,
            self.landCallback,
            queue_size=1
        )

        self.relock_sub = rospy.Subscriber(
            "~relock",
            Bool,
            self.relockCallback,
            queue_size=1
        )

        # ============================================================
        # Publishers
        # ============================================================

        self.nav_pub = rospy.Publisher(
            self.nav_out_topic,
            FlightNav,
            queue_size=1
        )

        self.attitude_pub = rospy.Publisher(
            self.final_target_baselink_rot_topic,
            QuaternionStamped,
            queue_size=1
        )

        self.robot_takeoff_pub = rospy.Publisher(
            self.robot_takeoff_topic,
            Empty,
            queue_size=1
        )

        self.robot_land_pub = rospy.Publisher(
            self.robot_land_topic,
            Empty,
            queue_size=1
        )

        self.diff_world_pub = rospy.Publisher(
            "~diff_position_world",
            Vector3Stamped,
            queue_size=1
        )

        self.diff_gimbal_pub = rospy.Publisher(
            "~diff_position_gimbal_frame",
            Vector3Stamped,
            queue_size=1
        )

        self.locked_radius_pub = rospy.Publisher(
            "~locked_radius",
            Float64,
            queue_size=1,
            latch=True
        )

        self.target_world_pub = rospy.Publisher(
            "~target_world",
            Vector3Stamped,
            queue_size=1
        )

        self.nominal_target_world_pub = rospy.Publisher(
            "~nominal_target_world",
            Vector3Stamped,
            queue_size=1
        )

        rospy.Timer(
            rospy.Duration(0.02),
            self.timerCallback
        )

        rospy.logwarn("[perch_branch_arc_takeoff] initialized")
        rospy.logwarn("  branch_pose_topic          : %s", self.branch_pose_topic)
        rospy.logwarn("  gimbal_odom_topic          : %s", self.gimbal_odom_topic)
        rospy.logwarn("  flight_state_topic         : %s", self.flight_state_topic)
        rospy.logwarn("  nav_out_topic              : %s", self.nav_out_topic)
        rospy.logwarn("  final_pitch_deg            : %.2f", self.final_pitch_deg)
        rospy.logwarn("  takeoff_hold_pitch_deg     : %.2f", self.takeoff_hold_pitch_deg)
        rospy.logwarn("  pitch ramp rate deg/s      : %.2f", self.arc_rate_deg_per_sec)
        rospy.logwarn("  hover_settle_time_sec      : %.2f", self.hover_settle_time_sec)
        rospy.logwarn("  publish_position_command   : %s", str(self.publish_position_command))
        rospy.logwarn("  adapt_roll_yaw_after_hover : %s", str(self.adapt_roll_yaw_after_hover))

    # ============================================================
    # TF helpers
    # ============================================================

    def normalizeFrameId(self, frame_id, fallback_frame):
        if frame_id is None or frame_id == "":
            return fallback_frame

        if frame_id[0] == "/":
            return frame_id[1:]

        return frame_id

    def odomToPoseStamped(self, odom_msg):
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose

        if pose_msg.header.frame_id == "":
            pose_msg.header.frame_id = self.robot_default_frame

        return pose_msg

    def transformPoseToTargetFrame(self, pose_msg, default_frame):
        source_frame = self.normalizeFrameId(
            pose_msg.header.frame_id,
            default_frame
        )

        target_frame = self.normalizeFrameId(
            self.target_world_frame,
            self.target_world_frame
        )

        pose_msg.header.frame_id = source_frame

        if source_frame == target_frame:
            out = PoseStamped()
            out.header = pose_msg.header
            out.header.frame_id = target_frame
            out.pose = pose_msg.pose
            return out

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(0.05)
            )

            return self.applyTransformToPose(
                pose_msg,
                transform
            )

        except Exception as e:
            rospy.logwarn_throttle(
                1.0,
                "[perch_branch_arc_takeoff] TF transform failed: %s -> %s. Error: %s",
                source_frame,
                target_frame,
                str(e)
            )
            return None

    def applyTransformToPose(self, pose_msg, transform_msg):
        tx = transform_msg.transform.translation.x
        ty = transform_msg.transform.translation.y
        tz = transform_msg.transform.translation.z

        tq = transform_msg.transform.rotation

        transform_q = [
            tq.x,
            tq.y,
            tq.z,
            tq.w
        ]

        px = pose_msg.pose.position.x
        py = pose_msg.pose.position.y
        pz = pose_msg.pose.position.z

        transform_mat = quaternion_matrix(transform_q)

        x_new = (
            transform_mat[0][0] * px +
            transform_mat[0][1] * py +
            transform_mat[0][2] * pz +
            tx
        )

        y_new = (
            transform_mat[1][0] * px +
            transform_mat[1][1] * py +
            transform_mat[1][2] * pz +
            ty
        )

        z_new = (
            transform_mat[2][0] * px +
            transform_mat[2][1] * py +
            transform_mat[2][2] * pz +
            tz
        )

        pq = pose_msg.pose.orientation

        pose_q = [
            pq.x,
            pq.y,
            pq.z,
            pq.w
        ]

        q_new = quaternion_multiply(
            transform_q,
            pose_q
        )

        out = PoseStamped()
        out.header.stamp = pose_msg.header.stamp
        out.header.frame_id = transform_msg.header.frame_id

        out.pose.position.x = x_new
        out.pose.position.y = y_new
        out.pose.position.z = z_new

        out.pose.orientation.x = q_new[0]
        out.pose.orientation.y = q_new[1]
        out.pose.orientation.z = q_new[2]
        out.pose.orientation.w = q_new[3]

        return out

    def getBranchAndRobotInTargetFrame(self):
        if self.branch_pose_msg is None:
            return None, None

        if self.odom_msg is None:
            return None, None

        branch_pose_raw = PoseStamped()
        branch_pose_raw.header = self.branch_pose_msg.header
        branch_pose_raw.pose = self.branch_pose_msg.pose

        if branch_pose_raw.header.frame_id == "":
            branch_pose_raw.header.frame_id = self.branch_default_frame

        robot_pose_raw = self.odomToPoseStamped(
            self.odom_msg
        )

        branch_pose_world = self.transformPoseToTargetFrame(
            branch_pose_raw,
            self.branch_default_frame
        )

        robot_pose_world = self.transformPoseToTargetFrame(
            robot_pose_raw,
            self.robot_default_frame
        )

        if branch_pose_world is None:
            return None, None

        if robot_pose_world is None:
            return None, None

        return branch_pose_world, robot_pose_world

    def getCurrentRobotPoseInTargetFrame(self):
        _, robot_pose_world = self.getBranchAndRobotInTargetFrame()

        if robot_pose_world is None:
            return None

        return robot_pose_world

    # ============================================================
    # Callbacks
    # ============================================================

    def branchPoseCallback(self, msg):
        self.branch_pose_msg = msg

    def gimbalOdomCallback(self, msg):
        self.odom_msg = msg

    def flightStateCallback(self, msg):
        previous = self.flight_state

        self.prev_flight_state = self.flight_state
        self.flight_state = msg.data

        if previous != self.flight_state:
            rospy.logwarn(
                "[perch_branch_arc_takeoff] flight_state changed: %d -> %d",
                previous,
                self.flight_state
            )

        if previous != HOVER_STATE and self.flight_state == HOVER_STATE:
            self.hover_enter_time = rospy.Time.now()
            self.current_pitch_cmd = self.safeTakeoffPitchRad()
            self.pitch_ramp_active = False

            rospy.logwarn(
                "[perch_branch_arc_takeoff] HOVER detected. Hold pitch %.2f deg for %.2f sec before ramp.",
                math.degrees(self.current_pitch_cmd),
                self.hover_settle_time_sec
            )

    def enableCallback(self, msg):
        self.enabled = bool(msg.data)

        if not self.enabled:
            self.takeoff_requested = False
            self.pitch_ramp_active = False
            self.land_requested = False
            self.land_after_pitch_return = False
            self.locked = False
            self.hover_enter_time = None
            self.send_takeoff_until = rospy.Time(0)
            self.send_land_until = rospy.Time(0)
            self.roll_offset_cmd = 0.0
            self.yaw_offset_cmd = 0.0

            rospy.logwarn("[perch_branch_arc_takeoff] disabled and unlocked")
            return

        ok = self.lockBranchGeometry()

        if ok:
            rospy.logwarn("[perch_branch_arc_takeoff] enabled and locked")
        else:
            rospy.logwarn("[perch_branch_arc_takeoff] enabled, waiting for branch/odom/TF")

    def takeoffCallback(self, msg):
        if not msg.data:
            self.takeoff_requested = False
            self.pitch_ramp_active = False
            rospy.logwarn("[perch_branch_arc_takeoff] takeoff stopped")
            return

        if not self.enabled:
            rospy.logwarn("[perch_branch_arc_takeoff] takeoff requested but node is not enabled")
            return

        if not self.locked:
            if not self.lockBranchGeometry():
                rospy.logwarn("[perch_branch_arc_takeoff] takeoff requested but lock failed")
                return

        self.takeoff_requested = True
        self.land_requested = False
        self.land_after_pitch_return = False
        self.pitch_ramp_active = False

        self.current_pitch_cmd = self.safeTakeoffPitchRad()

        if self.publish_robot_takeoff_land:
            self.send_takeoff_until = rospy.Time.now() + rospy.Duration(1.0)
            rospy.logwarn("[perch_branch_arc_takeoff] will publish robot takeoff for 1 second")

        rospy.logwarn(
            "[perch_branch_arc_takeoff] takeoff requested. Hold pitch %.2f deg until HOVER.",
            math.degrees(self.current_pitch_cmd)
        )

    def landCallback(self, msg):
        if not msg.data:
            self.land_requested = False
            self.land_after_pitch_return = False
            rospy.logwarn("[perch_branch_arc_takeoff] landing stopped")
            return

        if not self.enabled:
            rospy.logwarn("[perch_branch_arc_takeoff] land requested but node is not enabled")
            return

        if not self.locked:
            if not self.lockBranchGeometry():
                rospy.logwarn("[perch_branch_arc_takeoff] land requested but lock failed")
                return

        self.takeoff_requested = False
        self.pitch_ramp_active = False
        self.land_requested = True

        if self.arc_return_before_land and self.flight_state == HOVER_STATE:
            self.land_after_pitch_return = True
            rospy.logwarn(
                "[perch_branch_arc_takeoff] land requested. First return to safe takeoff pitch."
            )
        else:
            self.land_after_pitch_return = False
            self.send_land_until = rospy.Time.now() + rospy.Duration(1.0)
            rospy.logwarn("[perch_branch_arc_takeoff] will publish robot land for 1 second")

    def relockCallback(self, msg):
        if not msg.data:
            return

        self.locked = False
        self.takeoff_requested = False
        self.pitch_ramp_active = False
        self.land_requested = False
        self.land_after_pitch_return = False
        self.roll_offset_cmd = 0.0
        self.yaw_offset_cmd = 0.0
        self.hover_enter_time = None

        ok = self.lockBranchGeometry()

        if ok:
            rospy.logwarn("[perch_branch_arc_takeoff] relocked")
        else:
            rospy.logwarn("[perch_branch_arc_takeoff] relock failed")

    # ============================================================
    # Lock geometry
    # ============================================================

    def lockBranchGeometry(self):
        branch_pose_world, robot_pose_world = self.getBranchAndRobotInTargetFrame()

        if branch_pose_world is None or robot_pose_world is None:
            rospy.logwarn_throttle(
                1.0,
                "[perch_branch_arc_takeoff] waiting for branch/robot pose in target frame"
            )
            return False

        branch = branch_pose_world.pose
        robot = robot_pose_world.pose

        bx = branch.position.x
        by = branch.position.y
        bz = branch.position.z

        gx = robot.position.x
        gy = robot.position.y
        gz = robot.position.z

        dx = bx - gx
        dy = by - gy
        dz = bz - gz

        radius_3d = math.sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        )

        if radius_3d < self.min_valid_radius or radius_3d > self.max_valid_radius:
            rospy.logerr(
                "[perch_branch_arc_takeoff] invalid radius %.4f m. Check mocap/odom/TF.",
                radius_3d
            )
            return False

        radius_xz_sq = radius_3d * radius_3d - dy * dy

        if radius_xz_sq <= 1.0e-8:
            rospy.logerr(
                "[perch_branch_arc_takeoff] invalid X-Z radius. dx %.4f dz %.4f",
                dx,
                dz
            )
            return False

        q = robot.orientation

        quat = [
            q.x,
            q.y,
            q.z,
            q.w
        ]

        roll, pitch, yaw = euler_from_quaternion(quat)

        self.pivot = [
            bx,
            by,
            bz
        ]

        self.initial_robot = [
            gx,
            gy,
            gz
        ]

        self.initial_quat = quat

        self.initial_roll = roll
        self.initial_pitch = pitch
        self.initial_yaw = yaw

        self.roll_offset_cmd = 0.0
        self.yaw_offset_cmd = 0.0

        self.radius_3d = radius_3d
        self.radius_xz = math.sqrt(radius_xz_sq)

        self.initial_y_offset = gy - by

        self.initial_diff_x = dx
        self.initial_diff_y = dy
        self.initial_diff_z = dz

        self.initial_theta = math.atan2(
            dz,
            dx
        )

        self.current_pitch_cmd = self.safeTakeoffPitchRad()

        self.locked = True

        radius_msg = Float64()
        radius_msg.data = self.radius_3d
        self.locked_radius_pub.publish(radius_msg)

        rospy.logwarn(
            "[perch_branch_arc_takeoff] LOCKED in target frame: %s"
            "\n  branch pivot: x %.4f, y %.4f, z %.4f"
            "\n  robot initial: x %.4f, y %.4f, z %.4f"
            "\n  branch - robot: dx %.4f, dy %.4f, dz %.4f"
            "\n  radius_3d %.4f m, radius_xz %.4f m"
            "\n  initial theta %.2f deg"
            "\n  initial rpy: roll %.2f deg, pitch %.2f deg, yaw %.2f deg"
            "\n  takeoff hold pitch %.2f deg"
            "\n  final pitch target %.2f deg",
            self.target_world_frame,
            bx,
            by,
            bz,
            gx,
            gy,
            gz,
            dx,
            dy,
            dz,
            self.radius_3d,
            self.radius_xz,
            math.degrees(self.initial_theta),
            math.degrees(roll),
            math.degrees(pitch),
            math.degrees(yaw),
            math.degrees(self.current_pitch_cmd),
            self.final_pitch_deg
        )

        return True

    # ============================================================
    # Pitch / attitude
    # ============================================================

    def safeTakeoffPitchRad(self):
        safe_pitch = math.radians(
            abs(self.takeoff_hold_pitch_deg)
        )

        if self.initial_pitch >= 0.0:
            return safe_pitch

        return -safe_pitch

    def finalPitchRad(self):
        return math.radians(
            self.final_pitch_deg
        )

    def getCurrentPositionAndDistanceError(self):
        branch_pose_world, robot_pose_world = self.getBranchAndRobotInTargetFrame()

        if branch_pose_world is None or robot_pose_world is None:
            return None, 0.0

        branch = branch_pose_world.pose
        robot = robot_pose_world.pose

        current_pos = [
            robot.position.x,
            robot.position.y,
            robot.position.z
        ]

        dx = branch.position.x - robot.position.x
        dy = branch.position.y - robot.position.y
        dz = branch.position.z - robot.position.z

        current_distance = math.sqrt(
            dx * dx +
            dy * dy +
            dz * dz
        )

        distance_error = current_distance - self.radius_3d

        return current_pos, distance_error

    def updateAdaptiveRollYaw(self, distance_error):
        if not self.adapt_roll_yaw_after_hover:
            return

        if self.flight_state != HOVER_STATE:
            return

        if abs(distance_error) > self.adapt_distance_gate_m:
            return

        robot_pose_world = self.getCurrentRobotPoseInTargetFrame()

        if robot_pose_world is None:
            return

        q = robot_pose_world.pose.orientation

        quat = [
            q.x,
            q.y,
            q.z,
            q.w
        ]

        measured_roll, measured_pitch, measured_yaw = euler_from_quaternion(quat)

        desired_roll_offset = wrap_to_pi(
            measured_roll -
            self.initial_roll
        )

        desired_yaw_offset = wrap_to_pi(
            measured_yaw -
            self.initial_yaw
        )

        max_roll_offset = math.radians(
            self.max_roll_adapt_deg
        )

        max_yaw_offset = math.radians(
            self.max_yaw_adapt_deg
        )

        desired_roll_offset = clamp(
            desired_roll_offset,
            -max_roll_offset,
            max_roll_offset
        )

        desired_yaw_offset = clamp(
            desired_yaw_offset,
            -max_yaw_offset,
            max_yaw_offset
        )

        alpha = clamp(
            self.roll_yaw_adapt_alpha,
            0.0,
            1.0
        )

        self.roll_offset_cmd = (
            (1.0 - alpha) * self.roll_offset_cmd +
            alpha * desired_roll_offset
        )

        self.yaw_offset_cmd = (
            (1.0 - alpha) * self.yaw_offset_cmd +
            alpha * desired_yaw_offset
        )

    def commandQuaternionFromPitchRollYaw(self, target_pitch):
        target_roll = self.initial_roll + self.roll_offset_cmd
        target_yaw = self.initial_yaw + self.yaw_offset_cmd

        q_cmd = quaternion_from_euler(
            target_roll,
            target_pitch,
            target_yaw
        )

        return q_cmd

    # ============================================================
    # Publishers
    # ============================================================

    def publishRobotStateCommands(self):
        now = rospy.Time.now()

        if now < self.send_takeoff_until:
            self.robot_takeoff_pub.publish(
                Empty()
            )

        if now < self.send_land_until:
            self.robot_land_pub.publish(
                Empty()
            )

    def publishCommand(self):
        if not self.locked:
            return

        current_pos, distance_error = self.getCurrentPositionAndDistanceError()

        self.updateAdaptiveRollYaw(
            distance_error
        )

        target_pitch = self.current_pitch_cmd

        max_pitch = math.radians(
            self.max_commanded_pitch_deg
        )

        target_pitch = clamp(
            target_pitch,
            -max_pitch,
            max_pitch
        )

        q_cmd = self.commandQuaternionFromPitchRollYaw(
            target_pitch
        )

        roll_cmd, pitch_cmd, yaw_cmd = euler_from_quaternion(
            q_cmd
        )

        nav = FlightNav()

        # Critical:
        # Do not command position from this node.
        # This avoids branch-derived target_z jump.
        if self.publish_position_command and current_pos is not None:
            nav.pos_xy_nav_mode = NAV_MODE_POS
            nav.pos_z_nav_mode = NAV_MODE_POS
            nav.target_pos_x = current_pos[0]
            nav.target_pos_y = current_pos[1]
            nav.target_pos_z = current_pos[2]
        else:
            nav.pos_xy_nav_mode = NAV_MODE_NONE
            nav.pos_z_nav_mode = NAV_MODE_NONE
            nav.target_pos_x = 0.0
            nav.target_pos_y = 0.0
            nav.target_pos_z = 0.0

        nav.target_vel_x = 0.0
        nav.target_vel_y = 0.0
        nav.target_vel_z = 0.0

        nav.roll_nav_mode = NAV_MODE_POS
        nav.pitch_nav_mode = NAV_MODE_POS

        nav.target_roll = roll_cmd
        nav.target_pitch = pitch_cmd

        if self.hold_yaw_nav_mode:
            nav.yaw_nav_mode = NAV_MODE_POS
            nav.target_yaw = yaw_cmd
        else:
            nav.yaw_nav_mode = NAV_MODE_NONE
            nav.target_yaw = 0.0

        self.nav_pub.publish(
            nav
        )

        q_msg = QuaternionStamped()
        q_msg.header.stamp = rospy.Time.now()
        q_msg.header.frame_id = self.target_world_frame
        q_msg.quaternion.x = q_cmd[0]
        q_msg.quaternion.y = q_cmd[1]
        q_msg.quaternion.z = q_cmd[2]
        q_msg.quaternion.w = q_cmd[3]

        self.attitude_pub.publish(
            q_msg
        )

        # Debug only: publish current robot position as target_world.
        # It is not used as a position command unless publish_position_command is true.
        target_msg = Vector3Stamped()
        target_msg.header.stamp = rospy.Time.now()
        target_msg.header.frame_id = self.target_world_frame

        if current_pos is not None:
            target_msg.vector.x = current_pos[0]
            target_msg.vector.y = current_pos[1]
            target_msg.vector.z = current_pos[2]

        self.target_world_pub.publish(
            target_msg
        )

        nominal_msg = Vector3Stamped()
        nominal_msg.header.stamp = rospy.Time.now()
        nominal_msg.header.frame_id = self.target_world_frame

        if current_pos is not None:
            nominal_msg.vector.x = current_pos[0]
            nominal_msg.vector.y = current_pos[1]
            nominal_msg.vector.z = current_pos[2]

        self.nominal_target_world_pub.publish(
            nominal_msg
        )

        self.printDebug(
            target_pitch,
            distance_error,
            current_pos
        )

    def publishCurrentDistanceDebug(self):
        branch_pose_world, robot_pose_world = self.getBranchAndRobotInTargetFrame()

        if branch_pose_world is None or robot_pose_world is None:
            return

        branch = branch_pose_world.pose
        robot = robot_pose_world.pose

        dx = branch.position.x - robot.position.x
        dy = branch.position.y - robot.position.y
        dz = branch.position.z - robot.position.z

        now = rospy.Time.now()

        diff_world_msg = Vector3Stamped()
        diff_world_msg.header.stamp = now
        diff_world_msg.header.frame_id = self.target_world_frame
        diff_world_msg.vector.x = dx
        diff_world_msg.vector.y = dy
        diff_world_msg.vector.z = dz

        self.diff_world_pub.publish(
            diff_world_msg
        )

        q_robot = [
            robot.orientation.x,
            robot.orientation.y,
            robot.orientation.z,
            robot.orientation.w
        ]

        q_robot_inv = quaternion_inverse(
            q_robot
        )

        diff_g = quat_rotate_vector(
            q_robot_inv,
            [
                dx,
                dy,
                dz
            ]
        )

        diff_gimbal_msg = Vector3Stamped()
        diff_gimbal_msg.header.stamp = now
        diff_gimbal_msg.header.frame_id = "baselink"
        diff_gimbal_msg.vector.x = diff_g[0]
        diff_gimbal_msg.vector.y = diff_g[1]
        diff_gimbal_msg.vector.z = diff_g[2]

        self.diff_gimbal_pub.publish(
            diff_gimbal_msg
        )

    # ============================================================
    # Main timer
    # ============================================================

    def timerCallback(self, event):
        now = rospy.Time.now()

        dt = (
            now -
            self.last_time
        ).to_sec()

        self.last_time = now

        self.publishRobotStateCommands()
        self.publishCurrentDistanceDebug()

        if not self.enabled:
            return

        if not self.locked:
            if not self.lockBranchGeometry():
                return

        if self.stop_command_on_autoland:
            if self.flight_state in [
                LOW_BATTERY_STATE,
                FORCE_LANDING_STATE,
                LAND_STATE,
                STOP_STATE,
                ARM_OFF_STATE
            ]:
                return

        max_step = math.radians(
            self.arc_rate_deg_per_sec
        ) * max(0.0, dt)

        if self.flight_state == TAKEOFF_STATE:
            self.current_pitch_cmd = self.safeTakeoffPitchRad()

        elif self.flight_state == HOVER_STATE:
            if self.hover_enter_time is not None:
                elapsed = (
                    now -
                    self.hover_enter_time
                ).to_sec()
            else:
                elapsed = 0.0

            if (
                self.takeoff_requested and
                elapsed >= self.hover_settle_time_sec
            ):
                self.pitch_ramp_active = True

            if self.pitch_ramp_active:
                self.current_pitch_cmd = shortest_angle_step(
                    self.current_pitch_cmd,
                    self.finalPitchRad(),
                    max_step
                )
            else:
                self.current_pitch_cmd = self.safeTakeoffPitchRad()

        elif self.land_after_pitch_return:
            self.current_pitch_cmd = shortest_angle_step(
                self.current_pitch_cmd,
                self.safeTakeoffPitchRad(),
                max_step
            )

            err = abs(
                wrap_to_pi(
                    self.current_pitch_cmd -
                    self.safeTakeoffPitchRad()
                )
            )

            if err < math.radians(self.pitch_reached_thresh_deg):
                self.land_after_pitch_return = False
                self.send_land_until = rospy.Time.now() + rospy.Duration(1.0)
                rospy.logwarn(
                    "[perch_branch_arc_takeoff] returned to safe pitch. Now publish robot land."
                )

        else:
            if not self.publish_hold_when_enabled:
                return

            self.current_pitch_cmd = self.safeTakeoffPitchRad()

        self.publishCommand()

    def printDebug(self, target_pitch, distance_error, current_pos):
        now = rospy.Time.now()

        if (now - self.last_print_time).to_sec() < 1.0 / self.print_rate:
            return

        self.last_print_time = now

        if current_pos is None:
            pos_text = "None"
        else:
            pos_text = "x %.4f, y %.4f, z %.4f" % (
                current_pos[0],
                current_pos[1],
                current_pos[2]
            )

        rospy.loginfo(
            "\n[perch_branch_arc_takeoff]"
            "\n  enabled / locked:"
            "\n    %s / %s"
            "\n  flight_state:"
            "\n    %d"
            "\n  takeoff_requested / pitch_ramp_active / land_after_pitch_return:"
            "\n    %s / %s / %s"
            "\n  publish_position_command:"
            "\n    %s"
            "\n  branch radius:"
            "\n    radius_3d %.4f m"
            "\n    radius_xz %.4f m"
            "\n  branch distance error:"
            "\n    %.4f m"
            "\n  pitch command:"
            "\n    current %.2f deg"
            "\n    final   %.2f deg"
            "\n  current robot position:"
            "\n    %s"
            "\n  adaptive roll/yaw offset:"
            "\n    roll %.2f deg, yaw %.2f deg",
            str(self.enabled),
            str(self.locked),
            self.flight_state,
            str(self.takeoff_requested),
            str(self.pitch_ramp_active),
            str(self.land_after_pitch_return),
            str(self.publish_position_command),
            self.radius_3d,
            self.radius_xz,
            distance_error,
            math.degrees(target_pitch),
            self.final_pitch_deg,
            pos_text,
            math.degrees(self.roll_offset_cmd),
            math.degrees(self.yaw_offset_cmd)
        )


if __name__ == "__main__":
    try:
        node = PerchBranchArcTakeoff()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass