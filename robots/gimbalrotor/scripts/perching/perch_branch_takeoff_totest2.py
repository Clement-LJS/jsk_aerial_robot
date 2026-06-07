#!/usr/bin/env python3

import math
import rospy

from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, WrenchStamped
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


def norm2(x, y):
    return math.sqrt(x * x + y * y)


def norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


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
    Branch-constrained takeoff with force-relief hover search.

    Behavior:

      1. Lock branch + robot initial geometry.
      2. Trigger normal JSK takeoff.
      3. Wait until HOVER_STATE.
      4. Hold current pitch/attitude for a short settle time.
      5. Search small nearby hover positions.
      6. Choose the position with minimum external wrench.
      7. Hold that relaxed position.
      8. Slowly ramp pitch toward final_pitch_deg.
      9. Land command immediately publishes normal JSK land.

    Why this is needed:

      If the robot takes off while constrained by the branch,
      the normal hover target can become a high-preload target.
      This node searches around that hover point and finds a nearby
      position where external force/torque is lower.
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

        self.external_wrench_topic = rospy.get_param(
            "~external_wrench_topic",
            "/gimbalrotor/estimated_external_wrench"
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

        # If true, takeoff holds the measured initial pitch.
        # This prevents sudden command jump.
        self.use_locked_initial_pitch_for_takeoff = rospy.get_param(
            "~use_locked_initial_pitch_for_takeoff",
            True
        )

        self.takeoff_hold_pitch_deg = rospy.get_param(
            "~takeoff_hold_pitch_deg",
            20.0
        )

        self.pitch_rate_deg_per_sec = rospy.get_param(
            "~arc_rate_deg_per_sec",
            0.3
        )

        self.max_commanded_pitch_deg = rospy.get_param(
            "~max_commanded_pitch_deg",
            80.0
        )

        self.hover_settle_time_sec = rospy.get_param(
            "~hover_settle_time_sec",
            3.0
        )

        # ============================================================
        # Force-relief search settings
        # ============================================================

        self.enable_force_relief_search = rospy.get_param(
            "~enable_force_relief_search",
            True
        )

        # Search along the horizontal direction from branch to robot.
        # This is the "move back / move forward a little" direction.
        self.relief_search_max_m = rospy.get_param(
            "~relief_search_max_m",
            0.04
        )

        self.relief_search_step_m = rospy.get_param(
            "~relief_search_step_m",
            0.01
        )

        self.relief_candidate_dwell_sec = rospy.get_param(
            "~relief_candidate_dwell_sec",
            2.0
        )

        self.relief_settle_before_measure_sec = rospy.get_param(
            "~relief_settle_before_measure_sec",
            0.8
        )

        self.force_cost_torque_weight = rospy.get_param(
            "~force_cost_torque_weight",
            5.0
        )

        # If true, after finding best position, hold it with position command.
        self.hold_best_relief_position = rospy.get_param(
            "~hold_best_relief_position",
            True
        )

        # Safety: max allowed search motion.
        self.max_relief_position_jump_m = rospy.get_param(
            "~max_relief_position_jump_m",
            0.06
        )

        # ============================================================
        # Force gate during pitch ramp
        # ============================================================

        self.enable_force_gate = rospy.get_param(
            "~enable_force_gate",
            True
        )

        self.max_external_force_norm = rospy.get_param(
            "~max_external_force_norm",
            12.0
        )

        self.max_external_torque_norm = rospy.get_param(
            "~max_external_torque_norm",
            1.3
        )

        self.pause_pitch_ramp_on_high_force = rospy.get_param(
            "~pause_pitch_ramp_on_high_force",
            True
        )

        # ============================================================
        # Roll / yaw adaptation
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

        self.hold_yaw_nav_mode = rospy.get_param(
            "~hold_yaw_nav_mode",
            False
        )

        # ============================================================
        # Branch geometry safety
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
        # State machine
        # ============================================================

        self.print_rate = rospy.get_param(
            "~print_rate",
            5.0
        )

        self.publish_robot_takeoff_land = rospy.get_param(
            "~publish_robot_takeoff_land",
            True
        )

        self.immediate_land_on_request = rospy.get_param(
            "~immediate_land_on_request",
            True
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
        self.wrench_msg = None

        self.flight_state = ARM_OFF_STATE
        self.prev_flight_state = ARM_OFF_STATE

        self.enabled = False
        self.locked = False

        self.takeoff_requested = False
        self.pitch_ramp_active = False

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

        self.initial_diff_x = 0.0
        self.initial_diff_y = 0.0
        self.initial_diff_z = 0.0

        self.relief_search_started = False
        self.relief_search_done = False
        self.relief_candidates = []
        self.relief_candidate_index = 0
        self.relief_candidate_start_time = rospy.Time(0)
        self.relief_cost_sum = 0.0
        self.relief_cost_count = 0

        self.relief_start_pos = None
        self.relief_current_target = None
        self.relief_best_pos = None
        self.relief_best_cost = None
        self.relief_direction = [1.0, 0.0, 0.0]

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

        self.wrench_sub = rospy.Subscriber(
            self.external_wrench_topic,
            WrenchStamped,
            self.wrenchCallback,
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

        self.relief_target_pub = rospy.Publisher(
            "~relief_target_world",
            Vector3Stamped,
            queue_size=1
        )

        self.relief_best_pub = rospy.Publisher(
            "~relief_best_world",
            Vector3Stamped,
            queue_size=1,
            latch=True
        )

        rospy.Timer(
            rospy.Duration(0.02),
            self.timerCallback
        )

        rospy.logwarn("[perch_branch_arc_takeoff] initialized")
        rospy.logwarn("  final_pitch_deg                 : %.2f", self.final_pitch_deg)
        rospy.logwarn("  use_locked_initial_pitch        : %s", str(self.use_locked_initial_pitch_for_takeoff))
        rospy.logwarn("  pitch_rate_deg_per_sec          : %.2f", self.pitch_rate_deg_per_sec)
        rospy.logwarn("  enable_force_relief_search      : %s", str(self.enable_force_relief_search))
        rospy.logwarn("  relief_search_max_m             : %.3f", self.relief_search_max_m)
        rospy.logwarn("  relief_search_step_m            : %.3f", self.relief_search_step_m)
        rospy.logwarn("  relief_candidate_dwell_sec      : %.2f", self.relief_candidate_dwell_sec)
        rospy.logwarn("  hold_best_relief_position       : %s", str(self.hold_best_relief_position))

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

    # ============================================================
    # Callbacks
    # ============================================================

    def branchPoseCallback(self, msg):
        self.branch_pose_msg = msg

    def gimbalOdomCallback(self, msg):
        self.odom_msg = msg

    def wrenchCallback(self, msg):
        self.wrench_msg = msg

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
                "[perch_branch_arc_takeoff] HOVER detected. Wait %.2f sec, then force-relief search.",
                self.hover_settle_time_sec
            )

    def enableCallback(self, msg):
        self.enabled = bool(msg.data)

        if not self.enabled:
            self.takeoff_requested = False
            self.pitch_ramp_active = False
            self.locked = False
            self.hover_enter_time = None
            self.send_takeoff_until = rospy.Time(0)
            self.send_land_until = rospy.Time(0)
            self.roll_offset_cmd = 0.0
            self.yaw_offset_cmd = 0.0
            self.resetReliefSearch()

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
        self.pitch_ramp_active = False
        self.current_pitch_cmd = self.safeTakeoffPitchRad()
        self.resetReliefSearch()

        if self.publish_robot_takeoff_land:
            self.send_takeoff_until = rospy.Time.now() + rospy.Duration(1.0)
            rospy.logwarn("[perch_branch_arc_takeoff] will publish robot takeoff for 1 second")

        rospy.logwarn(
            "[perch_branch_arc_takeoff] takeoff requested. Hold pitch %.2f deg until HOVER.",
            math.degrees(self.current_pitch_cmd)
        )

    def landCallback(self, msg):
        if not msg.data:
            rospy.logwarn("[perch_branch_arc_takeoff] landing stopped")
            return

        self.takeoff_requested = False
        self.pitch_ramp_active = False

        if self.publish_robot_takeoff_land or self.immediate_land_on_request:
            self.send_land_until = rospy.Time.now() + rospy.Duration(1.0)
            rospy.logwarn("[perch_branch_arc_takeoff] immediate robot land command for 1 second")

    def relockCallback(self, msg):
        if not msg.data:
            return

        self.locked = False
        self.takeoff_requested = False
        self.pitch_ramp_active = False
        self.roll_offset_cmd = 0.0
        self.yaw_offset_cmd = 0.0
        self.hover_enter_time = None
        self.resetReliefSearch()

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

        self.initial_diff_x = dx
        self.initial_diff_y = dy
        self.initial_diff_z = dz

        self.current_pitch_cmd = self.safeTakeoffPitchRad()
        self.locked = True

        radius_msg = Float64()
        radius_msg.data = self.radius_3d
        self.locked_radius_pub.publish(radius_msg)

        rospy.logwarn(
            "[perch_branch_arc_takeoff] LOCKED"
            "\n  branch: x %.4f, y %.4f, z %.4f"
            "\n  robot : x %.4f, y %.4f, z %.4f"
            "\n  radius_3d %.4f m"
            "\n  initial pitch %.2f deg"
            "\n  takeoff pitch %.2f deg"
            "\n  final pitch %.2f deg",
            bx,
            by,
            bz,
            gx,
            gy,
            gz,
            self.radius_3d,
            math.degrees(self.initial_pitch),
            math.degrees(self.current_pitch_cmd),
            self.final_pitch_deg
        )

        return True

    # ============================================================
    # Pitch helpers
    # ============================================================

    def safeTakeoffPitchRad(self):
        if self.use_locked_initial_pitch_for_takeoff:
            return self.initial_pitch

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

    # ============================================================
    # Position / wrench helpers
    # ============================================================

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

    def externalForceTorqueNorm(self):
        if self.wrench_msg is None:
            return 0.0, 0.0

        f = self.wrench_msg.wrench.force
        t = self.wrench_msg.wrench.torque

        force_norm = math.sqrt(
            f.x * f.x +
            f.y * f.y +
            f.z * f.z
        )

        torque_norm = math.sqrt(
            t.x * t.x +
            t.y * t.y +
            t.z * t.z
        )

        return force_norm, torque_norm

    def wrenchCost(self):
        force_norm, torque_norm = self.externalForceTorqueNorm()

        return force_norm + self.force_cost_torque_weight * torque_norm

    def shouldPausePitchRamp(self):
        if not self.enable_force_gate:
            return False

        if not self.pause_pitch_ramp_on_high_force:
            return False

        force_norm, torque_norm = self.externalForceTorqueNorm()

        if force_norm > self.max_external_force_norm:
            return True

        if torque_norm > self.max_external_torque_norm:
            return True

        return False

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
    # Force-relief search
    # ============================================================

    def resetReliefSearch(self):
        self.relief_search_started = False
        self.relief_search_done = False
        self.relief_candidates = []
        self.relief_candidate_index = 0
        self.relief_candidate_start_time = rospy.Time(0)
        self.relief_cost_sum = 0.0
        self.relief_cost_count = 0
        self.relief_start_pos = None
        self.relief_current_target = None
        self.relief_best_pos = None
        self.relief_best_cost = None
        self.relief_direction = [1.0, 0.0, 0.0]

    def computeReliefDirection(self, current_pos):
        """
        Direction from branch to robot in horizontal XY plane.

        This is interpreted as:
            +direction = move slightly away/back from branch
            -direction = move slightly toward branch

        z is held constant during search to avoid altitude excitation.
        """
        dx = current_pos[0] - self.pivot[0]
        dy = current_pos[1] - self.pivot[1]

        length = math.sqrt(dx * dx + dy * dy)

        if length < 1.0e-5:
            return [1.0, 0.0, 0.0]

        return [
            dx / length,
            dy / length,
            0.0
        ]

    def startReliefSearch(self):
        current_pos, distance_error = self.getCurrentPositionAndDistanceError()

        if current_pos is None:
            rospy.logwarn("[perch_branch_arc_takeoff] cannot start relief search: no current position")
            return False

        self.relief_start_pos = current_pos
        self.relief_direction = self.computeReliefDirection(current_pos)

        offsets = []
        n = int(round(self.relief_search_max_m / self.relief_search_step_m))

        for i in range(-n, n + 1):
            offsets.append(i * self.relief_search_step_m)

        # Search order: current, small away, small toward, larger away, larger toward...
        ordered_offsets = [0.0]

        for k in range(1, n + 1):
            ordered_offsets.append(k * self.relief_search_step_m)
            ordered_offsets.append(-k * self.relief_search_step_m)

        self.relief_candidates = []

        for off in ordered_offsets:
            target = [
                self.relief_start_pos[0] + self.relief_direction[0] * off,
                self.relief_start_pos[1] + self.relief_direction[1] * off,
                self.relief_start_pos[2]
            ]

            jump = norm3(
                target[0] - self.relief_start_pos[0],
                target[1] - self.relief_start_pos[1],
                target[2] - self.relief_start_pos[2]
            )

            if jump <= self.max_relief_position_jump_m:
                self.relief_candidates.append(target)

        self.relief_candidate_index = 0
        self.relief_candidate_start_time = rospy.Time.now()
        self.relief_cost_sum = 0.0
        self.relief_cost_count = 0
        self.relief_current_target = self.relief_candidates[0]
        self.relief_best_pos = self.relief_current_target
        self.relief_best_cost = None

        self.relief_search_started = True
        self.relief_search_done = False

        rospy.logwarn(
            "[perch_branch_arc_takeoff] force-relief search started. candidates: %d, direction: %.3f %.3f %.3f",
            len(self.relief_candidates),
            self.relief_direction[0],
            self.relief_direction[1],
            self.relief_direction[2]
        )

        return True

    def updateReliefSearch(self):
        if not self.relief_search_started:
            return

        if self.relief_search_done:
            return

        now = rospy.Time.now()
        elapsed = (now - self.relief_candidate_start_time).to_sec()

        # Wait for candidate to settle before measuring wrench.
        if elapsed >= self.relief_settle_before_measure_sec:
            self.relief_cost_sum += self.wrenchCost()
            self.relief_cost_count += 1

        # Move to next candidate after dwell time.
        if elapsed < self.relief_candidate_dwell_sec:
            return

        if self.relief_cost_count > 0:
            avg_cost = self.relief_cost_sum / float(self.relief_cost_count)
        else:
            avg_cost = 999999.0

        current_candidate = self.relief_candidates[self.relief_candidate_index]

        rospy.logwarn(
            "[perch_branch_arc_takeoff] relief candidate %d/%d cost %.3f pos %.3f %.3f %.3f",
            self.relief_candidate_index + 1,
            len(self.relief_candidates),
            avg_cost,
            current_candidate[0],
            current_candidate[1],
            current_candidate[2]
        )

        if self.relief_best_cost is None or avg_cost < self.relief_best_cost:
            self.relief_best_cost = avg_cost
            self.relief_best_pos = current_candidate

        self.relief_candidate_index += 1

        if self.relief_candidate_index >= len(self.relief_candidates):
            self.relief_search_done = True
            self.relief_current_target = self.relief_best_pos

            self.publishReliefBest()

            rospy.logwarn(
                "[perch_branch_arc_takeoff] force-relief search done. best cost %.3f best pos %.3f %.3f %.3f",
                self.relief_best_cost,
                self.relief_best_pos[0],
                self.relief_best_pos[1],
                self.relief_best_pos[2]
            )

            return

        self.relief_current_target = self.relief_candidates[self.relief_candidate_index]
        self.relief_candidate_start_time = now
        self.relief_cost_sum = 0.0
        self.relief_cost_count = 0

    def publishReliefTarget(self):
        if self.relief_current_target is None:
            return

        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.target_world_frame
        msg.vector.x = self.relief_current_target[0]
        msg.vector.y = self.relief_current_target[1]
        msg.vector.z = self.relief_current_target[2]

        self.relief_target_pub.publish(msg)

    def publishReliefBest(self):
        if self.relief_best_pos is None:
            return

        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.target_world_frame
        msg.vector.x = self.relief_best_pos[0]
        msg.vector.y = self.relief_best_pos[1]
        msg.vector.z = self.relief_best_pos[2]

        self.relief_best_pub.publish(msg)

    # ============================================================
    # Publishers
    # ============================================================

    def publishRobotStateCommands(self):
        now = rospy.Time.now()

        if now < self.send_takeoff_until:
            self.robot_takeoff_pub.publish(Empty())

        if now < self.send_land_until:
            self.robot_land_pub.publish(Empty())

    def publishCommand(self):
        if not self.locked:
            return

        current_pos, distance_error = self.getCurrentPositionAndDistanceError()
        force_norm, torque_norm = self.externalForceTorqueNorm()

        self.updateAdaptiveRollYaw(distance_error)

        target_pitch = clamp(
            self.current_pitch_cmd,
            -math.radians(self.max_commanded_pitch_deg),
            math.radians(self.max_commanded_pitch_deg)
        )

        q_cmd = self.commandQuaternionFromPitchRollYaw(target_pitch)
        roll_cmd, pitch_cmd, yaw_cmd = euler_from_quaternion(q_cmd)

        nav = FlightNav()

        # Position command is only used during relief search or after best position found.
        use_position = False
        target_pos = None

        if self.relief_search_started and not self.relief_search_done:
            use_position = True
            target_pos = self.relief_current_target

        elif self.relief_search_done and self.hold_best_relief_position:
            use_position = True
            target_pos = self.relief_best_pos

        if use_position and target_pos is not None:
            nav.pos_xy_nav_mode = NAV_MODE_POS
            nav.pos_z_nav_mode = NAV_MODE_POS
            nav.target_pos_x = target_pos[0]
            nav.target_pos_y = target_pos[1]
            nav.target_pos_z = target_pos[2]
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

        self.nav_pub.publish(nav)

        q_msg = QuaternionStamped()
        q_msg.header.stamp = rospy.Time.now()
        q_msg.header.frame_id = self.target_world_frame
        q_msg.quaternion.x = q_cmd[0]
        q_msg.quaternion.y = q_cmd[1]
        q_msg.quaternion.z = q_cmd[2]
        q_msg.quaternion.w = q_cmd[3]

        self.attitude_pub.publish(q_msg)

        self.publishReliefTarget()

        self.printDebug(
            target_pitch,
            distance_error,
            current_pos,
            force_norm,
            torque_norm,
            use_position,
            target_pos
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
        self.diff_world_pub.publish(diff_world_msg)

        q_robot = [
            robot.orientation.x,
            robot.orientation.y,
            robot.orientation.z,
            robot.orientation.w
        ]

        q_robot_inv = quaternion_inverse(q_robot)
        diff_g = quat_rotate_vector(q_robot_inv, [dx, dy, dz])

        diff_gimbal_msg = Vector3Stamped()
        diff_gimbal_msg.header.stamp = now
        diff_gimbal_msg.header.frame_id = "baselink"
        diff_gimbal_msg.vector.x = diff_g[0]
        diff_gimbal_msg.vector.y = diff_g[1]
        diff_gimbal_msg.vector.z = diff_g[2]
        self.diff_gimbal_pub.publish(diff_gimbal_msg)

    # ============================================================
    # Timer
    # ============================================================

    def timerCallback(self, event):
        now = rospy.Time.now()

        dt = (now - self.last_time).to_sec()
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

        max_pitch_step = math.radians(self.pitch_rate_deg_per_sec) * max(0.0, dt)

        if self.flight_state == TAKEOFF_STATE:
            self.current_pitch_cmd = self.safeTakeoffPitchRad()

        elif self.flight_state == HOVER_STATE:
            if self.hover_enter_time is not None:
                elapsed = (now - self.hover_enter_time).to_sec()
            else:
                elapsed = 0.0

            if elapsed >= self.hover_settle_time_sec:
                if self.enable_force_relief_search and not self.relief_search_started:
                    self.startReliefSearch()

                if self.relief_search_started and not self.relief_search_done:
                    self.updateReliefSearch()

                if (not self.enable_force_relief_search) or self.relief_search_done:
                    self.pitch_ramp_active = True

            if self.pitch_ramp_active:
                if self.shouldPausePitchRamp():
                    pass
                else:
                    self.current_pitch_cmd = shortest_angle_step(
                        self.current_pitch_cmd,
                        self.finalPitchRad(),
                        max_pitch_step
                    )
            else:
                self.current_pitch_cmd = self.safeTakeoffPitchRad()

        else:
            self.current_pitch_cmd = self.safeTakeoffPitchRad()

        self.publishCommand()

    def printDebug(
        self,
        target_pitch,
        distance_error,
        current_pos,
        force_norm,
        torque_norm,
        use_position,
        target_pos
    ):
        now = rospy.Time.now()

        if (now - self.last_print_time).to_sec() < 1.0 / self.print_rate:
            return

        self.last_print_time = now

        if current_pos is None:
            current_pos_text = "None"
        else:
            current_pos_text = "x %.4f, y %.4f, z %.4f" % (
                current_pos[0],
                current_pos[1],
                current_pos[2]
            )

        if target_pos is None:
            target_pos_text = "None"
        else:
            target_pos_text = "x %.4f, y %.4f, z %.4f" % (
                target_pos[0],
                target_pos[1],
                target_pos[2]
            )

        rospy.loginfo(
            "\n[perch_branch_arc_takeoff]"
            "\n  enabled / locked:"
            "\n    %s / %s"
            "\n  flight_state:"
            "\n    %d"
            "\n  relief search:"
            "\n    started %s, done %s, index %d/%d"
            "\n    best cost %s"
            "\n  use_position:"
            "\n    %s"
            "\n  pitch command:"
            "\n    current %.2f deg"
            "\n    final   %.2f deg"
            "\n  wrench:"
            "\n    force %.2f N"
            "\n    torque %.2f Nm"
            "\n    cost %.2f"
            "\n    pause pitch %s"
            "\n  branch distance error:"
            "\n    %.4f m"
            "\n  current robot position:"
            "\n    %s"
            "\n  target position:"
            "\n    %s",
            str(self.enabled),
            str(self.locked),
            self.flight_state,
            str(self.relief_search_started),
            str(self.relief_search_done),
            self.relief_candidate_index,
            len(self.relief_candidates),
            "None" if self.relief_best_cost is None else "%.3f" % self.relief_best_cost,
            str(use_position),
            math.degrees(target_pitch),
            self.final_pitch_deg,
            force_norm,
            torque_norm,
            self.wrenchCost(),
            str(self.shouldPausePitchRamp()),
            distance_error,
            current_pos_text,
            target_pos_text
        )


if __name__ == "__main__":
    try:
        node = PerchBranchArcTakeoff()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass