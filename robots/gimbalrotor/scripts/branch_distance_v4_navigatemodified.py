#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys
import select
import termios
import tty
import threading

import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from aerial_robot_msgs.msg import FlightNav

from tf.transformations import (
    euler_from_quaternion,
    quaternion_inverse,
    quaternion_multiply,
)


class BranchDistanceNavigator(object):
    def __init__(self):
        rospy.init_node("branch_distance_navigator", anonymous=False)

        # ============================================================
        # Topics
        # ============================================================

        self.robot_ns = rospy.get_param("~robot_ns", "/gimbalrotor")

        # Branch mocap pose.
        #
        # For your branch:
        #   robot_id := 2
        #
        # From your mocap launch:
        #   pose: mocap/pose
        #
        # Therefore this node subscribes to:
        #   /mocap/pose
        #
        # If later you remap the branch mocap to another topic,
        # change this rosparam.
        self.branch_pose_topic = rospy.get_param(
            "~branch_pose_topic",
            "/mocap/pose"
        )

        # Robot odometry.
        self.gimbal_odom_topic = rospy.get_param(
            "~gimbal_odom_topic",
            self.robot_ns + "/uav/baselink/odom"
        )

        # Correct gimbalrotor navigation topic.
        # This follows your first file.
        self.nav_topic = rospy.get_param(
            "~nav_topic",
            self.robot_ns + "/uav/nav"
        )

        # Correct attitude target topic.
        # This follows your first file.
        self.baselink_rpy_topic = rospy.get_param(
            "~baselink_rpy_topic",
            self.robot_ns + "/final_target_baselink_rpy"
        )

        # Debug emergency topic.
        self.emergency_topic = rospy.get_param(
            "~emergency_topic",
            "~emergency_stop"
        )

        # ============================================================
        # Parameters
        # ============================================================

        self.print_rate = rospy.get_param("~print_rate", 5.0)
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 30.0)

        # Target offset from branch in world frame.
        #
        # Meaning:
        #   target_x = branch_x - x_offset
        #   target_y = branch_y + y_offset
        #   target_z = branch_z + z_offset
        #
        # If your robot goes to the wrong side of the branch,
        # change x_offset to -0.7.
        self.x_offset = rospy.get_param("~x_offset", 0.7)
        self.y_offset = rospy.get_param("~y_offset", 0.0)
        self.z_offset = rospy.get_param("~z_offset", 0.0)

        # ============================================================
        # Enable / disable each approach or alignment part
        # ============================================================
        # Requested behavior:
        #   enable_x_distance_approach := true/false
        #   enable_z_distance_approach := true/false
        #   enable_roll_alignment      := true/false
        #   enable_yaw_alignment       := true/false
        #
        # Backward compatibility:
        #   If old align_roll / align_yaw params are used, they still work.

        self.enable_x_distance_approach = rospy.get_param(
            "~enable_x_distance_approach",
            True
        )

        self.enable_z_distance_approach = rospy.get_param(
            "~enable_z_distance_approach",
            True
        )

        self.enable_roll_alignment = rospy.get_param(
            "~enable_roll_alignment",
            rospy.get_param("~align_roll", True)
        )

        self.enable_yaw_alignment = rospy.get_param(
            "~enable_yaw_alignment",
            rospy.get_param("~align_yaw", True)
        )

        # During this branch-distance navigation phase, the main body pitch
        # should be commanded to zero. This is NOT branch-pitch alignment.
        # Roll/yaw are used to make the robot orientation suitable for the
        # branch/perching phase, while pitch stays level.
        self.force_zero_pitch = rospy.get_param("~force_zero_pitch", True)
        self.target_pitch_zero = rospy.get_param("~target_pitch_zero", 0.0)

        # Arrival / stage tolerances.
        self.arrival_position_tolerance = rospy.get_param(
            "~arrival_position_tolerance",
            0.08
        )

        self.x_distance_tolerance = rospy.get_param(
            "~x_distance_tolerance",
            self.arrival_position_tolerance
        )

        self.z_distance_tolerance = rospy.get_param(
            "~z_distance_tolerance",
            self.arrival_position_tolerance
        )

        self.roll_alignment_tolerance_deg = rospy.get_param(
            "~roll_alignment_tolerance_deg",
            2.0
        )

        self.yaw_alignment_tolerance_deg = rospy.get_param(
            "~yaw_alignment_tolerance_deg",
            2.0
        )

        self.pitch_zero_tolerance_deg = rospy.get_param(
            "~pitch_zero_tolerance_deg",
            2.0
        )

        self.roll_alignment_tolerance = math.radians(
            self.roll_alignment_tolerance_deg
        )

        self.yaw_alignment_tolerance = math.radians(
            self.yaw_alignment_tolerance_deg
        )

        self.pitch_zero_tolerance = math.radians(
            self.pitch_zero_tolerance_deg
        )

        # ============================================================
        # Slow approach parameters
        # ============================================================
        # The node slowly moves a command target from the current robot pose
        # toward the active stage target.
        #
        # Example:
        #   approach_linear_speed = 0.10
        # means the published target moves at maximum 0.10 m/s.
        #
        # Example:
        #   approach_angular_speed_deg = 10.0
        # means roll/pitch/yaw command changes at maximum 10 deg/s.

        self.approach_linear_speed = rospy.get_param(
            "~approach_linear_speed",
            0.10
        )

        self.approach_angular_speed_deg = rospy.get_param(
            "~approach_angular_speed_deg",
            10.0
        )

        self.approach_angular_speed = math.radians(
            self.approach_angular_speed_deg
        )

        # ============================================================
        # State
        # ============================================================

        self.branch_pose_msg = None
        self.gimbal_odom_msg = None

        self.last_print_time = rospy.Time.now()

        self.confirmed = False
        self.confirmation_asked = False

        # Final target calculated from branch.
        self.target_position = None
        self.target_rpy = None

        # Slowly moving command target.
        # This is what is actually published.
        self.command_position = None
        self.command_rpy = None
        self.last_command_time = None

        # Navigation phase state.
        # Current requested behavior:
        #   1. Move position first.
        #      - x moves only if enable_x_distance_approach is true.
        #      - y always moves to branch_y + y_offset.
        #      - z moves only if enable_z_distance_approach is true.
        #   2. After position is reached, do orientation alignment.
        #      - roll aligns only if enable_roll_alignment is true.
        #      - yaw aligns only if enable_yaw_alignment is true.
        #      - pitch is commanded to zero if force_zero_pitch is true.
        self.navigation_phase = "position"
        self.last_phase_name = None

        self.emergency_stop = False
        self.hover_position = None
        self.hover_rpy = None

        self.keyboard_thread_started = False
        self.quit_flag = False

        self.state_lock = threading.Lock()

        # ============================================================
        # Subscribers
        # ============================================================

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

        # ============================================================
        # Publishers
        # ============================================================

        self.nav_pub = rospy.Publisher(
            self.nav_topic,
            FlightNav,
            queue_size=10
        )

        self.baselink_rpy_pub = rospy.Publisher(
            self.baselink_rpy_topic,
            Vector3Stamped,
            queue_size=1
        )

        self.emergency_pub = rospy.Publisher(
            self.emergency_topic,
            Bool,
            queue_size=1
        )

        # Debug publishers, same idea as your original distance file.
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

        rospy.logwarn("[branch_distance_nav] initialized")
        rospy.logwarn("[branch_distance_nav] branch pose topic: %s", self.branch_pose_topic)
        rospy.logwarn("[branch_distance_nav] gimbal odom topic: %s", self.gimbal_odom_topic)
        rospy.logwarn("[branch_distance_nav] nav topic: %s", self.nav_topic)
        rospy.logwarn("[branch_distance_nav] baselink rpy topic: %s", self.baselink_rpy_topic)
        rospy.logwarn("[branch_distance_nav] x_offset: %.3f", self.x_offset)
        rospy.logwarn("[branch_distance_nav] y_offset: %.3f", self.y_offset)
        rospy.logwarn("[branch_distance_nav] z_offset: %.3f", self.z_offset)
        rospy.logwarn("[branch_distance_nav] enable_x_distance_approach: %s", str(self.enable_x_distance_approach))
        rospy.logwarn("[branch_distance_nav] enable_z_distance_approach: %s", str(self.enable_z_distance_approach))
        rospy.logwarn("[branch_distance_nav] enable_roll_alignment: %s", str(self.enable_roll_alignment))
        rospy.logwarn("[branch_distance_nav] enable_yaw_alignment: %s", str(self.enable_yaw_alignment))
        rospy.logwarn("[branch_distance_nav] force_zero_pitch: %s", str(self.force_zero_pitch))
        rospy.logwarn("[branch_distance_nav] approach linear speed: %.3f m/s", self.approach_linear_speed)
        rospy.logwarn("[branch_distance_nav] approach angular speed: %.3f deg/s", self.approach_angular_speed_deg)

    # ================================================================
    # Callbacks
    # ================================================================

    def branchPoseCallback(self, msg):
        with self.state_lock:
            self.branch_pose_msg = msg

    def gimbalOdomCallback(self, msg):
        with self.state_lock:
            self.gimbal_odom_msg = msg

    # ================================================================
    # Helper functions
    # ================================================================

    def makeVector3Stamped(self, x, y, z, frame_id="world"):
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = z
        return msg

    def getCurrentRobotPositionAndRPY(self):
        if self.gimbal_odom_msg is None:
            return None, None

        pose = self.gimbal_odom_msg.pose.pose

        position = [
            pose.position.x,
            pose.position.y,
            pose.position.z
        ]

        q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]

        roll, pitch, yaw = euler_from_quaternion(q)

        rpy = [roll, pitch, yaw]

        return position, rpy

    def normalizeAngle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def angleDifference(self, target, current):
        return self.normalizeAngle(target - current)

    def moveScalarToward(self, current, target, max_step):
        error = target - current

        if abs(error) <= max_step:
            return target

        if error > 0.0:
            return current + max_step
        else:
            return current - max_step

    def moveAngleToward(self, current, target, max_step):
        error = self.angleDifference(target, current)

        if abs(error) <= max_step:
            return target

        if error > 0.0:
            return self.normalizeAngle(current + max_step)
        else:
            return self.normalizeAngle(current - max_step)

    def isPositionDone(self, data):
        errors = []

        # y is always part of position approach.
        errors.append(abs(data["y_error_to_target"]))

        if self.enable_x_distance_approach:
            errors.append(abs(data["x_error_to_target"]))

        if self.enable_z_distance_approach:
            errors.append(abs(data["z_error_to_target"]))

        if len(errors) == 0:
            return True

        return max(errors) <= self.arrival_position_tolerance

    def isOrientationDone(self, data):
        errors = []

        if self.enable_roll_alignment:
            errors.append(abs(data["roll_error_to_target"]))

        if self.force_zero_pitch:
            errors.append(abs(data["pitch_error_to_target"]))

        if self.enable_yaw_alignment:
            errors.append(abs(data["yaw_error_to_target"]))

        if len(errors) == 0:
            return True

        return max(errors) <= max(
            self.roll_alignment_tolerance,
            self.yaw_alignment_tolerance,
            self.pitch_zero_tolerance
        )

    def updateNavigationPhaseIfNeeded(self, data):
        if self.navigation_phase == "position":
            if self.isPositionDone(data):
                self.navigation_phase = "orientation"
                rospy.logwarn(
                    "[branch_distance_nav] position reached -> start orientation alignment"
                )

        elif self.navigation_phase == "orientation":
            if self.isOrientationDone(data):
                self.navigation_phase = "done"
                rospy.logwarn(
                    "[branch_distance_nav] position and orientation alignment complete"
                )

        return self.navigation_phase

    def buildPhasedTarget(self, data):
        """
        Build the current target.

        New requested behavior:
          - No more z -> yaw -> roll -> x sequential stage.
          - First move position directly.
          - After position is reached, align orientation.
          - Slow approach is still used.
          - Enable/disable flags are still respected.
        """

        if self.command_position is None or self.command_rpy is None:
            return data["target_position"], data["target_rpy"], "done"

        active_phase = self.updateNavigationPhaseIfNeeded(data)

        phased_position = list(self.command_position)
        phased_rpy = list(self.command_rpy)

        final_position = data["target_position"]
        final_rpy = data["target_rpy"]

        if active_phase == "position":
            # Move to the target position first.
            # x and z can be enabled/disabled.
            # y is always included because you now requested movement in all direction.
            if self.enable_x_distance_approach:
                phased_position[0] = final_position[0]

            phased_position[1] = final_position[1]

            if self.enable_z_distance_approach:
                phased_position[2] = final_position[2]

            # Do not newly align roll/yaw during position approach.
            # Keep the current slowly commanded attitude.

        elif active_phase == "orientation" or active_phase == "done":
            # Keep the final position after position phase.
            if self.enable_x_distance_approach:
                phased_position[0] = final_position[0]

            phased_position[1] = final_position[1]

            if self.enable_z_distance_approach:
                phased_position[2] = final_position[2]

            # After position is reached, align orientation.
            if self.enable_roll_alignment:
                phased_rpy[0] = final_rpy[0]

            if self.force_zero_pitch:
                phased_rpy[1] = final_rpy[1]

            if self.enable_yaw_alignment:
                phased_rpy[2] = final_rpy[2]

        return phased_position, phased_rpy, active_phase

    def updateSlowCommandTarget(self, final_position, final_rpy):
        """
        Slowly update command_position and command_rpy toward the current
        phased target.

        final_position / final_rpy here may be a partial phase target,
        not always the full final branch target.
        """

        now = rospy.Time.now()

        if self.last_command_time is None:
            self.last_command_time = now
            return

        dt = (now - self.last_command_time).to_sec()
        self.last_command_time = now

        if dt <= 0.0:
            return

        max_pos_step = self.approach_linear_speed * dt
        max_ang_step = self.approach_angular_speed * dt

        # ------------------------------------------------------------
        # Slowly update position
        # ------------------------------------------------------------

        dx = final_position[0] - self.command_position[0]
        dy = final_position[1] - self.command_position[1]
        dz = final_position[2] - self.command_position[2]

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist <= max_pos_step or dist < 1e-9:
            self.command_position = list(final_position)
        else:
            ratio = max_pos_step / dist
            self.command_position[0] += dx * ratio
            self.command_position[1] += dy * ratio
            self.command_position[2] += dz * ratio

        # ------------------------------------------------------------
        # Slowly update roll, pitch, yaw
        # ------------------------------------------------------------

        self.command_rpy[0] = self.moveAngleToward(
            self.command_rpy[0],
            final_rpy[0],
            max_ang_step
        )

        self.command_rpy[1] = self.moveAngleToward(
            self.command_rpy[1],
            final_rpy[1],
            max_ang_step
        )

        self.command_rpy[2] = self.moveAngleToward(
            self.command_rpy[2],
            final_rpy[2],
            max_ang_step
        )

    def publishNavAndRPY(self, position, rpy):
        now = rospy.Time.now()

        # ============================================================
        # 1. Publish attitude target
        #    Topic:
        #      /gimbalrotor/final_target_baselink_rpy
        # ============================================================

        rpy_msg = Vector3Stamped()
        rpy_msg.header.stamp = now
        rpy_msg.header.frame_id = "world"
        rpy_msg.vector.x = rpy[0]
        rpy_msg.vector.y = rpy[1]
        rpy_msg.vector.z = rpy[2]
        self.baselink_rpy_pub.publish(rpy_msg)

        # ============================================================
        # 2. Publish position target
        #    Topic:
        #      /gimbalrotor/uav/nav
        # ============================================================

        nav_msg = FlightNav()
        nav_msg.header.stamp = now

        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG

        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_x = position[0]
        nav_msg.target_pos_y = position[1]

        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_z = position[2]

        # Yaw is already controlled by final_target_baselink_rpy.
        # This follows your first working style.
        nav_msg.yaw_nav_mode = 0

        self.nav_pub.publish(nav_msg)

    def triggerEmergencyHover(self):
        with self.state_lock:
            if self.gimbal_odom_msg is None:
                rospy.logerr("[branch_distance_nav] emergency requested, but no odom available")
                return

            position, rpy = self.getCurrentRobotPositionAndRPY()

            if position is None or rpy is None:
                rospy.logerr("[branch_distance_nav] emergency requested, but current pose unavailable")
                return

            self.emergency_stop = True
            self.hover_position = position
            self.hover_rpy = rpy

            # Also reset slow command to current pose.
            # This avoids continuing from the old moving command after emergency.
            self.command_position = list(position)
            self.command_rpy = list(rpy)
            self.last_command_time = rospy.Time.now()

        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)

        rospy.logerr("")
        rospy.logerr("============================================================")
        rospy.logerr("[EMERGENCY HOVER]")
        rospy.logerr("Navigation target changed to current robot position.")
        rospy.logerr("The node is NOT killing the robot.")
        rospy.logerr("The robot should keep flying / hovering at current pose.")
        rospy.logerr("============================================================")
        rospy.logerr("")

    def keyboardEmergencyThread(self):
        old_settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())

            while not rospy.is_shutdown() and not self.quit_flag:
                if select.select([sys.stdin], [], [], 0.05)[0]:
                    key = sys.stdin.read(1)

                    if key == "e":
                        self.triggerEmergencyHover()

                    elif key == "q":
                        self.triggerEmergencyHover()

                    elif key == "\x03":
                        self.triggerEmergencyHover()

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def startKeyboardThreadOnce(self):
        if self.keyboard_thread_started:
            return

        self.keyboard_thread_started = True

        thread = threading.Thread(target=self.keyboardEmergencyThread)
        thread.daemon = True
        thread.start()

        rospy.logwarn("[branch_distance_nav] emergency keyboard enabled")
        rospy.logwarn("[branch_distance_nav] press 'e' or 'q' for emergency hover")

    # ================================================================
    # Main calculation
    # ================================================================

    def calculateTarget(self):
        with self.state_lock:
            branch_msg = self.branch_pose_msg
            odom_msg = self.gimbal_odom_msg

        if branch_msg is None:
            return None

        if odom_msg is None:
            return None

        now = rospy.Time.now()

        # ============================================================
        # 1. Branch pose from mocap
        # ============================================================

        branch_pose = branch_msg.pose

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

        branch_distance_from_world_origin = math.sqrt(
            branch_x * branch_x +
            branch_y * branch_y +
            branch_z * branch_z
        )

        # ============================================================
        # 2. Gimbalrotor pose from odom
        # ============================================================

        gimbal_pose = odom_msg.pose.pose

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

        gimbal_distance_from_world_origin = math.sqrt(
            gimbal_x * gimbal_x +
            gimbal_y * gimbal_y +
            gimbal_z * gimbal_z
        )

        # ============================================================
        # 3. Position difference
        # ============================================================

        diff_x = branch_x - gimbal_x
        diff_y = branch_y - gimbal_y
        diff_z = branch_z - gimbal_z

        distance_gimbal_to_branch = math.sqrt(
            diff_x * diff_x +
            diff_y * diff_y +
            diff_z * diff_z
        )

        # ============================================================
        # 4. Attitude difference
        # ============================================================

        q_relative = quaternion_multiply(
            quaternion_inverse(q_gimbal),
            q_branch
        )

        rel_roll, rel_pitch, rel_yaw = euler_from_quaternion(q_relative)

        simple_diff_roll = self.angleDifference(branch_roll, gimbal_roll)
        simple_diff_pitch = self.angleDifference(branch_pitch, gimbal_pitch)
        simple_diff_yaw = self.angleDifference(branch_yaw, gimbal_yaw)

        # ============================================================
        # 5. Target position
        # ============================================================

        target_x = branch_x - self.x_offset
        target_y = branch_y + self.y_offset
        target_z = branch_z + self.z_offset

        target_position = [
            target_x,
            target_y,
            target_z
        ]

        # ============================================================
        # 6. Target attitude
        # ============================================================
        # Roll and yaw can be enabled/disabled.
        # Pitch is NOT aligned to the branch. It is forced to zero in this
        # phase so the whole robot body does not keep a pitch angle.

        if self.enable_roll_alignment:
            target_roll = branch_roll
        else:
            target_roll = gimbal_roll

        if self.force_zero_pitch:
            target_pitch = self.target_pitch_zero
        else:
            target_pitch = gimbal_pitch

        if self.enable_yaw_alignment:
            target_yaw = branch_yaw
        else:
            target_yaw = gimbal_yaw

        target_rpy = [
            target_roll,
            target_pitch,
            target_yaw
        ]

        x_error_to_target = target_x - gimbal_x
        y_error_to_target = target_y - gimbal_y
        z_error_to_target = target_z - gimbal_z
        roll_error_to_target = self.angleDifference(target_roll, gimbal_roll)
        pitch_error_to_target = self.angleDifference(target_pitch, gimbal_pitch)
        yaw_error_to_target = self.angleDifference(target_yaw, gimbal_yaw)

        # ============================================================
        # 7. Publish debug topics
        # ============================================================

        self.branch_position_world_pub.publish(
            self.makeVector3Stamped(branch_x, branch_y, branch_z)
        )

        self.gimbal_position_world_pub.publish(
            self.makeVector3Stamped(gimbal_x, gimbal_y, gimbal_z)
        )

        self.position_difference_world_pub.publish(
            self.makeVector3Stamped(diff_x, diff_y, diff_z)
        )

        self.branch_rpy_world_pub.publish(
            self.makeVector3Stamped(branch_roll, branch_pitch, branch_yaw)
        )

        self.gimbal_rpy_world_pub.publish(
            self.makeVector3Stamped(gimbal_roll, gimbal_pitch, gimbal_yaw)
        )

        self.attitude_difference_rpy_pub.publish(
            self.makeVector3Stamped(rel_roll, rel_pitch, rel_yaw, "gimbal_to_branch")
        )

        active_phase_for_print = self.navigation_phase

        # ============================================================
        # 8. Print debug
        # ============================================================

        if (now - self.last_print_time).to_sec() > 1.0 / self.print_rate:
            self.last_print_time = now

            rospy.loginfo(
                "\n[branch_distance_nav]"
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
                "\n  ========== navigation final target =========="
                "\n  final target position:"
                "\n    x: %.4f"
                "\n    y: %.4f"
                "\n    z: %.4f"
                "\n  final target attitude:"
                "\n    roll : %.4f rad / %.2f deg"
                "\n    pitch: %.4f rad / %.2f deg"
                "\n    yaw  : %.4f rad / %.2f deg"
                "\n"
                "\n  ========== phase errors =========="
                "\n  active phase: %s"
                "\n  x_error_to_target    : %.4f m"
                "\n  y_error_to_target    : %.4f m"
                "\n  z_error_to_target    : %.4f m"
                "\n  roll_error_to_target : %.4f rad / %.2f deg"
                "\n  pitch_error_to_zero  : %.4f rad / %.2f deg"
                "\n  yaw_error_to_target  : %.4f rad / %.2f deg"
                "\n"
                "\n  ========== attitude difference =========="
                "\n  quaternion relative:"
                "\n    roll : %.4f rad / %.2f deg"
                "\n    pitch: %.4f rad / %.2f deg"
                "\n    yaw  : %.4f rad / %.2f deg"
                "\n  simple branch_rpy - gimbal_rpy:"
                "\n    roll : %.4f rad / %.2f deg"
                "\n    pitch: %.4f rad / %.2f deg"
                "\n    yaw  : %.4f rad / %.2f deg"
                "\n"
                "\n  ========== safety =========="
                "\n  confirmed: %s"
                "\n  emergency_stop: %s"
                "\n  slow approach speed: %.3f m/s"
                "\n  Press 'e' or 'q' for emergency hover.",
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
                target_x,
                target_y,
                target_z,
                target_roll,
                math.degrees(target_roll),
                target_pitch,
                math.degrees(target_pitch),
                target_yaw,
                math.degrees(target_yaw),
                active_phase_for_print,
                x_error_to_target,
                y_error_to_target,
                z_error_to_target,
                roll_error_to_target,
                math.degrees(roll_error_to_target),
                pitch_error_to_target,
                math.degrees(pitch_error_to_target),
                yaw_error_to_target,
                math.degrees(yaw_error_to_target),
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
                str(self.confirmed),
                str(self.emergency_stop),
                self.approach_linear_speed,
            )

        data = {
            "branch_x": branch_x,
            "branch_y": branch_y,
            "branch_z": branch_z,
            "branch_roll": branch_roll,
            "branch_pitch": branch_pitch,
            "branch_yaw": branch_yaw,

            "gimbal_x": gimbal_x,
            "gimbal_y": gimbal_y,
            "gimbal_z": gimbal_z,
            "gimbal_roll": gimbal_roll,
            "gimbal_pitch": gimbal_pitch,
            "gimbal_yaw": gimbal_yaw,

            "diff_x": diff_x,
            "diff_y": diff_y,
            "diff_z": diff_z,
            "distance_gimbal_to_branch": distance_gimbal_to_branch,

            "target_position": target_position,
            "target_rpy": target_rpy,

            "x_error_to_target": x_error_to_target,
            "y_error_to_target": y_error_to_target,
            "z_error_to_target": z_error_to_target,
            "roll_error_to_target": roll_error_to_target,
            "pitch_error_to_target": pitch_error_to_target,
            "yaw_error_to_target": yaw_error_to_target,
        }

        return data

    # ================================================================
    # Keyboard confirmation
    # ================================================================

    def askKeyboardConfirmation(self, data):
        rospy.logwarn("")
        rospy.logwarn("============================================================")
        rospy.logwarn("[CONFIRM BRANCH NAVIGATION TARGET]")
        rospy.logwarn("")
        rospy.logwarn("Current distance to branch:")
        rospy.logwarn("  %.4f m", data["distance_gimbal_to_branch"])
        rospy.logwarn("")
        rospy.logwarn("Current difference branch - gimbal:")
        rospy.logwarn("  diff_x: %.4f", data["diff_x"])
        rospy.logwarn("  diff_y: %.4f", data["diff_y"])
        rospy.logwarn("  diff_z: %.4f", data["diff_z"])
        rospy.logwarn("")
        rospy.logwarn("Final target position:")
        rospy.logwarn("  x: %.4f", data["target_position"][0])
        rospy.logwarn("  y: %.4f", data["target_position"][1])
        rospy.logwarn("  z: %.4f", data["target_position"][2])
        rospy.logwarn("")
        rospy.logwarn("Final target attitude:")
        rospy.logwarn("  roll : %.4f rad / %.2f deg",
                      data["target_rpy"][0],
                      math.degrees(data["target_rpy"][0]))
        rospy.logwarn("  pitch: %.4f rad / %.2f deg",
                      data["target_rpy"][1],
                      math.degrees(data["target_rpy"][1]))
        rospy.logwarn("  yaw  : %.4f rad / %.2f deg",
                      data["target_rpy"][2],
                      math.degrees(data["target_rpy"][2]))
        rospy.logwarn("")
        rospy.logwarn("Navigation phase setting:")
        rospy.logwarn("  1. position approach first")
        rospy.logwarn("     x distance approach : %s", str(self.enable_x_distance_approach))
        rospy.logwarn("     y distance approach : always true")
        rospy.logwarn("     z distance approach : %s", str(self.enable_z_distance_approach))
        rospy.logwarn("  2. orientation alignment after position")
        rospy.logwarn("     roll alignment      : %s", str(self.enable_roll_alignment))
        rospy.logwarn("     yaw alignment       : %s", str(self.enable_yaw_alignment))
        rospy.logwarn("     force main body pitch zero: %s", str(self.force_zero_pitch))
        rospy.logwarn("")
        rospy.logwarn("Tolerances:")
        rospy.logwarn("  position tolerance     : %.4f m", self.arrival_position_tolerance)
        rospy.logwarn("  x distance tolerance   : %.4f m", self.x_distance_tolerance)
        rospy.logwarn("  z distance tolerance   : %.4f m", self.z_distance_tolerance)
        rospy.logwarn("  roll tolerance         : %.4f deg", self.roll_alignment_tolerance_deg)
        rospy.logwarn("  yaw tolerance          : %.4f deg", self.yaw_alignment_tolerance_deg)
        rospy.logwarn("  pitch zero tolerance   : %.4f deg", self.pitch_zero_tolerance_deg)
        rospy.logwarn("")
        rospy.logwarn("Slow approach setting:")
        rospy.logwarn("  linear speed : %.4f m/s", self.approach_linear_speed)
        rospy.logwarn("  angular speed: %.4f deg/s", self.approach_angular_speed_deg)
        rospy.logwarn("")
        rospy.logwarn("This will publish slowly approaching command:")
        rospy.logwarn("  position -> %s", self.nav_topic)
        rospy.logwarn("  rpy      -> %s", self.baselink_rpy_topic)
        rospy.logwarn("")
        rospy.logwarn("Type 'y' then Enter to start slow navigation.")
        rospy.logwarn("Type anything else to cancel.")
        rospy.logwarn("============================================================")
        rospy.logwarn("")

        answer = input("Confirm navigation? [y/N]: ")

        if answer.lower() == "y":
            with self.state_lock:
                self.confirmed = True

                # Final target.
                self.target_position = list(data["target_position"])
                self.target_rpy = list(data["target_rpy"])

                # Start slow command from current robot pose, not from final target.
                current_position, current_rpy = self.getCurrentRobotPositionAndRPY()

                if current_position is None or current_rpy is None:
                    rospy.logerr("[branch_distance_nav] cannot start navigation: current pose unavailable")
                    self.confirmed = False
                    return False

                self.command_position = list(current_position)
                self.command_rpy = list(current_rpy)
                self.last_command_time = rospy.Time.now()

                self.navigation_phase = "position"
                self.last_phase_name = None

            rospy.logwarn("[branch_distance_nav] slow navigation confirmed")
            rospy.logwarn("[branch_distance_nav] phase order: position -> orientation")
            rospy.logwarn("[branch_distance_nav] command starts from current pose and slowly approaches each phase target")
            rospy.logwarn("[branch_distance_nav] press 'e' or 'q' for emergency hover")
            return True

        rospy.logwarn("[branch_distance_nav] navigation cancelled")
        return False

    # ================================================================
    # Main loop
    # ================================================================

    def run(self):
        rate = rospy.Rate(self.publish_rate_hz)

        rospy.logwarn("[branch_distance_nav] waiting for branch pose and gimbal odom...")

        while not rospy.is_shutdown():
            data = self.calculateTarget()

            if data is None:
                rate.sleep()
                continue

            if not self.confirmation_asked:
                self.confirmation_asked = True

                ok = self.askKeyboardConfirmation(data)

                if ok:
                    self.startKeyboardThreadOnce()
                else:
                    rospy.logwarn("[branch_distance_nav] no navigation command will be published")
                    rospy.logwarn("[branch_distance_nav] node will keep running for distance/debug only")

            with self.state_lock:
                confirmed = self.confirmed
                emergency_stop = self.emergency_stop

                target_position = None if self.target_position is None else list(self.target_position)
                target_rpy = None if self.target_rpy is None else list(self.target_rpy)

                command_position = None if self.command_position is None else list(self.command_position)
                command_rpy = None if self.command_rpy is None else list(self.command_rpy)

                hover_position = None if self.hover_position is None else list(self.hover_position)
                hover_rpy = None if self.hover_rpy is None else list(self.hover_rpy)

            # ========================================================
            # Emergency hover mode
            # ========================================================

            if emergency_stop:
                if hover_position is not None and hover_rpy is not None:
                    self.publishNavAndRPY(hover_position, hover_rpy)

                    rospy.logerr_throttle(
                        1.0,
                        "[branch_distance_nav] EMERGENCY HOVER publishing current pose: "
                        "pos=(%.3f, %.3f, %.3f), rpy_deg=(%.2f, %.2f, %.2f)",
                        hover_position[0],
                        hover_position[1],
                        hover_position[2],
                        math.degrees(hover_rpy[0]),
                        math.degrees(hover_rpy[1]),
                        math.degrees(hover_rpy[2]),
                    )

                rate.sleep()
                continue

            # ========================================================
            # Normal slow navigation mode
            # ========================================================

            if confirmed and target_position is not None and target_rpy is not None:
                with self.state_lock:
                    if self.command_position is not None and self.command_rpy is not None:
                        phased_position, phased_rpy, active_phase = self.buildPhasedTarget(data)

                        if active_phase != self.last_phase_name:
                            rospy.logwarn(
                                "[branch_distance_nav] active phase: %s",
                                active_phase
                            )
                            self.last_phase_name = active_phase

                        self.updateSlowCommandTarget(phased_position, phased_rpy)
                        command_position = list(self.command_position)
                        command_rpy = list(self.command_rpy)
                    else:
                        command_position = None
                        command_rpy = None
                        active_phase = "unknown"

                if command_position is not None and command_rpy is not None:
                    # IMPORTANT:
                    # Publish the slow intermediate command,
                    # not the final target directly.
                    self.publishNavAndRPY(command_position, command_rpy)

                    dx_final = data["gimbal_x"] - target_position[0]
                    dy_final = data["gimbal_y"] - target_position[1]
                    dz_final = data["gimbal_z"] - target_position[2]

                    distance_robot_to_final_target = math.sqrt(
                        dx_final * dx_final +
                        dy_final * dy_final +
                        dz_final * dz_final
                    )

                    dx_cmd = command_position[0] - target_position[0]
                    dy_cmd = command_position[1] - target_position[1]
                    dz_cmd = command_position[2] - target_position[2]

                    distance_command_to_final_target = math.sqrt(
                        dx_cmd * dx_cmd +
                        dy_cmd * dy_cmd +
                        dz_cmd * dz_cmd
                    )

                    rospy.logwarn_throttle(
                        1.0,
                        "[branch_distance_nav] publishing SLOW command | "
                        "phase=%s, "
                        "cmd_pos=(%.3f, %.3f, %.3f), "
                        "final_pos=(%.3f, %.3f, %.3f), "
                        "cmd_rpy_deg=(%.2f, %.2f, %.2f), "
                        "x_err=%.4f m, z_err=%.4f m, "
                        "roll_err=%.2f deg, pitch_err=%.2f deg, yaw_err=%.2f deg, "
                        "robot_to_final=%.4f m, "
                        "cmd_to_final=%.4f m",
                        active_phase,
                        command_position[0],
                        command_position[1],
                        command_position[2],
                        target_position[0],
                        target_position[1],
                        target_position[2],
                        math.degrees(command_rpy[0]),
                        math.degrees(command_rpy[1]),
                        math.degrees(command_rpy[2]),
                        data["x_error_to_target"],
                        data["z_error_to_target"],
                        math.degrees(data["roll_error_to_target"]),
                        math.degrees(data["pitch_error_to_target"]),
                        math.degrees(data["yaw_error_to_target"]),
                        distance_robot_to_final_target,
                        distance_command_to_final_target,
                    )

                    if active_phase == "done" and distance_robot_to_final_target < self.arrival_position_tolerance:
                        rospy.logwarn_throttle(
                            1.0,
                            "[branch_distance_nav] position and orientation done / hovering near final target, error = %.4f m",
                            distance_robot_to_final_target
                        )

            rate.sleep()


if __name__ == "__main__":
    try:
        node = BranchDistanceNavigator()
        node.run()
    except rospy.ROSInterruptException:
        pass
