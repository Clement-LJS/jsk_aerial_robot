#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool, Empty, Float64


class MujocoCuttingExperiment(object):
    EXPECTED_CONTROLLER = "aerial_robot_control/gimbalrotor_perching_admittance_controller"

    def __init__(self):
        self.controller_mode = rospy.get_param("~controller_mode", "pid")
        self.profile = rospy.get_param("~profile", "bag_1")
        self.hybrid_enabled = bool(rospy.get_param("~hybrid_enabled", False))
        self.step_deg = float(rospy.get_param("~step_deg", 0.8))
        self.hold_time = float(rospy.get_param("~hold_time", 15.0))
        self.number_of_steps = int(rospy.get_param("~number_of_steps", 15))
        self.tare_wait = float(rospy.get_param("~tare_wait", 3.0))
        self.start_delay = float(rospy.get_param("~start_delay", 3.0))
        self.valid_timeout = float(rospy.get_param("~valid_timeout", 30.0))
        self.zero_force_epsilon = float(rospy.get_param("~zero_force_epsilon", 1e-6))
        self.zero_force_samples_required = int(rospy.get_param("~zero_force_samples_required", 5))
        self.zero_force_timeout = float(rospy.get_param("~zero_force_timeout", 5.0))
        self.heartbeat_rate = float(rospy.get_param("~heartbeat_rate", 20.0))
        self.monitor_rate = float(rospy.get_param("~monitor_rate", 20.0))
        self.force_safety_limit = float(rospy.get_param("~force_safety_limit", 0.0))

        self.valid = False
        self.model_ready = False
        self.completed = False
        self.force_magnitude = None
        self.force_msg_count = 0
        self.force_msg_stamp = None
        self.locked_pivot_error = None
        self.locked_pivot = None
        self.locked_pose = None
        self.cutting_active_state = False
        self.safe_stop_started = False

        self._validate_inputs()

        self.perching_enable_pub = rospy.Publisher("perching/enable", Bool, queue_size=1, latch=True)
        self.admittance_enable_pub = rospy.Publisher("perching/admittance_enable", Bool, queue_size=1, latch=True)
        self.cutting_active_pub = rospy.Publisher("perching/cutting_active", Bool, queue_size=1, latch=True)
        self.manual_pitch_delta_pub = rospy.Publisher("perching/manual_pitch_delta", Float64, queue_size=1)
        self.reset_pub = rospy.Publisher("simulation/cutting/reset", Empty, queue_size=1)

        rospy.Subscriber("simulation/cutting/valid", Bool, self._valid_cb)
        rospy.Subscriber("simulation/cutting/model_ready", Bool, self._model_ready_cb)
        rospy.Subscriber("simulation/cutting/completed", Bool, self._completed_cb)
        rospy.Subscriber("simulation/cutting/force_magnitude", Float64, self._force_cb)
        rospy.Subscriber("simulation/cutting/locked_pivot_error", Float64, self._pivot_error_cb)
        rospy.Subscriber("perching/locked_pivot", PointStamped, self._locked_pivot_cb)
        rospy.Subscriber("perching/locked_pose", PoseStamped, self._locked_pose_cb)

        self.heartbeat_timer = rospy.Timer(
            rospy.Duration(1.0 / max(self.heartbeat_rate, 1.0)),
            self._heartbeat_cb,
        )
        rospy.on_shutdown(self.safe_stop)

    def _validate_inputs(self):
        if self.controller_mode not in ("pid", "admittance"):
            raise rospy.ROSInitException("controller_mode must be 'pid' or 'admittance'")
        if not self.profile:
            raise rospy.ROSInitException("profile must not be empty")
        if self.step_deg <= 0.0:
            raise rospy.ROSInitException("step_deg must be > 0")
        if self.hold_time < 0.0:
            raise rospy.ROSInitException("hold_time must be >= 0")
        if self.number_of_steps < 1:
            raise rospy.ROSInitException("number_of_steps must be >= 1")
        if self.tare_wait < 0.0:
            raise rospy.ROSInitException("tare_wait must be >= 0")
        if self.start_delay < 0.0:
            raise rospy.ROSInitException("start_delay must be >= 0")
        if self.valid_timeout <= 0.0:
            raise rospy.ROSInitException("valid_timeout must be > 0")
        if self.zero_force_samples_required < 1:
            raise rospy.ROSInitException("zero_force_samples_required must be >= 1")
        if self.zero_force_timeout <= 0.0:
            raise rospy.ROSInitException("zero_force_timeout must be > 0")

    def _valid_cb(self, msg):
        self.valid = msg.data

    def _model_ready_cb(self, msg):
        self.model_ready = msg.data

    def _completed_cb(self, msg):
        self.completed = msg.data

    def _force_cb(self, msg):
        self.force_magnitude = msg.data
        self.force_msg_count += 1
        self.force_msg_stamp = rospy.Time.now()

    def _pivot_error_cb(self, msg):
        self.locked_pivot_error = msg.data

    def _locked_pivot_cb(self, msg):
        self.locked_pivot = msg

    def _locked_pose_cb(self, msg):
        self.locked_pose = msg

    def _heartbeat_cb(self, _event):
        self.cutting_active_pub.publish(Bool(data=self.cutting_active_state))

    def _wait_until(self, predicate, timeout, description):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(max(self.monitor_rate, 1.0))
        while not rospy.is_shutdown():
            if predicate():
                return
            if rospy.Time.now() > deadline:
                raise rospy.ROSException(f"timeout waiting for {description}")
            rate.sleep()
        raise rospy.ROSInterruptException("shutdown while waiting")

    def _publish_mode(self, admittance_enabled, perching_enabled):
        self.admittance_enable_pub.publish(Bool(data=admittance_enabled))
        self.perching_enable_pub.publish(Bool(data=perching_enabled))

    def _publish_cutting_state(self, enabled, repeat=1):
        self.cutting_active_state = bool(enabled)
        for _ in range(max(repeat, 1)):
            self.cutting_active_pub.publish(Bool(data=self.cutting_active_state))
            rospy.sleep(0.05)

    def _sleep_with_checks(self, duration, description):
        if duration <= 0.0:
            return
        rate = rospy.Rate(max(self.monitor_rate, 1.0))
        deadline = rospy.Time.now() + rospy.Duration(duration)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.completed:
                return
            if not self.valid:
                raise rospy.ROSException(f"runtime validity lost during {description}")
            if (
                self.force_safety_limit > 0.0
                and self.force_magnitude is not None
                and math.isfinite(self.force_magnitude)
                and abs(self.force_magnitude) > self.force_safety_limit
            ):
                raise rospy.ROSException("force safety limit exceeded")
            rate.sleep()

    def _wait_for_zero_force_after_reset(self):
        start_count = self.force_msg_count
        deadline = rospy.Time.now() + rospy.Duration(self.zero_force_timeout)
        consecutive = 0
        rate = rospy.Rate(max(self.monitor_rate, 1.0))

        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                raise rospy.ROSException("timeout waiting for fresh zero-force samples after reset")

            if self.force_msg_count > start_count:
                if self.force_magnitude is not None and abs(self.force_magnitude) <= self.zero_force_epsilon:
                    consecutive += 1
                    if consecutive >= self.zero_force_samples_required:
                        return
                else:
                    consecutive = 0
            rate.sleep()

        raise rospy.ROSInterruptException("shutdown while waiting for zero force")

    def safe_stop(self):
        if self.safe_stop_started:
            return
        self.safe_stop_started = True

        self.cutting_active_state = False
        try:
            self._publish_cutting_state(False, repeat=5)
            for _ in range(5):
                self.admittance_enable_pub.publish(Bool(data=False))
                rospy.sleep(0.05)
            self.reset_pub.publish(Empty())
            if not rospy.is_shutdown():
                self._wait_for_zero_force_after_reset()
        except Exception as exc:
            rospy.logwarn("safe_stop did not complete cleanly: %s", exc)
        finally:
            for _ in range(5):
                self.perching_enable_pub.publish(Bool(data=False))
                rospy.sleep(0.05)

    def run(self):
        controller_name = rospy.get_param("aerial_robot_control_name", "")
        if controller_name != self.EXPECTED_CONTROLLER:
            raise rospy.ROSException(
                "unexpected aerial_robot_control_name: "
                f"expected '{self.EXPECTED_CONTROLLER}', got '{controller_name}'"
            )

        rospy.loginfo("waiting for simulation topics")
        rospy.wait_for_message("/clock", rospy.AnyMsg, timeout=self.valid_timeout)
        rospy.wait_for_message("mocap/pose", PoseStamped, timeout=self.valid_timeout)
        self._wait_until(lambda: self.model_ready, self.valid_timeout, "simulation/cutting/model_ready=true")

        self._publish_mode(False, True)
        self._publish_cutting_state(False, repeat=5)
        self.reset_pub.publish(Empty())

        self._wait_until(lambda: self.locked_pose is not None, self.valid_timeout, "perching/locked_pose")
        self._wait_until(lambda: self.locked_pivot is not None, self.valid_timeout, "perching/locked_pivot")
        self._wait_until(lambda: self.valid, self.valid_timeout, "simulation/cutting/valid=true")
        self._wait_until(
            lambda: self.locked_pivot_error is not None and self.locked_pivot_error <= 0.002,
            self.valid_timeout,
            "locked_pivot_error <= 0.002",
        )

        self._sleep_with_checks(self.start_delay, "start_delay")
        self._sleep_with_checks(self.tare_wait, "tare_wait")

        admittance_enabled = self.controller_mode == "admittance"
        self._publish_mode(admittance_enabled, True)
        rospy.sleep(0.5)

        self._publish_cutting_state(True, repeat=5)
        rospy.sleep(0.5)

        step_rad = math.radians(self.step_deg)
        for step_index in range(1, self.number_of_steps + 1):
            if rospy.is_shutdown() or self.completed:
                break
            target_delta = step_index * step_rad
            for _ in range(5):
                self.manual_pitch_delta_pub.publish(Float64(data=target_delta))
                rospy.sleep(0.02)
            self._sleep_with_checks(self.hold_time, f"hold step {step_index}")

        pivot_text = "unknown"
        if self.locked_pivot is not None:
            p = self.locked_pivot.point
            pivot_text = f"({p.x:.4f}, {p.y:.4f}, {p.z:.4f})"

        rospy.loginfo(
            "mujoco cutting run finished mode=%s profile=%s hybrid=%s pivot=%s completed=%s",
            self.controller_mode,
            self.profile,
            self.hybrid_enabled,
            pivot_text,
            self.completed,
        )


def main():
    rospy.init_node("mujoco_cutting_experiment")
    experiment = MujocoCuttingExperiment()
    try:
        experiment.run()
    finally:
        experiment.safe_stop()


if __name__ == "__main__":
    main()
