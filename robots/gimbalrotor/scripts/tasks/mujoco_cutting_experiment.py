#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool, Empty, Float64


class MujocoCuttingExperiment(object):
    def __init__(self):
        self.controller_mode = rospy.get_param("~controller_mode", "pid")
        if self.controller_mode not in ("pid", "admittance"):
            raise rospy.ROSInitException("controller_mode must be 'pid' or 'admittance'")

        self.profile = rospy.get_param("~profile", "bag_1")
        self.hybrid_enabled = bool(rospy.get_param("~hybrid_enabled", False))
        self.step_deg = float(rospy.get_param("~step_deg", 0.8))
        self.hold_time = float(rospy.get_param("~hold_time", 15.0))
        self.number_of_steps = int(rospy.get_param("~number_of_steps", 15))
        self.tare_wait = float(rospy.get_param("~tare_wait", 3.0))
        self.start_delay = float(rospy.get_param("~start_delay", 3.0))

        self.valid = False
        self.completed = False
        self.force_magnitude = None
        self.locked_pivot_error = None
        self.locked_pivot = None
        self.locked_pose = None

        self.perching_enable_pub = rospy.Publisher("perching/enable", Bool, queue_size=1, latch=True)
        self.admittance_enable_pub = rospy.Publisher("perching/admittance_enable", Bool, queue_size=1, latch=True)
        self.cutting_active_pub = rospy.Publisher("perching/cutting_active", Bool, queue_size=1, latch=True)
        self.manual_pitch_delta_pub = rospy.Publisher("perching/manual_pitch_delta", Float64, queue_size=1)
        self.reset_pub = rospy.Publisher("simulation/cutting/reset", Empty, queue_size=1)

        rospy.Subscriber("simulation/cutting/valid", Bool, self._valid_cb)
        rospy.Subscriber("simulation/cutting/completed", Bool, self._completed_cb)
        rospy.Subscriber("simulation/cutting/force_magnitude", Float64, self._force_cb)
        rospy.Subscriber("simulation/cutting/locked_pivot_error", Float64, self._pivot_error_cb)
        rospy.Subscriber("perching/locked_pivot", PointStamped, self._locked_pivot_cb)
        rospy.Subscriber("perching/locked_pose", PoseStamped, self._locked_pose_cb)

    def _valid_cb(self, msg):
        self.valid = msg.data

    def _completed_cb(self, msg):
        self.completed = msg.data

    def _force_cb(self, msg):
        self.force_magnitude = msg.data

    def _pivot_error_cb(self, msg):
        self.locked_pivot_error = msg.data

    def _locked_pivot_cb(self, msg):
        self.locked_pivot = msg

    def _locked_pose_cb(self, msg):
        self.locked_pose = msg

    def _wait_until(self, predicate, timeout, description):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if predicate():
                return
            if rospy.Time.now() > deadline:
                raise rospy.ROSException(f"timeout waiting for {description}")
            rate.sleep()

    def _publish_mode(self, admittance_enabled, cutting_active, perching_enabled):
        self.admittance_enable_pub.publish(Bool(data=admittance_enabled))
        self.cutting_active_pub.publish(Bool(data=cutting_active))
        self.perching_enable_pub.publish(Bool(data=perching_enabled))

    def run(self):
        rospy.loginfo("waiting for simulation topics")
        rospy.wait_for_message("/clock", rospy.AnyMsg, timeout=30.0)
        rospy.wait_for_message("mocap/pose", PoseStamped, timeout=30.0)
        self._wait_until(lambda: self.valid, 30.0, "simulation/cutting/valid=true")

        self.reset_pub.publish(Empty())
        rospy.sleep(self.start_delay)

        self._publish_mode(False, False, True)
        self._wait_until(lambda: self.locked_pose is not None, 30.0, "perching/locked_pose")
        self._wait_until(lambda: self.locked_pivot is not None, 30.0, "perching/locked_pivot")
        self._wait_until(
            lambda: self.locked_pivot_error is not None and self.locked_pivot_error <= 0.002,
            30.0,
            "locked_pivot_error <= 0.002",
        )

        rospy.sleep(self.tare_wait)

        admittance_enabled = self.controller_mode == "admittance"
        self._publish_mode(admittance_enabled, False, True)
        rospy.sleep(0.5)

        self.cutting_active_pub.publish(Bool(data=True))
        rospy.sleep(0.5)

        step_rad = self.step_deg * 3.14159265358979323846 / 180.0
        for step_index in range(1, self.number_of_steps + 1):
            if rospy.is_shutdown() or self.completed:
                break
            target_delta = step_index * step_rad
            for _ in range(5):
                self.manual_pitch_delta_pub.publish(Float64(data=target_delta))
                rospy.sleep(0.02)
            rospy.sleep(self.hold_time)

        self.cutting_active_pub.publish(Bool(data=False))
        self.admittance_enable_pub.publish(Bool(data=False))
        self.reset_pub.publish(Empty())
        rospy.sleep(0.5)

        if self.force_magnitude is not None and abs(self.force_magnitude) > 1e-6:
            raise rospy.ROSException("cutting force did not return to zero after reset")

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
    MujocoCuttingExperiment().run()


if __name__ == "__main__":
    main()
