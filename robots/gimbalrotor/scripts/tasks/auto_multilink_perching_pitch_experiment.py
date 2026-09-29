#!/usr/bin/env python

from __future__ import print_function

import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, Float64, UInt8


HOVER_STATE = 5

DEFAULT_STEP_DEG = 0.8
DEFAULT_HOLD_TIME = 15.0
DEFAULT_NUMBER_OF_STEPS = 15
DEFAULT_DIRECTION = 1.0
DEFAULT_MAX_DELTA_DEG = 20.0

DEFAULT_PUBLISH_REPEAT = 5
DEFAULT_PUBLISH_INTERVAL = 0.02

DEFAULT_LOCK_TIMEOUT = 3.0
DEFAULT_SECONDARY_SETTLE_TIMEOUT = 3.0
DEFAULT_TARGET_UPDATE_TIMEOUT = 1.0


def finite(value):
    if value is None:
        return False
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return not math.isnan(value) and not math.isinf(value)


def normalize_sign(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return 1.0
    if math.isnan(value) or math.isinf(value) or value == 0.0:
        return 1.0
    return 1.0 if value > 0.0 else -1.0


def topic_under(namespace, relative_name):
    return namespace.rstrip("/") + "/" + relative_name.lstrip("/")


class Feedback(object):
    def __init__(self):
        self.rezero = {}
        self.rezero_stamps = {}
        self.flight_state = None
        self.model_valid = None

        self.lock_stamp = None

        self.pitch_measured = None
        self.pitch_measured_receive_stamp = None
        self.pitch_nominal = None
        self.pitch_nominal_receive_stamp = None
        self.pitch_final = None
        self.pitch_final_receive_stamp = None

        self.secondary_measured = None
        self.secondary_measured_receive_stamp = None
        self.secondary_settled_receive_stamp = None
        self.secondary_settled = None

        self.body_target = None
        self.body_target_receive_stamp = None

    def rezero_callback(self, msg, name):
        self.rezero[name] = msg.data
        self.rezero_stamps[name] = rospy.Time.now()

    def flight_state_callback(self, msg):
        self.flight_state = msg.data

    def model_valid_callback(self, msg):
        self.model_valid = msg.data

    def locked_pose_callback(self, msg):
        if self.lock_stamp != msg.header.stamp:
            self.pitch_nominal_receive_stamp = None
        self.lock_stamp = msg.header.stamp

    def pitch_measured_callback(self, msg):
        self.pitch_measured = msg.data
        self.pitch_measured_receive_stamp = rospy.Time.now()

    def pitch_nominal_callback(self, msg):
        self.pitch_nominal = msg.data
        self.pitch_nominal_receive_stamp = rospy.Time.now()

    def pitch_final_callback(self, msg):
        self.pitch_final = msg.data
        self.pitch_final_receive_stamp = rospy.Time.now()

    def secondary_measured_callback(self, msg):
        self.secondary_measured = msg.data
        self.secondary_measured_receive_stamp = rospy.Time.now()

    def secondary_settled_callback(self, msg):
        self.secondary_settled_receive_stamp = rospy.Time.now()
        self.secondary_settled = msg.data

    def body_target_callback(self, msg):
        self.body_target = msg
        self.body_target_receive_stamp = rospy.Time.now()


class MultilinkPitchExperiment(object):
    def __init__(self):
        self.robot_namespace = rospy.get_param(
            "~robot_namespace", "/gimbalrotor")

        self.step_deg = abs(float(rospy.get_param(
            "~step_deg", DEFAULT_STEP_DEG)))
        self.hold_time = max(0.0, float(rospy.get_param(
            "~hold_time", DEFAULT_HOLD_TIME)))
        self.number_of_steps = max(1, int(rospy.get_param(
            "~number_of_steps", DEFAULT_NUMBER_OF_STEPS)))
        self.direction = normalize_sign(rospy.get_param(
            "~direction", DEFAULT_DIRECTION))
        self.max_delta_deg = abs(float(rospy.get_param(
            "~max_delta_deg", DEFAULT_MAX_DELTA_DEG)))

        self.publish_repeat = max(1, int(rospy.get_param(
            "~publish_repeat", DEFAULT_PUBLISH_REPEAT)))
        self.publish_interval = max(0.0, float(rospy.get_param(
            "~publish_interval", DEFAULT_PUBLISH_INTERVAL)))

        self.lock_timeout = max(0.1, float(rospy.get_param(
            "~lock_timeout", DEFAULT_LOCK_TIMEOUT)))
        self.secondary_settle_timeout = max(0.0, float(rospy.get_param(
            "~secondary_settle_timeout",
            DEFAULT_SECONDARY_SETTLE_TIMEOUT)))
        self.target_update_timeout = max(0.1, float(rospy.get_param(
            "~target_update_timeout",
            DEFAULT_TARGET_UPDATE_TIMEOUT)))

        self.perform_pitch_rezero = bool(rospy.get_param("~perform_pitch_rezero", True))
        self.rezero_timeout = float(rospy.get_param("~rezero_timeout", 8.0))
        if not finite(self.rezero_timeout) or self.rezero_timeout <= 0.0:
            raise ValueError("~rezero_timeout must be finite and positive")
        self.return_to_lock_at_end = bool(rospy.get_param(
            "~return_to_lock_at_end", False))
        self.disable_perching_at_end = bool(rospy.get_param(
            "~disable_perching_at_end", False))

        if self.step_deg <= 0.0:
            raise ValueError("~step_deg must be positive")
        if self.max_delta_deg <= 0.0:
            raise ValueError("~max_delta_deg must be positive")

        multilink_param_ns = topic_under(
            self.robot_namespace,
            "navigation/multilink_perching")

        self.pitch_command_sign = normalize_sign(rospy.get_param(
            multilink_param_ns + "/pitch_command_sign", 1.0))

        self.perching_enable_topic = rospy.get_param(
            "~perching_enable_topic",
            topic_under(self.robot_namespace, "perching/enable"))
        self.pitch_command_topic = rospy.get_param(
            "~pitch_command_topic",
            topic_under(
                self.robot_namespace,
                "perching/manual_pitch_delta"))

        self.feedback = Feedback()

        self.perching_enable_pub = rospy.Publisher(
            self.perching_enable_topic,
            Bool,
            queue_size=1)
        self.pitch_command_pub = rospy.Publisher(
            self.pitch_command_topic,
            Float64,
            queue_size=1)

        rezero_topic = rospy.get_param(
            multilink_param_ns + "/pitch_rezero_topic", "perching/multilink/pitch_rezero")
        if not rezero_topic.startswith("/"):
            rezero_topic = topic_under(self.robot_namespace, rezero_topic)
        self.pitch_rezero_pub = rospy.Publisher(
            rospy.get_param("~pitch_rezero_topic", rezero_topic), Empty, queue_size=1)
        self.relock_pub = rospy.Publisher(
            rospy.get_param("~relock_topic", topic_under(
                self.robot_namespace, "perching/relock")), Empty, queue_size=1)
        self.feedback_timeout = float(rospy.get_param("~feedback_timeout", 0.5))
        if not finite(self.feedback_timeout) or self.feedback_timeout <= 0.0:
            raise ValueError("~feedback_timeout must be finite and positive")
        self.subscribers = []
        for name in ("pitch_rezero_active", "pitch_rezero_ready", "pitch_rezero_failed",
                     "passive_pitch_delta", "body_pitch", "body_pitch_rate"):
            msg_type = Bool if name.startswith("pitch_rezero_") else Float64
            self.subscribers.append(rospy.Subscriber(
                topic_under(self.robot_namespace, "perching/multilink/" + name),
                msg_type, self.feedback.rezero_callback, callback_args=name, queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~flight_state_topic",
                topic_under(self.robot_namespace, "flight_state")),
            UInt8,
            self.feedback.flight_state_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~model_valid_topic",
                topic_under(
                    self.robot_namespace,
                    "perching/multilink/model_valid")),
            Bool,
            self.feedback.model_valid_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~locked_pose_topic",
                topic_under(
                    self.robot_namespace,
                    "perching/locked_pose")),
            PoseStamped,
            self.feedback.locked_pose_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~pitch_measured_topic",
                topic_under(
                    self.robot_namespace,
                    "perching/multilink/pitch_joint_measured")),
            Float64,
            self.feedback.pitch_measured_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~pitch_nominal_topic",
                topic_under(
                    self.robot_namespace,
                    "perching/multilink/pitch_joint_nominal")),
            Float64,
            self.feedback.pitch_nominal_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~pitch_final_topic",
                topic_under(
                    self.robot_namespace,
                    "perching/multilink/pitch_joint_final")),
            Float64,
            self.feedback.pitch_final_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~secondary_settled_topic",
                topic_under(
                    self.robot_namespace,
                    "perching/multilink/secondary_settled")),
            Bool,
            self.feedback.secondary_settled_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~target_body_pose_topic",
                topic_under(
                    self.robot_namespace,
                    "perching/multilink/target_body_pose")),
            PoseStamped,
            self.feedback.body_target_callback,
            queue_size=1))

        self.subscribers.append(rospy.Subscriber(
            topic_under(self.robot_namespace, "perching/multilink/secondary_joint_measured"),
            Float64, self.feedback.secondary_measured_callback, queue_size=1))
        self.final_lock_stamp = None
        self.lock_request_stamp = None
        self.locked_pitch = None
        self.pitch_delta = 0.0

    def fresh(self, stamp):
        return stamp is not None and 0.0 <= (rospy.Time.now() - stamp).to_sec() <= self.feedback_timeout

    def rezero_status_fresh(self, after=None):
        return all(self.fresh(self.feedback.rezero_stamps.get(name)) and
                   (after is None or self.feedback.rezero_stamps[name] > after)
                   for name in ("pitch_rezero_active", "pitch_rezero_ready", "pitch_rezero_failed"))

    def publish_pitch_delta(self, pitch_delta_rad):
        msg = Float64(data=pitch_delta_rad)

        for index in range(self.publish_repeat):
            if rospy.is_shutdown():
                return
            self.pitch_command_pub.publish(msg)
            if (index + 1 < self.publish_repeat and
                    self.publish_interval > 0.0):
                time.sleep(self.publish_interval)

    def wait_until(self, predicate, timeout, description):
        deadline = time.time() + timeout

        while not rospy.is_shutdown() and time.time() < deadline:
            if predicate():
                return True
            time.sleep(0.02)

        rospy.logerr("Timeout waiting for %s", description)
        return False

    def ready_for_lock(self):
        return (
            self.feedback.flight_state == HOVER_STATE and
            self.feedback.model_valid is True and
            finite(self.feedback.pitch_measured) and
            self.fresh(self.feedback.pitch_measured_receive_stamp) and
            finite(self.feedback.secondary_measured) and
            self.fresh(self.feedback.secondary_measured_receive_stamp) and
            self.perching_enable_pub.get_num_connections() > 0)

    def fresh_lock_ready(self):
        if self.lock_request_stamp is None:
            return False
        if self.feedback.lock_stamp is None:
            return False
        if self.feedback.lock_stamp.is_zero():
            return False
        if self.feedback.lock_stamp <= self.lock_request_stamp:
            return False

        if self.feedback.pitch_nominal_receive_stamp is None:
            return False
        if self.feedback.pitch_nominal_receive_stamp < self.feedback.lock_stamp:
            return False

        return finite(self.feedback.pitch_nominal)

    def enable_and_lock(self):
        if not self.wait_until(
                self.ready_for_lock,
                self.lock_timeout,
                "HOVER + valid multilink model + pitch joint feedback"):
            return False

        self.lock_request_stamp = rospy.Time.now()

        self.perching_enable_pub.publish(Bool(data=True))

        rospy.loginfo(
            "Requested NORMAL multilink perching lock. Waiting for fresh lock...")

        if not self.wait_until(
                self.fresh_lock_ready,
                self.lock_timeout,
                "fresh multilink locked_pose + pitch nominal diagnostic"):
            self.perching_enable_pub.publish(Bool(data=False))
            rospy.logerr(
                "Fresh multilink lock was not confirmed; perching was disabled.")
            return False

        self.final_lock_stamp = self.feedback.lock_stamp
        self.locked_pitch = self.feedback.pitch_nominal
        self.pitch_delta = 0.0

        rospy.loginfo(
            "Multilink lock confirmed. Locked pitch joint = %.2f deg",
            math.degrees(self.locked_pitch))

        return True

    def rezero_and_relock(self):
        if not self.wait_until(
                lambda: self.pitch_rezero_pub.get_num_connections() > 0 and
                self.relock_pub.get_num_connections() > 0 and self.rezero_status_fresh() and
                all(self.feedback.rezero.get(n) is False for n in
                    ("pitch_rezero_active", "pitch_rezero_ready", "pitch_rezero_failed")),
                self.lock_timeout, "rezero subscribers and IDLE status"):
            return False
        request_stamp = rospy.Time.now()
        self.pitch_rezero_pub.publish(Empty())
        deadline = time.time() + self.rezero_timeout
        seen_active = False
        ready = False
        while not rospy.is_shutdown() and time.time() < deadline:
            if not self.ready_for_lock():
                rospy.logerr("Lost HOVER/model/joint feedback during rezero; aborting without cutting.")
                return False
            if self.rezero_status_fresh(request_stamp):
                status = self.feedback.rezero
                if status.get("pitch_rezero_failed") is True:
                    rospy.logerr("Pitch rezero failed. Holding; disable/retry safely. No cutting.")
                    return False
                seen_active = seen_active or status.get("pitch_rezero_active") is True
                if (seen_active and status.get("pitch_rezero_ready") is True and
                        status.get("pitch_rezero_active") is False):
                    ready = True
                    break
            time.sleep(0.02)
        if not ready:
            rospy.logerr("Rezero ready not confirmed; no relock or cutting. Disable/retry safely.")
            return False
        self.lock_request_stamp = rospy.Time.now()
        self.relock_pub.publish(Empty())
        if not self.wait_until(self.fresh_lock_ready, self.lock_timeout, "fresh final lock"):
            return False
        self.final_lock_stamp = self.feedback.lock_stamp
        self.locked_pitch = self.feedback.pitch_nominal
        self.pitch_delta = 0.0
        return self.wait_until(self.cutting_ready, self.lock_timeout, "final lock IDLE status")

    def cutting_ready(self):
        return (self.ready_for_lock() and self.feedback.lock_stamp == self.final_lock_stamp and
                self.rezero_status_fresh(self.final_lock_stamp) and
                all(self.feedback.rezero.get(n) is False for n in
                    ("pitch_rezero_active", "pitch_rezero_ready", "pitch_rezero_failed")))

    def hold_cutting_target(self):
        deadline = time.time() + self.hold_time
        while not rospy.is_shutdown() and time.time() < deadline:
            if not self.cutting_ready():
                rospy.logerr("Lost final lock/IDLE/HOVER/feedback; stopping experiment.")
                return False
            time.sleep(0.02)
        return not rospy.is_shutdown()

    def wait_for_secondary_settle(self):
        if self.secondary_settle_timeout <= 0.0:
            return True

        return self.wait_until(
            lambda: (self.cutting_ready() and self.feedback.secondary_settled is True and
                     self.fresh(self.feedback.secondary_settled_receive_stamp) and
                     self.feedback.secondary_settled_receive_stamp > self.final_lock_stamp),
            self.secondary_settle_timeout,
            "secondary yaw/roll joint to settle")

    def expected_pitch_joint_target(self):
        if not finite(self.locked_pitch):
            return None

        return (
            self.locked_pitch +
            self.pitch_command_sign * self.pitch_delta)

    def wait_for_pitch_target_update(self, command_stamp):
        return self.wait_until(
            lambda: (
                self.cutting_ready() and
                self.feedback.pitch_nominal_receive_stamp is not None and
                self.feedback.pitch_nominal_receive_stamp >= command_stamp and
                finite(self.feedback.pitch_nominal) and
                abs(self.feedback.pitch_nominal - self.expected_pitch_joint_target()) < 1.0e-4 and
                self.feedback.body_target_receive_stamp is not None and
                self.feedback.body_target_receive_stamp >= command_stamp),
            self.target_update_timeout,
            "pitch target + fixed-contact body target update")

    def log_step_state(self, step_number):
        expected_target = self.expected_pitch_joint_target()

        expected_text = (
            "{:+.2f} deg".format(math.degrees(expected_target))
            if finite(expected_target) else "unknown")
        nominal_text = (
            "{:+.2f} deg".format(math.degrees(self.feedback.pitch_nominal))
            if finite(self.feedback.pitch_nominal) else "unknown")
        final_text = (
            "{:+.2f} deg".format(math.degrees(self.feedback.pitch_final))
            if finite(self.feedback.pitch_final) else "unknown")
        measured_text = (
            "{:+.2f} deg".format(math.degrees(self.feedback.pitch_measured))
            if finite(self.feedback.pitch_measured) else "unknown")

        body_text = "unknown"
        if self.feedback.body_target is not None:
            p = self.feedback.body_target.pose.position
            q = self.feedback.body_target.pose.orientation
            body_text = (
                "pos=[{:+.3f}, {:+.3f}, {:+.3f}] "
                "quat=[{:+.3f}, {:+.3f}, {:+.3f}, {:+.3f}]".format(
                    p.x, p.y, p.z, q.x, q.y, q.z, q.w))

        rospy.loginfo(
            "Step %d/%d | logical pitch delta=%+.2f deg | "
            "expected servo target=%s | navigator nominal=%s | "
            "navigator final=%s | measured=%s | body target %s",
            step_number,
            self.number_of_steps,
            math.degrees(self.pitch_delta),
            expected_text,
            nominal_text,
            final_text,
            measured_text,
            body_text)

    def run(self):
        rospy.loginfo(
            "Multilink automatic pitch experiment: step=%.2f deg, "
            "steps=%d, hold=%.2f s, direction=%+.0f, pitch_sign=%+.0f",
            self.step_deg,
            self.number_of_steps,
            self.hold_time,
            self.direction,
            self.pitch_command_sign)

        if not self.enable_and_lock():
            return

        if self.perform_pitch_rezero and not self.rezero_and_relock():
            return

        if not self.wait_for_secondary_settle():
            rospy.logerr(
                "Secondary yaw/roll joint did not settle. "
                "Pitch experiment aborted.")
            return

        step_rad = math.radians(self.step_deg)
        max_delta_rad = math.radians(self.max_delta_deg)

        for step_number in range(1, self.number_of_steps + 1):
            if rospy.is_shutdown():
                return
            if not self.cutting_ready():
                rospy.logerr("Cutting requires fresh final lock, IDLE, HOVER and mechanism feedback.")
                return

            next_delta = (
                self.pitch_delta +
                self.direction * step_rad)

            if abs(next_delta) > max_delta_rad + 1.0e-12:
                rospy.logwarn(
                    "Stopping before step %d because requested pitch delta "
                    "%+.2f deg exceeds max_delta_deg %.2f.",
                    step_number,
                    math.degrees(next_delta),
                    self.max_delta_deg)
                break

            self.pitch_delta = next_delta
            command_stamp = rospy.Time.now()

            self.publish_pitch_delta(self.pitch_delta)

            if not self.wait_for_pitch_target_update(command_stamp):
                rospy.logerr(
                    "Navigator did not produce a fresh pitch/body target. "
                    "Experiment aborted.")
                return

            self.log_step_state(step_number)

            if not self.hold_cutting_target():
                return

        rospy.loginfo(
            "Pitch stepping finished at logical delta %+.2f deg.",
            math.degrees(self.pitch_delta))

        if not self.cutting_ready():
            return
        if self.return_to_lock_at_end and not rospy.is_shutdown():
            rospy.loginfo("Returning pitch joint to locked angle (delta=0).")
            command_stamp = rospy.Time.now()
            self.pitch_delta = 0.0
            self.publish_pitch_delta(0.0)
            self.wait_for_pitch_target_update(command_stamp)

        if self.disable_perching_at_end and not rospy.is_shutdown():
            rospy.loginfo("Disabling multilink perching.")
            self.perching_enable_pub.publish(Bool(data=False))


def main():
    rospy.init_node("auto_multilink_perching_pitch_experiment")
    experiment = MultilinkPitchExperiment()
    experiment.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
