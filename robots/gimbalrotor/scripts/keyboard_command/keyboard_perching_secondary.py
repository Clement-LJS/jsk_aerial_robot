#!/usr/bin/env python

from __future__ import print_function

import math
import select
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, Float64, UInt8


HOVER_STATE = 5

HELP = """
Multilink perching keyboard command (navigation only)
-----------------------------------------------------
e       : enable NORMAL multilink perching / provisional lock
z       : rezero whole body about passive pivot (active joints frozen)
r       : final relock after rezero ready; enables cutting keys
d       : disable multilink perching

j / l   : secondary yaw/roll joint negative / positive
i / k   : pitch cutting joint positive / negative from locked pitch
space   : return pitch + secondary joints to their locked configuration

s       : print status
h       : print help
CTRL-C  : quit (leaves the current robot/perching state unchanged)

Notes:
  - This script is for GimbalrotorMultilinkPerchingNavigator.
  - Starting this keyboard does not adopt an externally enabled lock.
  - e requests a fresh navigator lock, even when perching is already enabled.
  - Physically perch, press e, then z; wait for ready, press r, then cut.
  - A confirmation timeout blocks keyboard motion without disabling perching.
  - On rezero failure, disable with d and retry safely. CTRL-C leaves the hold.
  - Calibrate the passive hinge offset/axis in MultilinkPerching.yaml.
  - The secondary joint can be either yaw or roll. Its actual configured
    joint name is read from navigation/multilink_perching/secondary_joint_name.
  - Joint motion is commanded through the navigator. The navigator keeps the
    branch/contact fixed and recomputes the robot body target using FK.
"""


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


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


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def topic_under(namespace, relative_name):
    return namespace.rstrip("/") + "/" + relative_name.lstrip("/")


class Feedback(object):
    def __init__(self):
        self.rezero = {}
        self.rezero_stamps = {}
        self.flight_state = None
        self.model_valid = None

        self.lock_stamp = None
        self.lock_receive_stamp = None

        self.secondary_measured = None
        self.secondary_measured_receive_stamp = None
        self.secondary_nominal = None
        self.secondary_nominal_receive_stamp = None
        self.secondary_settled = None

        self.pitch_measured = None
        self.pitch_measured_receive_stamp = None
        self.pitch_nominal = None
        self.pitch_nominal_receive_stamp = None
        self.pitch_final = None

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
            self.secondary_nominal_receive_stamp = None
        self.lock_stamp = msg.header.stamp
        self.lock_receive_stamp = rospy.Time.now()

    def secondary_measured_callback(self, msg):
        self.secondary_measured = msg.data
        self.secondary_measured_receive_stamp = rospy.Time.now()

    def secondary_nominal_callback(self, msg):
        self.secondary_nominal = msg.data
        self.secondary_nominal_receive_stamp = rospy.Time.now()

    def secondary_settled_callback(self, msg):
        self.secondary_settled = msg.data

    def pitch_measured_callback(self, msg):
        self.pitch_measured = msg.data
        self.pitch_measured_receive_stamp = rospy.Time.now()

    def pitch_nominal_callback(self, msg):
        self.pitch_nominal = msg.data
        self.pitch_nominal_receive_stamp = rospy.Time.now()

    def pitch_final_callback(self, msg):
        self.pitch_final = msg.data


class PerchingKeyboard(object):
    def __init__(self):
        robot_namespace = rospy.get_param("~robot_namespace", "/gimbalrotor")
        self.robot_namespace = robot_namespace

        self.perching_enable_topic = rospy.get_param(
            "~perching_enable_topic",
            topic_under(robot_namespace, "perching/enable"))
        self.pitch_command_topic = rospy.get_param(
            "~pitch_command_topic",
            topic_under(robot_namespace, "perching/manual_pitch_delta"))
        self.secondary_command_topic = rospy.get_param(
            "~secondary_command_topic",
            topic_under(
                robot_namespace,
                "perching/multilink/secondary_joint_target"))

        self.pitch_step = math.radians(
            float(rospy.get_param("~pitch_step_deg", 1.0)))
        self.pitch_delta_limit = math.radians(
            abs(float(rospy.get_param("~pitch_delta_limit_deg", 20.0))))
        self.secondary_step = math.radians(
            float(rospy.get_param("~secondary_step_deg", 1.0)))
        self.secondary_lower = math.radians(
            float(rospy.get_param("~secondary_lower_limit_deg", -90.0)))
        self.secondary_upper = math.radians(
            float(rospy.get_param("~secondary_upper_limit_deg", 90.0)))
        self.repeat_count = max(
            1, int(rospy.get_param("~repeat_count", 3)))
        self.repeat_period = max(
            0.0, float(rospy.get_param("~repeat_period", 0.02)))
        self.lock_timeout = max(
            0.1, float(rospy.get_param("~lock_timeout", 2.0)))

        if self.pitch_step <= 0.0:
            raise ValueError("~pitch_step_deg must be positive")
        if self.pitch_delta_limit <= 0.0:
            raise ValueError("~pitch_delta_limit_deg must be positive")
        if self.secondary_step <= 0.0:
            raise ValueError("~secondary_step_deg must be positive")
        if self.secondary_lower >= self.secondary_upper:
            raise ValueError(
                "~secondary_lower_limit_deg must be below "
                "~secondary_upper_limit_deg")

        multilink_param_ns = topic_under(
            robot_namespace, "navigation/multilink_perching")

        self.pitch_command_sign = normalize_sign(
            rospy.get_param(
                multilink_param_ns + "/pitch_command_sign", 1.0))
        self.secondary_command_sign = normalize_sign(
            rospy.get_param(
                multilink_param_ns + "/secondary_command_sign", 1.0))
        self.secondary_joint_name = rospy.get_param(
            multilink_param_ns + "/secondary_joint_name",
            "secondary_joint")

        self.feedback = Feedback()

        self.perching_requested = False
        self.lock_confirmed = False
        self.enable_request_stamp = None
        self.previous_lock_stamp = None
        self.rezero_pending = False
        self.rezero_request_stamp = None
        self.confirmed_lock_stamp = None
        self.confirmed_lock_receive_stamp = None

        self.pitch_delta = 0.0
        self.pitch_lock = None

        self.secondary_lock = None
        self.secondary_target = None

        self.perching_enable_pub = rospy.Publisher(
            self.perching_enable_topic, Bool, queue_size=1)
        self.pitch_command_pub = rospy.Publisher(
            self.pitch_command_topic, Float64, queue_size=1)
        self.secondary_command_pub = rospy.Publisher(
            self.secondary_command_topic, Float64, queue_size=1)

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
                topic_under(robot_namespace, "flight_state")),
            UInt8,
            self.feedback.flight_state_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~model_valid_topic",
                topic_under(
                    robot_namespace,
                    "perching/multilink/model_valid")),
            Bool,
            self.feedback.model_valid_callback,
            queue_size=1))
        self.locked_pose_topic = rospy.get_param(
            "~locked_pose_topic",
            topic_under(robot_namespace, "perching/locked_pose"))
        self.subscribers.append(rospy.Subscriber(
            self.locked_pose_topic,
            PoseStamped,
            self.feedback.locked_pose_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~secondary_measured_topic",
                topic_under(
                    robot_namespace,
                    "perching/multilink/secondary_joint_measured")),
            Float64,
            self.feedback.secondary_measured_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~secondary_nominal_topic",
                topic_under(
                    robot_namespace,
                    "perching/multilink/secondary_joint_nominal")),
            Float64,
            self.feedback.secondary_nominal_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~secondary_settled_topic",
                topic_under(
                    robot_namespace,
                    "perching/multilink/secondary_settled")),
            Bool,
            self.feedback.secondary_settled_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~pitch_measured_topic",
                topic_under(
                    robot_namespace,
                    "perching/multilink/pitch_joint_measured")),
            Float64,
            self.feedback.pitch_measured_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~pitch_nominal_topic",
                topic_under(
                    robot_namespace,
                    "perching/multilink/pitch_joint_nominal")),
            Float64,
            self.feedback.pitch_nominal_callback,
            queue_size=1))
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~pitch_final_topic",
                topic_under(
                    robot_namespace,
                    "perching/multilink/pitch_joint_final")),
            Float64,
            self.feedback.pitch_final_callback,
            queue_size=1))

    def fresh(self, stamp):
        return stamp is not None and 0.0 <= (rospy.Time.now() - stamp).to_sec() <= self.feedback_timeout

    def rezero_status_fresh(self, after=None):
        return all(self.fresh(self.feedback.rezero_stamps.get(name)) and
                   (after is None or self.feedback.rezero_stamps[name] > after)
                   for name in ("pitch_rezero_active", "pitch_rezero_ready", "pitch_rezero_failed"))

    def publish_repeated(self, publisher, message):
        for index in range(self.repeat_count):
            if rospy.is_shutdown():
                return
            publisher.publish(message)
            if index + 1 < self.repeat_count and self.repeat_period > 0.0:
                time.sleep(self.repeat_period)

    def publish_float(self, publisher, value):
        self.publish_repeated(publisher, Float64(data=value))

    def ready_to_enable(self):
        if self.feedback.flight_state != HOVER_STATE:
            print(
                "Rejected: flight state is {}, not HOVER_STATE ({}).".format(
                    self.feedback.flight_state, HOVER_STATE))
            return False

        if self.feedback.model_valid is not True:
            print(
                "Rejected: multilink model_valid is {}.".format(
                    self.feedback.model_valid))
            return False

        if (not finite(self.feedback.pitch_measured) or
                not self.fresh(self.feedback.pitch_measured_receive_stamp)):
            print("Rejected: no valid pitch-joint measurement has arrived.")
            return False

        if (not finite(self.feedback.secondary_measured) or
                not self.fresh(self.feedback.secondary_measured_receive_stamp)):
            print("Rejected: no valid secondary-joint measurement has arrived.")
            return False

        return True

    def begin_lock_request(self):
        # Capture a pre-existing latched lock before sending a request. This
        # prevents a late initial latch from being accepted as a new lock.
        if self.feedback.lock_stamp is None:
            try:
                previous = rospy.wait_for_message(
                    self.locked_pose_topic, PoseStamped,
                    timeout=min(0.25, self.lock_timeout))
            except rospy.ROSException:
                # Before the first lock there may be no latched pose yet.
                pass
            else:
                self.feedback.locked_pose_callback(previous)
        # Source stamps identify sessions only. Keyboard receive/request times
        # share this process's ROS clock; never compare these two clocks.
        self.previous_lock_stamp = self.feedback.lock_stamp
        self.enable_request_stamp = rospy.Time.now()

    def fresh_lock_received(self):
        if self.enable_request_stamp is None:
            return False
        if self.feedback.lock_stamp is None or self.feedback.lock_stamp.is_zero():
            return False
        if self.feedback.lock_stamp == self.previous_lock_stamp:
            return False
        received = self.feedback.lock_receive_stamp
        if received is None or received < self.enable_request_stamp:
            return False

        if self.feedback.pitch_nominal_receive_stamp is None:
            return False
        if self.feedback.secondary_nominal_receive_stamp is None:
            return False
        if self.feedback.pitch_nominal_receive_stamp < received:
            return False
        if self.feedback.secondary_nominal_receive_stamp < received:
            return False

        return (
            finite(self.feedback.pitch_nominal) and
            finite(self.feedback.secondary_nominal))

    def wait_for_fresh_lock(self):
        deadline = time.time() + self.lock_timeout

        while not rospy.is_shutdown() and time.time() < deadline:
            if self.fresh_lock_received():
                return True
            time.sleep(0.02)

        return False

    def ready_to_move(self, allow_rezero=False):
        if self.feedback.flight_state != HOVER_STATE:
            self.lock_confirmed = False
            self.perching_requested = False
            print(
                "Rejected: flight state is {}, not HOVER_STATE ({}). "
                "Re-enable perching after returning to HOVER.".format(
                    self.feedback.flight_state, HOVER_STATE))
            return False

        if self.feedback.model_valid is not True:
            print("Rejected: multilink model is invalid.")
            return False

        if not self.perching_requested:
            print("Rejected: press 'e' to request a multilink perching lock.")
            return False

        if self.feedback.lock_stamp != self.confirmed_lock_stamp:
            self.lock_confirmed = False
        if not self.lock_confirmed:
            print("Rejected: no confirmed fresh multilink lock.")
            return False

        if (not finite(self.feedback.pitch_measured) or
                not finite(self.feedback.secondary_measured) or
                not self.fresh(self.feedback.pitch_measured_receive_stamp) or
                not self.fresh(self.feedback.secondary_measured_receive_stamp)):
            print("Rejected: fresh mechanism measurements are required.")
            return False
        if not allow_rezero and (self.rezero_pending or
                not self.rezero_status_fresh(self.confirmed_lock_receive_stamp) or
                any(self.feedback.rezero.get(name) is not False for name in
                    ("pitch_rezero_active", "pitch_rezero_ready", "pitch_rezero_failed"))):
            print("Rejected: rezero ACTIVE/HOLD/pending or stale status; wait ready and final relock.")
            return False
        return True

    def enable_perching(self):
        if self.perching_requested:
            print("Perching already requested. Use z/r, or d before a new enable.")
            return
        if self.perching_enable_pub.get_num_connections() == 0:
            print("Rejected: navigator enable subscriber is unavailable.")
            return
        if not self.ready_to_enable():
            return

        self.pitch_delta = 0.0
        self.pitch_lock = None
        self.secondary_lock = None
        self.secondary_target = None
        self.lock_confirmed = False

        self.begin_lock_request()

        self.perching_enable_pub.publish(Bool(data=True))
        self.perching_requested = True

        print("Multilink perching enable requested; waiting for fresh lock...")

        if not self.wait_for_fresh_lock():
            # Missing confirmation is not evidence that the navigator failed.
            # Do not change robot mode on a keyboard-only acknowledgement error.
            self.perching_requested = False
            self.lock_confirmed = False
            print(
                "Keyboard lock confirmation timed out after {:.2f} s. "
                "Navigator perching state is unconfirmed; no disable was sent. "
                "Motion keys remain blocked. Use s to inspect feedback, "
                "e to request another fresh lock, or d to disable explicitly."
                .format(self.lock_timeout))
            return

        self.pitch_lock = self.feedback.pitch_nominal
        self.secondary_lock = self.feedback.secondary_nominal
        self.secondary_target = self.secondary_lock
        self.lock_confirmed = True
        self.confirmed_lock_stamp = self.feedback.lock_stamp
        self.confirmed_lock_receive_stamp = self.feedback.lock_receive_stamp
        self.rezero_pending = False

        print(
            "LOCK CONFIRMED: pitch={:+.2f} deg, {}={:+.2f} deg.".format(
                math.degrees(self.pitch_lock),
                self.secondary_joint_name,
                math.degrees(self.secondary_lock)))

    def trigger_rezero(self):
        if not self.ready_to_move():
            return
        if self.pitch_rezero_pub.get_num_connections() == 0:
            print("Rejected: pitch rezero subscriber unavailable; check navigator configuration.")
            return
        self.rezero_pending = True
        self.rezero_request_stamp = rospy.Time.now()
        self.pitch_rezero_pub.publish(Empty())
        print("Rezero requested. Joints blocked until ready + final relock. Use s for status.")

    def final_relock(self):
        if not self.ready_to_move(allow_rezero=True):
            return
        if (not self.rezero_status_fresh(self.rezero_request_stamp) or
                self.feedback.rezero.get("pitch_rezero_ready") is not True or
                self.feedback.rezero.get("pitch_rezero_active") is not False or
                self.feedback.rezero.get("pitch_rezero_failed") is not False):
            print("Rejected: require fresh rezero ready without failure. On failure, d and retry safely.")
            return
        if self.relock_pub.get_num_connections() == 0:
            print("Rejected: relock subscriber unavailable.")
            return
        self.rezero_pending = True
        self.lock_confirmed = False
        self.begin_lock_request()
        self.relock_pub.publish(Empty())
        if not self.wait_for_fresh_lock():
            self.perching_requested = False
            print("Final lock unconfirmed; cutting remains blocked and no disable was sent. Use s for feedback; e requests a new lock.")
            return
        self.pitch_lock = self.feedback.pitch_nominal
        self.secondary_lock = self.feedback.secondary_nominal
        self.secondary_target = self.secondary_lock
        self.pitch_delta = 0.0
        self.lock_confirmed = True
        self.confirmed_lock_stamp = self.feedback.lock_stamp
        self.confirmed_lock_receive_stamp = self.feedback.lock_receive_stamp
        self.rezero_pending = False
        print("Fresh final lock confirmed. Cutting keys available once IDLE status arrives.")

    def disable_perching(self):
        self.perching_enable_pub.publish(Bool(data=False))

        self.perching_requested = False
        self.lock_confirmed = False
        self.enable_request_stamp = None
        self.rezero_pending = False
        self.rezero_request_stamp = None
        self.confirmed_lock_stamp = None
        self.confirmed_lock_receive_stamp = None
        self.previous_lock_stamp = None

        self.pitch_delta = 0.0
        self.pitch_lock = None
        self.secondary_lock = None
        self.secondary_target = None

        print("Multilink perching disable requested.")

    def move_pitch(self, direction):
        if not self.ready_to_move():
            return

        self.pitch_delta = clamp(
            self.pitch_delta + direction * self.pitch_step,
            -self.pitch_delta_limit,
            self.pitch_delta_limit)

        self.publish_float(self.pitch_command_pub, self.pitch_delta)

        expected_joint = None
        if finite(self.pitch_lock):
            expected_joint = (
                self.pitch_lock +
                self.pitch_command_sign * self.pitch_delta)

        if expected_joint is None:
            print(
                "Pitch delta command: {:+.2f} deg.".format(
                    math.degrees(self.pitch_delta)))
        else:
            print(
                "Pitch delta: {:+.2f} deg -> expected joint_pitch target "
                "{:+.2f} deg.".format(
                    math.degrees(self.pitch_delta),
                    math.degrees(expected_joint)))

    def physical_secondary_to_command(self, physical_target):
        return physical_target / self.secondary_command_sign

    def move_secondary(self, direction):
        if not self.ready_to_move():
            return

        if not finite(self.secondary_target):
            print("Rejected: secondary lock/target is unavailable.")
            return

        self.secondary_target = clamp(
            self.secondary_target + direction * self.secondary_step,
            self.secondary_lower,
            self.secondary_upper)

        command_value = self.physical_secondary_to_command(
            self.secondary_target)
        self.publish_float(
            self.secondary_command_pub,
            command_value)

        print(
            "{} physical target: {:+.2f} deg "
            "(published command {:+.2f} deg, sign {:+.0f}).".format(
                self.secondary_joint_name,
                math.degrees(self.secondary_target),
                math.degrees(command_value),
                self.secondary_command_sign))

    def return_to_lock(self):
        if not self.ready_to_move():
            return

        if not finite(self.secondary_lock):
            print("Rejected: locked secondary angle is unavailable.")
            return

        self.pitch_delta = 0.0
        self.secondary_target = self.secondary_lock

        self.publish_float(self.pitch_command_pub, 0.0)
        self.publish_float(
            self.secondary_command_pub,
            self.physical_secondary_to_command(self.secondary_lock))

        print(
            "Return-to-lock requested: pitch delta=0.00 deg, "
            "{}={:+.2f} deg.".format(
                self.secondary_joint_name,
                math.degrees(self.secondary_lock)))

    @staticmethod
    def format_angle(value):
        if not finite(value):
            return "unknown"
        return "{:+.2f} deg".format(math.degrees(value))

    def print_status(self):
        print("")
        print("Multilink perching feedback / keyboard session")
        print("Keyboard confirmation is not physical contact detection.")
        print("-------------------------")
        print("flight_state       : {}".format(self.feedback.flight_state))
        print("model_valid        : {}".format(self.feedback.model_valid))
        print("keyboard_requested : {}".format(self.perching_requested))
        print("keyboard_lock_ok   : {}".format(self.lock_confirmed))
        print("")
        print("rezero pending     : {}".format(self.rezero_pending))
        for name in ("pitch_rezero_active", "pitch_rezero_ready", "pitch_rezero_failed",
                     "passive_pitch_delta", "body_pitch", "body_pitch_rate"):
            print("{}: {}".format(name, self.feedback.rezero.get(name, "unknown")))
        print("pitch:")
        print("  measured         : {}".format(
            self.format_angle(self.feedback.pitch_measured)))
        print("  locked           : {}".format(
            self.format_angle(self.pitch_lock)))
        print("  command delta    : {}".format(
            self.format_angle(self.pitch_delta)))
        print("  nominal target   : {}".format(
            self.format_angle(self.feedback.pitch_nominal)))
        print("  final target     : {}".format(
            self.format_angle(self.feedback.pitch_final)))
        print("")
        print("{}:".format(self.secondary_joint_name))
        print("  measured         : {}".format(
            self.format_angle(self.feedback.secondary_measured)))
        print("  locked           : {}".format(
            self.format_angle(self.secondary_lock)))
        print("  script target    : {}".format(
            self.format_angle(self.secondary_target)))
        print("  navigator target : {}".format(
            self.format_angle(self.feedback.secondary_nominal)))
        print("  settled          : {}".format(
            self.feedback.secondary_settled))
        print("")

    def print_configuration(self):
        print(HELP)
        print("Configuration:")
        print("  robot namespace   : {}".format(self.robot_namespace))
        print("  perching topic    : {}".format(self.perching_enable_topic))
        print("  pitch topic       : {}".format(self.pitch_command_topic))
        print("  secondary topic   : {}".format(self.secondary_command_topic))
        print("  secondary joint   : {}".format(self.secondary_joint_name))
        print("  pitch sign        : {:+.0f}".format(self.pitch_command_sign))
        print("  secondary sign    : {:+.0f}".format(self.secondary_command_sign))
        print("  pitch step        : {:.2f} deg".format(
            math.degrees(self.pitch_step)))
        print("  secondary step    : {:.2f} deg".format(
            math.degrees(self.secondary_step)))
        print("")

    def run(self):
        settings = termios.tcgetattr(sys.stdin)

        self.print_configuration()
        print("Waiting briefly for ROS feedback...")
        time.sleep(0.75)
        self.print_status()

        try:
            while not rospy.is_shutdown():
                key = get_key(settings)

                if key == "\x03":
                    break
                if key == "e":
                    self.enable_perching()
                elif key == "z":
                    self.trigger_rezero()
                elif key == "r":
                    self.final_relock()
                elif key == "d":
                    self.disable_perching()
                elif key == "j":
                    self.move_secondary(-1.0)
                elif key == "l":
                    self.move_secondary(1.0)
                elif key == "i":
                    self.move_pitch(1.0)
                elif key == "k":
                    self.move_pitch(-1.0)
                elif key == " ":
                    self.return_to_lock()
                elif key == "s":
                    self.print_status()
                elif key == "h":
                    print(HELP)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print("")


def main():
    rospy.init_node("keyboard_perching_secondary")
    PerchingKeyboard().run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

