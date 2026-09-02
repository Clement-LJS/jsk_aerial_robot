#!/usr/bin/env python

from __future__ import print_function

import math
import select
import sys
import termios
import time
import tty

import rospy
from std_msgs.msg import Bool, Float64, UInt8


HOVER_STATE = 5

HELP = """
Multilink perching keyboard command
-----------------------------------
e       : enable perching and lock the current contact/joint configuration
d       : disable perching (admittance is disabled first)

j / l   : secondary joint (joint_yaw) negative / positive
i / k   : active pitch joint positive / negative from its locked angle
space   : return both joints to the angles captured by 'e'

t       : disable perching admittance and request a fresh tare
a       : request perching admittance enable
s       : print current status
h       : print this help
CTRL-C  : quit (leaves the current perching state unchanged)

Safety/ownership:
  Joint commands are accepted only after this script has sent 'e', while the
  navigator reports HOVER_STATE and a valid multilink model.  Every joint move
  first disables perching admittance and starts a fresh tare.  After the move
  has settled, wait for the controller's "Equilibrium pivot wrench ready" log
  before pressing 'a'.
"""


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


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
        self.flight_state = None
        self.model_valid = None
        self.secondary_measured = None
        self.secondary_settled = None
        self.pitch_measured = None
        self.pitch_nominal = None
        self.pitch_final = None
        self.contact_active = None

    def flight_state_callback(self, message):
        self.flight_state = message.data

    def model_valid_callback(self, message):
        self.model_valid = message.data

    def secondary_measured_callback(self, message):
        self.secondary_measured = message.data

    def secondary_settled_callback(self, message):
        self.secondary_settled = message.data

    def pitch_measured_callback(self, message):
        self.pitch_measured = message.data

    def pitch_nominal_callback(self, message):
        self.pitch_nominal = message.data

    def pitch_final_callback(self, message):
        self.pitch_final = message.data

    def contact_active_callback(self, message):
        self.contact_active = message.data


class PerchingKeyboard(object):
    def __init__(self):
        robot_namespace = rospy.get_param("~robot_namespace", "/gimbalrotor")

        self.perching_enable_topic = rospy.get_param(
            "~perching_enable_topic",
            topic_under(robot_namespace, "perching/enable"))
        self.admittance_enable_topic = rospy.get_param(
            "~admittance_enable_topic",
            topic_under(robot_namespace, "perching/admittance_enable"))
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

        if self.pitch_step <= 0.0:
            raise ValueError("~pitch_step_deg must be positive")
        if self.secondary_step <= 0.0:
            raise ValueError("~secondary_step_deg must be positive")
        if self.secondary_lower >= self.secondary_upper:
            raise ValueError(
                "~secondary_lower_limit_deg must be below "
                "~secondary_upper_limit_deg")

        self.feedback = Feedback()
        self.perching_requested = False
        self.admittance_requested = False
        self.pitch_delta = 0.0
        self.secondary_lock = None
        self.secondary_target = None

        self.perching_enable_pub = rospy.Publisher(
            self.perching_enable_topic, Bool, queue_size=1)
        self.admittance_enable_pub = rospy.Publisher(
            self.admittance_enable_topic, Bool, queue_size=1)
        self.pitch_command_pub = rospy.Publisher(
            self.pitch_command_topic, Float64, queue_size=1)
        self.secondary_command_pub = rospy.Publisher(
            self.secondary_command_topic, Float64, queue_size=1)

        self.subscribers = []
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
        self.subscribers.append(rospy.Subscriber(
            rospy.get_param(
                "~contact_active_topic",
                topic_under(
                    robot_namespace,
                    "perching/contact_admittance/contact_active")),
            Bool,
            self.feedback.contact_active_callback,
            queue_size=1))

    def publish_repeated(self, publisher, message):
        for index in range(self.repeat_count):
            publisher.publish(message)
            if index + 1 < self.repeat_count and self.repeat_period > 0.0:
                time.sleep(self.repeat_period)

    def publish_bool(self, publisher, value):
        publisher.publish(Bool(data=value))

    def publish_float(self, publisher, value):
        self.publish_repeated(publisher, Float64(data=value))

    def ready_to_enable(self):
        if self.feedback.flight_state != HOVER_STATE:
            print(
                "Rejected: flight state is {}, not HOVER_STATE (5).".format(
                    self.feedback.flight_state))
            return False
        if self.feedback.model_valid is not True:
            print(
                "Rejected: multilink model_valid is {}.".format(
                    self.feedback.model_valid))
            return False
        if self.feedback.secondary_measured is None:
            print("Rejected: no secondary-joint measurement has arrived.")
            return False
        return True

    def ready_to_move(self):
        if not self.perching_requested:
            print("Rejected: press 'e' to create a perching lock first.")
            return False
        return self.ready_to_enable()

    def disable_admittance_for_motion(self):
        self.publish_bool(self.admittance_enable_pub, False)
        self.admittance_requested = False

    def enable_perching(self):
        if not self.ready_to_enable():
            return

        self.pitch_delta = 0.0
        self.secondary_lock = self.feedback.secondary_measured
        self.secondary_target = self.secondary_lock

        self.disable_admittance_for_motion()
        self.publish_bool(self.perching_enable_pub, True)
        self.perching_requested = True

        print(
            "Perching enable requested; locked secondary reference "
            "{:+.2f} deg. Verify the navigator reports a successful lock.".format(
                math.degrees(self.secondary_lock)))

    def disable_perching(self):
        self.disable_admittance_for_motion()
        self.publish_bool(self.perching_enable_pub, False)
        self.perching_requested = False
        self.pitch_delta = 0.0
        self.secondary_lock = None
        self.secondary_target = None
        print("Perching and perching admittance disable requested.")

    def move_pitch(self, direction):
        if not self.ready_to_move():
            return

        self.pitch_delta = clamp(
            self.pitch_delta + direction * self.pitch_step,
            -self.pitch_delta_limit,
            self.pitch_delta_limit)
        self.disable_admittance_for_motion()
        self.publish_float(self.pitch_command_pub, self.pitch_delta)
        print(
            "Pitch target: locked pitch {:+.2f} deg (admittance disabled; "
            "fresh tare collecting).".format(math.degrees(self.pitch_delta)))

    def move_secondary(self, direction):
        if not self.ready_to_move():
            return
        if self.secondary_target is None:
            print("Rejected: secondary target was not initialized by 'e'.")
            return

        self.secondary_target = clamp(
            self.secondary_target + direction * self.secondary_step,
            self.secondary_lower,
            self.secondary_upper)
        self.disable_admittance_for_motion()
        self.publish_float(
            self.secondary_command_pub, self.secondary_target)
        print(
            "Secondary target: {:+.2f} deg (admittance disabled; fresh tare "
            "collecting).".format(math.degrees(self.secondary_target)))

    def return_to_lock(self):
        if not self.ready_to_move():
            return

        self.pitch_delta = 0.0
        self.secondary_target = self.secondary_lock
        self.disable_admittance_for_motion()
        self.publish_float(self.pitch_command_pub, self.pitch_delta)
        self.publish_float(
            self.secondary_command_pub, self.secondary_target)
        print("Both active joints commanded back to the angles captured by 'e'.")

    def request_fresh_tare(self):
        if not self.ready_to_move():
            return
        self.disable_admittance_for_motion()
        print(
            "Perching admittance disabled; fresh tare requested. Wait for the "
            "equilibrium-wrench-ready log before pressing 'a'.")

    def request_admittance_enable(self):
        if not self.ready_to_move():
            return
        if self.feedback.secondary_settled is False:
            print(
                "Rejected: the secondary joint is not settled. Wait, request "
                "a fresh tare with 't', then arm with 'a'.")
            return

        self.publish_bool(self.admittance_enable_pub, True)
        self.admittance_requested = True
        print(
            "Perching admittance enable requested. The controller will accept "
            "it only when the lock and equilibrium-wrench tare are valid.")

    @staticmethod
    def format_angle(value):
        if value is None:
            return "unknown"
        return "{:+.2f} deg".format(math.degrees(value))

    def print_status(self):
        print(
            "flight={} model_valid={} perching_requested={} "
            "admittance_requested={} secondary_measured={} "
            "secondary_target={} secondary_settled={} pitch_measured={} "
            "pitch_nominal={} pitch_final={} contact_active={}".format(
                self.feedback.flight_state,
                self.feedback.model_valid,
                self.perching_requested,
                self.admittance_requested,
                self.format_angle(self.feedback.secondary_measured),
                self.format_angle(self.secondary_target),
                self.feedback.secondary_settled,
                self.format_angle(self.feedback.pitch_measured),
                self.format_angle(self.feedback.pitch_nominal),
                self.format_angle(self.feedback.pitch_final),
                self.feedback.contact_active))

    def print_configuration(self):
        print(HELP)
        print("Command topics:")
        print("  perching:  {}".format(self.perching_enable_topic))
        print("  admittance: {}".format(self.admittance_enable_topic))
        print("  pitch:      {}".format(self.pitch_command_topic))
        print("  secondary:  {}".format(self.secondary_command_topic))
        print(
            "Steps: pitch {:.2f} deg, secondary {:.2f} deg".format(
                math.degrees(self.pitch_step),
                math.degrees(self.secondary_step)))

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        self.print_configuration()
        print("Waiting briefly for ROS topic connections/feedback...")
        time.sleep(0.75)
        self.print_status()

        try:
            while not rospy.is_shutdown():
                key = get_key(settings)

                if key == "\x03":
                    break
                if key == "e":
                    self.enable_perching()
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
                elif key == "t":
                    self.request_fresh_tare()
                elif key == "a":
                    self.request_admittance_enable()
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
    main()
