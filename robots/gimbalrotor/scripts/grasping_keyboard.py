#!/usr/bin/env python
from __future__ import print_function

import rospy
import sys
import select
import termios
import tty

from std_msgs.msg import Bool
from spinal.msg import ServoControlCmd, ServoStates


HELP_MSG = """
Keyboard control:
----------------------------------------
o : move OPEN direction
c : move CLOSE direction
q : quit
Ctrl+C : quit
----------------------------------------

Behavior:
- OPEN always uses the fixed open direction
- CLOSE always uses the fixed close direction
- torque/load limit is checked for both directions
- if CLOSE torque limit is reached:
    publish /isGrasping = True
- otherwise:
    publish /isGrasping = False
"""


def getKey(settings, timeout=0.1):
    """
    Read one key with timeout.
    Returns '' if no key was pressed.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def printMsg(text, msg_len=100):
    print(text.ljust(msg_len) + "\r", end="")
    sys.stdout.flush()


class ManualServoKeyboardControl:
    def __init__(self):
        rospy.init_node("manual_servo_keyboard_control")

        # =========================================================
        # Local settings only (no rosparam)
        # =========================================================

        self.target_servo_index = 0
        self.servo_move_range = 100

        # Direction definition:
        # +1 means target = current + step
        # -1 means target = current - step
        self.open_direction_sign = -1
        self.close_direction_sign = -self.open_direction_sign

        # Torque/load safety limits
        self.negative_load_limit = -180
        self.positive_load_limit = 100

        # Topics
        self.state_topic = '/gimbalrotor/spinal_dynamixel/servo/states'
        self.command_topic = '/gimbalrotor/spinal_dynamixel/servo/target_states'
        self.is_grasping_topic = '/isGrasping'

        # =========================================================
        # State variables
        # =========================================================
        self.current_servo_load = 0.0
        self.current_servo_position = 0.0
        self.target_servo_position = 0.0

        self.last_callback_time = rospy.get_time()
        self.received_state = False
        self.is_grasping = False

        # =========================================================
        # ROS interfaces
        # =========================================================
        self.sub = rospy.Subscriber(
            self.state_topic,
            ServoStates,
            self.callback,
            queue_size=10
        )

        self.pub = rospy.Publisher(
            self.command_topic,
            ServoControlCmd,
            queue_size=10
        )

        self.grasp_pub = rospy.Publisher(
            self.is_grasping_topic,
            Bool,
            queue_size=10
        )

        rospy.loginfo("manual_servo_keyboard_control started")
        rospy.loginfo("Target servo index: %d", self.target_servo_index)
        rospy.loginfo("servo_move_range: %d", self.servo_move_range)
        rospy.loginfo("open_direction_sign: %d", self.open_direction_sign)
        rospy.loginfo("close_direction_sign: %d", self.close_direction_sign)
        rospy.loginfo("negative_load_limit: %.2f", self.negative_load_limit)
        rospy.loginfo("positive_load_limit: %.2f", self.positive_load_limit)
        rospy.loginfo("state_topic: %s", self.state_topic)
        rospy.loginfo("command_topic: %s", self.command_topic)
        rospy.loginfo("is_grasping_topic: %s", self.is_grasping_topic)

    def callback(self, data):
        self.last_callback_time = rospy.get_time()

        target_servo_data = next(
            (servo for servo in data.servos if servo.index == self.target_servo_index),
            None
        )

        if target_servo_data is not None:
            self.current_servo_load = target_servo_data.load
            self.current_servo_position = target_servo_data.angle
            self.received_state = True

    def check_subscription(self):
        current_time = rospy.get_time()
        if current_time - self.last_callback_time > 5.0:
            rospy.logwarn_throttle(2.0, "No servo data received for more than 5 seconds.")
            return False
        return True

    def publish_is_grasping(self, value):
        self.is_grasping = value
        msg = Bool()
        msg.data = value
        self.grasp_pub.publish(msg)

    def torque_blocked(self, move_sign):
        """
        move_sign:
            +1 -> target = current + step
            -1 -> target = current - step
        """
        load = self.current_servo_load

        if move_sign < 0:
            if load <= self.negative_load_limit:
                return True, "Blocked: negative torque/load limit reached"

        elif move_sign > 0:
            if load >= self.positive_load_limit:
                return True, "Blocked: positive torque/load limit reached"

        return False, ""

    def move_servo_step(self, move_sign, action_name):
        blocked, reason = self.torque_blocked(move_sign)

        if blocked:
            rospy.logwarn("%s | current load = %.2f", reason, self.current_servo_load)

            if action_name == "close":
                self.publish_is_grasping(True)
                rospy.loginfo("Published /isGrasping = True")
            else:
                self.publish_is_grasping(False)

            return False

        self.publish_is_grasping(False)

        self.target_servo_position = int(
            self.current_servo_position + move_sign * self.servo_move_range
        )

        cmd = ServoControlCmd()
        cmd.index = [self.target_servo_index]
        cmd.angles = [int(self.target_servo_position)]
        self.pub.publish(cmd)

        rospy.loginfo(
            "Publish servo[%d] target: %d | action: %s | current_pos: %.2f | current_load: %.2f | isGrasping: %s",
            self.target_servo_index,
            int(self.target_servo_position),
            action_name,
            self.current_servo_position,
            self.current_servo_load,
            str(self.is_grasping)
        )

        return True

    def wait_for_first_servo_state(self, settings):
        """
        Wait for first servo state, but still allow quitting with q or Ctrl+C.
        """
        rospy.loginfo("Waiting for servo state on %s ...", self.state_topic)
        printMsg("Waiting for first servo data... press q or Ctrl+C to quit")

        while not rospy.is_shutdown() and not self.received_state:
            key = getKey(settings, timeout=0.1)

            if key == 'q':
                rospy.loginfo("Quit requested while waiting for first servo state.")
                return False

            if key == '\x03':  # Ctrl+C
                raise KeyboardInterrupt

        if rospy.is_shutdown():
            return False

        rospy.loginfo(
            "Servo state received. position=%.2f load=%.2f",
            self.current_servo_position,
            self.current_servo_load
        )
        print("")
        return True

    def run(self):
        print(HELP_MSG)
        settings = termios.tcgetattr(sys.stdin)

        try:
            if not self.wait_for_first_servo_state(settings):
                return

            self.publish_is_grasping(False)

            while not rospy.is_shutdown():
                key = getKey(settings, timeout=0.1)
                status_msg = "Use o / c / q"

                if key == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt

                if key == 'q':
                    rospy.loginfo("Quit keyboard control")
                    break

                if not self.check_subscription():
                    status_msg = "No servo data | press q to quit"
                    printMsg(status_msg)
                    rospy.sleep(0.01)
                    continue

                if key == 'o':
                    moved = self.move_servo_step(self.open_direction_sign, "open")
                    if moved:
                        status_msg = "o pressed: move OPEN direction"
                    else:
                        status_msg = "OPEN blocked by torque/load limit"

                elif key == 'c':
                    moved = self.move_servo_step(self.close_direction_sign, "close")
                    if moved:
                        status_msg = "c pressed: move CLOSE direction"
                    else:
                        status_msg = "CLOSE blocked -> /isGrasping = True"

                printMsg(status_msg)
                rospy.sleep(0.01)

        except KeyboardInterrupt:
            rospy.loginfo("Keyboard interrupt received. Exiting...")

        except Exception as e:
            rospy.logerr("Exception: %s", repr(e))

        finally:
            try:
                self.publish_is_grasping(False)
            except Exception:
                pass

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print("")


if __name__ == "__main__":
    try:
        controller = ManualServoKeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    sys.exit(0)