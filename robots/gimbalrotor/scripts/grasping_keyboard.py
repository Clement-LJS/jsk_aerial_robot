#!/usr/bin/env python
from __future__ import print_function

import rospy
import sys
import select
import termios
import tty

from spinal.msg import ServoControlCmd, ServoStates


msg = """
Instruction:
--------------------------------------------------
1. First type: open  or  close
   This defines your intended main direction.

2. Then use keyboard:
   i : move in the intended direction
   k : move in the opposite direction
   q : quit

Example:
- if you typed "open"
    i -> open direction
    k -> opposite direction

- if you typed "close"
    i -> close direction
    k -> opposite direction
--------------------------------------------------
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def printMsg(text, msg_len=80):
    print(text.ljust(msg_len) + "\r", end="")


class ManualServoKeyboardControl:
    def __init__(self):
        rospy.init_node("manual_servo_keyboard_control")

        # ---------------------------------
        # Servo config
        # ---------------------------------
        self.target_servo_index = 0
        self.servoMoveRange = rospy.get_param("~servo_move_range", 100)

        # ---------------------------------
        # Safety torque/load thresholds
        # Tune these if needed
        # ---------------------------------
        # If moving in negative-load direction, do not continue
        # when load is already too negative.
        self.negative_load_limit = rospy.get_param("~negative_load_limit", -180)

        # If moving in positive-load direction, do not continue
        # when load is already too positive.
        self.positive_load_limit = rospy.get_param("~positive_load_limit", 100)

        # ---------------------------------
        # Servo state (only servo index 0)
        # ---------------------------------
        self.currentServoLoad_0 = 0.0
        self.currentServoPosition_0 = 0.0
        self.targetServoPosition_0 = 0.0
        self.last_callback_time = rospy.get_time()
        self.received_state = False

        # ---------------------------------
        # User-defined motion meaning
        # open_direction_sign:
        #   +1 means open = current + step
        #   -1 means open = current - step
        #
        # close will be the opposite sign.
        # If your real hardware moves opposite to your expectation,
        # just swap this sign.
        # ---------------------------------
        self.open_direction_sign = rospy.get_param("~open_direction_sign", -1)
        self.close_direction_sign = -self.open_direction_sign

        self.user_mode = None  # "open" or "close"

        # ---------------------------------
        # ROS
        # ---------------------------------
        self.sub = rospy.Subscriber(
            '/gimbalrotor/spinal_dynamixel/servo/states',
            ServoStates,
            self.callback,
            queue_size=10
        )
        self.pub = rospy.Publisher(
            '/gimbalrotor/spinal_dynamixel/servo/target_states',
            ServoControlCmd,
            queue_size=10
        )

        rospy.loginfo("manual_servo_keyboard_control started")
        rospy.loginfo("Target servo index: %d", self.target_servo_index)
        rospy.loginfo("servoMoveRange: %d", self.servoMoveRange)
        rospy.loginfo("open_direction_sign: %d", self.open_direction_sign)
        rospy.loginfo("close_direction_sign: %d", self.close_direction_sign)
        rospy.loginfo("negative_load_limit: %.2f", self.negative_load_limit)
        rospy.loginfo("positive_load_limit: %.2f", self.positive_load_limit)

    def callback(self, data):
        self.last_callback_time = rospy.get_time()

        index0_data = next((servo for servo in data.servos if servo.index == 0), None)

        if index0_data is not None:
            self.currentServoLoad_0 = index0_data.load
            self.currentServoPosition_0 = index0_data.angle
            self.received_state = True

    def check_subscription(self):
        current_time = rospy.get_time()
        if current_time - self.last_callback_time > 5.0:
            rospy.logwarn("No servo data received for more than 5 seconds.")
            return False
        return True

    def choose_mode(self):
        while not rospy.is_shutdown():
            print("\nType mode: open / close")
            try:
                targetInput = input().strip().lower()
            except KeyboardInterrupt:
                rospy.loginfo("Interrupted from keyboard.")
                rospy.signal_shutdown("")
                return False

            if targetInput == "open":
                self.user_mode = "open"
                rospy.loginfo("User mode set to OPEN")
                return True

            elif targetInput == "close":
                self.user_mode = "close"
                rospy.loginfo("User mode set to CLOSE")
                return True

            else:
                print("Incorrect input. Please enter open or close.")

        return False

    def get_main_direction(self):
        if self.user_mode == "open":
            return self.open_direction_sign
        elif self.user_mode == "close":
            return self.close_direction_sign
        else:
            return 0

    def torque_blocked(self, move_sign):
        """
        move_sign:
            +1 -> target = current + step
            -1 -> target = current - step

        We use load sign to block motion if torque is already too high
        in that direction.
        """

        load = self.currentServoLoad_0

        # If commanding more in the negative direction,
        # block when load is already too negative.
        if move_sign < 0:
            if load <= self.negative_load_limit:
                return True, "Blocked: negative torque/load limit reached"

        # If commanding more in the positive direction,
        # block when load is already too positive.
        elif move_sign > 0:
            if load >= self.positive_load_limit:
                return True, "Blocked: positive torque/load limit reached"

        return False, ""

    def move_servo_step(self, move_sign):
        blocked, reason = self.torque_blocked(move_sign)
        if blocked:
            rospy.logwarn("%s | current load = %.2f", reason, self.currentServoLoad_0)
            return

        self.targetServoPosition_0 = int(
            self.currentServoPosition_0 + move_sign * self.servoMoveRange
        )

        cmd = ServoControlCmd()
        cmd.index = [0]
        cmd.angles = [int(self.targetServoPosition_0)]
        self.pub.publish(cmd)

        rospy.loginfo(
            "Publish servo[0] target: %d | current_pos: %.2f | current_load: %.2f",
            int(self.targetServoPosition_0),
            self.currentServoPosition_0,
            self.currentServoLoad_0
        )

    def run(self):
        if not self.choose_mode():
            return

        print(msg)
        settings = termios.tcgetattr(sys.stdin)

        # Wait for first servo state
        rospy.loginfo("Waiting for servo state on /gimbalrotor/spinal_dynamixel/servo/states ...")
        wait_rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self.received_state:
            wait_rate.sleep()

        if rospy.is_shutdown():
            return

        rospy.loginfo(
            "Servo state received. position=%.2f load=%.2f",
            self.currentServoPosition_0,
            self.currentServoLoad_0
        )

        try:
            while not rospy.is_shutdown():
                key = getKey(settings)
                status_msg = ""

                if not self.check_subscription():
                    status_msg = "No servo data"
                    printMsg(status_msg)
                    rospy.sleep(0.001)
                    continue

                main_dir = self.get_main_direction()
                opposite_dir = -main_dir

                if key == 'i':
                    self.move_servo_step(main_dir)
                    if self.user_mode == "open":
                        status_msg = "i pressed: move OPEN direction"
                    else:
                        status_msg = "i pressed: move CLOSE direction"

                elif key == 'k':
                    self.move_servo_step(opposite_dir)
                    if self.user_mode == "open":
                        status_msg = "k pressed: move CLOSE direction"
                    else:
                        status_msg = "k pressed: move OPEN direction"

                elif key == 'q' or key == '\x03':
                    rospy.loginfo("Quit keyboard control")
                    break

                else:
                    status_msg = "Use i / k / q"

                printMsg(status_msg)
                rospy.sleep(0.001)

        except Exception as e:
            print(repr(e))

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    try:
        controller = ManualServoKeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        sys.exit(1)
