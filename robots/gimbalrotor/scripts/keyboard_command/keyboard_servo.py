#!/usr/bin/env python

from __future__ import print_function

import select
import sys
import termios
import time
import tty

import rospy
from spinal.msg import ServoControlCmd, ServoStates


HELP = """
Servo keyboard command
----------------------
o: move one positive step
p: move one negative step
CTRL-C: quit
"""


class ServoState(object):
    def __init__(self, servo_index):
        self.servo_index = servo_index
        self.current_angle = None

    def callback(self, message):
        for servo in message.servos:
            if servo.index == self.servo_index:
                self.current_angle = servo.angle
                return


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("keyboard_servo_command")

    target_topic = rospy.get_param(
        "~target_topic", "/servo/target_states")
    current_topic = rospy.get_param(
        "~current_topic", "/servo/states")
    servo_index = rospy.get_param("~servo_index", 1)
    angle_step = rospy.get_param("~angle_step", 50)
    key_repeat_timeout = rospy.get_param("~key_repeat_timeout", 0.10)

    publisher = rospy.Publisher(target_topic, ServoControlCmd, queue_size=1)
    servo_state = ServoState(servo_index)
    rospy.Subscriber(
        current_topic, ServoStates, servo_state.callback, queue_size=1)

    print(HELP)
    print("Waiting for current state of servo index {0}...".format(
        servo_index))

    last_target = None
    last_direction = None
    last_key = None
    last_key_time = 0.0

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)

            if key == "\x03":
                break

            if key not in ("o", "p"):
                continue

            key_time = time.time()
            time_since_last_key = key_time - last_key_time
            repeated_too_quickly = (
                key == last_key and time_since_last_key < key_repeat_timeout)
            last_key = key
            last_key_time = key_time

            if repeated_too_quickly:
                continue

            if servo_state.current_angle is None:
                print("No current state received for servo index {0}".format(
                    servo_index))
                continue

            current_angle = servo_state.current_angle
            if key == "o":
                if last_direction == "positive" and last_target is not None:
                    base_angle = max(current_angle, last_target)
                else:
                    base_angle = current_angle
                target_angle = base_angle + angle_step
                last_direction = "positive"
            else:
                if last_direction == "negative" and last_target is not None:
                    base_angle = min(current_angle, last_target)
                else:
                    base_angle = current_angle
                target_angle = base_angle - angle_step
                last_direction = "negative"

            last_target = target_angle

            command = ServoControlCmd()
            command.index = [servo_index]
            command.angles = [target_angle]
            publisher.publish(command)

            print("Published: index={0}, current={1}, base={2}, target={3}".format(
                servo_index, current_angle, base_angle, target_angle))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()