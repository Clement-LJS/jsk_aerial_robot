#!/usr/bin/env python

from __future__ import print_function

import rospy

from spinal.msg import ServoTorqueCmd, ServoTorqueStates
from spinal.srv import GetBoardInfo, SetDirectServoConfig


HAND_HARDWARE_ID = 5

# Increase static closing force.
NEW_P_GAIN = 1400

# Preserve your existing values.
NEW_I_GAIN = 0
NEW_D_GAIN = 0

BOARD_INFO_SERVICE = "gimbalrotor/get_board_info"
CONFIG_SERVICE = "gimbalrotor/direct_servo_config"
TORQUE_TOPIC = "gimbalrotor/servo/torque_enable"
TORQUE_STATE_TOPIC = "gimbalrotor/servo/torque_states"


def resolve_hardware_id(hardware_id):
    rospy.wait_for_service(BOARD_INFO_SERVICE, timeout=5.0)

    get_board_info = rospy.ServiceProxy(
        BOARD_INFO_SERVICE,
        GetBoardInfo
    )

    response = get_board_info()

    id_to_index = {}
    internal_index = 0

    for board in response.boards:
        for servo in board.servos:
            id_to_index[int(servo.id)] = internal_index
            internal_index += 1

    rospy.loginfo(
        "Discovered DYNAMIXEL IDs: %s",
        sorted(id_to_index.keys())
    )

    if hardware_id not in id_to_index:
        raise RuntimeError(
            "DYNAMIXEL ID {0} was not found".format(hardware_id)
        )

    return id_to_index[hardware_id]


def disable_torque(internal_index):
    publisher = rospy.Publisher(
        TORQUE_TOPIC,
        ServoTorqueCmd,
        queue_size=10
    )

    deadline = rospy.Time.now() + rospy.Duration(3.0)

    while (
        publisher.get_num_connections() == 0 and
        rospy.Time.now() < deadline and
        not rospy.is_shutdown()
    ):
        rospy.sleep(0.05)

    command = ServoTorqueCmd()
    command.index = [internal_index]
    command.torque_enable = [0]

    rate = rospy.Rate(20.0)

    for unused in range(10):
        publisher.publish(command)
        rate.sleep()

    torque_state = rospy.wait_for_message(
        TORQUE_STATE_TOPIC,
        ServoTorqueStates,
        timeout=3.0
    )

    if internal_index >= len(torque_state.torque_enable):
        raise RuntimeError(
            "Torque-state array does not contain the hand servo"
        )

    if torque_state.torque_enable[internal_index] != 0:
        raise RuntimeError(
            "Hand torque did not turn off"
        )

    rospy.loginfo(
        "Hand ID %d torque is OFF",
        HAND_HARDWARE_ID
    )


def set_hand_gain(internal_index):
    rospy.wait_for_service(CONFIG_SERVICE, timeout=5.0)

    configure = rospy.ServiceProxy(
        CONFIG_SERVICE,
        SetDirectServoConfig
    )

    # Command 2:
    # [internal_index, P gain, I gain, D gain]
    response = configure(
        2,
        [
            internal_index,
            NEW_P_GAIN,
            NEW_I_GAIN,
            NEW_D_GAIN
        ]
    )

    if not response.success:
        raise RuntimeError(
            "Spinal rejected the PID gain configuration"
        )

    rospy.loginfo(
        "Hand ID %d gains set: P=%d I=%d D=%d",
        HAND_HARDWARE_ID,
        NEW_P_GAIN,
        NEW_I_GAIN,
        NEW_D_GAIN
    )


def verify_gain():
    get_board_info = rospy.ServiceProxy(
        BOARD_INFO_SERVICE,
        GetBoardInfo
    )

    response = get_board_info()

    for board in response.boards:
        for servo in board.servos:
            if int(servo.id) == HAND_HARDWARE_ID:
                rospy.loginfo(
                    "Verified ID %d: P=%d I=%d D=%d current_limit=%d",
                    servo.id,
                    servo.p_gain,
                    servo.i_gain,
                    servo.d_gain,
                    servo.current_limit
                )

                if int(servo.p_gain) != NEW_P_GAIN:
                    raise RuntimeError(
                        "P gain verification failed"
                    )

                return

    raise RuntimeError(
        "Hand servo disappeared during verification"
    )


def main():
    rospy.init_node("set_hand_gain_once")

    try:
        hand_index = resolve_hardware_id(
            HAND_HARDWARE_ID
        )

        disable_torque(hand_index)
        set_hand_gain(hand_index)
        verify_gain()

        rospy.loginfo(
            "Configuration completed. Now run close_hand_then_lock.py"
        )

    except (
        RuntimeError,
        rospy.ROSException,
        rospy.ServiceException
    ) as error:
        rospy.logfatal("%s", error)


if __name__ == "__main__":
    main()