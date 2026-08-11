#!/usr/bin/env python

from __future__ import print_function

import threading

import rospy
from spinal.msg import ServoControlCmd, ServoStates
from spinal.srv import GetBoardInfo


HAND_SERVO_ID = 6
LOCK_SERVO_ID = 5

HAND_CLOSED_POSITION = 4019
LOCK_CLOSED_POSITION = 849

# Temporary hand-only target used if the hand stalls.
# Start with 4060. If insufficient, try 4080. Never exceed 4095.
HAND_OVERDRIVE_POSITION = 4095

HAND_POSITION_MIN = 3000
HAND_POSITION_MAX = 4095
LOCK_POSITION_MIN = 0
LOCK_POSITION_MAX = 1000

STEP_TICKS = 100
STEP_HOLD_TIME = 0.6
MAX_FOLLOWING_ERROR = 250

FINAL_POSITION_TOLERANCE = 10
SETTLED_LOAD = 30
SETTLE_TIME = 0.4
FINAL_TIMEOUT = 45.0

# Hand overdrive activates when position stops changing under moderate load.
HAND_STALL_LOAD_MIN = 70
HAND_STALL_TIME = 1.5
HAND_STALL_PROGRESS = 3
HAND_OVERDRIVE_TIMEOUT = 8.0

LOAD_WARNING = 200
LOAD_ABORT = 300
LOAD_ABORT_TIME = 0.5
TEMPERATURE_ABORT = 55

STATE_TIMEOUT = 1.0
PUBLISH_RATE = 20.0

COMMAND_TOPIC = "/servo/target_states"
STATE_TOPIC = "/servo/states"
BOARD_INFO_SERVICE = "/get_board_info"


class HandLockController(object):

    def __init__(self):
        self.state_lock = threading.Lock()
        self.servo_states = {}
        self.overload_start = {}

        self.hand_index = None
        self.lock_index = None

        self.command_pub = rospy.Publisher(
            COMMAND_TOPIC,
            ServoControlCmd,
            queue_size=10
        )

        self.state_sub = rospy.Subscriber(
            STATE_TOPIC,
            ServoStates,
            self.state_callback,
            queue_size=10
        )

    def state_callback(self, msg):
        now = rospy.Time.now()

        with self.state_lock:
            for servo in msg.servos:
                self.servo_states[int(servo.index)] = {
                    "angle": int(servo.angle),
                    "load": int(servo.load),
                    "temperature": int(servo.temp),
                    "error": int(servo.error),
                    "received": now,
                }

    def get_state(self, index):
        with self.state_lock:
            state = self.servo_states.get(index)

            if state is None:
                return None

            return dict(state)

    def resolve_hardware_ids(self):
        rospy.loginfo("Waiting for %s", BOARD_INFO_SERVICE)

        try:
            rospy.wait_for_service(
                BOARD_INFO_SERVICE,
                timeout=5.0
            )

            get_board_info = rospy.ServiceProxy(
                BOARD_INFO_SERVICE,
                GetBoardInfo
            )

            response = get_board_info()

        except (rospy.ROSException, rospy.ServiceException) as error:
            raise RuntimeError(
                "Cannot read DYNAMIXEL hardware IDs: {0}".format(error)
            )

        id_to_index = {}
        internal_index = 0

        for board in response.boards:
            for servo_info in board.servos:
                hardware_id = int(servo_info.id)

                if hardware_id in id_to_index:
                    raise RuntimeError(
                        "Duplicate DYNAMIXEL ID {0}".format(hardware_id)
                    )

                id_to_index[hardware_id] = internal_index
                internal_index += 1

        rospy.loginfo(
            "Discovered DYNAMIXEL hardware IDs: %s",
            sorted(id_to_index.keys())
        )

        if HAND_SERVO_ID not in id_to_index:
            raise RuntimeError(
                "Hand servo ID {0} was not found".format(HAND_SERVO_ID)
            )

        if LOCK_SERVO_ID not in id_to_index:
            raise RuntimeError(
                "Lock servo ID {0} was not found".format(LOCK_SERVO_ID)
            )

        self.hand_index = id_to_index[HAND_SERVO_ID]
        self.lock_index = id_to_index[LOCK_SERVO_ID]

        if self.hand_index == self.lock_index:
            raise RuntimeError(
                "Hand and lock resolved to the same servo"
            )

        rospy.loginfo(
            "Verified hand hardware ID %d and lock hardware ID %d",
            HAND_SERVO_ID,
            LOCK_SERVO_ID
        )

    def wait_for_servo_states(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        rate = rospy.Rate(50.0)

        while not rospy.is_shutdown():
            if rospy.Time.now() >= deadline:
                raise RuntimeError(
                    "Timed out waiting for servo states"
                )

            if (
                self.get_state(self.hand_index) is not None and
                self.get_state(self.lock_index) is not None
            ):
                return

            rate.sleep()

    def publish_position(self, internal_index, position):
        msg = ServoControlCmd()

        # The internal index is resolved from hardware ID 5 or 6.
        # There is no fixed 0/1 fallback.
        msg.index = [int(internal_index)]
        msg.angles = [int(position)]

        self.command_pub.publish(msg)

    def check_safety(self, name, index, state):
        now = rospy.Time.now()

        state_age = (now - state["received"]).to_sec()

        if state_age > STATE_TIMEOUT:
            rospy.logerr(
                "%s state is stale: %.2f seconds",
                name,
                state_age
            )
            return False

        if state["error"] != 0:
            rospy.logerr(
                "%s hardware error: 0x%02x",
                name,
                state["error"]
            )
            return False

        if state["temperature"] >= TEMPERATURE_ABORT:
            rospy.logerr(
                "%s temperature is too high: %d C",
                name,
                state["temperature"]
            )
            return False

        absolute_load = abs(state["load"])

        if absolute_load >= LOAD_WARNING:
            rospy.logwarn_throttle(
                1.0,
                "%s load/current is %d",
                name,
                state["load"]
            )

        if absolute_load >= LOAD_ABORT:
            if index not in self.overload_start:
                self.overload_start[index] = now

            elif (
                now - self.overload_start[index]
            ).to_sec() >= LOAD_ABORT_TIME:
                rospy.logerr(
                    "%s load/current remained above %d",
                    name,
                    LOAD_ABORT
                )
                return False
        else:
            self.overload_start.pop(index, None)

        return True

    def hold_present_position(self, name, index):
        state = self.get_state(index)

        if state is None:
            return

        present_position = state["angle"]

        rospy.logwarn(
            "%s: stopping at present position %d",
            name,
            present_position
        )

        rate = rospy.Rate(PUBLISH_RATE)

        for unused in range(5):
            if rospy.is_shutdown():
                return

            self.publish_position(
                index,
                present_position
            )

            rate.sleep()

    def command_ramp_step(self, name, index, goal):
        start_time = rospy.Time.now()
        rate = rospy.Rate(PUBLISH_RATE)

        while not rospy.is_shutdown():
            state = self.get_state(index)

            if state is None:
                rate.sleep()
                continue

            if not self.check_safety(name, index, state):
                return False

            following_error = abs(goal - state["angle"])

            if following_error > MAX_FOLLOWING_ERROR:
                rospy.logerr(
                    "%s following error is too large: "
                    "goal=%d actual=%d error=%d",
                    name,
                    goal,
                    state["angle"],
                    following_error
                )
                return False

            self.publish_position(index, goal)

            rospy.loginfo_throttle(
                1.0,
                "%s ramp: goal=%d actual=%d error=%d load=%d temp=%d C",
                name,
                goal,
                state["angle"],
                following_error,
                state["load"],
                state["temperature"]
            )

            if (
                rospy.Time.now() - start_time
            ).to_sec() >= STEP_HOLD_TIME:
                return True

            rate.sleep()

        return False

    def wait_for_final_position(
        self,
        name,
        index,
        final_goal,
        overdrive_goal=None
    ):
        deadline = (
            rospy.Time.now() +
            rospy.Duration(FINAL_TIMEOUT)
        )

        rate = rospy.Rate(PUBLISH_RATE)

        settled_start = None
        stall_start = None
        stall_position = None

        overdrive_active = False
        overdrive_used = False
        overdrive_start = None

        active_goal = final_goal

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if now >= deadline:
                rospy.logerr(
                    "%s timed out while moving to %d",
                    name,
                    final_goal
                )
                return False

            state = self.get_state(index)

            if state is None:
                rate.sleep()
                continue

            if not self.check_safety(name, index, state):
                return False

            actual = state["angle"]
            absolute_load = abs(state["load"])
            final_error = abs(final_goal - actual)

            if overdrive_active:
                if actual >= (
                    final_goal - FINAL_POSITION_TOLERANCE
                ):
                    overdrive_active = False
                    active_goal = final_goal

                    rospy.loginfo(
                        "%s reached closed region; returning target to %d",
                        name,
                        final_goal
                    )

                elif (
                    now - overdrive_start
                ).to_sec() >= HAND_OVERDRIVE_TIMEOUT:
                    rospy.logerr(
                        "%s overdrive timed out: actual=%d load=%d",
                        name,
                        actual,
                        state["load"]
                    )
                    return False

            following_error = abs(active_goal - actual)

            if following_error > MAX_FOLLOWING_ERROR:
                rospy.logerr(
                    "%s following error is too large: "
                    "goal=%d actual=%d error=%d",
                    name,
                    active_goal,
                    actual,
                    following_error
                )
                return False

            self.publish_position(
                index,
                active_goal
            )

            position_ok = (
                final_error <= FINAL_POSITION_TOLERANCE
            )

            load_ok = (
                absolute_load <= SETTLED_LOAD
            )

            if position_ok and load_ok and not overdrive_active:
                if settled_start is None:
                    settled_start = now

                elif (
                    now - settled_start
                ).to_sec() >= SETTLE_TIME:
                    rospy.loginfo(
                        "%s reached %d: actual=%d load=%d",
                        name,
                        final_goal,
                        actual,
                        state["load"]
                    )
                    return True
            else:
                settled_start = None

            # Detect a hand stalled near its closed position.
            if (
                overdrive_goal is not None and
                not overdrive_active and
                not overdrive_used and
                final_error > FINAL_POSITION_TOLERANCE and
                absolute_load >= HAND_STALL_LOAD_MIN and
                absolute_load < LOAD_ABORT
            ):
                if stall_start is None:
                    stall_start = now
                    stall_position = actual

                elif abs(actual - stall_position) > HAND_STALL_PROGRESS:
                    stall_start = now
                    stall_position = actual

                elif (
                    now - stall_start
                ).to_sec() >= HAND_STALL_TIME:
                    overdrive_active = True
                    overdrive_used = True
                    overdrive_start = now
                    active_goal = overdrive_goal

                    rospy.logwarn(
                        "%s stalled at %d with load %d; "
                        "temporarily increasing target to %d",
                        name,
                        actual,
                        state["load"],
                        overdrive_goal
                    )
            else:
                if not overdrive_active:
                    stall_start = None
                    stall_position = None

            rospy.loginfo_throttle(
                1.0,
                "%s final: command=%d final=%d actual=%d "
                "error=%d load=%d temp=%d C",
                name,
                active_goal,
                final_goal,
                actual,
                final_error,
                state["load"],
                state["temperature"]
            )

            rate.sleep()

        return False

    def close_servo(
        self,
        name,
        hardware_id,
        internal_index,
        closed_position,
        minimum_position,
        maximum_position,
        overdrive_position=None
    ):
        state = self.get_state(internal_index)

        if state is None:
            rospy.logerr(
                "%s servo ID %d has no state",
                name,
                hardware_id
            )
            return False

        start_position = state["angle"]

        if (
            start_position < minimum_position or
            start_position > maximum_position
        ):
            rospy.logerr(
                "%s servo ID %d starting position %d "
                "is outside [%d, %d]",
                name,
                hardware_id,
                start_position,
                minimum_position,
                maximum_position
            )
            return False

        relative_movement = (
            closed_position - start_position
        )

        rospy.loginfo(
            "%s servo ID %d: start=%d relative=%+d final=%d",
            name,
            hardware_id,
            start_position,
            relative_movement,
            closed_position
        )

        if relative_movement == 0:
            return self.wait_for_final_position(
                name,
                internal_index,
                closed_position,
                overdrive_position
            )

        direction = 1 if relative_movement > 0 else -1
        command_position = start_position

        while (
            command_position != closed_position and
            not rospy.is_shutdown()
        ):
            if direction > 0:
                next_position = min(
                    command_position + STEP_TICKS,
                    closed_position
                )
            else:
                next_position = max(
                    command_position - STEP_TICKS,
                    closed_position
                )

            final_step = (
                next_position == closed_position
            )

            if final_step:
                success = self.wait_for_final_position(
                    name,
                    internal_index,
                    closed_position,
                    (
                        overdrive_position
                        if direction > 0
                        else None
                    )
                )
            else:
                success = self.command_ramp_step(
                    name,
                    internal_index,
                    next_position
                )

            if not success:
                self.hold_present_position(
                    name,
                    internal_index
                )
                return False

            command_position = next_position

        return True

    def run(self):
        self.resolve_hardware_ids()
        self.wait_for_servo_states()

        rospy.loginfo(
            "Closing hand servo ID %d",
            HAND_SERVO_ID
        )

        hand_success = self.close_servo(
            "hand",
            HAND_SERVO_ID,
            self.hand_index,
            HAND_CLOSED_POSITION,
            HAND_POSITION_MIN,
            HAND_POSITION_MAX,
            HAND_OVERDRIVE_POSITION
        )

        if not hand_success:
            raise RuntimeError(
                "Hand did not close; lock will not be commanded"
            )

        rospy.sleep(0.5)

        rospy.loginfo(
            "Closing lock servo ID %d",
            LOCK_SERVO_ID
        )

        lock_success = self.close_servo(
            "lock",
            LOCK_SERVO_ID,
            self.lock_index,
            LOCK_CLOSED_POSITION,
            LOCK_POSITION_MIN,
            LOCK_POSITION_MAX
        )

        if not lock_success:
            raise RuntimeError(
                "Lock did not close"
            )

        rospy.loginfo(
            "Hand and lock sequence completed successfully"
        )


def main():
    rospy.init_node("close_hand_then_lock")

    try:
        controller = HandLockController()
        controller.run()

    except RuntimeError as error:
        rospy.logfatal("%s", error)
        rospy.signal_shutdown(str(error))

    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()