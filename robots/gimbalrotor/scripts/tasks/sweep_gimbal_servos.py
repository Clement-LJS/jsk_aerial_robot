#!/usr/bin/env python3
"""Sweep selected gimbals together: current position -> upper limit -> lower limit.

Run after `roslaunch spinal bridge.launch`, with the flight controller inactive:
    rosrun gimbalrotor sweep_gimbal_servos.py --speed-deg-s 0.1
    rosrun gimbalrotor sweep_gimbal_servos.py --cycles 3 --speed-deg-s 0.1
    rosrun gimbalrotor sweep_gimbal_servos.py --indices 0 --speed-deg-s 1.0
    rosrun gimbalrotor sweep_gimbal_servos.py --dry-run

The default topics are /servo/target_states and /servo/states.
The Beetle defaults are indices 0-3 and URDF limits +/-1.57 rad (+/-89.95 deg).
With bridge.launch alone, read calibration and limits from the local Beetle
files and publish the selected gimbals' profiles on /joint_profiles before motion.
If servo_controller parameters exist in --namespace, use those and the loaded
URDF instead. Use --namespace /gimbalrotor for the full robot bringup.
One cycle is positive limit -> negative limit. Between cycles, return to the
positive limit; finish at the negative limit after the last cycle (default: 1).
Approach, returns, and sweeps all use the requested speed. Encoder quantization makes
very slow motion discrete (about 0.088 deg per tick with the Beetle calibration).
Ctrl+C stops publishing; the servos retain their last commanded positions.
"""

import argparse
import math
from pathlib import Path
import sys
import threading
import time
import xml.etree.ElementTree as ET

import rospy
from spinal.msg import JointProfile, JointProfiles, ServoControlCmd, ServoStates


DEFAULT_SPEED_DEG_S = 240.0
DEFAULT_CYCLES = 2
DEFAULT_INDICES = [0, 1, 2, 3]


def positive_number(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be a finite number greater than zero")
    return number


def positive_integer(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be an integer greater than zero")
    return number


class Gimbal:
    def __init__(self, settings, joint):
        self.index = int(settings["id"])
        self.name = settings["name"]
        self.scale = float(settings["angle_scale"])
        self.sign = int(settings["angle_sgn"])
        self.offset = int(settings["zero_point_offset"])
        limit = joint.find("limit")
        if limit is None:
            raise ValueError("No URDF limits for " + self.name)
        self.lower = float(limit.attrib["lower"])
        self.upper = float(limit.attrib["upper"])
        self.velocity = float(limit.attrib["velocity"])
        if (not all(math.isfinite(v) for v in
                    (self.scale, self.lower, self.upper, self.velocity))
                or self.scale <= 0 or self.sign not in (-1, 1)
                or not self.lower < 0 < self.upper or self.velocity <= 0):
            raise ValueError("Invalid calibration or limits for " + self.name)

        # Round inward so even endpoint commands stay inside the URDF limits.
        endpoints = [self.offset + angle * self.sign / self.scale
                     for angle in (self.lower, self.upper)]
        self.raw_lower = max(math.ceil(min(endpoints)),
                             int(settings.get("raw_lower_limit", 0)), -32768)
        self.raw_upper = min(math.floor(max(endpoints)),
                             int(settings.get("raw_upper_limit", 4095)), 32767)
        if self.raw_lower >= self.raw_upper:
            raise ValueError("No usable encoder range for " + self.name)
        reachable = sorted(self.angle(raw) for raw in (self.raw_lower, self.raw_upper))
        if (reachable[0] - self.lower > self.scale * 1.001
                or self.upper - reachable[1] > self.scale * 1.001):
            raise ValueError("Raw encoder limits prevent a full sweep of " + self.name)

    def angle(self, raw):
        return (raw - self.offset) * self.scale * self.sign

    def ticks(self, angle):
        raw = round(self.offset + angle * self.sign / self.scale)
        return max(self.raw_lower, min(self.raw_upper, raw))


def load_gimbals(namespace, indices):
    local_config = not rospy.has_param(namespace + "/servo_controller")
    if local_config:
        import xacro
        import yaml

        package = Path(__file__).resolve().parents[2]
        config_path = package / "config/beetle/Servo.yaml"
        model_path = package / "robots/beetle/gimbalrotor.urdf.xacro"
        controller = yaml.safe_load(config_path.read_text())["servo_controller"]
        model = xacro.process_file(str(model_path)).toxml()
        rospy.loginfo("Using local Beetle calibration: %s", config_path)
    else:
        controller = rospy.get_param(namespace + "/servo_controller")
        description = namespace + "/robot_description"
        if not rospy.has_param(description):
            description = "/robot_description"
        model = rospy.get_param(description)
    group = controller["gimbals"]
    joints = {joint.attrib["name"]: joint
              for joint in ET.fromstring(model).findall("joint")}
    configured = {}
    for key, entry in group.items():
        if "controller" not in key or not isinstance(entry, dict):
            continue
        index = int(entry["id"])
        if index in configured:
            raise ValueError("Duplicate gimbal index: {}".format(index))
        configured[index] = dict(group, **entry)
    missing = sorted(set(indices) - set(configured))
    if missing:
        raise ValueError("Indices {} are not configured gimbals; available: {}".format(
            missing, sorted(configured)))
    gimbals = [Gimbal(configured[index], joints[configured[index]["name"]])
               for index in indices]

    def topic(key, default):
        name = group.get(key, controller.get(key, default))
        return name if name.startswith("/") else namespace + "/" + name

    return (gimbals, topic("pos_pub_topic", "servo/target_states"),
            topic("state_sub_topic", "servo/states"), local_config)


def joint_profiles(gimbals):
    message = JointProfiles()
    for gimbal in gimbals:
        profile = JointProfile()
        profile.type = JointProfile.GIMBAL
        profile.servo_id = gimbal.index
        profile.angle_sgn = gimbal.sign
        profile.angle_scale = gimbal.scale
        profile.zero_point_offset = gimbal.offset
        profile.lower_limit = gimbal.lower
        profile.upper_limit = gimbal.upper
        profile.raw_lower_limit = gimbal.raw_lower
        profile.raw_upper_limit = gimbal.raw_upper
        message.joints.append(profile)
    return message


class Feedback:
    def __init__(self):
        self.lock = threading.Lock()
        self.states = {}

    def callback(self, message):
        now = time.monotonic()
        with self.lock:
            for servo in message.servos:
                self.states[servo.index] = (servo.angle, servo.error, now)

    def positions(self, gimbals, timeout):
        with self.lock:
            states = dict(self.states)
        angles = []
        for gimbal in gimbals:
            state = states.get(gimbal.index)
            if state is None or time.monotonic() - state[2] > timeout:
                raise RuntimeError("Missing/stale feedback for servo {}".format(gimbal.index))
            if state[1]:
                raise RuntimeError("Servo {} reports error 0x{:02x}".format(
                    gimbal.index, state[1]))
            angles.append(gimbal.angle(state[0]))
        return angles


def advance(current, goal, step):
    return [position + max(-step, min(step, target - position))
            for position, target in zip(current, goal)]


def sweep_phases(gimbals, cycles):
    upper = [g.upper for g in gimbals]
    lower = [g.lower for g in gimbals]
    for cycle in range(1, cycles + 1):
        prefix = "Cycle {}/{}: ".format(cycle, cycles)
        action = "Approaching" if cycle == 1 else "Returning to"
        yield prefix + action + " positive limits", upper
        yield prefix + "Sweeping to negative limits", lower


def run_sweep(args, gimbals, target_topic, state_topic, profile_topic=None):
    feedback = Feedback()
    subscriber = rospy.Subscriber(state_topic, ServoStates, feedback.callback, queue_size=1)
    publisher = rospy.Publisher(target_topic, ServoControlCmd, queue_size=1)
    profile_publisher = (rospy.Publisher(profile_topic, JointProfiles, queue_size=1, latch=True)
                         if profile_topic else None)
    try:
        deadline = time.monotonic() + args.wait_timeout
        while not rospy.is_shutdown():
            try:
                current = feedback.positions(gimbals, args.state_timeout)
                if not publisher.get_num_connections():
                    problem = "No subscriber on " + target_topic
                elif profile_publisher is not None and not profile_publisher.get_num_connections():
                    problem = "No subscriber on " + profile_topic
                else:
                    break
            except RuntimeError as error:
                problem = str(error)
            if time.monotonic() >= deadline:
                raise RuntimeError(problem)
            time.sleep(0.05)
        else:
            return

        for gimbal, angle in zip(gimbals, current):
            if not gimbal.lower - gimbal.scale <= angle <= gimbal.upper + gimbal.scale:
                raise RuntimeError("Servo {} starts outside its URDF limits ({:.2f} deg)".format(
                    gimbal.index, math.degrees(angle)))

        if profile_publisher is not None:
            rospy.loginfo("Sending selected gimbal calibration and limits on %s", profile_topic)
            profiles = joint_profiles(gimbals)
            for _ in range(3):
                if rospy.is_shutdown():
                    return
                feedback.positions(gimbals, args.state_timeout)
                profile_publisher.publish(profiles)
                time.sleep(0.1)

        def publish(angles):
            message = ServoControlCmd()
            message.index = [gimbal.index for gimbal in gimbals]
            message.angles = [gimbal.ticks(angle) for gimbal, angle in zip(gimbals, angles)]
            publisher.publish(message)

        period = 1.0 / args.rate
        speed = math.radians(args.speed_deg_s)
        for label, goal in sweep_phases(gimbals, args.cycles):
            duration = max(abs(target - start) for start, target in zip(current, goal)) / speed
            rospy.loginfo("%s at %.4f deg/s (about %.1f minutes)",
                          label, args.speed_deg_s, duration / 60.0)
            previous = time.monotonic()
            while not rospy.is_shutdown():
                feedback.positions(gimbals, args.state_timeout)
                if not publisher.get_num_connections():
                    raise RuntimeError("Lost command subscriber on " + target_topic)
                now = time.monotonic()
                # Never catch up with a large jump after a scheduling delay.
                step = speed * min(now - previous, period)
                previous = now
                current = advance(current, goal, step)
                publish(current)
                rospy.loginfo_throttle(10.0, "%s: commanded degrees %s", label,
                                       [round(math.degrees(a), 3) for a in current])
                time.sleep(period)
                if current == goal:
                    break
            if rospy.is_shutdown():
                return
        # Give the queued endpoint time to reach the transport before exiting.
        for _ in range(3):
            if rospy.is_shutdown():
                return
            publish(current)
            time.sleep(period)
        rospy.loginfo("Completed %d cycle(s); last command remains at the negative limits.",
                      args.cycles)
    finally:
        subscriber.unregister()
        publisher.unregister()
        if profile_publisher is not None:
            profile_publisher.unregister()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed-deg-s", type=positive_number, default=DEFAULT_SPEED_DEG_S,
                        help="approach and sweep speed in degrees/second (default: 0.1)")
    parser.add_argument("--cycles", type=positive_integer, default=DEFAULT_CYCLES,
                        help="number of positive-to-negative sweeps (default: 1); "
                             "return to positive between sweeps")
    parser.add_argument("--indices", type=int, nargs="+", choices=range(4),
                        default=DEFAULT_INDICES, help="servo indices to move (default: 0 1 2 3)")
    parser.add_argument("--namespace", default="/",
                        help="robot ROS namespace (default: / for spinal bridge.launch)")
    parser.add_argument("--target-topic", help="override the configured raw command topic")
    parser.add_argument("--state-topic", help="override the configured raw feedback topic")
    parser.add_argument("--profile-topic", help="override the bridge-only joint profile topic")
    parser.add_argument("--rate", type=positive_number, default=50.0,
                        help="command publishing frequency in Hz (default: 50)")
    parser.add_argument("--state-timeout", type=positive_number, default=2.0,
                        help="stop if a servo's feedback is older than this many seconds")
    parser.add_argument("--wait-timeout", type=positive_number, default=10.0,
                        help="startup connection/feedback timeout in seconds")
    parser.add_argument("--dry-run", action="store_true",
                        help="read and print configuration without publishing commands")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if len(set(args.indices)) != len(args.indices):
        parser.error("--indices must not contain duplicates")
    namespace = "/" + args.namespace.strip("/") if args.namespace.strip("/") else ""
    rospy.init_node("sweep_gimbal_servos", anonymous=True)
    try:
        gimbals, target_topic, state_topic, local_config = load_gimbals(namespace, args.indices)
        for gimbal in gimbals:
            if math.radians(args.speed_deg_s) > gimbal.velocity:
                raise ValueError("Requested speed exceeds the URDF velocity limit for " + gimbal.name)
            rospy.loginfo("Servo %d (%s): %+.4f -> %+.4f deg, encoder %d -> %d",
                          gimbal.index, gimbal.name, math.degrees(gimbal.upper),
                          math.degrees(gimbal.lower), gimbal.ticks(gimbal.upper),
                          gimbal.ticks(gimbal.lower))
        target_topic = args.target_topic or target_topic
        state_topic = args.state_topic or state_topic
        profile_topic = (args.profile_topic or namespace + "/joint_profiles") if local_config else None
        rospy.loginfo("Command: %s; feedback: %s; speed: %.4f deg/s; cycles: %d",
                      target_topic, state_topic, args.speed_deg_s, args.cycles)
        if profile_topic:
            rospy.loginfo("Joint profiles before motion: %s (selected gimbals only)", profile_topic)
        if not args.dry_run:
            rospy.logwarn("Bench sweep: keep the flight controller inactive to avoid competing commands.")
            run_sweep(args, gimbals, target_topic, state_topic, profile_topic)
    except (KeyError, ValueError, TypeError, OSError, ET.ParseError, RuntimeError, rospy.ROSException) as error:
        rospy.logerr("Sweep stopped: %s", error)
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
