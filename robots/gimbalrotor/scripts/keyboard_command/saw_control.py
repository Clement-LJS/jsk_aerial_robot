#!/usr/bin/env python3

from __future__ import print_function

import math
import select
import sys
import termios
import tty

import rospy
from std_msgs.msg import Bool

try:
    from spinal.msg import PwmTest
except Exception:
    rospy.logerr("Could not import spinal.msg.PwmTest")
    raise


abort_requested = False


def abort_callback(msg):
    global abort_requested

    if msg.data and not abort_requested:
        abort_requested = True

        rospy.logerr(
            "Hybrid controller abort received. "
            "Saw abort is now latched."
        )


def get_key(settings, timeout):
    key = ""

    tty.setraw(sys.stdin.fileno())

    try:
        readable, _, _ = select.select(
            [sys.stdin],
            [],
            [],
            timeout,
        )

        if readable:
            key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings,
        )

    return key


def publish_pwm_and_flags(
    pwm_pub,
    cut_pub,
    cutting_active_pub,
    motor_index,
    value,
    threshold,
    cutting_phase_requested,
):
    pwm_msg = PwmTest()
    pwm_msg.motor_index = [int(motor_index)]
    pwm_msg.pwms = [float(value)]
    pwm_pub.publish(pwm_msg)

    saw_running = value > threshold

    cutting_msg = Bool()
    cutting_msg.data = saw_running
    cut_pub.publish(cutting_msg)

    cutting_active_msg = Bool()
    cutting_active_msg.data = (
        saw_running and
        cutting_phase_requested
    )
    cutting_active_pub.publish(
        cutting_active_msg
    )

    return (
        saw_running,
        cutting_active_msg.data,
    )


def validate_parameters(
    initial_value,
    step,
    vmin,
    vmax,
    threshold,
    publish_rate,
    safe_pwm,
):
    values = {
        "initial_value": initial_value,
        "step": step,
        "min": vmin,
        "max": vmax,
        "threshold": threshold,
        "publish_rate": publish_rate,
        "safe_pwm": safe_pwm,
    }

    for name, value in values.items():
        if not math.isfinite(value):
            rospy.logfatal(
                "~%s must be finite; got %r",
                name,
                value,
            )
            return False

    if vmin > vmax:
        rospy.logfatal(
            "~min %.6f must not exceed ~max %.6f",
            vmin,
            vmax,
        )
        return False

    if initial_value < vmin or initial_value > vmax:
        rospy.logfatal(
            "~initial_value %.6f must be within "
            "[%.6f, %.6f]",
            initial_value,
            vmin,
            vmax,
        )
        return False

    if step <= 0.0:
        rospy.logfatal(
            "~step must be greater than zero"
        )
        return False

    if publish_rate <= 0.0:
        rospy.logfatal(
            "~publish_rate must be greater than zero"
        )
        return False

    if threshold < vmin or threshold > vmax:
        rospy.logfatal(
            "~threshold %.6f must be within "
            "[%.6f, %.6f]",
            threshold,
            vmin,
            vmax,
        )
        return False

    if safe_pwm < vmin or safe_pwm > vmax:
        rospy.logfatal(
            "~safe_pwm %.6f must be within "
            "[%.6f, %.6f]",
            safe_pwm,
            vmin,
            vmax,
        )
        return False

    if safe_pwm > threshold:
        rospy.logfatal(
            "~safe_pwm %.6f must be <= "
            "~threshold %.6f",
            safe_pwm,
            threshold,
        )
        return False

    return True


def main():
    global abort_requested

    rospy.init_node("saw_control", anonymous=False,)

    if not sys.stdin.isatty():
        rospy.logfatal(
            "saw_control requires an interactive terminal. "
            "Run it from a terminal because keyboard input is used "
            "to control the saw."
        )
        return

    try:
        settings = termios.tcgetattr(sys.stdin)
    except (termios.error, OSError) as exc:
        rospy.logfatal("Failed to configure terminal input: %s", str(exc),)
        return

    pwm_topic = rospy.get_param(
        "~pwm_topic",
        "/gimbalrotor/pwm_test",
    )

    is_cutting_topic = rospy.get_param(
        "~is_cutting_topic",
        "/isCutting",
    )

    cutting_active_topic = rospy.get_param(
        "~cutting_active_topic",
        "/gimbalrotor/perching/cutting_active",
    )

    abort_topic = rospy.get_param(
        "~abort_topic",
        "/gimbalrotor/perching/hybrid/abort_request",
    )

    motor_index = int(
        rospy.get_param(
            "~motor_index",
            4,
        )
    )

    step = float(
        rospy.get_param(
            "~step",
            0.1,
        )
    )

    vmin = float(
        rospy.get_param(
            "~min",
            0.5,
        )
    )

    vmax = float(
        rospy.get_param(
            "~max",
            0.9,
        )
    )

    threshold = float(
        rospy.get_param(
            "~threshold",
            0.5,
        )
    )

    publish_rate = float(
        rospy.get_param(
            "~publish_rate",
            10.0,
        )
    )

    if not rospy.has_param(
        "~safe_pwm"
    ):
        rospy.logfatal(
            "~safe_pwm must be explicitly configured "
            "to a verified saw-off PWM value"
        )
        return

    safe_pwm = float(
        rospy.get_param(
            "~safe_pwm"
        )
    )

    if not validate_parameters(
        initial_value,
        step,
        vmin,
        vmax,
        threshold,
        publish_rate,
        safe_pwm,
    ):
        return

    pwm_pub = rospy.Publisher(
        pwm_topic,
        PwmTest,
        queue_size=1,
    )

    cut_pub = rospy.Publisher(
        is_cutting_topic,
        Bool,
        queue_size=1,
    )

    cutting_active_pub = rospy.Publisher(
        cutting_active_topic,
        Bool,
        queue_size=1,
    )

    abort_sub = rospy.Subscriber(
        abort_topic,
        Bool,
        abort_callback,
        queue_size=1,
    )

    # Keep the subscriber object alive for the complete lifetime
    # of main(). Do not delete or unregister this handle.

    current_value = safe_pwm
    cutting_phase_requested = False

    rospy.loginfo(
        "Starting saw_control"
    )

    rospy.loginfo(
        "PWM topic: %s",
        pwm_topic,
    )

    rospy.loginfo(
        "Cutting-active topic: %s",
        cutting_active_topic,
    )

    rospy.loginfo(
        "Abort topic: %s",
        abort_topic,
    )

    rospy.loginfo(
        "Verified safe PWM: %.6f",
        safe_pwm,
    )

    try:
        publish_pwm_and_flags(
            pwm_pub,
            cut_pub,
            cutting_active_pub,
            motor_index,
            current_value,
            threshold,
            cutting_phase_requested,
        )

        rate = rospy.Rate(
            publish_rate
        )

        key_timeout = (
            1.0 / publish_rate
        )

        while not rospy.is_shutdown():
            if abort_requested:
                cutting_phase_requested = False
                current_value = safe_pwm

            key = get_key(
                settings,
                key_timeout,
            )

            if key:
                key_lower = key.lower()

                if key_lower in [
                    "q",
                    "\x03",
                ]:
                    rospy.loginfo(
                        "Quit requested"
                    )
                    break

                if abort_requested:
                    rospy.logwarn_throttle(
                        1.0,
                        "Controller abort is latched. "
                        "Only quit is accepted.",
                    )
                elif key_lower == "i":
                    candidate = (
                        current_value +
                        step
                    )

                    if candidate > vmax:
                        rospy.logwarn(
                            "Cannot exceed max %.6f",
                            vmax,
                        )
                    else:
                        current_value = round(
                            candidate,
                            6,
                        )

                elif key_lower == "k":
                    candidate = (
                        current_value -
                        step
                    )

                    if candidate < vmin:
                        rospy.logwarn(
                            "Cannot go below min %.6f",
                            vmin,
                        )
                    else:
                        current_value = round(
                            candidate,
                            6,
                        )

                elif key_lower == "c":
                    cutting_phase_requested = (
                        not
                        cutting_phase_requested
                    )


            if current_value <= threshold:
                cutting_phase_requested = False

            # Recheck immediately before publishing.
            # The abort callback may have executed while get_key()
            # was waiting for terminal input.
            if abort_requested:
                cutting_phase_requested = False
                current_value = safe_pwm

            publish_pwm_and_flags(
                pwm_pub,
                cut_pub,
                cutting_active_pub,
                motor_index,
                current_value,
                threshold,
                cutting_phase_requested,
            )

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    except Exception as exc:
        rospy.logerr(
            "saw_control exception: %s",
            str(exc),
        )

    finally:
        try:
            publish_pwm_and_flags(
                pwm_pub,
                cut_pub,
                cutting_active_pub,
                motor_index,
                safe_pwm,
                threshold,
                False,
            )

            rospy.logwarn(
                "Saw shutdown PWM published: %.6f",
                safe_pwm,
            )

        except Exception as exc:
            rospy.logerr(
                "Failed to publish safe shutdown PWM: %s",
                str(exc),
            )

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            settings,
        )


if __name__ == "__main__":
    main()