#!/usr/bin/env python3
from __future__ import print_function  # safe for Python2/3
import sys
import select
import termios
import tty
import rospy
from std_msgs.msg import Bool

try:
    from spinal.msg import PwmTest
except Exception:
    rospy.logerr("Could not import spinal.msg.PwmTest")
    raise

def getKey(settings, timeout):
    """
    Read a single keypress with a timeout (seconds).
    Returns '' if no key was pressed within timeout.
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def publish_pwm_and_flag(pwm_pub, cut_pub, motor_index, value, threshold):
    """
    Build and publish PwmTest and Bool messages.
    This function publishes every time it's called (used for continuous publishing).
    """
    msg = PwmTest()
    msg.motor_index = [int(motor_index)]
    msg.pwms = [float(value)]
    pwm_pub.publish(msg)

    cut = Bool()
    cut.data = (value > threshold)
    cut_pub.publish(cut)
    return cut.data

def main():
    # Save terminal state to restore on exit
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('saw_control', anonymous=False)

    # IMPORTANT: use the actual topic that the saw listens to
    pwm_topic = '/gimbalrotor/pwm_test'
    isCutting_topic = '/isCutting'

    pwm_pub = rospy.Publisher(pwm_topic, PwmTest, queue_size=1)
    cut_pub = rospy.Publisher(isCutting_topic, Bool, queue_size=1)

    # Parameters (can override with rosparam)
    motor_index = rospy.get_param('~motor_index', 4)
    value = float(rospy.get_param('~initial_value', 0.5))
    step = float(rospy.get_param('~step', 0.1))
    vmin = float(rospy.get_param('~min', 0.5))
    vmax = float(rospy.get_param('~max', 0.9))
    threshold = float(rospy.get_param('~threshold', 0.5))
    publish_rate = float(rospy.get_param('~publish_rate', 10.0))  # Hz

    rospy.loginfo("Starting saw_control node")
    rospy.loginfo("Publishing to: %s  (iscutting -> %s)", pwm_topic, isCutting_topic)
    rospy.loginfo("Initial PWM=%.3f (min=%.3f max=%.3f) motor_index=%d step=%.3f publish_rate=%.1fHz",
                  value, vmin, vmax, motor_index, step, publish_rate)
    rospy.loginfo("Controls: 'i' increase, 'k' decrease, 'q' or Ctrl-C quit")

    # Publish initial state once immediately (so subscribers see initial value)
    try:
        initial_cutting = publish_pwm_and_flag(pwm_pub, cut_pub, motor_index, value, threshold)
        print("Initial -> PWM={:.3f} cutting={}".format(value, initial_cutting))
    except Exception as e:
        rospy.logwarn("Failed to publish initial state: %s", str(e))

    # State variables
    current_value = value
    current_cutting = (current_value > threshold)

    rate = rospy.Rate(publish_rate)
    dt = 1.0 / publish_rate  # timeout for key read; keeps getKey in sync with publish rate

    try:
        while not rospy.is_shutdown():
            # Check keyboard (wait up to dt seconds). This is non-blocking beyond dt.
            key = getKey(settings, dt)

            printed = False  # whether we printed status this loop (only True when i/k pressed)

            if key:
                k = key.lower()
                if k == 'i':
                    new_value = current_value + step
                    if new_value > vmax:
                        rospy.logwarn("Cannot increase: would exceed max %.3f (current %.3f)", vmax, current_value)
                    else:
                        current_value = round(new_value, 6)
                        current_cutting = (current_value > threshold)
                        print("Increased -> PWM={:.3f} cutting={}".format(current_value, current_cutting))
                        printed = True

                elif k == 'k':
                    new_value = current_value - step
                    if new_value < vmin:
                        rospy.logwarn("Cannot decrease: would go below min %.3f (current %.3f)", vmin, current_value)
                    else:
                        current_value = round(new_value, 6)
                        current_cutting = (current_value > threshold)
                        print("Decreased -> PWM={:.3f} cutting={}".format(current_value, current_cutting))
                        printed = True

                elif k in ['q', '\x03']:
                    rospy.loginfo("Quit requested by user.")
                    break
                else:
                    # ignore other keys silently (no terminal spam)
                    pass

            # Continuously publish the current pwm and iscutting flag at the configured rate
            try:
                publish_pwm_and_flag(pwm_pub, cut_pub, motor_index, current_value, threshold)
            except Exception as e:
                rospy.logwarn_throttle(5, "Publish failed: %s", str(e))  # throttle to avoid log spam

            # If you want a periodic status print while running (uncomment):
            # if not printed:
            #     print("\rPWM={:.3f} cutting={} (press i/k to change)".format(current_value, current_cutting), end='')

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Exception in saw_control: %s", str(e))
    finally:
        # On exit publish safe value and restore terminal
        try:
            safe_value = vmin
            publish_pwm_and_flag(pwm_pub, cut_pub, motor_index, safe_value, threshold)
            print("\nExiting. Set PWM to safe value {:.3f}".format(safe_value))
        except Exception as e:
            rospy.logwarn("Failed to publish safe shutdown pwm: %s", str(e))
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()

