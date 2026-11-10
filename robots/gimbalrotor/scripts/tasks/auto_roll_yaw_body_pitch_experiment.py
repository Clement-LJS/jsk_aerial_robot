#!/usr/bin/env python3
"""Publish periodic relative main-body local pitch requests after perching setup.

Private parameters (read once): interval_sec=15.0, step_deg=0.8, num_steps=15,
robot_namespace=/gimbalrotor, debug_prefix=perching/roll_yaw. Positive pitch is
down, negative is up. Each request is the same signed increment in radians.

From the repository root, after the usual perching setup and positioning:
  python3 robots/gimbalrotor/scripts/tasks/auto_roll_yaw_body_pitch_experiment.py \
    _interval_sec:=15.0 _step_deg:=0.8 _num_steps:=15
  python3 robots/gimbalrotor/scripts/tasks/auto_roll_yaw_body_pitch_experiment.py \
    _interval_sec:=5.0 _step_deg:=0.5 _num_steps:=20

The first request follows a one-second startup delay. Sleep interval_sec after
each publication, including the last. num_steps counts requests published, not
verified movements. The navigator enforces its per-jog limit (at most one degree,
possibly configured smaller) and may reject requests, including while busy.
This publisher does not wait for completion, retry, or adjust its schedule.
Ctrl+C stops future publications; an accepted jog remains the navigator's
responsibility. This script sends no cancellation.
"""

import math

import rospy
from std_msgs.msg import Float64


def finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + ' must be a finite number')
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(name + ' must be a finite number') from exc
    if not math.isfinite(number):
        raise ValueError(name + ' must be a finite number')
    return number


def main():
    rospy.init_node('auto_roll_yaw_body_pitch_experiment')

    interval_sec = finite_number(rospy.get_param('~interval_sec', 15.0), '~interval_sec')
    step_deg = finite_number(rospy.get_param('~step_deg', 0.8), '~step_deg')
    num_steps = rospy.get_param('~num_steps', 15)
    if interval_sec <= 0:
        raise ValueError('~interval_sec must be strictly positive')
    if step_deg == 0:
        raise ValueError('~step_deg must be nonzero')
    if isinstance(num_steps, bool) or not isinstance(num_steps, int) or num_steps <= 0:
        raise ValueError('~num_steps must be a positive integer (not a boolean or fraction)')
    step_rad = math.radians(step_deg)
    if step_rad == 0:
        raise ValueError('~step_deg is too small to represent in radians')

    ns = rospy.get_param('~robot_namespace', '/gimbalrotor')
    prefix = rospy.get_param('~debug_prefix', 'perching/roll_yaw')
    if not isinstance(ns, str) or not isinstance(prefix, str) or not prefix.strip('/'):
        raise ValueError('namespace and debug_prefix must be strings; prefix must be nonempty')
    ns, prefix = ns.rstrip('/'), prefix.rstrip('/')
    topic_prefix = prefix if prefix.startswith('/') else ns + '/' + prefix
    publisher = rospy.Publisher(topic_prefix + '/body_pitch_delta', Float64, queue_size=1)

    rospy.loginfo('Publishing %d local-body pitch requests on %s: %+.3f deg, interval %.3f s',
                  num_steps, topic_prefix + '/body_pitch_delta', step_deg, interval_sec)
    rospy.sleep(1.0)

    for request_number in range(1, num_steps + 1):
        if rospy.is_shutdown():
            break
        publisher.publish(Float64(data=step_rad))
        rospy.loginfo('Published local-body pitch request %d/%d: %+.3f deg',
                      request_number, num_steps, step_deg)
        rospy.sleep(interval_sec)

    rospy.loginfo('Request publishing finished.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except ValueError as exc:
        rospy.logerr('%s', exc)
        raise SystemExit(1)
