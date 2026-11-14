#!/usr/bin/env python3
"""Refresh a signed main-body local +Y angular velocity after perching setup.

Required: ~angular_velocity_deg_s (finite numeric degrees/second, zero allowed).
Defaults: ~interval_sec=8.0, ~num_steps=8, ~robot_namespace=/gimbalrotor,
~debug_prefix=perching/roll_yaw. ~step_deg is rejected. num_steps counts requests.
The selected robot's navigation/roll_yaw_perching/body_pitch_velocity_command_duration
must match interval_sec. This script reads it once and never changes it.

Illustrative only, not a hardware-tuned recommendation:
  rosrun gimbalrotor auto_roll_yaw_body_pitch_experiment.py \
    _angular_velocity_deg_s:=0.1 _interval_sec:=8.0 _num_steps:=8

Publish the same velocity in rad/s once per interval after a one-second startup
sleep. Sleep after every request, including the eighth: default publication times
are approximately 0, 8, ..., 56 s relative to the first request. The last accepted
command expires about 64 s after the first, then brakes within navigator limits.
Signs follow the commanded main body's local +Y, including nonzero roll/yaw;
this is neither world Euler pitch nor passive-alpha velocity. The navigator uses
the coherent applied posture and coordinates all joints about the fixed contact.
Refreshes preserve orientation/rate; actual displacement integrates the bounded
applied rate, including ramps and holds. Logs are requests, not confirmed motion.
Ctrl+C stops future publications. The last request retains its remaining lifetime
and then brakes to hold. No enable, relock, retry, polling or cancellation is sent.
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

    interval_sec = finite_number(rospy.get_param('~interval_sec', 8.0), '~interval_sec')
    if rospy.has_param('~step_deg'):
        raise ValueError('~step_deg is an obsolete angle parameter; supply ~angular_velocity_deg_s explicitly')
    if not rospy.has_param('~angular_velocity_deg_s'):
        raise ValueError('~angular_velocity_deg_s is required (degrees/second; no default speed)')
    angular_velocity_deg_s = finite_number(rospy.get_param('~angular_velocity_deg_s'), '~angular_velocity_deg_s')
    num_steps = rospy.get_param('~num_steps', 8)
    if interval_sec <= 0:
        raise ValueError('~interval_sec must be strictly positive')
    if isinstance(num_steps, bool) or not isinstance(num_steps, int) or num_steps <= 0:
        raise ValueError('~num_steps must be a positive integer (not a boolean or fraction)')
    angular_velocity_rad_s = math.radians(angular_velocity_deg_s)
    if angular_velocity_deg_s != 0 and angular_velocity_rad_s == 0:
        raise ValueError('~angular_velocity_deg_s is too small to represent in radians/second')

    ns = rospy.get_param('~robot_namespace', '/gimbalrotor')
    prefix = rospy.get_param('~debug_prefix', 'perching/roll_yaw')
    if not isinstance(ns, str) or not isinstance(prefix, str) or not prefix.strip('/'):
        raise ValueError('namespace and debug_prefix must be strings; prefix must be nonempty')
    ns, prefix = ns.rstrip('/'), prefix.rstrip('/')
    topic_prefix = prefix if prefix.startswith('/') else ns + '/' + prefix
    duration_param = ns + '/navigation/roll_yaw_perching/body_pitch_velocity_command_duration'
    if not rospy.has_param(duration_param):
        raise ValueError('navigator command duration is missing: ' + duration_param)
    command_duration = finite_number(rospy.get_param(duration_param), duration_param)
    if command_duration <= 0 or not math.isclose(command_duration, interval_sec, rel_tol=1e-9, abs_tol=1e-9):
        raise ValueError('{} must be positive and match ~interval_sec ({} s); got {} s'.format(
            duration_param, interval_sec, command_duration))
    publisher = rospy.Publisher(topic_prefix + '/body_pitch_velocity', Float64, queue_size=1)

    rospy.loginfo('Publishing %d body-local pitch velocity requests on %s: %+.6g deg/s, interval %.3f s',
                  num_steps, topic_prefix + '/body_pitch_velocity', angular_velocity_deg_s, interval_sec)
    rospy.sleep(1.0)

    for request_number in range(1, num_steps + 1):
        if rospy.is_shutdown():
            break
        publisher.publish(Float64(data=angular_velocity_rad_s))
        rospy.loginfo('Published body-local pitch velocity request %d/%d: %+.6g deg/s (%+.6g rad/s)',
                      request_number, num_steps, angular_velocity_deg_s, angular_velocity_rad_s)
        rospy.sleep(interval_sec)

    rospy.loginfo('Request publishing finished; the last accepted command expires onboard, then brakes to hold.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except ValueError as exc:
        rospy.logerr('%s', exc)
        raise SystemExit(1)
