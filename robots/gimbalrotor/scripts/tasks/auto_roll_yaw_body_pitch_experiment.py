#!/usr/bin/env python3
"""Stream signed main-body local-Y velocity for num_steps * interval_sec.

Required ~angular_velocity_deg_s is degrees/second (published as rad/s).
~interval_sec=8 and ~num_steps=8 define a 64-second experiment, independently
of keepalives. ~refresh_period_sec defaults to one quarter of the onboard lease
and must be strictly shorter than it. No prepare/tare/enable is performed.
At the normal experiment deadline send zero once for bounded braking. Ctrl+C
ends refreshes; the last accepted command expires onboard. Rejected requests do
not renew the onboard lease. Logs describe requests, not confirmed movement.
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
    if command_duration <= 0:
        raise ValueError(duration_param + ' must be positive')
    refresh_period = finite_number(rospy.get_param('~refresh_period_sec', command_duration / 4),
                                   '~refresh_period_sec')
    if not 0 < refresh_period < command_duration:
        raise ValueError('~refresh_period_sec must be positive and shorter than the navigator lease')
    experiment_duration = interval_sec * num_steps
    if not math.isfinite(experiment_duration):
        raise ValueError('experiment duration must be finite')
    publisher = rospy.Publisher(topic_prefix + '/body_pitch_velocity', Float64, queue_size=1)

    rospy.loginfo('Requesting body-local pitch velocity on %s: %+.6g deg/s for %.3f s; refresh %.3f s, lease %.3f s',
                  topic_prefix + '/body_pitch_velocity', angular_velocity_deg_s, experiment_duration,
                  refresh_period, command_duration)
    rospy.sleep(1.0)
    started = rospy.get_time()
    deadline = started + experiment_duration
    previous = started
    messages = 0
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if now < previous:
            publisher.publish(Float64(data=0.0))
            raise ValueError('ROS time moved backwards; stop requested, experiment abandoned')
        if now >= deadline:
            break
        publisher.publish(Float64(data=angular_velocity_rad_s))
        messages += 1
        previous = now
        # Schedule from the current time: no burst of missed keepalives.
        rospy.sleep(min(refresh_period, deadline - now))
    if not rospy.is_shutdown():
        publisher.publish(Float64(data=0.0))
        rospy.loginfo('Experiment duration finished after %d keepalives; zero requests bounded braking.', messages)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except ValueError as exc:
        rospy.logerr('%s', exc)
        raise SystemExit(1)
