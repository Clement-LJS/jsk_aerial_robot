#!/usr/bin/env python3
"""Request cumulative absolute body-local pitch targets, then wait interval_sec.

body_pitch_delta is an absolute target within a persistent command frame.
Requests do not wait for completion; motion uses navigation/perching/pitch_rate.
Normal completion leaves the final destination held; interruption requests braking.
Optional use_admittance performs unloaded tare/enable once before the first step.
"""

import math
import signal
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool, Empty, Float64


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


class Feedback:
    def __init__(self, prefix):
        self.lock = threading.Lock()
        self.status = None
        self.subscriber = rospy.Subscriber(prefix + '/diagnostics_status', DiagnosticArray, self.diagnostic, queue_size=1)

    def diagnostic(self, message):
        for status in message.status:
            if status.name.endswith('/roll_yaw_perching'):
                values = {v.key: v.value for v in status.values}
                with self.lock:
                    self.status = (time.monotonic(), status.message, values)

    def snapshot(self):
        with self.lock:
            return self.status


class AdmittanceStartup:
    """Collect while disarmed, then verify arming, as in no-joint perching.

    Retry only a processed refusal. Missing acknowledgements must not produce
    duplicate tare requests that could later erase an accepted calibration.
    """
    def __init__(self):
        self.started = self.generation = None
        self.pending = None
        self.retry_at = 0.
        self.baseline_seen = False

    def ready(self, now, values, tare, enable):
        generation = int(values['tare_generation'])
        if self.started is None:
            self.started = now
        # The controller validates contact/model identity; startup pins the tare.
        if values.get('admittance_configured') != '1':
            raise ValueError('Admittance configuration unavailable: ' + values.get('admittance_status', ''))
        if self.pending:
            action, sequence, requested, previous_generation = self.pending
            processed = int(values[action + '_request_sequence'])
            if processed == sequence:
                if now - requested > 2.:
                    raise ValueError(action + ' acknowledgement missing; request is not repeated')
                return False
            if processed != sequence + 1:
                raise ValueError('Admittance startup interrupted by another request')
            self.pending = None
            self.retry_at = now + .5
            if action == 'tare' and generation != previous_generation:
                if generation != previous_generation + 1:
                    raise ValueError('Tare changed unexpectedly during startup')
                self.generation = generation
        if self.generation is not None and generation != self.generation:
            raise ValueError('Admittance baseline changed; task abandoned')
        baseline = values.get('baseline_ready') == '1'
        if self.baseline_seen and not baseline:
            raise ValueError('Admittance baseline lost; automatic re-tare is forbidden')
        self.baseline_seen |= baseline
        if values.get('admittance_enabled') == '1':
            if not baseline:
                raise ValueError('Armed admittance has no baseline; disarm and tare unloaded')
            self.generation = generation
            return values.get('admittance_hold_eligible') == '1'
        if now - self.started > 30.:
            raise ValueError('No verified admittance baseline/enable within 30 seconds: ' + values.get('admittance_status', ''))
        collecting = values.get('tare_collecting') == '1'
        if baseline or collecting:
            self.generation = generation
        # Controller admission uses nominal hold/contact/tracking, independent of
        # preparation quietness. Fresh diagnostics are required by the caller.
        if values.get('admittance_hold_eligible') != '1' or now < self.retry_at:
            return False
        if baseline:
            if not enable.get_num_connections():
                return False
            action, publisher, message = 'enable', enable, Bool(data=True)
        else:
            if self.generation is not None or collecting:
                return False  # An accepted collection must never be restarted by retry.
            if not tare.get_num_connections():
                return False
            action, publisher, message = 'tare', tare, Empty()
        self.pending = (action, int(values[action + '_request_sequence']), now, generation)
        publisher.publish(message)
        return False


def main():
    rospy.init_node('auto_roll_yaw_body_pitch_experiment', disable_signals=True)
    if rospy.has_param('~body_pitch_rate_deg_s'):
        raise ValueError('~body_pitch_rate_deg_s is unsupported by direct body_pitch_delta commands; '
                         'the rate is set by navigation/perching/pitch_rate')
    if any(rospy.has_param(key) for key in ('~angular_velocity_deg_s', '~feed_duration_sec')):
        raise ValueError('Pulse parameters removed: use ~step_deg and ~interval_sec for step and hold')
    step = math.radians(finite_number(rospy.get_param('~step_deg', None), '~step_deg'))
    limit = math.radians(finite_number(rospy.get_param('~body_pitch_limit_deg', 20.), '~body_pitch_limit_deg'))
    hold = finite_number(rospy.get_param('~interval_sec', 8.), '~interval_sec')
    if rospy.has_param('~hold_duration_sec'):
        alias = finite_number(rospy.get_param('~hold_duration_sec'), '~hold_duration_sec')
        if rospy.has_param('~interval_sec') and alias != hold:
            raise ValueError('~hold_duration_sec and ~interval_sec conflict; specify one hold duration')
        hold = alias
    count = rospy.get_param('~num_steps', 8)
    if step <= 0 or not 0 < limit < math.pi or hold <= 0:
        raise ValueError('Require positive step_deg and interval_sec, and 0 < body_pitch_limit_deg < 180')
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError('~num_steps must be a positive integer')
    try:
        total = finite_number(step * count, 'Total requested angle')
    except OverflowError as exc:
        raise ValueError('Total requested travel must be finite') from exc
    if total > limit:
        raise ValueError('Requested total %+.6g deg exceeds body-pitch command limit %.6g deg' %
                         (math.degrees(total), math.degrees(limit)))
    ns = rospy.get_param('~robot_namespace', '/gimbalrotor').rstrip('/')
    prefix = rospy.get_param('~debug_prefix', 'perching/roll_yaw').rstrip('/')
    prefix = prefix if prefix.startswith('/') else ns + '/' + prefix
    feedback_timeout = finite_number(rospy.get_param('~feedback_timeout_sec', 1.), '~feedback_timeout_sec')
    timeout = finite_number(rospy.get_param('~timeout_sec', 30.), '~timeout_sec')
    if min(feedback_timeout, timeout) <= 0:
        raise ValueError('Startup timeouts must be positive')
    use_admittance = rospy.get_param('~use_admittance', False)
    if not isinstance(use_admittance, bool):
        raise ValueError('~use_admittance must be a boolean')
    motion = rospy.Publisher(prefix + '/body_pitch_delta', Float64, queue_size=1)
    stop = rospy.Publisher(prefix + '/body_pitch_velocity', Float64, queue_size=1)
    feedback = Feedback(prefix)
    tare = rospy.Publisher(prefix + '/admittance_tare', Empty, queue_size=1)
    enable = rospy.Publisher(ns + '/perching/admittance_enable', Bool, queue_size=1)
    interrupted = threading.Event()
    handlers = {number: signal.signal(number, lambda *_: interrupted.set())
                for number in (signal.SIGINT, signal.SIGTERM)}
    started = time.monotonic()
    normal_exit = False
    startup = AdmittanceStartup()
    startup_complete = not use_admittance
    rospy.loginfo('Requesting %d cumulative absolute body-local pitch targets in %+.6g deg steps, with %.3g s holds; '
                  'navigator pitch_rate applies within a persistent command frame; no completion wait.',
                  count, math.degrees(step), hold)
    try:
        initial_target = None
        while not rospy.is_shutdown() and not interrupted.is_set():
            now = time.monotonic()
            if now - started > timeout:
                raise ValueError('Timed out waiting for perched feedback, command subscribers or unloaded admittance startup')
            status = feedback.snapshot()
            if status:
                received, state, values = status
                if 0 <= now - received <= feedback_timeout and state.startswith('CONTACT_LOCKED') and int(values.get('session_id', '0')) > 0:
                    if not startup_complete:
                        startup_complete = startup.ready(now, values, tare, enable)
                    if startup_complete and motion.get_num_connections() and stop.get_num_connections():
                        initial_target = float(values.get('body_pitch_target_rad', '0')) if values.get('body_pitch_target_valid') == '1' else 0.0
                        if not math.isfinite(initial_target) or abs(initial_target + total) > limit:
                            raise ValueError('Requested cumulative body pitch exceeds the configured script limit')
                        break
            interrupted.wait(.02)

        for number in range(1, count + 1):
            if rospy.is_shutdown() or interrupted.is_set():
                return
            target = initial_target + step * number
            for repeat in range(5):
                if rospy.is_shutdown() or interrupted.is_set():
                    return
                motion.publish(Float64(data=target))
                if repeat < 4:
                    interrupted.wait(.02)
            rospy.loginfo('Step %d/%d: requested absolute body-local pitch target %+.6g deg; hold %.3g s.',
                          number, count, math.degrees(target), hold)
            deadline = time.monotonic() + hold
            while not rospy.is_shutdown() and not interrupted.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                interrupted.wait(min(.1, remaining))
        if rospy.is_shutdown() or interrupted.is_set():
            return
        normal_exit = True
        rospy.loginfo('Finished %d step requests and holds; leaving the nominal destination with the navigator.', count)
    finally:
        try:
            if not normal_exit and not rospy.is_shutdown():
                stop.publish(Float64(data=0.))
                rospy.loginfo('Requested brake/hold with one zero body_pitch_velocity message.')
        finally:
            for number, handler in handlers.items():
                signal.signal(number, handler)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except (ValueError, RuntimeError) as exc:
        rospy.logerr('%s', exc)
        raise SystemExit(1)
