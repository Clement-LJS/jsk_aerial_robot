"""Offline ROS message/launch checks; no node or ROS master is started."""
import importlib.util
import io
import math
import signal
from pathlib import Path
import struct
import threading
import unittest
from unittest.mock import patch, MagicMock
import time

from replay_perching_bags import Stroke

import roslaunch.config
import roslaunch.xmlloader

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('roll_yaw_command', ROOT / 'scripts/keyboard_command/roll_yaw_command.py')
command = importlib.util.module_from_spec(spec)
spec.loader.exec_module(command)


keyboard_spec = importlib.util.spec_from_file_location('roll_yaw_keyboard', ROOT / 'scripts/keyboard_command/keyboard_roll_yaw_perching.py')
keyboard = importlib.util.module_from_spec(keyboard_spec)
keyboard_spec.loader.exec_module(keyboard)

experiment_spec = importlib.util.spec_from_file_location('roll_yaw_experiment', ROOT / 'scripts/tasks/auto_roll_yaw_body_pitch_experiment.py')
experiment = importlib.util.module_from_spec(experiment_spec)
experiment_spec.loader.exec_module(experiment)


def simulated_motion(clock, publisher, consume):
    """Actual client protocol with an in-memory receiver, no ROS master."""
    def create(prefix):
        motion = keyboard.MotionClient(prefix)
        motion.epoch_callback(keyboard.UInt64(1))

        def receive(message):
            consume(message)
            epoch = message.epoch + (message.mode in (keyboard.RollYawMotion.CLAIM, keyboard.RollYawMotion.RELEASE,
                                                      keyboard.RollYawMotion.STOP))
            motion.ack_callback(keyboard.String(
                'action=motion session={} epoch={} source={} sequence={} mode={} accepted=true motion_epoch={} accepted_value={}'.format(
                    message.session, message.epoch, message.source, message.sequence, message.mode, epoch, message.value)))
            motion.epoch_callback(keyboard.UInt64(epoch))
            if message.mode == keyboard.RollYawMotion.STOP:
                motion.epoch_callback(keyboard.UInt64(message.epoch + 1))
                motion.status = (clock[0], {'motion_epoch': str(message.epoch + 1),
                                          'stopping': '0', 'body_pitch_velocity_active': '0'})
        publisher.publish.side_effect = receive
        return motion
    return create


class VelocityExperiment(unittest.TestCase):
    @staticmethod
    def pitch(degrees):
        angle = math.radians(degrees)
        return (0., math.sin(angle / 2), 0., math.cos(angle / 2))

    def make_task(self, rate=.1):
        return Stroke(math.radians(rate), 24., math.radians(.5), 30.)

    def step(self, task, now, reference=0., measured=0., **kwargs):
        values = dict(session=123, state='CONTACT_LOCKED: test',
                      reference=self.pitch(reference) if isinstance(reference, (float, int)) else reference,
                      measured=self.pitch(measured) if isinstance(measured, (float, int)) else measured,
                      fresh=True, holding=False, stopping=False, horizon=1.)
        values.update(kwargs)
        return task.step(now, **values)

    def test_timer_and_accepted_command_are_not_completed_motion(self):
        task = self.make_task()
        self.assertGreater(self.step(task, 0), 0)
        # Run 3 accepted velocity while its reference stayed essentially still.
        for now in range(1, 25):
            self.assertGreater(self.step(task, now, .001, .415), 0)
            self.assertFalse(task.complete)
        with self.assertRaisesRegex(ValueError, 'no measured progress'):
            self.step(task, 56, .001, .415)

    def test_outage_retains_goal_and_resumes_without_perching_reset(self):
        task = self.make_task()
        self.step(task, 0)
        self.step(task, 5, .5, .45)
        for now in (5.1, 5.5, 6.61):  # Run 1's 1.51-second processed-feedback gap.
            self.assertEqual(self.step(task, now, .5, .45, fresh=False, holding=True), 0)
        self.assertGreater(self.step(task, 6.7, .5, .45), 0)
        self.assertEqual(task.session, 123)
        self.assertAlmostEqual(task.goal, math.radians(2.4))
        self.step(task, 28, 2.4, 2.4, stopping=True)
        self.assertFalse(task.complete)
        self.step(task, 28.2, 2.4, 2.4)
        self.step(task, 28.6, 2.4, 2.4)
        self.assertTrue(task.complete)

    def test_reference_completion_requires_measured_endpoint_and_advancing_samples(self):
        task = self.make_task()
        self.step(task, 0)
        self.step(task, 24, 2.4, 1.)
        self.assertFalse(task.complete)
        self.step(task, 25, 2.4, 2.4, sample_stamp=25)
        self.step(task, 25.5, 2.4, 2.4, sample_stamp=25)
        self.assertFalse(task.complete)
        self.step(task, 25.6, 2.4, 2.4, sample_stamp=25.6)
        self.assertTrue(task.complete)
        # A delayed acknowledgement must not certify an endpoint after feedback is lost.
        self.step(task, 26, 2.4, 2.4, fresh=False)
        self.assertFalse(task.complete)

    def test_fault_and_session_change_cannot_be_automatically_recaptured(self):
        for changes in ({'session': 124}, {'state': 'FAULT_HOLD: fixed model mismatch'},
                        {'state': 'FREE_FLIGHT: disabled'}):
            task = self.make_task()
            self.step(task, 0)
            with self.assertRaises(ValueError):
                self.step(task, 1, **changes)
            self.assertFalse(task.complete)

    def test_negative_stroke_uses_local_rotation_at_nonzero_attitude(self):
        from tf.transformations import quaternion_from_euler, quaternion_multiply
        start = quaternion_from_euler(.2, -.1, .5)
        task = self.make_task(-.1)
        self.assertLess(self.step(task, 0, reference=start, measured=start), 0)
        endpoint = quaternion_multiply(start, self.pitch(-2.4))
        self.step(task, 24, reference=endpoint, measured=endpoint)
        self.step(task, 24.5, reference=endpoint, measured=endpoint)
        self.assertTrue(task.complete)
        self.assertAlmostEqual(task.measured_progress, math.radians(-2.4))

    def test_small_stroke_does_not_finish_with_stationary_measurement(self):
        task = Stroke(math.radians(.1), 1., math.radians(.5), 30.)
        self.step(task, 0)
        self.step(task, 5, .1, 0)
        self.step(task, 6, .1, 0)
        self.assertFalse(task.complete)

    def run_angles(self, interrupt_step=0, fault=False, blocked=False, admittance=False, yielding=False,
                   refuse_tare=False, refuse_enable=False, baseline=False, vibrating=False, step=.3,
                   interrupt_phase='movement', stale_status=False, stale_ack=False, stale_pose=False,
                   not_ready_until=0., new_owner=False):
        from contextlib import ExitStack
        clock, messages, clients = [0.], [], []
        plant = dict(angle=0., nominal=0., target=0., rate=0., sequence=0, steps=0, sent_at=0., tare_at=None,
                     enabled=False, tare_requests=0, enable_requests=0, released=False, completed_at=[],
                     first_sent=None, completed_angles=[])
        publisher, tare, enable = MagicMock(), MagicMock(), MagicMock()
        for pub in (publisher, tare, enable):
            pub.get_num_connections.return_value = 1
        params = {'~step_deg': step, '~interval_sec': 8., '~num_steps': 3, '~timeout_sec': 65.,
                  '~use_admittance': admittance}

        def consume(message):
            if message.mode not in (keyboard.RollYawMotion.CLAIM, keyboard.RollYawMotion.RELEASE):
                messages.append((clock[0], message))
            if message.mode == keyboard.RollYawMotion.BODY_PITCH:
                if plant['first_sent'] is None:
                    plant['first_sent'] = clock[0]
                plant.update(target=math.degrees(message.value), rate=math.degrees(message.rate_limit),
                             sequence=message.sequence, sent_at=clock[0], steps=plant['steps'] + 1)
                if interrupt_phase == 'acknowledgement':
                    signal.raise_signal(signal.SIGTERM)
            elif message.mode == keyboard.RollYawMotion.STOP:
                plant['target'] = plant['nominal']
            elif message.mode == keyboard.RollYawMotion.RELEASE:
                plant['released'] = True

        def advance(dt):
            before = plant['nominal']
            if not blocked:
                difference = plant['target'] - before
                plant['nominal'] += max(-plant['rate'] * dt, min(plant['rate'] * dt, difference))
                # Nominal +0.9 can complete while physical motion remains behind its start.
                correction = (2.2 * min(1., (clock[0] - plant['first_sent']) / 2.)
                              if yielding and plant['enabled'] and plant['first_sent'] is not None else 0)
                plant['angle'] = plant['nominal'] - correction
            clock[0] += dt
            if before != plant['target'] and plant['nominal'] == plant['target']:
                plant['completed_at'].append(clock[0])
                plant['completed_angles'].append(plant['angle'])
            if interrupt_step and plant['steps'] == interrupt_step:
                moving = plant['nominal'] != plant['target']
                if ((interrupt_phase == 'movement' and moving and clock[0] - plant['sent_at'] > .3) or
                        (interrupt_phase == 'hold' and not moving)):
                    signal.raise_signal(signal.SIGINT)
            if interrupt_phase == 'startup' and clock[0] > .2:
                signal.raise_signal(signal.SIGTERM)

        def snapshot():
            client = clients[0]
            active = plant['nominal'] != plant['target']
            collected = plant['tare_at'] is not None and clock[0] - plant['tare_at'] >= 1
            values = {'session_id': '123', 'feedback_waiting': '0', 'stopping': '0',
                      'motion_source': client.source, 'motion_epoch': str(client.current_epoch),
                      'motion_sequence': str(plant['sequence']), 'body_pitch_target_valid': '1',
                      'body_pitch_target_frame': str(client.frame),
                      'body_pitch_target_rad': str(math.radians(plant['target'])),
                      'body_pitch_target_progress_rad': str(math.radians(plant['nominal'])),
                      'body_pitch_target_reference_complete': str(int(not active)),
                      'accepted_destination_pending': '1' if clock[0] < .5 else '0',
                      'body_pitch_velocity_active': str(int(active)),
                      'current_body_rate_rad_s': str(.08 if vibrating else math.radians(plant['rate']) if active else 0),
                      'current_joint_rate_rad_s': '.06' if vibrating else '0', 'effective_body_tracking_error_rad': '0',
                      'admittance_enabled': str(int(plant['enabled'])), 'baseline_ready': str(int(baseline or collected)),
                      'tare_generation': '0' if plant['tare_at'] is None else '1',
                      'producer_generation': '1', 'admittance_configured': '1',
                      'admittance_quiet': str(int(not vibrating and clock[0] >= .7 and not active)),
                      'admittance_hold_eligible': str(int(clock[0] >= .5 and not active and
                                                        (not plant['steps'] or clock[0] >= not_ready_until))),
                      'tare_collecting': str(int(plant['tare_at'] is not None and not collected)),
                      'tare_request_sequence': str(plant['tare_requests']),
                      'enable_request_sequence': str(plant['enable_requests']),
                      'admittance_enable_requested': '1',
                      'offset_rad': str(math.radians(plant['angle'] - plant['nominal'])),
                      'effective_offset_rad': str(math.radians(plant['angle'] - plant['nominal'])),
                      'offset_rate_rad_s': '-.01' if yielding and plant['enabled'] else '0',
                      'offset_acceleration_rad_s2': '0', 'task_torque_valid': '1',
                      'hold_reason': '4' if blocked else '0',
                      'body_pitch_velocity_limit': 'tracking feed scaled' if blocked else 'coordinated'}
            if plant['steps']:
                if stale_status:
                    values['body_pitch_target_frame'] = '0'
                if stale_ack:
                    client.ack[1]['mode'] = str(keyboard.RollYawMotion.PULSE)
                if new_owner:
                    client.current_epoch += 1
            pose = (0., 100., self.pitch(0)) if stale_pose and plant['steps'] else (
                clock[0], 100 + clock[0], self.pitch(plant['angle']))
            state = 'FAULT_HOLD: test' if fault and plant['sequence'] else 'CONTACT_LOCKED: test'
            return (clock[0], 100 + clock[0], state, values), {'reference': pose, 'measured': pose}

        def collect(msg):
            plant['tare_requests'] += 1
            if not refuse_tare or plant['tare_requests'] > 1:
                plant['tare_at'] = clock[0]
        tare.publish.side_effect = collect
        def arm(msg):
            if not baseline:
                self.assertGreaterEqual(clock[0] - plant['tare_at'], 1.)
            plant['enable_requests'] += 1
            if not refuse_enable or plant['enable_requests'] > 1:
                plant['enabled'] = msg.data
        enable.publish.side_effect = arm
        factory = simulated_motion(clock, publisher, consume)
        def create(prefix):
            client = factory(prefix)
            clients.append(client)
            return client
        old_handler = signal.getsignal(signal.SIGINT)
        with ExitStack() as stack:
            patches = [
                patch.object(experiment.rospy, 'init_node'),
                patch.object(experiment.rospy, 'get_param', side_effect=lambda key, default=None: params.get(key, default)),
                patch.object(experiment.rospy, 'has_param', side_effect=lambda key: key in params),
                patch.object(experiment.rospy, 'get_time', side_effect=lambda: 100 + clock[0]),
                patch.object(experiment.rospy, 'is_shutdown', return_value=False),
                patch.object(experiment.rospy, 'Publisher', side_effect=lambda topic, *args, **kw:
                             tare if topic.endswith('/admittance_tare') else enable if topic.endswith('/admittance_enable') else publisher),
                patch.object(experiment.rospy, 'Subscriber', return_value=MagicMock()),
                patch.object(experiment, 'MotionClient', wraps=keyboard.MotionClient, side_effect=create),
                patch.object(experiment.rospy, 'loginfo'),
                patch.object(experiment.time, 'monotonic', side_effect=lambda: clock[0]),
                patch.object(experiment.time, 'sleep', side_effect=advance)]
            for context in patches:
                stack.enter_context(context)
            feedback = stack.enter_context(patch.object(experiment, 'Feedback'))
            feedback.return_value.snapshot.side_effect = snapshot
            error = ('active contact' if fault else 'blocked/timed out.*remaining.*tracking feed scaled' if blocked else
                     'identity/frame/target' if stale_status else 'acknowledgement missing' if stale_ack else
                     'pose feedback lost' if stale_pose else 'cancelled' if new_owner else None)
            if error:
                with self.assertRaisesRegex(ValueError, error):
                    experiment.main()
            else:
                experiment.main()
        self.assertEqual(signal.getsignal(signal.SIGINT), old_handler)
        if not error and not interrupt_step and interrupt_phase not in ('startup', 'acknowledgement'):
            self.assertTrue(plant['released'])
        return clock[0], messages, plant

    def test_absolute_angles_share_frame_and_hold_after_each_completion(self):
        for sign in (1, -1):
            elapsed, messages, plant = self.run_angles(step=sign * .3)
            self.assertEqual(len(messages), 3)
            for i, (stamp, msg) in enumerate(messages):
                self.assertEqual(msg.mode, keyboard.RollYawMotion.BODY_PITCH)
                self.assertEqual(msg.duration, 0.)
                self.assertEqual(msg.frame, messages[0][1].frame)
                self.assertAlmostEqual(msg.value, math.radians(sign * .3 * (i + 1)))
                self.assertAlmostEqual(msg.rate_limit, math.radians(.5))
                if i:
                    self.assertGreaterEqual(stamp - plant['completed_at'][i-1], 8.)
            self.assertAlmostEqual(plant['angle'], sign * .9, places=5)
            self.assertGreaterEqual(elapsed - plant['completed_at'][-1], 8.)

    def test_interruption_stops_destination_and_sends_no_successor(self):
        for phase in ('movement', 'hold', 'acknowledgement'):
            _, messages, _ = self.run_angles(interrupt_step=1, interrupt_phase=phase)
            self.assertEqual([msg.mode for _, msg in messages], [keyboard.RollYawMotion.BODY_PITCH, keyboard.RollYawMotion.STOP])
        _, messages, _ = self.run_angles(interrupt_phase='startup')
        self.assertFalse(any(msg.mode == keyboard.RollYawMotion.BODY_PITCH for _, msg in messages))

    def test_fault_blockage_and_stale_feedback_stop_without_successor(self):
        for problem in ('fault', 'blocked', 'stale_status', 'stale_ack', 'stale_pose'):
            _, messages, _ = self.run_angles(**{problem: True})
            self.assertEqual([msg.mode for _, msg in messages], [keyboard.RollYawMotion.BODY_PITCH, keyboard.RollYawMotion.STOP])

    def test_epoch_change_does_not_stop_new_owner(self):
        _, messages, _ = self.run_angles(new_owner=True)
        self.assertEqual([msg.mode for _, msg in messages], [keyboard.RollYawMotion.BODY_PITCH])

    def test_hold_waits_for_effective_tracking_readiness(self):
        _, messages, _ = self.run_angles(not_ready_until=15.)
        self.assertGreaterEqual(messages[1][0], 15.3)

    def test_admittance_waits_for_new_tare_generation_before_enable(self):
        _, messages, plant = self.run_angles(admittance=True)
        self.assertTrue(plant['enabled'])
        self.assertGreaterEqual(messages[0][0] - plant['tare_at'], 1.)

    def test_admittance_retreat_and_vibration_allow_nominal_completion(self):
        elapsed, messages, plant = self.run_angles(admittance=True, yielding=True, vibrating=True)
        self.assertEqual(len(messages), 3)
        self.assertLess(plant['completed_angles'][0], 0.)
        self.assertAlmostEqual(plant['nominal'], .9, places=5)
        self.assertAlmostEqual(plant['angle'], -1.3, places=5)
        self.assertEqual(plant['tare_requests'], 1)
        self.assertEqual(plant['enable_requests'], 1)
        for i in range(1, len(messages)):
            self.assertGreaterEqual(messages[i][0] - plant['completed_at'][i-1], 8.)
        self.assertGreaterEqual(elapsed - plant['completed_at'][-1], 8.)

    def test_admittance_retries_processed_tare_and_enable_refusals(self):
        _, messages, plant = self.run_angles(admittance=True, refuse_tare=True, refuse_enable=True)
        self.assertEqual(len(messages), 3)
        self.assertEqual(plant['tare_requests'], 2)
        self.assertEqual(plant['enable_requests'], 2)

    def test_admittance_reuses_valid_disarmed_baseline_like_nojoint(self):
        _, _, plant = self.run_angles(admittance=True, baseline=True)
        self.assertIsNone(plant['tare_at'])
        self.assertEqual(plant['tare_requests'], 0)
        self.assertTrue(plant['enabled'])

    def test_completion_needs_matching_identity_angle_and_bounded_brake(self):
        task = experiment.AngleSequence(3, 8.)
        message = keyboard.RollYawMotion(session=123, epoch=2, source='test', sequence=5,
                                        frame=5, mode=keyboard.RollYawMotion.BODY_PITCH, value=math.radians(.3))
        task.submitted(message, 0.)
        ack = {key: str(getattr(message, key)) for key in ('session', 'epoch', 'source', 'sequence', 'mode')}
        ack.update(accepted='true', accepted_value=str(message.value))
        for key in ('session', 'epoch', 'source', 'sequence', 'mode'):
            self.assertFalse(task.acknowledge((1., dict(ack, **{key: 'old'}), 'test')))
        self.assertTrue(task.acknowledge((1., ack, 'test')))
        state = dict(session_id='123', motion_source='test', motion_epoch='2', motion_sequence='5',
                     body_pitch_target_valid='1', body_pitch_target_frame='5', body_pitch_target_rad=str(message.value),
                     body_pitch_target_progress_rad='.000001', body_pitch_target_reference_complete='1',
                     body_pitch_velocity_active='0', stopping='0')
        self.assertFalse(task.ready(100., state, True))  # Tiny advancement is not arrival.
        state['body_pitch_target_progress_rad'] = str(message.value)
        task.accepted = False
        self.assertFalse(task.ready(100., state, True))
        self.assertIn('remaining +0.000000 deg', task.describe('waiting for acknowledgement', state))
        self.assertIn('remaining', task.describe('invalid feedback', dict(state, body_pitch_target_rad='invalid')))
        task.accepted = True
        for key in ('session_id', 'motion_source', 'motion_epoch', 'motion_sequence', 'body_pitch_target_frame', 'body_pitch_target_rad'):
            self.assertFalse(task.matches_status(dict(state, **{key: '0'})))
        self.assertFalse(task.ready(100., dict(state, body_pitch_velocity_active='1'), True))
        self.assertFalse(task.ready(101., state, True))
        for _ in range(100):
            self.assertFalse(task.ready(101., state, True))
        self.assertFalse(task.ready(108., state, False))
        self.assertFalse(task.ready(109., state, True))
        self.assertTrue(task.ready(109.4, state, True))

    def test_invalid_angle_parameters_are_rejected_before_dispatch(self):
        for invalid in ({'~step_deg': 0}, {'~step_deg': float('nan')}, {'~step_deg': float('inf')},
                        {'~step_deg': 3}, {'~step_deg': True}, {'~num_steps': 0}, {'~num_steps': 1.5},
                        {'~num_steps': 10**400}, {'~body_pitch_rate_deg_s': 0}, {'~body_pitch_rate_deg_s': float('inf')},
                        {'~interval_sec': -1}, {'~hold_duration_sec': 7, '~interval_sec': 8},
                        {'~angular_velocity_deg_s': .5}, {'~feed_duration_sec': 1.}):
            params = dict({'~step_deg': .3}, **invalid)
            with patch.object(experiment.rospy, 'init_node'), \
                 patch.object(experiment.rospy, 'has_param', side_effect=lambda key: key in params), \
                 patch.object(experiment.rospy, 'get_param', side_effect=lambda key, default=None: params.get(key, default)), \
                 patch.object(experiment, 'MotionClient') as client:
                with self.assertRaises(ValueError):
                    experiment.main()
                client.assert_not_called()

    def test_old_client_does_not_refresh_or_stop_new_owner(self):
        with patch.object(keyboard.rospy, 'Publisher', return_value=MagicMock()), \
             patch.object(keyboard.rospy, 'Subscriber', return_value=MagicMock()):
            client = keyboard.MotionClient('/test')
        client.epoch_callback(keyboard.UInt64(7))
        client.send(123, keyboard.RollYawMotion.VELOCITY, .01)
        message = client.publisher.publish.call_args.args[0]
        wire = io.BytesIO()
        message.serialize(wire)
        restored = keyboard.RollYawMotion().deserialize(wire.getvalue())
        self.assertEqual((restored.session, restored.epoch, restored.source), (123, 7, client.source))
        client.epoch_callback(keyboard.UInt64(8))
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            client.send(123, keyboard.RollYawMotion.VELOCITY, .01)
        self.assertTrue(client.stop(wait=True))
        self.assertEqual(client.publisher.publish.call_count, 1)

    def test_stop_retries_until_observed_hold_even_if_ack_was_lost(self):
        with patch.object(keyboard.rospy, 'Publisher', return_value=MagicMock()), \
             patch.object(keyboard.rospy, 'Subscriber', return_value=MagicMock()):
            client = keyboard.MotionClient('/test')
        client.epoch_callback(keyboard.UInt64(1))
        client.send(123, keyboard.RollYawMotion.VELOCITY, .01)
        client.claimed = True  # Exercise STOP after an ownership grant.
        clock, stops = [0.], []

        def receive(message):
            stops.append(message)
            if len(stops) == 2:
                client.epoch_callback(keyboard.UInt64(2))
                client.status = (clock[0], {'motion_epoch': '2', 'stopping': '0', 'body_pitch_velocity_active': '0'})
        client.publisher.publish.side_effect = receive
        with patch.object(keyboard.rospy, 'is_shutdown', return_value=False), \
             patch.object(keyboard.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(keyboard.time, 'sleep', side_effect=lambda dt: clock.__setitem__(0, clock[0] + dt)):
            self.assertTrue(client.stop(wait=True))
        self.assertEqual(len(stops), 2)
        self.assertEqual(stops[0].sequence, stops[1].sequence)

    def test_operator_zero_cancels_other_owner_without_repeated_cancellation(self):
        publisher = MagicMock()
        with patch.object(keyboard.rospy, 'Publisher', return_value=publisher), \
             patch.object(keyboard.rospy, 'Subscriber', return_value=MagicMock()):
            client = keyboard.MotionClient('/test')
        client.epoch_callback(keyboard.UInt64(1))
        clock = [0.]
        publisher.publish.side_effect = lambda message: client.epoch_callback(keyboard.UInt64(2))

        def advance(dt):
            clock[0] += dt
            if clock[0] > .35:
                client.status = (clock[0], {'motion_epoch': '2', 'stopping': '0', 'body_pitch_velocity_active': '0'})
        with patch.object(keyboard.rospy, 'Publisher', return_value=publisher), \
             patch.object(keyboard.rospy, 'is_shutdown', return_value=False), \
             patch.object(keyboard.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(keyboard.time, 'sleep', side_effect=advance):
            self.assertTrue(client.cancel(123))
        publisher.publish.assert_called_once()
        self.assertEqual(publisher.publish.call_args.args[0].data, 123)


class AdmittanceAdmission(unittest.TestCase):
    def setUp(self):
        self.startup = experiment.AdmittanceStartup()
        self.values = dict(tare_generation='4', producer_generation='7', admittance_configured='1',
                           admittance_enabled='0', baseline_ready='0', tare_collecting='0', offset_rad='0',
                           admittance_quiet='0', admittance_hold_eligible='1',
                           offset_rate_rad_s='0', offset_acceleration_rad_s2='0',
                           task_torque_valid='1', tare_request_sequence='0',
                           enable_request_sequence='0')
        self.values['admittance_enable_requested'] = '1'
        self.tare, self.enable = MagicMock(), MagicMock()
        self.tare.get_num_connections.return_value = self.enable.get_num_connections.return_value = 1

    def ready(self, now):
        return self.startup.ready(now, self.values, self.tare, self.enable)

    def test_held_contact_required_without_quiet_and_missing_ack_does_not_repeat_tare(self):
        self.values['admittance_hold_eligible'] = '0'
        self.assertFalse(self.ready(0))
        self.tare.publish.assert_not_called()
        self.values['admittance_hold_eligible'] = '1'
        self.assertFalse(self.ready(1))
        self.assertFalse(self.ready(2))
        with self.assertRaisesRegex(ValueError, 'acknowledgement missing'):
            self.ready(3.1)
        self.tare.publish.assert_called_once()

    def test_accepted_collection_is_not_restarted_and_enable_waits_for_load(self):
        self.ready(0)
        self.values.update(tare_generation='5', tare_request_sequence='1', tare_collecting='1')
        for now in (1, 2, 3):
            self.assertFalse(self.ready(now))
        self.tare.publish.assert_called_once()
        self.values.update(baseline_ready='1', tare_collecting='0', task_torque_valid='0')
        self.assertFalse(self.ready(4))
        self.enable.publish.assert_not_called()
        self.values['task_torque_valid'] = '1'
        self.assertFalse(self.ready(5))
        self.values.update(enable_request_sequence='1', admittance_enabled='1')
        self.assertTrue(self.ready(5.1))
        self.enable.publish.assert_called_once()

    def test_enabled_readiness_requires_held_contact_and_load_but_allows_correction(self):
        self.values.update(baseline_ready='1', admittance_enabled='1', offset_rad='-.02',
                           offset_rate_rad_s='-.01', task_torque_valid='0')
        self.assertFalse(self.ready(0))
        self.values.update(task_torque_valid='1', admittance_hold_eligible='0')
        self.assertFalse(self.ready(1))
        self.values['admittance_hold_eligible'] = '1'
        self.assertTrue(self.ready(2))
        self.tare.publish.assert_not_called()
        self.enable.publish.assert_not_called()

    def test_disarmed_correction_must_finish_before_tare_or_enable(self):
        for baseline in ('0', '1'):
            self.setUp()
            self.values.update(baseline_ready=baseline, offset_rate_rad_s='.001')
            self.assertFalse(self.ready(0))
            self.tare.publish.assert_not_called()
            self.enable.publish.assert_not_called()

    def test_baseline_change_aborts_without_automatic_retare(self):
        for changes in (dict(tare_generation='6'), dict(baseline_ready='0')):
            self.setUp()
            self.values.update(baseline_ready='1', admittance_enabled='1')
            self.assertTrue(self.ready(0))
            self.values.update(changes)
            with self.assertRaises(ValueError):
                self.ready(1)
            self.tare.publish.assert_not_called()

    def test_planner_retirement_preserves_completed_baseline(self):
        self.values.update(baseline_ready='1', admittance_enabled='1')
        self.assertTrue(self.ready(0))
        self.values['producer_generation'] = '8'
        self.assertTrue(self.ready(1))
        self.tare.publish.assert_not_called()

    def test_operator_disable_is_not_retried_as_an_enable_refusal(self):
        self.values['baseline_ready'] = '1'
        self.assertFalse(self.ready(0))
        self.values.update(enable_request_sequence='1', admittance_enable_requested='0')
        with self.assertRaisesRegex(ValueError, 'interrupted by another request'):
            self.ready(1)
        self.enable.publish.assert_called_once()


class MotionHandover(unittest.TestCase):
    def setUp(self):
        self.epoch, self.owner, self.until = 1, None, 0.
        self.clock, self.clients, self.messages = [0.], [], []

    def client(self):
        publisher = MagicMock()
        publisher.get_num_connections.return_value = 1
        with patch.object(keyboard.rospy, 'Publisher', return_value=publisher), \
             patch.object(keyboard.rospy, 'Subscriber', return_value=MagicMock()):
            client = keyboard.MotionClient('/test')
        client.epoch_callback(keyboard.UInt64(self.epoch))
        self.clients.append(client)

        def receive(message):
            self.messages.append(message)
            accepted, reason = message.epoch == self.epoch, 'old epoch'
            if accepted and message.mode == keyboard.RollYawMotion.CLAIM:
                accepted, reason = self.clock[0] >= self.until, 'busy'
                if accepted:
                    self.epoch += 1
                    self.owner = message.source
            elif accepted:
                accepted, reason = message.source == self.owner, 'another source'
                if accepted and message.mode == keyboard.RollYawMotion.RELEASE:
                    self.epoch += 1
                    self.owner = None
                elif accepted and message.mode == keyboard.RollYawMotion.PULSE:
                    self.until = self.clock[0] + message.duration
            client.ack_callback(keyboard.String(
                'action=motion session={} epoch={} source={} sequence={} mode={} accepted={} motion_epoch={} reason={}'.format(
                    message.session, message.epoch, message.source, message.sequence, message.mode,
                    str(accepted).lower(), self.epoch, reason)))
            for peer in self.clients:
                peer.epoch_callback(keyboard.UInt64(self.epoch))
        publisher.publish.side_effect = receive
        return client

    def test_release_during_pulse_keeps_deadline_and_next_client_waits(self):
        first, second = self.client(), self.client()
        first.claim(123)
        first.send(123, keyboard.RollYawMotion.PULSE, .01, duration=1.)
        self.assertTrue(first.release())
        self.assertEqual(self.until, 1.)
        with self.assertRaisesRegex(RuntimeError, 'busy'):
            second.claim(123)
        self.assertTrue(second.stop())  # A refused claimant cannot cancel another pulse.
        self.clock[0] = 1.1
        second.claim(123)  # Explicit new attempt, now idle.
        second.send(123, keyboard.RollYawMotion.PULSE, .01, duration=1.)
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            first.send(123, keyboard.RollYawMotion.PULSE, .01, duration=1.)
        self.assertTrue(first.stop())
        self.assertNotIn(keyboard.RollYawMotion.STOP, [msg.mode for msg in self.messages])

    def test_killed_client_can_be_replaced_only_after_finite_motion(self):
        first = self.client()
        first.claim(123)
        first.send(123, keyboard.RollYawMotion.PULSE, .01, duration=1.)
        second = self.client()
        with self.assertRaisesRegex(RuntimeError, 'busy'):
            second.claim(123)
        self.clock[0] = 1.1
        second.claim(123)
        second.send(123, keyboard.RollYawMotion.BODY_PITCH, .02, rate_limit=.01)
        self.assertEqual(self.owner, second.source)
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            first.send(123, keyboard.RollYawMotion.PULSE, .01, duration=1.)


class Interfaces(unittest.TestCase):
    def test_uint64_keeps_all_bits(self):
        for session in [1, 1793890000123456789, 2**64 - 1]:
            message = command.session_message(session)
            wire = io.BytesIO()
            message.serialize(wire)
            self.assertEqual(struct.unpack('<Q', wire.getvalue())[0], session)

    def test_reject_invalid_session_types(self):
        for session in [0, -1, 2**64, 1.5, '123', True]:
            with self.assertRaises(ValueError):
                command.session_message(session)

    def make_keyboard(self):
        with patch.object(keyboard.rospy, 'get_param', side_effect=lambda key, default: default), \
             patch.object(keyboard.rospy, 'Publisher', side_effect=lambda *a, **kw: MagicMock()), \
             patch.object(keyboard.rospy, 'Subscriber', return_value=MagicMock()):
            k = keyboard.Keyboard()
        k.session_callback(keyboard.UInt64(123))
        k.motion.epoch_callback(keyboard.UInt64(1))
        k.flight_callback(keyboard.UInt8(5))
        k.state_callback(keyboard.String('CONTACT_LOCKED: test'))
        for bound, value in [('lower', -1.), ('upper', 1.)]:
            m = keyboard.Vector3Stamped()
            m.header.frame_id = 'perching_lock_123'
            m.vector.x = m.vector.y = m.vector.z = value
            k.limits_callback(m, bound)
        m = keyboard.Vector3Stamped()
        m.header.frame_id = 'perching_lock_123'
        k.angles_callback(m)
        # Keyboard tests below exercise target semantics. The real acquisition
        # exchange is covered separately by the multi-client protocol tests.
        k.motion.claim = MagicMock()
        return k

    def test_keyboard_independent_axes_and_immediate_limit_reversal(self):
        k = self.make_keyboard()
        k.manual('yaw', 1)
        yaw = k.commanded[2]
        k.manual('roll', 1)
        k.manual('pitch', -1)
        self.assertEqual(k.commanded[2], yaw)
        self.assertGreater(k.commanded[0], 0)
        self.assertLess(k.commanded[1], 0)
        k.commanded[2] = 1.
        k.manual('yaw', 1)
        self.assertEqual(k.commanded[2], 1.)
        k.manual('yaw', -1)
        self.assertAlmostEqual(k.commanded[2], 1. - k.steps['yaw'])
        self.assertEqual(k.motion.publisher.publish.call_args[0][0].value, k.commanded[2])

    def test_keyboard_claim_does_not_block_its_ros_acknowledgement_thread(self):
        k = self.make_keyboard()
        k.motion.claim = type(k.motion).claim.__get__(k.motion)
        workers = []

        def receive(message):
            if message.mode != keyboard.RollYawMotion.CLAIM:
                return
            def acknowledge():
                # ROS invokes subscribers on this connection in sequence. The
                # queued receipt reaches the keyboard before the final reply.
                k.ack_callback(keyboard.String('action=owner_claim session=123 received=true'))
                reply = keyboard.String(
                    'action=motion session=123 epoch={} source={} sequence={} mode={} accepted=true motion_epoch=2'.format(
                        message.epoch, message.source, message.sequence, message.mode))
                k.motion.ack_callback(reply)
                k.motion.epoch_callback(keyboard.UInt64(2))
                k.ack_callback(reply)
            worker = threading.Thread(target=acknowledge, daemon=True)
            workers.append(worker)
            worker.start()
        k.motion.publisher.publish.side_effect = receive
        k.key('o')
        for worker in workers:
            worker.join(1.)
            self.assertFalse(worker.is_alive())
        messages = [call.args[0] for call in k.motion.publisher.publish.call_args_list]
        self.assertEqual([m.mode for m in messages], [keyboard.RollYawMotion.CLAIM, keyboard.RollYawMotion.BODY_PITCH])
        self.assertEqual(messages[-1].epoch, 2)

    def test_keyboard_stale_feedback_and_new_session_do_not_queue(self):
        k = self.make_keyboard()
        k.state_time = time.monotonic() - 2
        with self.assertRaises(RuntimeError):
            k.manual('yaw', 1)
        k.motion.publisher.publish.assert_not_called()
        k.session_callback(keyboard.UInt64(124))
        self.assertIsNone(k.commanded)
        with self.assertRaises(RuntimeError):
            k.manual('roll', 1)
        k.motion.publisher.publish.assert_not_called()

    def test_pitch_frame_survives_updates_but_changes_after_axis_takeover(self):
        k = self.make_keyboard()
        k.key('o')
        first = k.motion.publisher.publish.call_args.args[0]
        k.key('o')
        second = k.motion.publisher.publish.call_args.args[0]
        self.assertEqual(first.frame, second.frame)
        # Fresh accepted nominal feedback permits a mechanism-axis takeover.
        nominal = keyboard.Vector3Stamped()
        nominal.header.frame_id = 'perching_lock_123'
        k.angles_callback(nominal)
        k.manual('roll', 1)
        k.key('o')
        third = k.motion.publisher.publish.call_args.args[0]
        self.assertGreater(third.frame, second.frame)
        self.assertAlmostEqual(third.value, math.radians(.2))

    def test_stop_rearms_from_applied_pose_not_cancelled_nominal(self):
        k = self.make_keyboard()
        k.manual('yaw', 1)
        k.angles = (0., 0., .8)
        k.applied, k.applied_time = (0., 0., .1), time.monotonic()
        # Simulate a very fast epoch callback during cancel publication.
        k.pubs['stop'].publish.side_effect = lambda _: k.motion.epoch_callback(keyboard.UInt64(2))
        k.manual()
        k.manual('yaw', 1)
        self.assertAlmostEqual(k.motion.publisher.publish.call_args.args[0].value, .1 + math.radians(.2))

    def test_stale_keyboard_display_does_not_cancel_an_automatic_owner(self):
        k = self.make_keyboard()
        k.had_feedback = True
        k.state_time = time.monotonic() - 2
        with patch.object(k, 'show_status'):
            k.tick()
        k.pubs['stop'].publish.assert_not_called()

    def test_body_pitch_keys_replace_total_destination_and_reverse(self):
        k = self.make_keyboard()
        k.key('o')
        self.assertAlmostEqual(k.motion.publisher.publish.call_args[0][0].value, math.radians(.2))
        k.key('o')
        k.key('o')
        self.assertAlmostEqual(k.motion.publisher.publish.call_args[0][0].value, math.radians(.6))
        k.key('u')
        self.assertAlmostEqual(k.motion.publisher.publish.call_args[0][0].value, math.radians(.4))
        self.assertIsNone(k.commanded)  # Reconcile the navigator's coordinated nominal vector.
        self.assertEqual(k.pending_axes, [False] * 3)
        k.pubs['stop'].publish.assert_not_called()
        k.state_callback(keyboard.String('PRE_ALIGN: body-local pitch jog moving'))
        k.key('o')  # Navigator accepts a replacement through the normal nominal path.
        self.assertEqual(k.motion.publisher.publish.call_count, 5)
        k.pubs['stop'].publish.assert_not_called()
        k.had_feedback = True
        k.state_time = time.monotonic() - 2
        with patch.object(k, 'show_status'):
            k.tick()
        k.pubs['stop'].publish.assert_not_called()  # Preserve automatic pause/resume.
        k.key(' ')
        k.pubs['stop'].publish.assert_called_once()
        k.state_callback(keyboard.String('CONTACT_LOCKED: test'))
        with self.assertRaisesRegex(RuntimeError, 'Waiting'):
            k.key('o')
        k.motion.epoch_callback(keyboard.UInt64(2))
        k.key('u')
        self.assertAlmostEqual(k.motion.publisher.publish.call_args[0][0].value, -math.radians(.2))

    def test_body_pitch_parameter_namespace_and_acknowledgements(self):
        params = {'~robot_namespace': '/other', '~debug_prefix': 'custom',
                  '~body_pitch_step_deg': .4, '~pitch_step_deg': .1}
        with patch.object(keyboard.rospy, 'get_param', side_effect=lambda key, default: params.get(key, default)), \
             patch.object(keyboard.rospy, 'Publisher', return_value=MagicMock()) as publisher, \
             patch.object(keyboard.rospy, 'Subscriber', return_value=MagicMock()):
            k = keyboard.Keyboard()
        self.assertEqual(k.prefix, '/other/custom')
        self.assertAlmostEqual(k.steps['body_pitch'], math.radians(.4))
        self.assertAlmostEqual(k.steps['pitch'], math.radians(.1))
        self.assertIn('/other/custom/motion', [c.args[0] for c in publisher.call_args_list])
        k.session = 123
        with patch('builtins.print') as output:
            k.ack_callback(keyboard.String('action=body_pitch_delta session=122 accepted=false phase=rejected reason=busy'))
            output.assert_not_called()
            k.ack_callback(keyboard.String('action=body_pitch_delta session=123 accepted=false phase=rejected reason=busy'))
            self.assertIn('reason=busy', output.call_args.args[0])
        for invalid in (0, -1, 1.1, float('nan'), float('inf')):
            params['~body_pitch_step_deg'] = invalid
            with patch.object(keyboard.rospy, 'get_param', side_effect=lambda key, default: params.get(key, default)):
                with self.assertRaises(ValueError):
                    keyboard.Keyboard()

    def load_launch(self, simulation, robot_id=''):
        config = roslaunch.config.ROSLaunchConfig()
        namespace = '/gimbalrotor' + robot_id
        launch = 'roll_yaw_perching.launch'
        args = ['simulation:=' + simulation]
        if robot_id:
            # robot_id is exposed by bringup, not the roll-yaw convenience launch.
            launch = 'bringup.launch'
            args += ['robot_id:=' + robot_id, 'real_machine:=true', 'airframe:=beetle',
                     'roll_yaw_perching:=true', 'direct_model_name:=unused']
        roslaunch.xmlloader.XmlLoader().load(
            str(ROOT / 'launch' / launch), config, argv=args,
            verbose=False)
        self.assertEqual(config.params[namespace + '/aerial_robot_control_name'].value,
                         'aerial_robot_control/gimbalrotor_multilink_perching_admittance_controller')
        self.assertEqual(config.params[namespace + '/flight_navigation_plugin_name'].value,
                         'aerial_robot_navigation/gimbalrotor_perching_navigation')
        self.assertEqual(config.params[namespace + '/navigation/perching_configuration'].value,
                         'roll_yaw')
        return config

    def test_hardware_offline(self):
        config = self.load_launch('false')
        self.assertTrue(config.params['/gimbalrotor/navigation/roll_yaw_perching/settle_use_window'].value)
        self.check_effort_source(config, '/gimbalrotor', 'servo_bridge_joint_states')

    def check_effort_source(self, config, namespace, source):
        prefix = namespace + '/navigation/roll_yaw_perching/'
        self.assertEqual(config.params[prefix + 'admittance/source'].value, 'articulated_momentum')
        self.assertEqual(config.params[prefix + 'dynamics/joint_effort/source'].value, source)
        self.assertEqual(config.params[prefix + 'dynamics/joint_effort/topic'].value,
                         config.params[prefix + 'joint_state_topic'].value)
        if source == 'servo_bridge_joint_states':
            self.assertNotIn(prefix + 'dynamics/joint_effort/producer', config.params)
            self.assertNotIn(prefix + 'dynamics/joint_effort_contract', config.params)
            self.assertTrue(any(node.name == 'servo_bridge' and node.namespace.rstrip('/') == namespace
                                for node in config.nodes))

    def test_hardware_effort_uses_robot_namespace(self):
        config = self.load_launch('false', '7')
        self.check_effort_source(config, '/gimbalrotor7', 'servo_bridge_joint_states')

    def test_simulation_override(self):
        config = self.load_launch('true')
        self.check_effort_source(config, '/gimbalrotor', 'gazebo_ode_joint_states')
        prefix = '/gimbalrotor/navigation/roll_yaw_perching/dynamics/'
        self.assertEqual(config.params[prefix + 'joint_effort_contract'].value, 'joint_side_actuation_nm')
        self.assertEqual(config.params[prefix + 'joint_effort/producer'].value, '/gazebo')
        self.assertFalse(config.params['/gimbalrotor/navigation/roll_yaw_perching/settle_use_window'].value)
        self.assertEqual(config.params['/gimbalrotor/navigation/roll_yaw_perching/settle_rate'].value, .08)


if __name__ == '__main__':
    unittest.main()
