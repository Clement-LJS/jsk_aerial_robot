from perching_bag_task import Stroke
"""Offline ROS message/launch checks; no node or ROS master is started."""
import importlib.util
import io
import math
import signal
from pathlib import Path
import struct
import unittest
from unittest.mock import patch, MagicMock
import time

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
            motion.ack_callback(keyboard.String(
                'action=motion session={} epoch={} source={} sequence={} accepted=true'.format(
                    message.session, message.epoch, message.source, message.sequence)))
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

    def run_pulses(self, interrupt_step=0, fault=False, blocked=False, admittance=False, yielding=False):
        from contextlib import ExitStack
        clock, messages = [0.], []
        plant = dict(angle=0., nominal=0., until=0., rate=0., sequence=0, sent_at=0., tare_at=None, enabled=False)
        publisher, tare, enable = MagicMock(), MagicMock(), MagicMock()
        for pub in (publisher, tare, enable):
            pub.get_num_connections.return_value = 1
        params = {'~angular_velocity_deg_s': .5, '~interval_sec': 8., '~num_steps': 8,
                  '~use_admittance': admittance}

        def consume(message):
            messages.append((clock[0], message))
            if message.mode == keyboard.RollYawMotion.PULSE:
                plant.update(until=clock[0] + message.duration, rate=message.value,
                             sequence=message.sequence, sent_at=clock[0])
            elif message.mode == keyboard.RollYawMotion.STOP:
                plant['until'] = clock[0]

        def advance(dt):
            if not blocked:
                plant['nominal'] += math.degrees(plant['rate']) * max(0., min(dt, plant['until'] - clock[0]))
                plant['angle'] = plant['nominal'] - (1.5 * min(.5, plant['nominal']) if yielding else 0)
            clock[0] += dt
            if interrupt_step and plant['sequence'] == interrupt_step and clock[0] - plant['sent_at'] > .5:
                signal.raise_signal(signal.SIGINT)

        def snapshot():
            active = clock[0] < plant['until']
            values = {'session_id': '123', 'feedback_waiting': '0', 'stopping': '0',
                      'accepted_destination_pending': '1' if clock[0] < .5 else '0',
                      'body_pitch_velocity_active': str(int(active)),
                      'current_body_rate_rad_s': str(abs(plant['rate']) if active else 0),
                      'current_joint_rate_rad_s': '0', 'effective_body_tracking_error_rad': '0',
                      'pulse_sequence': str(plant['sequence']), 'pulse_complete': str(int(not active)),
                      'admittance_enabled': str(int(plant['enabled'])), 'baseline_ready': '1',
                      'tare_generation': '0' if plant['tare_at'] is None or clock[0] - plant['tare_at'] < 1 else '1',
                      'offset_rad': '0', 'task_torque_valid': '1', 'local_pitch_nominal_rad': str(math.radians(plant['nominal']))}
            pose = (clock[0], 100 + clock[0], self.pitch(plant['angle']))
            state = 'FAULT_HOLD: test' if fault and plant['sequence'] else 'CONTACT_LOCKED: test'
            return (clock[0], 100 + clock[0], state, values), {'reference': pose, 'measured': pose}

        tare.publish.side_effect = lambda msg: plant.update(tare_at=clock[0])
        def arm(msg):
            self.assertGreaterEqual(clock[0] - plant['tare_at'], 1.)  # An old ready baseline must not authorize enable.
            plant['enabled'] = msg.data
        enable.publish.side_effect = arm
        old_handler = signal.getsignal(signal.SIGINT)
        with ExitStack() as stack:
            patches = [
                patch.object(experiment.rospy, 'init_node'),
                patch.object(experiment.rospy, 'get_param', side_effect=lambda key, default=None: params.get(key, default)),
                patch.object(experiment.rospy, 'get_time', side_effect=lambda: 100 + clock[0]),
                patch.object(experiment.rospy, 'is_shutdown', return_value=False),
                patch.object(experiment.rospy, 'Publisher', side_effect=lambda topic, *args, **kw:
                             tare if topic.endswith('/admittance_tare') else enable if topic.endswith('/admittance_enable') else publisher),
                patch.object(experiment.rospy, 'Subscriber', return_value=MagicMock()),
                patch.object(experiment, 'MotionClient', side_effect=simulated_motion(clock, publisher, consume)),
                patch.object(experiment.rospy, 'loginfo'),
                patch.object(experiment.time, 'monotonic', side_effect=lambda: clock[0]),
                patch.object(experiment.time, 'sleep', side_effect=advance)]
            for context in patches:
                stack.enter_context(context)
            feedback = stack.enter_context(patch.object(experiment, 'Feedback'))
            feedback.return_value.snapshot.side_effect = snapshot
            if fault or blocked:
                with self.assertRaisesRegex(ValueError, 'active contact' if fault else 'no measured forward progress'):
                    experiment.main()
            else:
                experiment.main()
        self.assertEqual(signal.getsignal(signal.SIGINT), old_handler)
        return clock[0], messages, plant

    def test_pulses_feed_one_second_and_hold_eight_without_refresh(self):
        elapsed, messages, plant = self.run_pulses()
        self.assertEqual(len(messages), 8)
        for i, (stamp, msg) in enumerate(messages):
            self.assertEqual(msg.mode, keyboard.RollYawMotion.PULSE)
            self.assertEqual(msg.duration, 1.)
            self.assertAlmostEqual(msg.value, math.radians(.5))
            if i:
                self.assertGreaterEqual(stamp - messages[i-1][0], 9.)
        self.assertAlmostEqual(plant['angle'], 4., places=5)
        self.assertGreaterEqual(elapsed - messages[-1][0], 9.)

    def test_ctrl_c_during_fifth_pulse_leaves_it_to_expire_without_sixth(self):
        elapsed, messages, plant = self.run_pulses(interrupt_step=5)
        self.assertEqual(len(messages), 5)
        self.assertTrue(all(message.mode == keyboard.RollYawMotion.PULSE for _, message in messages))
        self.assertLess(elapsed, plant['until'])
        self.assertLess(plant['until'] - elapsed, .5)

    def test_runtime_fault_sends_stop_and_no_successor(self):
        _, messages, _ = self.run_pulses(fault=True)
        self.assertEqual([msg.mode for _, msg in messages], [keyboard.RollYawMotion.PULSE, keyboard.RollYawMotion.STOP])

    def test_blocked_pulse_does_not_claim_progress_or_send_another(self):
        _, messages, _ = self.run_pulses(blocked=True)
        self.assertEqual([msg.mode for _, msg in messages], [keyboard.RollYawMotion.PULSE, keyboard.RollYawMotion.STOP])

    def test_admittance_waits_for_new_tare_generation_before_enable(self):
        _, messages, plant = self.run_pulses(admittance=True)
        self.assertTrue(plant['enabled'])
        self.assertGreaterEqual(messages[0][0] - plant['tare_at'], 1.)

    def test_admittance_retreat_is_not_misreported_as_a_failed_pulse(self):
        _, messages, plant = self.run_pulses(admittance=True, yielding=True)
        self.assertEqual(len(messages), 8)
        self.assertAlmostEqual(plant['angle'], 3.25, places=5)

    def test_hold_and_settling_require_advancing_diagnostics(self):
        task = experiment.PulseSequence(8, 8.)
        task.submitted(5)
        state = {'pulse_sequence': '5', 'pulse_complete': '0'}
        self.assertFalse(task.ready(100., state, True))
        state['pulse_complete'] = '1'
        self.assertFalse(task.ready(101., state, True))
        for _ in range(100):
            self.assertFalse(task.ready(101., state, True))
        self.assertFalse(task.ready(108., state, False))
        self.assertFalse(task.ready(109., state, True))
        self.assertTrue(task.ready(109.4, state, True))

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
