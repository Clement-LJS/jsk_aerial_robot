"""Offline ROS message/launch checks; no node or ROS master is started."""
import importlib.util
import io
import math
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


class VelocityExperiment(unittest.TestCase):
    @staticmethod
    def pitch(degrees):
        angle = math.radians(degrees)
        return (0., math.sin(angle / 2), 0., math.cos(angle / 2))

    def make_task(self, rate=.1):
        return experiment.Stroke(math.radians(rate), 24., math.radians(.5), 30.)

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
        task = experiment.Stroke(math.radians(.1), 1., math.radians(.5), 30.)
        self.step(task, 0)
        self.step(task, 5, .1, 0)
        self.step(task, 6, .1, 0)
        self.assertFalse(task.complete)

    def test_runner_waits_for_previous_destination_and_stops_on_fault(self):
        clock = [0.]
        published = []
        publisher = MagicMock()
        publisher.get_num_connections.return_value = 1
        publisher.publish.side_effect = lambda message: published.append((clock[0], message.data))
        params = {'~angular_velocity_deg_s': .1,
                  '/gimbalrotor/navigation/roll_yaw_perching/body_pitch_velocity_command_duration': 8.}

        def snapshot():
            state = 'FAULT_HOLD: fixed model mismatch' if clock[0] >= .8 else 'CONTACT_LOCKED: test'
            values = {'session_id': '123', 'feedback_waiting': '0', 'stopping': '0',
                      'accepted_destination_pending': '1' if clock[0] < .5 else '0'}
            pose = (clock[0], 100 + clock[0], self.pitch(0))
            return (clock[0], 100 + clock[0], state, values), {'reference': pose, 'measured': pose}, None

        with patch.object(experiment.rospy, 'init_node'), \
             patch.object(experiment.rospy, 'get_param', side_effect=lambda key, default=None: params.get(key, default)), \
             patch.object(experiment.rospy, 'has_param', side_effect=lambda key: key in params), \
             patch.object(experiment.rospy, 'get_time', side_effect=lambda: 100 + clock[0]), \
             patch.object(experiment.rospy, 'is_shutdown', return_value=False), \
             patch.object(experiment.rospy, 'Publisher', return_value=publisher), \
             patch.object(experiment.rospy, 'loginfo'), patch.object(experiment.rospy, 'loginfo_throttle'), \
             patch.object(experiment, 'Feedback') as feedback, \
             patch.object(experiment.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(experiment.time, 'sleep', side_effect=lambda duration: clock.__setitem__(0, clock[0] + duration)):
            feedback.return_value.snapshot.side_effect = snapshot
            with self.assertRaisesRegex(ValueError, 'active contact'):
                experiment.main()
        self.assertGreaterEqual(published[0][0], .5)
        self.assertGreater(published[0][1], 0)
        self.assertEqual(published[-1][1], 0)

    def test_runner_completes_measured_stroke_after_feedback_outage(self):
        clock = [0.]
        plant = {'reference': 0., 'measured': 0., 'rate': 0., 'ack': None, 'sample': 0.}
        published = []
        publisher = MagicMock()
        publisher.get_num_connections.return_value = 1
        params = {'~angular_velocity_deg_s': .1, '~interval_sec': 8., '~num_steps': 1,
                  '/gimbalrotor/navigation/roll_yaw_perching/body_pitch_velocity_command_duration': 8.}

        def publish(message):
            plant['rate'] = message.data
            plant['ack'] = (clock[0], {'session': '123', 'accepted': 'true'}, 'accepted')
            published.append((clock[0], message.data))

        def advance(duration):
            clock[0] += duration
            plant['reference'] += math.degrees(plant['rate']) * duration
            plant['measured'] += (plant['reference'] - plant['measured']) * min(1., duration * 5)
            if not 2. <= clock[0] < 3.51:
                plant['sample'] = math.floor(clock[0] * 5) / 5  # Recorded diagnostic/pose rate: 5 Hz.

        def snapshot():
            waiting = 2. <= clock[0] < 3.51
            stamp = plant['sample']
            values = {'session_id': '123', 'feedback_waiting': str(int(waiting)), 'stopping': '0',
                      'body_pitch_velocity_active': str(int(plant['rate'] != 0))}
            poses = {kind: (stamp, 100 + stamp, self.pitch(plant[kind])) for kind in ('reference', 'measured')}
            return (clock[0], 100 + clock[0], 'CONTACT_LOCKED: test', values), poses, plant['ack']

        publisher.publish.side_effect = publish
        with patch.object(experiment.rospy, 'init_node'), \
             patch.object(experiment.rospy, 'get_param', side_effect=lambda key, default=None: params.get(key, default)), \
             patch.object(experiment.rospy, 'has_param', side_effect=lambda key: key in params), \
             patch.object(experiment.rospy, 'get_time', side_effect=lambda: 100 + clock[0]), \
             patch.object(experiment.rospy, 'is_shutdown', return_value=False), \
             patch.object(experiment.rospy, 'Publisher', return_value=publisher), \
             patch.object(experiment.rospy, 'loginfo'), patch.object(experiment.rospy, 'loginfo_throttle'), \
             patch.object(experiment, 'Feedback') as feedback, \
             patch.object(experiment.time, 'monotonic', side_effect=lambda: clock[0]), \
             patch.object(experiment.time, 'sleep', side_effect=advance):
            feedback.return_value.snapshot.side_effect = snapshot
            experiment.main()
        self.assertGreater(clock[0], 9.51)  # Nominal eight seconds plus the outage are insufficient.
        self.assertAlmostEqual(plant['measured'], .8, delta=.01)
        self.assertEqual(published[-1][1], 0)
        self.assertTrue(any(2. <= stamp < 3.51 and rate == 0 for stamp, rate in published))
        self.assertTrue(any(stamp >= 3.51 and rate > 0 for stamp, rate in published))


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
        self.assertEqual(k.pubs['yaw'].publish.call_args[0][0].data, k.commanded[2])

    def test_keyboard_stale_feedback_and_new_session_do_not_queue(self):
        k = self.make_keyboard()
        k.state_time = time.monotonic() - 2
        with self.assertRaises(RuntimeError):
            k.manual('yaw', 1)
        k.pubs['yaw'].publish.assert_not_called()
        k.session_callback(keyboard.UInt64(124))
        self.assertIsNone(k.commanded)
        with self.assertRaises(RuntimeError):
            k.manual('roll', 1)
        k.pubs['roll'].publish.assert_not_called()

    def test_body_pitch_keys_are_separate_relative_inputs(self):
        k = self.make_keyboard()
        k.key('o')
        self.assertAlmostEqual(k.pubs['body_pitch'].publish.call_args[0][0].data, math.radians(.2))
        k.key('u')
        self.assertAlmostEqual(k.pubs['body_pitch'].publish.call_args[0][0].data, -math.radians(.2))
        self.assertIsNone(k.commanded)  # Reconcile the navigator's coordinated nominal vector.
        self.assertEqual(k.pending_axes, [False] * 3)
        for axis in ('pitch', 'roll', 'yaw', 'stop'):
            k.pubs[axis].publish.assert_not_called()
        k.state_callback(keyboard.String('PRE_ALIGN: body-local pitch jog moving'))
        k.key('o')  # Navigator accepts a replacement through the normal nominal path.
        self.assertEqual(k.pubs['body_pitch'].publish.call_count, 3)
        k.pubs['stop'].publish.assert_not_called()
        k.had_feedback = True
        k.state_time = time.monotonic() - 2
        with patch.object(k, 'show_status'):
            k.tick()
        k.pubs['stop'].publish.assert_not_called()  # Preserve automatic pause/resume.
        k.key(' ')
        k.pubs['stop'].publish.assert_called_once()

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
        self.assertIn('/other/custom/body_pitch_delta', [c.args[0] for c in publisher.call_args_list])
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
