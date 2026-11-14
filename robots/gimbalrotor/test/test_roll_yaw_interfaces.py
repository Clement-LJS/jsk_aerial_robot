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
                         'aerial_robot_control/gimbalrotor_controller')
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
