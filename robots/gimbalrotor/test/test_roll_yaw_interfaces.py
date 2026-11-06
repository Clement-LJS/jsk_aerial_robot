"""Offline ROS message/launch checks; no node or ROS master is started."""
import importlib.util
import io
from pathlib import Path
import struct
import unittest

import roslaunch.config
import roslaunch.xmlloader

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('roll_yaw_command', ROOT / 'scripts/keyboard_command/roll_yaw_command.py')
command = importlib.util.module_from_spec(spec)
spec.loader.exec_module(command)


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

    def load_launch(self, simulation, qp, online):
        config = roslaunch.config.ROSLaunchConfig()
        roslaunch.xmlloader.XmlLoader().load(
            str(ROOT / 'launch/roll_yaw_perching.launch'), config,
            argv=['simulation:=' + simulation, 'qp_planning:=' + qp, 'online_planning:=' + online],
            verbose=False)
        self.assertEqual(config.params['/gimbalrotor/navigation/roll_yaw_perching/qp_planning'].value, qp == 'true')
        self.assertEqual(config.params['/gimbalrotor/navigation/roll_yaw_perching/online_planning'].value, online == 'true')
        self.assertEqual(config.params['/gimbalrotor/aerial_robot_control_name'].value,
                         'aerial_robot_control/gimbalrotor_controller')
        return config

    def test_hardware_offline(self):
        config = self.load_launch('false', 'true', 'false')
        self.assertTrue(config.params['/gimbalrotor/navigation/roll_yaw_perching/settle_use_window'].value)

    def test_hardware_online(self):
        self.load_launch('false', 'true', 'true')

    def test_simulation_override(self):
        config = self.load_launch('true', 'false', 'false')
        self.assertFalse(config.params['/gimbalrotor/navigation/roll_yaw_perching/settle_use_window'].value)
        self.assertEqual(config.params['/gimbalrotor/navigation/roll_yaw_perching/settle_rate'].value, .08)


if __name__ == '__main__':
    unittest.main()
