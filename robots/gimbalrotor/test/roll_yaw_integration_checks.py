#!/usr/bin/env python3
"""Read/parse-only ROS integration checks. Never starts a ROS node or master.

Run with ROS_HOME pointing into a disposable directory in this repository.
"""
import pathlib
import unittest
import xml.etree.ElementTree as ET

import roslaunch.config
import roslaunch.xmlloader
import xacro
import yaml

PACKAGE = pathlib.Path(__file__).resolve().parents[1]


def load_launch(*args):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(
        str(PACKAGE / 'launch/bringup.launch'), config,
        argv=['airframe:=beetle', 'rm:=false', 'sim:=false',
              'direct_model_name:=unused'] + list(args), verbose=False)
    return config


class RollYawIntegration(unittest.TestCase):
    def test_plugin_controller_and_mechanism_selection(self):
        config = load_launch('roll_yaw_perching:=true')
        params = config.params
        self.assertEqual(params['/gimbalrotor/flight_navigation_plugin_name'].value,
                         'aerial_robot_navigation/gimbalrotor_roll_yaw_perching_navigation')
        self.assertEqual(params['/gimbalrotor/aerial_robot_control_name'].value,
                         'aerial_robot_control/gimbalrotor_controller')
        self.assertFalse(params['/gimbalrotor/controller/gimbal_calc_in_fc'].value)
        root = ET.fromstring(params['/gimbalrotor/robot_description'].value)
        joints = {j.attrib['name']: j for j in root.findall('joint')}
        self.assertEqual(joints['joint_roll'].find('axis').attrib['xyz'], '1 0 0')
        self.assertEqual(joints['joint_yaw'].find('axis').attrib['xyz'], '0 0 1')
        self.assertNotIn('joint_pitch', joints)
        self.assertNotIn('joint_pitch_only', joints)
        for name in ('joint_roll', 'joint_yaw'):
            self.assertEqual(float(joints[name].find('limit').attrib['velocity']), .1)
        for joint in joints.values():
            self.assertIn(joint.find('parent').attrib['link'],
                          {link.attrib['name'] for link in root.findall('link')})
        # Check it is a connected tree, not just a parseable collection of links.
        children = {j.find('child').attrib['link'] for j in joints.values()}
        links = {link.attrib['name'] for link in root.findall('link')}
        self.assertEqual(len(links - children), 1)

    def test_modes_are_mutually_exclusive(self):
        for a, b in [('roll_yaw_perching', 'pitch_only_perching'),
                     ('roll_yaw_perching', 'multilink_perching'),
                     ('pitch_only_perching', 'multilink_perching')]:
            with self.assertRaises(Exception):
                load_launch(a + ':=true', b + ':=true')

    def test_defaults_fail_closed_and_servo_mapping_unchanged(self):
        cfg = yaml.safe_load((PACKAGE / 'config/beetle/RollYawPerching.yaml').read_text())
        self.assertFalse(cfg['navigation']['perching_enable'])
        cfg = cfg['navigation']['roll_yaw_perching']
        self.assertFalse(cfg['calibration_confirmed'])
        self.assertNotIn('pivot_offset', cfg)
        self.assertLess(cfg['body_max_tilt'], 20 * 3.141592653589793 / 180)
        servo = yaml.safe_load((PACKAGE / 'config/beetle/Servo.yaml').read_text())['servo_controller']['joints']
        self.assertEqual((servo['controller1']['id'], servo['controller1']['name']), (6, 'joint_yaw'))
        self.assertEqual((servo['controller2']['id'], servo['controller2']['name']), (7, 'joint_roll'))
        plugins = ET.parse(PACKAGE / 'plugins/navigation_plugin.xml')
        names = [x.attrib['name'] for x in plugins.findall('class')]
        for suffix in ('pitch_only', 'multilink', 'roll_yaw'):
            self.assertIn('aerial_robot_navigation/gimbalrotor_' + suffix + '_perching_navigation', names)


if __name__ == '__main__':
    unittest.main()
