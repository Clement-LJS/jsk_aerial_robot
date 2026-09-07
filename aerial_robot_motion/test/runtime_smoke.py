#!/usr/bin/env python3
"""Isolated ROS integration test; publishes only to its private synthetic robot."""
import math
import os
import signal
import subprocess
import threading
import time
import unittest

import rospkg
import rospy
import rostest
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Pose, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from std_srvs.srv import SetBool
from aerial_robot_msgs.msg import FlightNav


def robot_description():
    inertial = "<inertial><mass value='1'/><inertia ixx='.1' iyy='.1' izz='.1' ixy='0' ixz='0' iyz='0'/></inertial>"
    xml = "<robot name='test_quad'><baselink name='body'/><thrust_link name='thrust'/><m_f_rate value='.02'/>"
    xml += "<link name='test_root'/><link name='body'>" + inertial + "</link>"
    xml += "<joint name='mount' type='fixed'><parent link='test_root'/><child link='body'/></joint>"
    for i, (x, y) in enumerate(((.2, .2), (-.2, .2), (-.2, -.2), (.2, -.2)), 1):
        xml += ("<link name='thrust{0}'/><joint name='mount{0}' type='fixed'><parent link='body'/>"
                "<child link='thrust{0}'/><origin xyz='{1} {2} 0'/></joint><link name='prop{0}'/>"
                "<joint name='rotor{0}' type='continuous'><parent link='thrust{0}'/><child link='prop{0}'/>"
                "<axis xyz='0 0 {3}'/><limit lower='0' upper='20' velocity='100' effort='20'/></joint>").format(i, x, y, 1 if i % 2 else -1)
    return xml + "</robot>"


class RuntimeSmoke(unittest.TestCase):
    def test_online_feedback_and_gates(self):
        ns = "/motion_runtime_test"
        private = ns + "/motion"
        package = rospkg.RosPack().get_path("aerial_robot_motion")
        with open(os.path.join(package, "config/default.yaml"), encoding="utf-8") as stream:
            params = yaml.safe_load(stream)
        default_types = {entry["type"] for entry in params["constraint_plugins"]}
        self.assertNotIn("aerial_robot_motion/RevoluteContact", default_types)
        self.assertIn("aerial_robot_motion/AccelerationLimit", default_types)
        self.assertNotIn("admittance", params)
        params.update(publish_commands=True, tool_frame="body", contact_frame="body")
        params["constraint_plugins"].append(
            {"name": "revolute_contact", "type": "aerial_robot_motion/RevoluteContact"})
        params["constraints"]["revolute_contact"] = {
            "position_gain": 3.0,
            "orientation_gain": 3.0,
            "max_position_correction_velocity": 0.05,
            "max_orientation_correction_velocity": 0.2,
        }
        params["hinge_axis"] = [0, 1, 0]
        params["bridge"]["full_attitude"] = True
        params["contact_offset_xyz"] = [-0.3, 0, 0]
        rospy.set_param(private, params)
        rospy.set_param(ns + "/robot_description", robot_description())
        rospy.set_param(ns + "/robot_model_plugin_name", "multirotor_robot_model")
        pose = Pose(); pose.position.z = 1; pose.orientation.w = 1
        state = {"pose": pose, "stream": False, "hover": False, "track": False, "refs": [], "nav": [], "status": None}
        lock = threading.Lock()

        def reference(message):
            with lock:
                state["refs"].append(message)
                if state["track"]:
                    state["pose"] = message.pose.pose

        def status(message):
            with lock:
                state["status"] = message.status[0] if message.status else None

        subscribers = [rospy.Subscriber(private + "/reference/body", Odometry, reference),
                       rospy.Subscriber(private + "/status", DiagnosticArray, status),
                       rospy.Subscriber(ns + "/uav/nav", FlightNav, lambda m: state["nav"].append(m))]
        odom = rospy.Publisher(ns + "/uav/baselink/odom", Odometry, queue_size=1)
        cog = rospy.Publisher(ns + "/uav/cog/odom", Odometry, queue_size=1)
        flight = rospy.Publisher(ns + "/flight_state", UInt8, queue_size=1)
        command = rospy.Publisher(private + "/command/increment", TwistStamped, queue_size=1)

        def publish(_):
            with lock:
                if not state["stream"]:
                    return
                msg = Odometry(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = "world"
                msg.child_frame_id = ns[1:] + "/body"; msg.pose.pose = state["pose"]
                odom.publish(msg)
                msg.child_frame_id = ns[1:] + "/cog"; cog.publish(msg)
                flight.publish(UInt8(5 if state["hover"] else 0))

        def wait_for(predicate, timeout=6):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.assertIsNone(process.poll(), "motion node exited unexpectedly")
                if predicate():
                    return
                time.sleep(.02)
            self.fail("timed out; last status: " + str(state["status"]))

        process = subprocess.Popen(["rosrun", "aerial_robot_motion", "aerial_robot_motion_node",
                                    "__ns:=" + ns, "__name:=motion"], start_new_session=True)
        timer = rospy.Timer(rospy.Duration(.025), publish)
        try:
            rospy.wait_for_service(private + "/perching/enable", timeout=10)
            perch = rospy.ServiceProxy(private + "/perching/enable", SetBool)
            self.assertFalse(perch(True).success)  # Cannot lock before measured state.
            state["stream"] = True
            wait_for(lambda: state["status"] is not None)
            time.sleep(.25)
            self.assertEqual(len(state["refs"]), 0)  # Flight state gates coupled output.
            state["hover"] = True
            wait_for(lambda: len(state["refs"]) > 3 and command.get_num_connections() > 0)
            self.assertTrue(state["nav"])
            self.assertEqual(state["nav"][-1].target, FlightNav.COG)
            state["track"] = True
            cmd = TwistStamped(); cmd.header.stamp = rospy.Time.now(); cmd.header.frame_id = "world"
            cmd.twist.linear.x = .03; command.publish(cmd)
            wait_for(lambda: state["pose"].position.x > .01)
            self.assertTrue(perch(True).success)
            cmd = TwistStamped(); cmd.header.stamp = rospy.Time.now(); cmd.header.frame_id = "tool"
            cmd.twist.angular.y = -math.radians(3); command.publish(cmd)
            wait_for(lambda: state["pose"].orientation.y < -.005)
            values = {entry.key: entry.value for entry in state["status"].values}
            self.assertEqual(int(values["constraints"]), 5)
            self.assertLess(float(values["contact_position_error"]), .005)
            self.assertLess(float(values["contact_angle_error"]), .02)
            self.assertTrue(perch(False).success)
            state["stream"] = False
            time.sleep(.35)
            count = len(state["refs"])
            time.sleep(.15)
            self.assertEqual(len(state["refs"]), count)
            self.assertNotEqual(state["status"].level, 0)
            self.assertIn("stale", state["status"].message)
        finally:
            timer.shutdown()
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
            for subscriber in subscribers:
                subscriber.unregister()
            rospy.delete_param(ns)


if __name__ == "__main__":
    rospy.init_node("motion_runtime_smoke")
    rostest.rosrun("aerial_robot_motion", "runtime_smoke", RuntimeSmoke)
