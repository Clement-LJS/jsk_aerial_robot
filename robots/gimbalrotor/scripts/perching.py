#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import select
import termios
import tty
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import rospkg
import rospy
from geometry_msgs.msg import PoseStamped, QuaternionStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


msg = """
Perched Pitch-Joint Control using URDF geometry
--------------------------------------------------
Keyboard:
  z : lock current perch pose in world
  i : pitch joint +
  k : pitch joint -
  x : reset target joint to locked joint angle
  p : print current status
  q : quit

Behavior:
  - Only the pitch joint is commanded by keyboard
  - The DRONE / BASE pose is computed and published
  - The robot pitches about the perch / pitch-joint mechanism,
    not about the CoG
--------------------------------------------------
Publishes:
  /gimbalrotor/target_pose
  /gimbalrotor/final_target_baselink_rot
  /gimbalrotor/final_target_baselink_rpy
  /gimbalrotor/joints_ctrl
--------------------------------------------------
"""


current_base_pos = [0.0, 0.0, 0.0]
current_base_quat = [0.0, 0.0, 0.0, 1.0]
got_odom = False

current_joint_angle = 0.0
got_joint = False


def odom_cb(msg):
    global current_base_pos, current_base_quat, got_odom
    current_base_pos = [
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    ]
    current_base_quat = [
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    ]
    got_odom = True


def joint_states_cb(msg, pitch_joint_name):
    global current_joint_angle, got_joint
    try:
        idx = msg.name.index(pitch_joint_name)
        current_joint_angle = msg.position[idx]
        got_joint = True
    except ValueError:
        pass


# blocking keyboard style
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def printMsg(text, msg_len=170):
    print(text.ljust(msg_len) + "\r", end="")


def parse_xyz(xyz_str):
    return [float(v) for v in xyz_str.strip().split()]


def parse_rpy(rpy_str):
    return [float(v) for v in rpy_str.strip().split()]


def normalize(v):
    n = math.sqrt(sum(x * x for x in v))
    if n < 1e-12:
        raise ValueError("Cannot normalize near-zero vector")
    return [x / n for x in v]


def rot_x(a):
    c = math.cos(a)
    s = math.sin(a)
    return [
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ]


def rot_y(a):
    c = math.cos(a)
    s = math.sin(a)
    return [
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
    ]


def rot_z(a):
    c = math.cos(a)
    s = math.sin(a)
    return [
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0],
    ]


def rpy_to_rot(roll, pitch, yaw):
    return matmul(matmul(rot_z(yaw), rot_y(pitch)), rot_x(roll))


def rot_to_rpy(R):
    # ZYX convention
    sy = -R[2][0]
    if abs(sy) < 1.0 - 1e-9:
        pitch = math.asin(sy)
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        # gimbal lock fallback
        pitch = math.copysign(math.pi / 2.0, sy)
        roll = 0.0
        yaw = math.atan2(-R[0][1], R[1][1])
    return [roll, pitch, yaw]


def quat_to_rot(x, y, z, w):
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz),       2.0 * (xz + wy)],
        [2.0 * (xy + wz),       1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy),       2.0 * (yz + wx),       1.0 - 2.0 * (xx + yy)],
    ]


def rot_to_quat(R):
    trace = R[0][0] + R[1][1] + R[2][2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2][1] - R[1][2]) / s
        y = (R[0][2] - R[2][0]) / s
        z = (R[1][0] - R[0][1]) / s
    elif (R[0][0] > R[1][1]) and (R[0][0] > R[2][2]):
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s

    return [x, y, z, w]


def axis_angle_rot(axis, angle):
    ax = normalize(axis)
    x, y, z = ax
    c = math.cos(angle)
    s = math.sin(angle)
    C = 1.0 - c

    return [
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C    ],
    ]


def transpose(A):
    return [
        [A[0][0], A[1][0], A[2][0]],
        [A[0][1], A[1][1], A[2][1]],
        [A[0][2], A[1][2], A[2][2]],
    ]


def matmul(A, B):
    out = [[0.0] * len(B[0]) for _ in range(len(A))]
    for i in range(len(A)):
        for j in range(len(B[0])):
            s = 0.0
            for k in range(len(B)):
                s += A[i][k] * B[k][j]
            out[i][j] = s
    return out


def matvec(A, v):
    return [
        A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
        A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
        A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2],
    ]


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def get_joint_info(root, joint_name):
    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            parent_elem = joint.find("parent")
            child_elem = joint.find("child")
            origin_elem = joint.find("origin")
            axis_elem = joint.find("axis")

            parent_link = parent_elem.get("link") if parent_elem is not None else None
            child_link = child_elem.get("link") if child_elem is not None else None

            if origin_elem is not None:
                xyz = parse_xyz(origin_elem.get("xyz", "0 0 0"))
                rpy = parse_rpy(origin_elem.get("rpy", "0 0 0"))
            else:
                xyz = [0.0, 0.0, 0.0]
                rpy = [0.0, 0.0, 0.0]

            if axis_elem is not None:
                axis = parse_xyz(axis_elem.get("xyz", "0 0 1"))
            else:
                axis = [0.0, 0.0, 1.0]

            return {
                "name": joint_name,
                "parent": parent_link,
                "child": child_link,
                "xyz": xyz,
                "rpy": rpy,
                "axis": axis,
            }

    raise ValueError("Joint '{}' not found in URDF".format(joint_name))


def load_urdf_root():
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("gimbalrotor")

    xacro_file = os.path.join(pkg_path, "urdf", "beetle", "gimbalrotor.urdf.xacro")
    rospy.loginfo("Loading xacro file: %s", xacro_file)

    tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".urdf")
    tmp.close()

    with open(tmp.name, "w") as f:
        subprocess.run(["xacro", xacro_file], stdout=f, check=True)

    tree = ET.parse(tmp.name)
    root = tree.getroot()

    rospy.loginfo("URDF loaded successfully from expanded xacro")

    if os.path.exists(tmp.name):
        os.remove(tmp.name)

    return root


def compute_urdf_geometry(root,
                          base_to_handbase_joint_name,
                          handbase_to_pitchpipe_joint_name,
                          pitch_joint_name,
                          perch_offset_in_joint_zero):
    j_base_to_handbase = get_joint_info(root, base_to_handbase_joint_name)
    j_handbase_to_pitchpipe = get_joint_info(root, handbase_to_pitchpipe_joint_name)
    j_pitch = get_joint_info(root, pitch_joint_name)

    t_b_hb = j_base_to_handbase["xyz"]
    R_b_hb = rpy_to_rot(*j_base_to_handbase["rpy"])

    t_hb_pjl = j_handbase_to_pitchpipe["xyz"]
    R_hb_pjl = rpy_to_rot(*j_handbase_to_pitchpipe["rpy"])

    t_pjl_pj = j_pitch["xyz"]
    R_pjl_pj = rpy_to_rot(*j_pitch["rpy"])

    p_pitch_in_handbase = vec_add(t_hb_pjl, matvec(R_hb_pjl, t_pjl_pj))
    p_pitch_in_base = vec_add(t_b_hb, matvec(R_b_hb, p_pitch_in_handbase))

    R_b_pjframe0 = matmul(matmul(R_b_hb, R_hb_pjl), R_pjl_pj)
    joint_axis_local = normalize(j_pitch["axis"])

    perch_zero_in_base = vec_add(
        p_pitch_in_base,
        matvec(R_b_pjframe0, perch_offset_in_joint_zero)
    )

    return p_pitch_in_base, R_b_pjframe0, joint_axis_local, perch_zero_in_base


def compute_base_to_perch_position(p_pitch_in_base,
                                   R_b_pjframe0,
                                   joint_axis_local,
                                   perch_offset_in_joint_zero,
                                   joint_angle):
    R_joint = axis_angle_rot(joint_axis_local, joint_angle)
    rotated_local_offset = matvec(R_joint, perch_offset_in_joint_zero)
    return vec_add(p_pitch_in_base, matvec(R_b_pjframe0, rotated_local_offset))


def compute_base_to_perch_orientation(R_b_pjframe0,
                                      joint_axis_local,
                                      joint_angle):
    R_joint = axis_angle_rot(joint_axis_local, joint_angle)
    return matmul(R_b_pjframe0, R_joint)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("perched_pitch_from_urdf")
    print(msg)

    robot_ns = rospy.get_param("~robot_ns", "/gimbalrotor")

    odom_topic = rospy.get_param("~odom_topic", robot_ns + "/uav/baselink/odom")
    target_pose_topic = rospy.get_param("~target_pose_topic", robot_ns + "/target_pose")
    baselink_rot_topic = rospy.get_param("~baselink_rot_topic", robot_ns + "/final_target_baselink_rot")
    baselink_rpy_topic = rospy.get_param("~baselink_rpy_topic", robot_ns + "/final_target_baselink_rpy")
    joints_topic = rospy.get_param("~joints_topic", robot_ns + "/joints_ctrl")
    joint_states_topic = rospy.get_param("~joint_states_topic", robot_ns + "/joint_states")

    pitch_joint_name = rospy.get_param("~pitch_joint_name", "pitch_joint")
    base_to_handbase_joint_name = rospy.get_param("~base_to_handbase_joint_name", "handbase_baselink_joint")
    handbase_to_pitchpipe_joint_name = rospy.get_param("~handbase_to_pitchpipe_joint_name", "handbase_pitchpipe_joint")

    # pitch joint to perch point at theta=0, x=10 cm
    perch_offset_x = rospy.get_param("~perch_offset_x", 0.25754)
    perch_offset_y = rospy.get_param("~perch_offset_y", 0.02932)
    perch_offset_z = rospy.get_param("~perch_offset_z", 0.06664)
    perch_offset_in_joint_zero = [perch_offset_x, perch_offset_y, perch_offset_z]

    joint_step_deg = rospy.get_param("~joint_step_deg", 1.0)
    joint_min_deg = rospy.get_param("~joint_min_deg", -60.0)
    joint_max_deg = rospy.get_param("~joint_max_deg", 60.0)

    joint_step = math.radians(joint_step_deg)
    joint_min = math.radians(joint_min_deg)
    joint_max = math.radians(joint_max_deg)

    rospy.Subscriber(odom_topic, Odometry, odom_cb, queue_size=1)
    rospy.Subscriber(joint_states_topic, JointState, joint_states_cb,
                     callback_args=pitch_joint_name, queue_size=1)

    pose_pub = rospy.Publisher(target_pose_topic, PoseStamped, queue_size=1)
    baselink_rot_pub = rospy.Publisher(baselink_rot_topic, QuaternionStamped, queue_size=1)
    baselink_rpy_pub = rospy.Publisher(baselink_rpy_topic, Vector3Stamped, queue_size=1)
    joints_pub = rospy.Publisher(joints_topic, JointState, queue_size=1)

    rospy.loginfo("Waiting for odometry and joint state ...")
    while not rospy.is_shutdown() and (not got_odom or not got_joint):
        rospy.sleep(0.05)

    root = load_urdf_root()
    p_pitch_in_base, R_b_pjframe0, joint_axis_local, perch_zero_in_base = compute_urdf_geometry(
        root,
        base_to_handbase_joint_name,
        handbase_to_pitchpipe_joint_name,
        pitch_joint_name,
        perch_offset_in_joint_zero
    )

    rospy.loginfo("URDF geometry loaded.")
    rospy.loginfo("Pitch joint origin in base frame: [%.6f, %.6f, %.6f]",
                  p_pitch_in_base[0], p_pitch_in_base[1], p_pitch_in_base[2])
    rospy.loginfo("Joint axis in local frame: [%.6f, %.6f, %.6f]",
                  joint_axis_local[0], joint_axis_local[1], joint_axis_local[2])
    rospy.loginfo("Perch point at zero angle in base frame: [%.6f, %.6f, %.6f]",
                  perch_zero_in_base[0], perch_zero_in_base[1], perch_zero_in_base[2])

    target_joint_angle = current_joint_angle
    locked_joint_angle = current_joint_angle

    perch_locked = False
    world_perch_point = None
    world_perch_rot = None

    try:
        while True:
            key = getKey()
            text = ""

            if key == 'z':
                R_wb_lock = quat_to_rot(
                    current_base_quat[0],
                    current_base_quat[1],
                    current_base_quat[2],
                    current_base_quat[3]
                )
                p_wb_lock = list(current_base_pos)

                theta_lock = current_joint_angle
                locked_joint_angle = theta_lock
                target_joint_angle = theta_lock

                p_bp_lock = compute_base_to_perch_position(
                    p_pitch_in_base,
                    R_b_pjframe0,
                    joint_axis_local,
                    perch_offset_in_joint_zero,
                    theta_lock
                )

                R_b_perch_lock = compute_base_to_perch_orientation(
                    R_b_pjframe0,
                    joint_axis_local,
                    theta_lock
                )

                # lock the perch in world
                world_perch_point = vec_add(p_wb_lock, matvec(R_wb_lock, p_bp_lock))
                world_perch_rot = matmul(R_wb_lock, R_b_perch_lock)
                perch_locked = True

                text = "perch locked at world point ({:.3f}, {:.3f}, {:.3f})".format(
                    world_perch_point[0], world_perch_point[1], world_perch_point[2]
                )

            elif key == 'i':
                target_joint_angle += joint_step
                if target_joint_angle > joint_max:
                    target_joint_angle = joint_max
                text = "pitch joint +"

            elif key == 'k':
                target_joint_angle -= joint_step
                if target_joint_angle < joint_min:
                    target_joint_angle = joint_min
                text = "pitch joint -"

            elif key == 'x':
                target_joint_angle = locked_joint_angle if perch_locked else current_joint_angle
                text = "reset target joint"

            elif key == 'p':
                if perch_locked:
                    text = "locked={} joint[deg]={:.2f} perch=({:.3f}, {:.3f}, {:.3f})".format(
                        perch_locked,
                        math.degrees(target_joint_angle),
                        world_perch_point[0], world_perch_point[1], world_perch_point[2]
                    )
                else:
                    text = "locked={} joint[deg]={:.2f} (press z to lock perch)".format(
                        perch_locked,
                        math.degrees(target_joint_angle)
                    )

            elif key == 'q' or key == '\x03':
                break

            now = rospy.Time.now()

            # always publish pitch joint command
            joint_msg = JointState()
            joint_msg.header.stamp = now
            joint_msg.name = [pitch_joint_name]
            joint_msg.position = [target_joint_angle]
            joint_msg.velocity = []
            joint_msg.effort = []
            joints_pub.publish(joint_msg)

            if perch_locked:
                # compute DRONE / BASE target orientation
                R_b_perch = compute_base_to_perch_orientation(
                    R_b_pjframe0,
                    joint_axis_local,
                    target_joint_angle
                )

                # body orientation in world
                R_wb_target = matmul(world_perch_rot, transpose(R_b_perch))

                # compute DRONE / BASE target position
                p_bp = compute_base_to_perch_position(
                    p_pitch_in_base,
                    R_b_pjframe0,
                    joint_axis_local,
                    perch_offset_in_joint_zero,
                    target_joint_angle
                )

                p_wb_target = vec_sub(world_perch_point, matvec(R_wb_target, p_bp))

                quat = rot_to_quat(R_wb_target)
                rpy = rot_to_rpy(R_wb_target)

                # publish base pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = now
                pose_msg.header.frame_id = "world"
                pose_msg.pose.position.x = p_wb_target[0]
                pose_msg.pose.position.y = p_wb_target[1]
                pose_msg.pose.position.z = p_wb_target[2]
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]
                pose_pub.publish(pose_msg)

                # publish base orientation explicitly
                rot_msg = QuaternionStamped()
                rot_msg.header.stamp = now
                rot_msg.header.frame_id = "world"
                rot_msg.quaternion.x = quat[0]
                rot_msg.quaternion.y = quat[1]
                rot_msg.quaternion.z = quat[2]
                rot_msg.quaternion.w = quat[3]
                baselink_rot_pub.publish(rot_msg)

                rpy_msg = Vector3Stamped()
                rpy_msg.header.stamp = now
                rpy_msg.header.frame_id = "world"
                rpy_msg.vector.x = rpy[0]
                rpy_msg.vector.y = rpy[1]
                rpy_msg.vector.z = rpy[2]
                baselink_rpy_pub.publish(rpy_msg)

                if text:
                    printMsg(
                        "{} | joint[deg]={:.2f} | drone_rpy[deg]=({:.2f}, {:.2f}, {:.2f}) | target_pos=({:.3f}, {:.3f}, {:.3f})".format(
                            text,
                            math.degrees(target_joint_angle),
                            math.degrees(rpy[0]),
                            math.degrees(rpy[1]),
                            math.degrees(rpy[2]),
                            p_wb_target[0], p_wb_target[1], p_wb_target[2]
                        )
                    )
            else:
                if text:
                    printMsg("{} | joint[deg]={:.2f}".format(
                        text,
                        math.degrees(target_joint_angle)
                    ))

            rospy.sleep(0.001)

    except Exception as e:
        print(repr(e))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)