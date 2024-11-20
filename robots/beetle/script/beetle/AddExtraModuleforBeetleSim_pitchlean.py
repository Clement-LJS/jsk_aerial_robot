#!/usr/bin/env python

import os
import tf
import math
import rospy
import xml.etree.ElementTree as ET
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform, Quaternion, Pose, Point
from std_srvs.srv import SetBool
from aerial_robot_model.srv import AddExtraModule, AddExtraModuleRequest
from gazebo_msgs.srv import SpawnModel, DeleteModel
from spinal.msg import Imu


def euler_to_quaternion(roll, pitch, yaw):
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def get_pitch_angle_from_imu(imu_data):
    # Extract the pitch angle from the IMU quaternion
    orientation = imu_data.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    _, pitch, _ = tf.transformations.euler_from_quaternion(quaternion)
    pitch_deg = math.degrees(pitch)
    return pitch_deg


def spawn_object(module_name, model_path, link_name):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        tree = ET.parse(model_path)
        root = tree.getroot()
        pose_element = root.find('.//pose')
        position_element = root.find('.//position')

        if pose_element is not None:
            pose_values = list(map(float, pose_element.text.split()))
            position_values = list(map(float, position_element.text.split()))
            x, y, z = pose_values[0:3]
            roll, pitch, yaw = pose_values[3:6]
            position_x, position_y, position_z = position_values[0:3]

            pose = Point(x, y, z)
            position = Point(position_x, position_y, position_z)
            orientation = euler_to_quaternion(roll, pitch, yaw)
            initial_pose = Pose(pose, orientation)
            initial_position = Pose(position, orientation)
            print(f"\033[1;32m[Msg] Initial pose/position set.")

        else:
            print("\033[1;91m[Warn] No <pose> / <position> tag found in the model file.\033[0m")
            return None, None, None

        inertia_element = root.find('.//inertia')
        if inertia_element is not None:
            inertia = {
                'm': float(inertia_element.find('./mass').text),
                'ixx': float(inertia_element.find('./ixx').text),
                'ixy': float(inertia_element.find('./ixy').text),
                'ixz': float(inertia_element.find('./ixz').text),
                'iyy': float(inertia_element.find('./iyy').text),
                'iyz': float(inertia_element.find('./iyz').text),
                'izz': float(inertia_element.find('./izz').text),
            }
            print(f"\033[1;32m[Msg] Inertia set.\033[0m")
        else:
            print("\033[1;91m[Warn] No <inertia> tag found in the model file.\033[0m")
            return None, None, None

        config = {
            'module_name': module_name,
            'transform': Transform(translation=pose, rotation=orientation),
            'inertia': inertia
        }

        with open(model_path, 'r') as model_file:
            model_sdf = model_file.read()

        return model_sdf, initial_position, config

    except rospy.ServiceException as e:
        rospy.logerr("\033[1;91m[Warn] Spawn service call failed: %s\033[0m" % e)
        return None, None, None


def delete_object(model_name):
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        response = delete_model_service(model_name)
        if response.success:
            print(f"\033[1;32m[Msg] {model_name} has been deleted successfully!\033[0m")
        else:
            print(f"\033[1;91m[Warning] Failed to delete {model_name}.\033[0m")
    except rospy.ServiceException as e:
        print(f"\033[1;91m[Warning] Service call failed: {e}\033[0m")


def call_add_extra_module(action, link_name, config):
    rospy.wait_for_service('/beetle1/add_extra_module')
    try:
        add_extra_module = rospy.ServiceProxy('/beetle1/add_extra_module', AddExtraModule)

        request = AddExtraModuleRequest()
        request.action = action
        request.module_name = config['module_name']
        request.parent_link_name = link_name
        request.transform = config['transform']
        request.inertia.m = config['inertia']['m']
        request.inertia.ixx = config['inertia']['ixx']
        request.inertia.ixy = config['inertia']['ixy']
        request.inertia.ixz = config['inertia']['ixz']
        request.inertia.iyy = config['inertia']['iyy']
        request.inertia.iyz = config['inertia']['iyz']
        request.inertia.izz = config['inertia']['izz']

        response = add_extra_module(request)
        if response.status:
            print("\033[1;32m[Msg] Module added/deleted successfully!\033[0m")
            return True
        else:
            print("\033[1;91m[Error] Failed to add/delete module.\033[0m")
            return False
    except rospy.ServiceException as e:
        print("\033[1;91m[Error] Service call failed.\033[0m")
        return False


pub = rospy.Publisher('/beetle1/docking_cmd', Bool, queue_size=10)


def send_docking_command(is_docking):
    rospy.sleep(0.5)
    pub.publish(is_docking)
    rospy.sleep(0.5)


def monitor_imu():
    imu_data = rospy.wait_for_message('/beetle1/imu', Imu)
    pitch = imu_data.orientation.y
    return abs(pitch) > math.radians(5)  # 5度をラジアンに変換

def main():
    rospy.init_node('add_module_client')
    rospy.sleep(1)

    while True:
        print("\033[1mSelect parent link, plus or minus: \033[0m")
        link = input().strip().lower()
        if link == "plus":
            link_name = "dummy_gripper_1"
            break
        elif link == "minus":
            link_name = "dummy_gripper_2"
            break
        else:
            print(f"\033[1;91m[Warn] Wrong selection. Please select a valid link.\033[0m")
            continue

    while True:
        print("\033[1mEnter the module name: \033[0m")

        module_name = input().strip().lower()
        model_path = f'/home/iida/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/models/sim/{module_name}_model.sdf'
        if not os.path.exists(model_path):
            print(f"\033[1;91m[Warning] '{module_name}' model not found. Please enter a valid module name.\033[0m")
            continue
        else:
            model_sdf, initial_position, config = spawn_object(module_name, model_path, link_name)
            if model_sdf is None:
                continue
            break

    while True:
        print("\033[1mEnter add or delete: \033[0m")
        operation = input().strip().lower()
        
        if operation in ["add", "delete"]:
            action = 1 if operation == "add" else -1
            send_docking_command(action == 1)

            try:
                # モデルのスポーンまたは削除
                if operation == "add":
                    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                    spawn_model(module_name, model_sdf, 'obstacle_namespace', initial_position, link_name)
                else:
                    delete_object(module_name)


                # ピッチ角監視とサービス呼び出し
                print("nyanyanyanyanya!!")
                if monitor_imu():
                    if call_add_extra_module(action, link_name, config):
                        print(f"\033[1;32m[Msg] {module_name} module {'added' if action == 1 else 'deleted'} successfully!\033[0m")
                    else:
                        print(f"\033[1;91m[Error] Failed to {'add' if action == 1 else 'delete'} module. Please try again.\033[0m")
                        continue
                else:
                    print("\033[1;91m[Warn] Pitch angle is within the safe range. No action taken.\033[0m")
                    continue

                break

            except rospy.ServiceException as e:
                print(f"\033[1;91m[Warn] Service call failed: {e}\033[0m")
                continue
        
        else:
            print("\033[1;91mInvalid operation. Please enter 'add' or 'delete'.\033[0m")

if __name__ == "__main__":
    main()
