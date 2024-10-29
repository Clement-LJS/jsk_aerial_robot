#!/usr/bin/env python

import os, tf, math, rospy
import xml.etree.ElementTree as ET
from geometry_msgs.msg import Transform, Quaternion, Pose, Point
from aerial_robot_model.srv import AddExtraModule, AddExtraModuleRequest

def euler_to_quaternion(roll, pitch, yaw):
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def spawn_object(module_name, model_path, link_name):        
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
            print(f"\033[1;32m[Msg] Initial pose/position set.\033[0m")

        else:
            print("\033[1;91m[Warn] No <pose> / <position> tag found in the model file.\033[0m")
            return None, None, None

        inertia_element = root.find('.//inertia')
        if inertia_element is not None:
            inertia = {}
            inertia['m'] = float(inertia_element.find('./mass').text)
            inertia['ixx'] = float(inertia_element.find('./ixx').text)
            inertia['ixy'] = float(inertia_element.find('./ixy').text)
            inertia['ixz'] = float(inertia_element.find('./ixz').text)
            inertia['iyy'] = float(inertia_element.find('./iyy').text)
            inertia['iyz'] = float(inertia_element.find('./iyz').text)
            inertia['izz'] = float(inertia_element.find('./izz').text)

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
        rospy.logerr("\033[1;91m[Warn] Service call failed: %s\033[0m" % e)
        return None, None, None

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
            print("\033[1mModule added successfully!\033[0m")
            return True
        else:
            print("\033[1mFailed to add module!\033[0m")
            return False
    except rospy.ServiceException as e:
        print("\033[1mService call failed!\033[0m")
        return False

def main():
    rospy.init_node('add_module_client')
    rospy.sleep(1)

    link_name = "dummy_gripper"

    print("\033[1mEnter the module name: \033[0m")
    module_name = input().strip().lower()
    model_path = f'/home/iida/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/models/{module_name}_model.sdf'
    if not os.path.exists(model_path):
        print(f"\033[1;91m[Warning] '{module_name}' model not found. Please enter a valid module name.\033[0m")
        return

    model_sdf, initial_position, config = spawn_object(module_name, model_path, link_name)
    if model_sdf is None:
        return

    action = 1
    if call_add_extra_module(action, link_name, config):
        print(f"\033[1;32m[Msg] {module_name} model processed successfully!\033[0m")
    else:
        print("\033[1;91m[Error] Failed to process the module.\033[0m")

if __name__ == "__main__":
    main()

