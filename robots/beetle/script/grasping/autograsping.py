#!/usr/bin/env python
import tf, math, rospy
import numpy as np
from spinal.msg import ServoControlCmd, ServoTorqueCmd, ServoStates, ServoTorqueStates

def __init__(self):
    rospy.init_node("autoGrasping")

    # Parameters
    self.currentServoLoad = np.array([0.0])
    self.targetServoIndex = np.array([0.0])
    self.targetServoPosition = np.array([0.0])
    
    # ros
    self.sub = rospy.Subscriber('/beetle1/servo/states', ServoStates, self.callback)
    self.pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size = 10)
    self.rate = rospy.Rate(40)

def sendGraspingCommand(self):
    rospy.sleep(0.5)
    cmd = ServoControlCmd()
    cmd.index = [self.targetServoIndex]
    cmd.angles = [self.targetServoPosition]
    pub.publish(cmd)
    print("\033[1;32m[Msg] Hand command sent.\033[0m")

def callback(self, data):
    if data.
    currentServoPos = 

def main():
    rospy.init_node('add_module_client')
    rospy.sleep(1)

    link_name = "dummy_gripper_1"

    print("\033[1mEnter the module name: \033[0m")
    module_name = input().strip().lower()
    model_path = f'/home/iida/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/models/real/{module_name}_model.sdf'
    if not os.path.exists(model_path):
        print(f"\033[1;91m[Warning] '{module_name}' model not found. Please enter a valid module name.\033[0m")
        return

    model_sdf, config = spawn_object(module_name, model_path, link_name)
    if model_sdf is None:
        return

    while True:
        print("\033[1mEnter add or delete: \033[0m")
        operation = input().strip().lower()
        
        if operation == "add":
            action = 1
            if call_add_extra_module(action, link_name, config):
                print(f"\033[1;32m[Msg] {module_name} model added successfully!\033[0m")
            else:
                print("\033[1;91m[Error] Failed to add module.\033[0m")
            break

        elif operation == "delete":
            action = -1
            send_openhand_command()
            if call_add_extra_module(action, link_name, config):
                print(f"\033[1;32m[Msg] {module_name} model deleted successfully!\033[0m")
            else:
                print("\033[1;91m[Error] Failed to delete module.\033[0m")
            break

        else:
            print("\033[1;91mInvalid operation. Please enter 'add' or 'delete'.\033[0m")

if __name__ == "__main__":
    main()
