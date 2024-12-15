#!/usr/bin/env python
import rospy, sys
import numpy as np
from spinal.msg import ServoControlCmd, ServoStates

class AutoGrasping:
    def __init__(self):
        rospy.init_node("autoGrasping")

        # Prepared value
        self.limitLoadRange = np.array([20, 50])
        self.servoMoveRange = np.array([50])
    
        # Parameters
        self.currentServoLoad_0 = 0.0
        self.currentServoPosition_0 = 0.0
        self.targetServoPosition_0 = 0.0

        self.currentServoLoad_1 = 0.0
        self.currentServoPosition_1 = 0.0
        self.targetServoPosition_1 = 0.0

        # Flags
        self.targetReachedFlag = False
        self.handOpenFlag_0 = rospy.get_param('handOpenFlag_0')
        self.handOpenFlag_1 = rospy.get_param('handOpenFlag_1')

        # ROS
        self.last_callback_time = rospy.get_time()
        self.sub = rospy.Subscriber('/servo/states', ServoStates, self.callback, queue_size=10)
        self.pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=10)
        self.rate = rospy.Rate(40)

    def check_subscription(self):
        current_time = rospy.get_time()
        if current_time - self.last_callback_time > 5.0:
            print(f"\033[1;91m[Error] No servo data!! \033[0m") 
        
    def calculatePosition(self, targetServoIndex):
        if targetServoIndex == 0:
            if self.limitLoadRange[0] > self.currentServoLoad_0:
                if self.handOpenFlag_0:
                    self.targetServoPosition_0 = int(self.currentServoPosition_0 - self.servoMoveRange)
                else:
                    self.targetServoPosition_0 = int(self.currentServoPosition_0 + self.servoMoveRange)
            elif self.limitLoadRange[0] <= self.currentServoLoad_0 < self.limitLoadRange[1]:
                self.handOpenFlag_0 = not self.handOpenFlag_0
                rospy.set_param('handOpenFlag_0', self.handOpenFlag_0)                
                self.targetReachedFlag = True
            else:
                if self.handOpenFlag_0:
                    self.targetServoPosition_0 = int(self.currentServoPosition_0 + self.servoMoveRange)
                else:
                    self.targetServoPosition_0 = int(self.currentServoPosition_0 - self.servoMoveRange)                
            self.sendGraspingCommand(targetServoIndex)
        
        elif targetServoIndex == 1:
            if self.limitLoadRange[0] > self.currentServoLoad_1:
                if self.handOpenFlag_1:
                    self.targetServoPosition_1 = int(self.currentServoPosition_1 - self.servoMoveRange)
                else:
                    self.targetServoPosition_1 = int(self.currentServoPosition_1 + self.servoMoveRange)
            elif self.limitLoadRange[0] <= self.currentServoLoad_1 < self.limitLoadRange[1]:
                self.handOpenFlag_1 = not self.handOpenFlag_1
                rospy.set_param('handOpenFlag_1', self.handOpenFlag_1)
                self.targetReachedFlag = True

            else:
                if self.handOpenFlag_1:
                    self.targetServoPosition_1 = int(self.currentServoPosition_1 + self.servoMoveRange)
                else:
                    self.targetServoPosition_1 = int(self.currentServoPosition_1 - self.servoMoveRange)                
                
            self.sendGraspingCommand(targetServoIndex)

    def sendGraspingCommand(self, targetServoIndex):
        cmd = ServoControlCmd()
        if targetServoIndex == 0:
            cmd.index = [0]
            cmd.angles = [int(self.targetServoPosition_0)]
        else:
            cmd.index = [1]
            cmd.angles = [int(self.targetServoPosition_1)]
        self.pub.publish(cmd)
            
    def callback(self, data):
        self.last_callback_time = rospy.get_time()

        index0_data = next((servo for servo in data.servos if servo.index == 0), None)
        index1_data = next((servo for servo in data.servos if servo.index == 1), None)

        if index0_data:
            self.currentServoLoad_0 = abs(index0_data.load)
            self.currentServoPosition_0 = index0_data.angle
        if index1_data:
            self.currentServoLoad_1 = abs(index1_data.load)
            self.currentServoPosition_1 = index1_data.angle

        # print("nyanyanyanya")
            
    def main(self):
        while not rospy.is_shutdown():
            print("\033[1mEnter the servo index, + or -: \033[0m")
            try:
                targetInput = input().strip().lower()
            except KeyboardInterrupt:
                rospy.loginfo("Interrupted from keyboard.")
                rospy.signal_shutdown("")
                break
            
            if targetInput == "+":
                targetServoIndex = 1
                break
            elif targetInput == "-":
                targetServoIndex = 0
                break
            else:
                print("Incorrect input. Please enter + or -.")
                continue

        while not self.targetReachedFlag:
            self.check_subscription()
            self.calculatePosition(targetServoIndex)
            if targetServoIndex == 0 and self.targetServoPosition_0 == self.currentServoPosition_0:
                rospy.loginfo("Servo 0 reached target position. Exiting...")
                break
            elif targetServoIndex == 1 and self.targetServoPosition_1 == self.currentServoPosition_1:
                rospy.loginfo("Servo 1 reached target position. Exiting...")
                break
            self.rate.sleep()

        print(f"\033[1;32m[Msg] Target reached. Exiting... \033[0m")
        rospy.signal_shutdown("")

if __name__ == "__main__":
    try:
        auto_grasping = AutoGrasping()
        auto_grasping.main()
        rospy.spin()
    except rospy.ROSInterruptException:
        sys.exit(1)
    except KeyError:
        rospy.logerr(f"Error: hand flags not found!")
        sys.exit(1)
