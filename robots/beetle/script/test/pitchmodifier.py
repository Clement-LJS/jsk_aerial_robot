#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord

class PitchModifier:
    def __init__(self):
        rospy.init_node("pitchModifier", anonymous=True)

        self.vel = 0.3
        self.targetPitch = -0.3
        self.targetYaw = 0.3
        
        self.pub = rospy.Publisher('/beetle1/uav/nav', FlightNav, queue_size=10)
        self.rot_pub = rospy.Publisher('/beetle1/final_target_baselink_rot', DesireCoord, queue_size=10)
        self.rate = rospy.Rate(40)

        self.nav_msg = FlightNav()
        self.rot_msg = DesireCoord()

        self.switch_direction = True
        self.timer = rospy.Timer(rospy.Duration(3), self.timer_callback)

    def timer_callback(self, event):
        if self.switch_direction:
            self.vel = 0.2
        else:
            self.vel = -0.2
        self.switch_direction = not self.switch_direction
        print("Switching...")

    def main(self):
        while not rospy.is_shutdown():
            try:
                self.nav_msg.control_frame = 0
                self.nav_msg.target = 0
                self.nav_msg.pos_xy_nav_mode = 1
                self.nav_msg.target_vel_x = self.vel
                self.nav_msg.pos_z_nav_mode = 1
                self.pub.publish(self.nav_msg)

                self.rot_msg.pitch = self.targetPitch            
                self.rot_pub.publish(self.rot_msg)

                self.rate.sleep()
            except rospy.ROSInterruptException:
                break

if __name__ == "__main__":
    try:        
        pitch_modifier = PitchModifier()
        pitch_modifier.main()
    except rospy.ROSInterruptException:
        pass
