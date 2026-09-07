#include <aerial_robot_motion/core/motion_core.h>
int main(int argc, char** argv)
{
  ros::init(argc, argv, "aerial_robot_motion");
  try
  {
    aerial_robot_motion::MotionCore node(ros::NodeHandle{}, ros::NodeHandle{"~"});
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL("Motion generation could not start: %s", error.what());
    return 1;
  }
  return 0;
}
