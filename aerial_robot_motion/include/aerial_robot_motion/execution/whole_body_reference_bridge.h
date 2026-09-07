#pragma once
#include <aerial_robot_motion/core/motion_state.h>
#include <ros/ros.h>
namespace aerial_robot_motion
{
class WholeBodyReferenceBridge
{
public:
  void initialize(ros::NodeHandle robot_nh, ros::NodeHandle private_nh, const ModelInfo& info,
                  const std::string& world);
  void publish(const WholeBodyReference& reference, bool execute);
  bool fullAttitude() const { return full_attitude_; }
private:
  ModelInfo info_;
  std::string world_;
  bool full_attitude_ = false;
  ros::Publisher body_, joints_, navigation_, joint_command_;
};
}
