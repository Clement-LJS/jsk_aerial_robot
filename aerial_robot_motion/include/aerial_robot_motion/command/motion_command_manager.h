#pragma once

#include <aerial_robot_motion/core/motion_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

namespace aerial_robot_motion::command
{
// Commands change a persistent nominal target; measured state defines command axes.
class MotionCommandManager
{
public:
  void initialize(ros::NodeHandle nh, tf2_ros::Buffer& tf, const std::string& world_frame);
  void setMeasuredPose(const Eigen::Isometry3d& pose, const ros::Time& stamp);
  bool ready() const { return ready_; }
  const Eigen::Isometry3d& target() const { return nominal_; }
  void resetTarget(const Eigen::Isometry3d& pose);

  // Pure command math, also usable by non-ROS clients. Angular increments are radians.
  void applyIncrement(const Vector6& increment, const Eigen::Matrix3d& world_R_command);
  void setAbsolutePose(const Eigen::Isometry3d& world_T_target);

private:
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& message);
  void incrementCallback(const geometry_msgs::TwistStampedConstPtr& message);
  void requireFreshMeasurement() const;
  Eigen::Isometry3d commandFrame(const std::string& frame, const ros::Time& stamp) const;

  tf2_ros::Buffer* tf_ = nullptr;
  ros::Subscriber pose_subscriber_, increment_subscriber_;
  std::string world_frame_ = "world", default_frame_ = "tool";
  double measured_timeout_ = 0.25;
  double max_translation_increment_ = 0.1, max_rotation_increment_ = 0.35;
  Eigen::Isometry3d measured_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d nominal_ = Eigen::Isometry3d::Identity();
  ros::Time measured_stamp_;
  bool ready_ = false;
};
}  // namespace aerial_robot_motion::command
