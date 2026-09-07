#pragma once

#include <aerial_robot_motion/core/motion_state.h>
#include <aerial_robot_model/model/aerial_robot_model.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <map>

namespace aerial_robot_motion
{
/** A private model instance: only this adapter updates its kinematic state. */
class ModelAdapter
{
public:
  ModelAdapter();
  void initialize(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  void initializeModel(const boost::shared_ptr<aerial_robot_model::RobotModel>& model,
                       const ros::NodeHandle& private_nh);
  const ModelInfo& info() const { return info_; }
  std::string baselink() const { return model_->getBaselinkName(); }
  bool needsJointState() const { return !required_joints_.empty(); }
  bool readJoints(const sensor_msgs::JointState& message, KDL::JntArray& full,
                  std::string& error) const;
  void setCogDesiredOrientation(const Eigen::Matrix3d& cog_R_baselink);
  MotionState measured(const Eigen::Isometry3d& world_T_baselink,
                       const KDL::JntArray& joints, const ros::Time& stamp);
  MotionState integrate(const MotionState& measured, const Eigen::VectorXd& velocity,
                        double dt, const ros::Time& stamp);
  Eigen::Isometry3d framePose(const MotionState& state, const std::string& segment,
                             const Eigen::Isometry3d& offset) const;
  Eigen::MatrixXd frameJacobian(const MotionState& state, const std::string& segment,
                               const Eigen::Isometry3d& offset) const;

private:
  void update(MotionState& state);
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> loader_;
  boost::shared_ptr<aerial_robot_model::RobotModel> model_;
  ModelInfo info_;
  std::string tool_frame_, contact_frame_;
  Eigen::Isometry3d tool_offset_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d contact_offset_ = Eigen::Isometry3d::Identity();
  std::map<std::string, int> required_joints_;
  std::map<int, double> fixed_joints_;
};
}  // namespace aerial_robot_motion
