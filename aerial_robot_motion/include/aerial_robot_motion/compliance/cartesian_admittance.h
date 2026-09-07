#pragma once

#include <aerial_robot_motion/core/motion_state.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>

namespace aerial_robot_motion::compliance
{
struct AdmittanceParameters
{
  Vector6 mass = Vector6::Ones();
  Vector6 damping = Vector6::Constant(20.0);
  Vector6 stiffness = Vector6::Constant(50.0);
  Vector6 axes = Vector6::Zero();
  Vector6 displacement_limit = Vector6::Constant(0.05);
  Vector6 velocity_limit = Vector6::Constant(0.1);
  Vector6 disable_rate = Vector6::Constant(0.05);
  double max_dt = 0.1;
};

// Independent diagonal backward-Euler dynamics. State is expressed in compliance axes.
class AdmittanceDynamics
{
public:
  void configure(const AdmittanceParameters& parameters);
  void setEnabled(bool enabled);
  bool enabled() const { return enabled_; }
  void reset();
  void step(const Vector6& wrench, double dt);
  const Vector6& displacement() const { return displacement_; }
  const Vector6& velocity() const { return velocity_; }
  const AdmittanceParameters& parameters() const { return parameters_; }
  static Vector6 transformWrench(const Eigen::Isometry3d& target_T_source, const Vector6& source_wrench);

private:
  AdmittanceParameters parameters_;
  Vector6 displacement_ = Vector6::Zero(), velocity_ = Vector6::Zero();
  bool enabled_ = false;
};

class CartesianAdmittance
{
public:
  void initialize(ros::NodeHandle nh, tf2_ros::Buffer& tf, const std::string& world_frame);
  Eigen::Isometry3d update(const Eigen::Isometry3d& nominal, const Eigen::Isometry3d& measured_tool,
                           double dt, const ros::Time& now,
                           const ros::Time& measured_tool_stamp = ros::Time());
  Vector6 targetTwist() const { return Vector6::Zero(); }
  void reset();
  bool enabled() const { return dynamics_.enabled(); }
  const Vector6& displacement() const { return dynamics_.displacement(); }

private:
  void wrenchCallback(const geometry_msgs::WrenchStampedConstPtr& message);
  bool enableCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  bool resetCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);
  Eigen::Isometry3d worldFrame(const std::string& frame, const ros::Time& stamp,
                               const Eigen::Isometry3d& tool) const;
  AdmittanceDynamics dynamics_;
  tf2_ros::Buffer* tf_ = nullptr;
  ros::Subscriber wrench_subscriber_;
  ros::ServiceServer enable_service_, reset_service_;
  geometry_msgs::WrenchStamped wrench_;
  ros::Time received_stamp_;
  std::string world_frame_ = "world", frame_ = "tool";
  double wrench_timeout_ = 0.25;
  bool have_wrench_ = false;
  Vector6 wrench_reference_ = Vector6::Zero();
  Eigen::Matrix3d last_world_R_frame_ = Eigen::Matrix3d::Identity();
};
}  // namespace aerial_robot_motion::compliance
