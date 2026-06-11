// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/control/gimbalrotor_impedance_controller.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <tf/transform_datatypes.h>

#include <string>

namespace aerial_robot_control
{

class GimbalrotorPerchingImpedanceController
  : public GimbalrotorImpedanceController
{
public:
  GimbalrotorPerchingImpedanceController();
  virtual ~GimbalrotorPerchingImpedanceController() = default;

  virtual void initialize(
      ros::NodeHandle nh,
      ros::NodeHandle nhp,
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
      boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
      double ctrl_loop_rate) override;

  virtual void reset() override;

protected:
  virtual const char* controllerName() const override
  {
    return "GimbalrotorPerchingImpedanceController";
  }

  virtual void applyImpedanceOutputToNavigator(
      const tf::Vector3& original_target_pos,
      const tf::Vector3& original_target_rpy,
      const ImpedanceCoreOutput& output) override;

private:
  void perchingRosParamInit();

  void perchingEnableCallback(
      const std_msgs::Bool::ConstPtr& msg);

  void perchingPointCallback(
      const geometry_msgs::PointStamped::ConstPtr& msg);

  void branchPoseCallback(
      const geometry_msgs::PoseStamped::ConstPtr& msg);

  void lockedPoseCallback(
      const geometry_msgs::PoseStamped::ConstPtr& msg);

  bool hasValidPerchingConstraint() const;

  tf::Vector3 computePerchingArcPositionFromPitch(
      double target_pitch,
      const tf::Vector3& original_target_pos) const;

  double clamp(
      double value,
      double min_value,
      double max_value) const;

  double normalizeAngle(
      double angle) const;

  double norm2D(
      double x,
      double z) const;

  void poseMsgToTfPosRpy(
      const geometry_msgs::PoseStamped& msg,
      tf::Vector3& pos,
      tf::Vector3& rpy) const;

private:
  ros::Subscriber perching_enable_sub_for_constraint_;
  ros::Subscriber perching_point_sub_;
  ros::Subscriber branch_pose_sub_;
  ros::Subscriber locked_pose_sub_;

  std::string perching_enable_topic_for_constraint_;
  std::string perching_point_topic_;
  std::string perching_branch_pose_topic_;
  std::string perching_locked_pose_topic_;

  bool perching_enabled_for_constraint_;
  bool has_perching_point_;
  bool has_branch_pose_;
  bool has_locked_pose_;

  bool use_branch_pose_if_no_point_;
  bool require_perching_lock_;

  double min_valid_radius_;
  double max_pitch_delta_;
  double arc_pitch_sign_;

  tf::Vector3 perching_point_world_;
  tf::Vector3 branch_pos_world_;

  tf::Vector3 locked_robot_pos_world_;
  tf::Vector3 locked_robot_rpy_;
  tf::Vector3 locked_radius_vec_world_;

  double locked_radius_;
  double locked_x_side_;
};

} // namespace aerial_robot_control