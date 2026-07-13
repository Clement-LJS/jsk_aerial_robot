// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/gimbalrotor_navigation.h>

#include <aerial_robot_control/control/spatial_constraint.h>
#include <aerial_robot_msgs/FlightNav.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace aerial_robot_navigation
{

/*
 * Perching navigator responsibilities:
 *
 *   1. choose/capture the fixed perching pivot,
 *   2. explicitly define the allowed constraint DOF,
 *   3. generate the nominal constrained robot pose,
 *   4. expose the same SpatialConstraint to the interaction controller.
 *
 * The interaction controller never duplicates the perching geometry.
 */
class GimbalrotorPerchingNavigator : public GimbalrotorNavigator
{
public:
  GimbalrotorPerchingNavigator();
  ~GimbalrotorPerchingNavigator() override = default;

  void initialize(
      ros::NodeHandle nh,
      ros::NodeHandle nhp,
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
      double loop_du) override;

  void update() override;

  bool isPerchingEnabled() const;
  bool isPerchingLocked() const;

  const aerial_robot_control::SpatialConstraint&
  getSpatialConstraint() const;

  double getActiveConstraintCoordinate() const;

  aerial_robot_control::SpatialConstraintTarget
  calculateConstrainedTarget(
      double coordinate,
      double coordinate_velocity = 0.0,
      double coordinate_acceleration = 0.0) const;

protected:
  void rosParamInit() override;
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;
  void reset() override;

private:
  void perchingEnableCallback(const std_msgs::BoolConstPtr& msg);
  void relockCallback(const std_msgs::EmptyConstPtr& msg);
  void resetConstraintCallback(const std_msgs::EmptyConstPtr& msg);

  void branchPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void perchingPointCallback(const geometry_msgs::PointStampedConstPtr& msg);

  void targetAngleDegCallback(const std_msgs::Float64ConstPtr& msg);
  void addAngleDegCallback(const std_msgs::Float64ConstPtr& msg);
  void addAngleRadCallback(const std_msgs::Float64ConstPtr& msg);

  bool tryLockConstraint(const std::string& reason);
  void clearConstraint();
  void applyActiveConstraintTarget();

  bool hasConstraintAngleCommand(const aerial_robot_msgs::FlightNav& msg) const;
  double getConstraintAngleCommand(const aerial_robot_msgs::FlightNav& msg) const;

  void setActiveCoordinateFromAngle(double angle_rad);
  void addActiveCoordinate(double delta_rad);

  Eigen::Vector3d getCurrentCogPositionWorld() const;
  Eigen::Matrix3d getCurrentCogRotationWorld() const;
  Eigen::Vector3d getCurrentBaselinkPositionWorld() const;
  Eigen::Matrix3d getCurrentBaselinkRotationWorld() const;
  Eigen::Vector3d computePivotWorld() const;

  bool isManualPivotSource() const;
  bool isBranchPivotSource() const;
  bool hasBranchPivot() const;

  bool loadVector6Param(
      const ros::NodeHandle& nh,
      const std::string& name,
      aerial_robot_control::Vector6d& value);

  bool loadVector3Param(
      const ros::NodeHandle& nh,
      const std::string& name,
      Eigen::Vector3d& value);

  double lockedEulerComponent() const;
  double normalizeAngle(double angle) const;

  void publishLockedState();
  void publishCommandedState(const aerial_robot_control::SpatialConstraintTarget& target);

private:
  aerial_robot_control::SpatialConstraint spatial_constraint_;

  bool perching_enabled_ = false;
  bool perching_locked_ = false;
  bool lock_once_ = true;
  bool active_hold_enabled_ = true;
  bool accept_uav_nav_angle_command_ = false;
  bool command_angle_as_delta_ = true;

  double minimum_valid_radius_ = 0.05;
  double maximum_coordinate_ = 0.5235987756;
  double coordinate_sign_ = 1.0;
  double command_sign_ = 1.0;

  double active_coordinate_ = 0.0;

  aerial_robot_control::Vector6d allowed_dof_ = aerial_robot_control::Vector6d::Zero();

  Eigen::Vector3d constraint_frame_rpy_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d hand_center_offset_baselink_ = Eigen::Vector3d::Zero();

  std::string pivot_source_ = "manual";

  std::string perching_enable_topic_ = "perching/enable";
  std::string relock_topic_ = "perching/relock";
  std::string reset_topic_ = "perching/reset";
  std::string branch_pose_topic_ = "perching/branch_pose";
  std::string perching_point_topic_ = "perching/point";
  std::string target_angle_topic_ = "perching/target_angle_deg";
  std::string add_angle_topic_ = "perching/add_angle_deg";
  std::string legacy_target_pitch_topic_ = "perching/target_pitch_deg";
  std::string legacy_add_pitch_topic_ = "perching/add_pitch_deg";
  std::string legacy_manual_pitch_delta_topic_ = "perching/manual_pitch_delta";

  bool has_branch_pose_ = false;
  bool has_perching_point_ = false;

  Eigen::Vector3d branch_position_world_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d perching_point_world_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d locked_rpy_ = Eigen::Vector3d::Zero();

  ros::Subscriber perching_enable_sub_;
  ros::Subscriber relock_sub_;
  ros::Subscriber reset_sub_;
  ros::Subscriber branch_pose_sub_;
  ros::Subscriber perching_point_sub_;
  ros::Subscriber target_angle_sub_;
  ros::Subscriber add_angle_sub_;
  ros::Subscriber legacy_target_pitch_sub_;
  ros::Subscriber legacy_add_pitch_sub_;
  ros::Subscriber legacy_manual_pitch_delta_sub_;

  ros::Publisher locked_pose_pub_;
  ros::Publisher locked_pivot_pub_;
  ros::Publisher commanded_pose_pub_;
};

}  // namespace aerial_robot_navigation
