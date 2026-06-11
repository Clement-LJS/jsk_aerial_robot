// -*- mode: c++ -*-

#pragma once

#include <cmath>
#include <string>

#include <gimbalrotor/gimbalrotor_navigation.h>

#include <aerial_robot_msgs/FlightNav.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#include <tf/tf.h>

#include <ros/ros.h>

namespace aerial_robot_navigation
{

class GimbalrotorPerchingNavigator : public GimbalrotorNavigator
{
public:
  GimbalrotorPerchingNavigator();
  ~GimbalrotorPerchingNavigator() {}

  void initialize(
      ros::NodeHandle nh,
      ros::NodeHandle nhp,
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
      double loop_du) override;

  /*
   * Important:
   *
   * Perching navigation is an active constrained navigation mode.
   *
   * While perching is enabled:
   *   - the robot keeps a branch-relative target
   *   - pitch delta command is converted into:
   *       1. body pitch target
   *       2. CoG position target on X-Z arc around branch/perching point
   *
   * Manual perching pitch command is intentionally separated from /uav/nav.
   * This avoids /perching_cutting_mission overwriting manual pitch by sending
   * target_pitch = 0.0 with pitch_nav_mode = POS.
   */
  void update() override;

private:
  void rosParamInit() override;
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;

  void perchingEnableCallback(const std_msgs::BoolConstPtr& msg);
  void branchPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void perchingPointCallback(const geometry_msgs::PointStampedConstPtr& msg);
  void relockCallback(const std_msgs::EmptyConstPtr& msg);
  void resetCallback(const std_msgs::EmptyConstPtr& msg);

  /*
   * Manual perching pitch delta command.
   *
   * msg->data is relative pitch angle [rad].
   *
   * Example:
   *   +0.10 rad means add +0.10 rad from locked pitch
   *   -0.10 rad means add -0.10 rad from locked pitch
   *    0.00 rad means return to locked pitch
   */
  void manualPitchDeltaCallback(const std_msgs::Float64ConstPtr& msg);

  bool tryLockPerching(const std::string& reason);
  void resetPerchingLock();

  void applyPerchingConstraint(aerial_robot_msgs::FlightNav& nav_msg);

  void applyActivePerchingTarget();
  aerial_robot_msgs::FlightNav buildActivePerchingNavCommand();
  tf::Vector3 computeActiveHoldPosition() const;
  double computeActiveHoldPitch() const;
  double computeCompliantTargetY() const;

  bool hasPitchCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;
  bool hasPositionCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;
  bool hasVelocityCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;

  double getCommandedPitch(const aerial_robot_msgs::FlightNav& nav_msg) const;

  tf::Vector3 getCurrentRobotPos() const;
  tf::Vector3 getCurrentRobotRPY() const;

  tf::Vector3 computeArcPositionFromPitch(double target_pitch) const;
  tf::Vector3 projectPositionToPitchArc(const tf::Vector3& desired_pos) const;
  tf::Vector3 projectVelocityToPitchArcTangent(const tf::Vector3& desired_vel) const;

  tf::Vector3 getDesiredPosition(const aerial_robot_msgs::FlightNav& nav_msg) const;
  tf::Vector3 getDesiredVelocity(const aerial_robot_msgs::FlightNav& nav_msg) const;

  double clamp(double value, double min_value, double max_value) const;
  double normalizeAngle(double angle) const;
  double norm2D(double x, double z) const;
  double norm3D(const tf::Vector3& v) const;

  void publishLockedDebugPose();
  void publishCommandedDebugPose(const tf::Vector3& pos, double pitch);

  ros::Subscriber perching_enable_sub_;
  ros::Subscriber branch_pose_sub_;
  ros::Subscriber perching_point_sub_;
  ros::Subscriber relock_sub_;
  ros::Subscriber reset_sub_;
  ros::Subscriber manual_pitch_delta_sub_;

  ros::Publisher locked_pose_pub_;
  ros::Publisher commanded_pose_pub_;

  bool perching_enable_;
  bool perching_locked_;
  bool perching_lock_once_;

  bool require_branch_point_;
  bool command_pitch_as_delta_;
  bool constrain_position_command_;
  bool constrain_velocity_command_;
  bool use_pitch_command_for_arc_;
  bool hold_locked_pose_without_pitch_command_;

  /*
   * If false, /uav/nav pitch_nav_mode == POS is NOT treated as a perching
   * arc pitch command.
   *
   * This should normally be false for your current test, because mission nodes
   * often publish target_pitch = 0.0 with pitch_nav_mode = POS.
   *
   * Manual pitch should use:
   *   /gimbalrotor/perching/manual_pitch_delta
   */
  bool accept_uav_nav_pitch_command_;

  bool active_perching_hold_enable_;

  double min_valid_radius_;
  double max_pitch_delta_;
  double arc_pitch_sign_;
  double command_pitch_sign_;
  double y_compliance_deadband_;

  std::string perching_enable_topic_;
  std::string branch_pose_topic_;
  std::string perching_point_topic_;
  std::string relock_topic_;
  std::string reset_topic_;
  std::string manual_pitch_delta_topic_;

  bool has_branch_pose_;
  bool has_perching_point_;

  bool has_active_pitch_target_;
  double active_target_pitch_;

  tf::Vector3 branch_pos_world_;
  tf::Vector3 perching_point_world_;

  tf::Vector3 locked_robot_pos_world_;
  tf::Vector3 locked_robot_rpy_;
  tf::Vector3 locked_radius_vec_world_;

  double locked_radius_;
  double locked_y_offset_;
  double locked_x_side_;
};

}  // namespace aerial_robot_navigation