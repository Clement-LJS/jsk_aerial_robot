// -*- mode: c++ -*-
#pragma once

#include <cmath>
#include <string>

#include <ros/ros.h>

#include <aerial_robot_msgs/FlightNav.h>
#include <gimbalrotor/gimbalrotor_navigation.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <tf/tf.h>

namespace aerial_robot_navigation
{

class GimbalrotorPerchingNavigator : public GimbalrotorNavigator
{
public:
  GimbalrotorPerchingNavigator();
  ~GimbalrotorPerchingNavigator() {}

  void initialize(ros::NodeHandle nh,
                  ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  double loop_du) override;

private:
  /*
   * Reusable perching navigation.
   *
   * This class is NOT for branch takeoff.
   *
   * Intended mission:
   *   1. Normal takeoff from ground.
   *   2. Normal navigation to branch.
   *   3. Enable perching navigation near branch.
   *   4. Lock current robot pose + branch/perching point.
   *   5. Convert pitch command into:
   *        - body pitch target
   *        - CoG position target on arc around branch
   */

  void rosParamInit() override;
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;

  void perchingEnableCallback(const std_msgs::BoolConstPtr& msg);
  void branchPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void perchingPointCallback(const geometry_msgs::PointStampedConstPtr& msg);
  void relockCallback(const std_msgs::EmptyConstPtr& msg);
  void resetCallback(const std_msgs::EmptyConstPtr& msg);

  bool tryLockPerching(const std::string& reason);
  void resetPerchingLock();

  void applyPerchingConstraint(aerial_robot_msgs::FlightNav& nav_msg);

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

  double min_valid_radius_;
  double max_pitch_delta_;
  double arc_pitch_sign_;

  std::string perching_enable_topic_;
  std::string branch_pose_topic_;
  std::string perching_point_topic_;
  std::string relock_topic_;
  std::string reset_topic_;

  bool has_branch_pose_;
  bool has_perching_point_;

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