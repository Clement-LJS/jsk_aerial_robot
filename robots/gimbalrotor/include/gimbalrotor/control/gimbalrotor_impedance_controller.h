// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/control/gimbalrotor_multilink_controller.h>
#include <gimbalrotor/model/gimbalrotor_multilink_robot_model.h>

#include <ros/ros.h>

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/treefksolverpos_recursive.hpp>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>

#include <algorithm>
#include <string>

namespace aerial_robot_control
{

class GimbalrotorImpedanceController : public GimbalrotorMultilinkController
{
public:
  GimbalrotorImpedanceController();

  void initialize(
      ros::NodeHandle nh,
      ros::NodeHandle nhp,
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
      boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
      double ctrl_loop_rate) override;

  void reset() override;

protected:
  void rosParamInit() override;
  void controlCore() override;

private:
  void externalWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
  void isCuttingCallback(const std_msgs::Bool::ConstPtr& msg);

  bool getToolRotationWorld(Eigen::Matrix3d& R_world_tool);
  Eigen::Matrix3d rpyToRot(double roll, double pitch, double yaw) const;

  double clampValue(double value, double min_value, double max_value) const;

  Eigen::Vector3d clampVectorElementwise(
      const Eigen::Vector3d& value,
      const Eigen::Vector3d& limit) const;

  tf::Vector3 eigenToTfVector(const Eigen::Vector3d& v) const;
  Eigen::Vector3d tfToEigenVector(const tf::Vector3& v) const;

  void resetImpedanceMemory();

private:
  ros::Subscriber external_wrench_sub_;
  ros::Subscriber is_cutting_sub_;

  /*
   * Multilink robot model pointer.
   *
   * This controller is specifically for:
   *   gimbalrotor + pitch_joint / saw / multilink body
   *
   * Therefore we explicitly require GimbalrotorMultilinkRobotModel.
   */
  boost::shared_ptr<GimbalrotorMultilinkRobotModel> gimbalrotor_multilink_robot_model_for_impedance_;

  bool use_impedance_;

  /*
   * isCutting meaning:
   *
   *   false:
   *     normal multilink controller only
   *
   *   true:
   *     saw is spinning / cutting mode is armed
   *     robot can still move normally
   *     impedance only adds small compliance offset
   */
  bool is_cutting_;
  bool prev_is_cutting_;

  std::string external_wrench_topic_;
  std::string is_cutting_topic_;
  std::string external_wrench_frame_;

  std::string tool_link_name_;

  double tool_frame_roll_;
  double tool_frame_pitch_;
  double tool_frame_yaw_;

  Eigen::VectorXd est_external_wrench_;

  Eigen::Vector3d force_raw_;
  Eigen::Vector3d force_world_lpf_;
  Eigen::Vector3d force_tool_lpf_;

  /*
   * Tool-frame impedance states.
   */
  Eigen::Vector3d dx_tool_;
  Eigen::Vector3d dx_dot_tool_;
  Eigen::Vector3d dx_ddot_tool_;

  /*
   * World-frame offset memory.
   *
   * This prevents target drift while still allowing live motion commands.
   *
   * Each cycle:
   *   current_nav_target may already include previous impedance offset.
   *
   * Therefore:
   *   live_base_target = current_nav_target - prev_dx_world_
   *
   * Then:
   *   new_cmd_target = live_base_target + new_dx_world
   */
  Eigen::Vector3d prev_dx_world_;
  Eigen::Vector3d prev_modified_target_world_;
  bool prev_modified_target_valid_;

  /*
   * If current navigator target differs from previous modified target by more
   * than this value, assume an external navigation command updated the target.
   *
   * Then we do NOT subtract prev_dx_world_.
   */
  double target_override_threshold_;

  Eigen::Vector3d axis_enable_;

  Eigen::Vector3d mass_;
  Eigen::Vector3d damping_;
  Eigen::Vector3d stiffness_;

  Eigen::Vector3d force_ref_tool_;
  Eigen::Vector3d force_limit_tool_;
  Eigen::Vector3d displacement_limit_tool_;
  Eigen::Vector3d velocity_limit_tool_;

  double force_lpf_alpha_;

  ros::Time prev_time_;
};

} // namespace aerial_robot_control
