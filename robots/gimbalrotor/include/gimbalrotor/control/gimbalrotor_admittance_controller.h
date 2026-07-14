// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/control/gimbalrotor_controller.h>

#include <aerial_robot_control/control/admittance_core.h>

#include <ros/ros.h>

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <string>

#include <mutex>

namespace aerial_robot_control
{

class GimbalrotorAdmittanceController : public GimbalrotorController
{
public:
  GimbalrotorAdmittanceController();
  virtual ~GimbalrotorAdmittanceController() = default;

  virtual void initialize(
      ros::NodeHandle nh,
      ros::NodeHandle nhp,
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
      boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
      double ctrl_loop_rate) override;

  virtual void reset() override;

protected:
  virtual void rosParamInit() override;
  virtual void controlCore() override;

  /*
   * This is the important refactor.
   *
   * Normal GimbalrotorAdmittanceController uses normal admittance injection.
   * A subclass, such as GimbalrotorPerchingAdmittanceController, can override
   * only this function and keep all existing admittance/wrench/core logic.
   */
  virtual void applyAdmittanceOutputToNavigator(
      const tf::Vector3& original_target_pos,
      const tf::Vector3& original_target_rpy,
      const AdmittanceCoreOutput& output);

  virtual const char* controllerName() const
  {
    return "GimbalrotorAdmittanceController";
  }

  void externalWrenchCallback(
      const geometry_msgs::WrenchStamped::ConstPtr& msg);

  void admittanceEnableCallback(
      const std_msgs::Bool::ConstPtr& msg);

  virtual Eigen::Matrix<double, 6, 1> getExternalWrenchWorld() const;

  virtual Eigen::Matrix3d getComplianceToWorldRotation() const;

  tf::Vector3 eigenToTfVector3(
      const Eigen::Vector3d& v) const;

  Eigen::Vector3d tfVector3ToEigen(
      const tf::Vector3& v) const;

  mutable std::mutex external_wrench_mutex_;

  ros::Time last_external_wrench_receive_time_;

  bool has_external_wrench_;

  double external_wrench_timeout_;

protected:
  ros::Subscriber external_wrench_sub_;
  ros::Subscriber admittance_enable_sub_;

  std::string external_wrench_topic_;
  std::string admittance_enable_topic_;

  /*
   * external_wrench_frame:
   *   world / map / odom:
   *     input wrench already uses world frame
   *
   *   cog / body / base_link:
   *     input wrench uses robot CoG/body frame
   */
  std::string external_wrench_frame_;

  /*
   * compliance_frame:
   *   world / map / odom:
   *     admittance axes are fixed in world frame
   *
   *   cog / body / base_link:
   *     admittance axes rotate with robot body
   */
  std::string compliance_frame_;

  bool admittance_enabled_;

  Eigen::Matrix<double, 6, 1> raw_external_wrench_;

  ros::Time prev_admittance_time_;

  AdmittanceCore admittance_core_;
  AdmittanceCoreConfig admittance_config_;
  AdmittanceCoreOutput admittance_output_;
};

} // namespace aerial_robot_control