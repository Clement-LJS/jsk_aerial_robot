// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/control/gimbalrotor_controller.h>

#include <aerial_robot_control/control/impedance_core.h>

#include <ros/ros.h>

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <string>

namespace aerial_robot_control
{

class GimbalrotorImpedanceController : public GimbalrotorController
{
public:
  GimbalrotorImpedanceController();
  virtual ~GimbalrotorImpedanceController() = default;

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

private:
  void externalWrenchCallback(
      const geometry_msgs::WrenchStamped::ConstPtr& msg);

  void impedanceEnableCallback(
      const std_msgs::Bool::ConstPtr& msg);

  Eigen::Matrix<double, 6, 1> getExternalWrenchWorld() const;

  Eigen::Matrix3d getComplianceToWorldRotation() const;

  tf::Vector3 eigenToTfVector3(
      const Eigen::Vector3d& v) const;

  Eigen::Vector3d tfVector3ToEigen(
      const tf::Vector3& v) const;

private:
  ros::Subscriber external_wrench_sub_;
  ros::Subscriber impedance_enable_sub_;

  std::string external_wrench_topic_;
  std::string impedance_enable_topic_;

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
   *     impedance axes are fixed in world frame
   *
   *   cog / body / base_link:
   *     impedance axes rotate with robot body
   *
   * Robot-specific tool frame should be added later in a robot-specific
   * subclass, not in ImpedanceCore.
   */
  std::string compliance_frame_;

  bool impedance_enabled_;

  Eigen::Matrix<double, 6, 1> raw_external_wrench_;

  ros::Time prev_impedance_time_;

  ImpedanceCore impedance_core_;
  ImpedanceCoreConfig impedance_config_;
  ImpedanceCoreOutput impedance_output_;
};

} // namespace aerial_robot_control