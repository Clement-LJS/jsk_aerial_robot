// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/control/gimbalrotor_perching_admittance_controller.h>
#include <gimbalrotor/gimbalrotor_multilink_perching_navigation.h>

#include <std_msgs/Float64.h>

#include <cstdint>
#include <limits>
#include <mutex>

namespace aerial_robot_control
{

class GimbalrotorMultilinkPerchingAdmittanceController
  : public GimbalrotorPerchingAdmittanceController
{
public:
  GimbalrotorMultilinkPerchingAdmittanceController();
  ~GimbalrotorMultilinkPerchingAdmittanceController() override = default;

  void initialize(
      ros::NodeHandle nh,
      ros::NodeHandle nhp,
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
      boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
      double ctrl_loop_rate) override;

  void reset() override;

protected:
  void controlCore() override;

  bool computeResidualPerchingPitchTorque(
      const Eigen::Matrix<double, 6, 1>& wrench_cog_world,
      const Eigen::Matrix<double, 6, 1>& wrench_pivot_world,
      const Eigen::Matrix<double, 6, 1>& equilibrium_wrench_pivot_world,
      const Eigen::Vector3d& cog_pos_world,
      const Eigen::Vector3d& pivot_pos_world,
      const Eigen::Matrix3d& R_world_constraint,
      const Eigen::Vector3d& constraint_axis_world,
      double& residual_pitch_torque,
      Eigen::Vector3d& pitch_axis_world) const override;

  bool allowPerchingEquilibriumTareCollection() const override;
  bool allowPerchingAdmittanceArming() const override;
  bool useLegacyPlanarPerchingConstraintFrame() const override
  {
    return false;
  }

  void applyAdmittanceOutputToNavigator(
      const tf::Vector3& original_target_pos,
      const tf::Vector3& original_target_rpy,
      const AdmittanceCoreOutput& output) override;

  Eigen::Matrix3d getComplianceToWorldRotation() const override;

  const char* controllerName() const override
  {
    return "GimbalrotorMultilinkPerchingAdmittanceController";
  }

private:
  static constexpr std::uint64_t INVALID_GENERATION =
      std::numeric_limits<std::uint64_t>::max();

  void publishJointTorqueDiagnostics() const;

  boost::shared_ptr<
      aerial_robot_navigation::GimbalrotorMultilinkPerchingNavigator>
      multilink_navigator_;
  bool multilink_navigator_valid_;

  std::uint64_t last_seen_mechanism_generation_;
  std::uint64_t last_tared_mechanism_generation_;

  ros::Publisher pitch_external_torque_pub_;
  ros::Publisher pitch_equilibrium_torque_pub_;
  ros::Publisher pitch_residual_torque_pub_;

  mutable std::mutex multilink_controller_mutex_;
  mutable double pitch_external_torque_;
  mutable double pitch_equilibrium_torque_;
  mutable double pitch_residual_torque_;
};

}  // namespace aerial_robot_control
