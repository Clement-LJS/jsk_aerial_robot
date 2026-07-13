// -*- mode: c++ -*-
#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <string>

namespace aerial_robot_control
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/*
 * Common ordering used by every vector in this file:
 *
 *   [x, y, z, roll, pitch, yaw]
 *
 * For a wrench, the same indices mean:
 *
 *   [force_x, force_y, force_z, torque_x, torque_y, torque_z]
 */

struct AdmittanceConfig
{
  Matrix6d virtual_mass = Matrix6d::Identity();
  Matrix6d damping = Matrix6d::Zero();
  Matrix6d stiffness = Matrix6d::Zero();

  Vector6d enabled_dof = Vector6d::Ones();
  Vector6d reference_wrench = Vector6d::Zero();

  Vector6d wrench_error_limit = Vector6d::Constant(1.0e6);
  Vector6d position_offset_limit = Vector6d::Constant(1.0e6);
  Vector6d velocity_offset_limit = Vector6d::Constant(1.0e6);
  Vector6d acceleration_offset_limit = Vector6d::Constant(1.0e6);

  // First-order low-pass filter time constant [s].
  // <= 0 disables filtering.
  double wrench_filter_time_constant = 0.0;

  // Reject a cycle if its dt is larger than this value.
  double maximum_dt = 0.1;

  bool reset_on_disable = true;
};

struct AdmittanceInput
{
  Vector6d measured_wrench = Vector6d::Zero();

  // Extra robot/application-specific mask.
  // Example: the perching constraint can supply only pitch DOF here.
  Vector6d active_dof = Vector6d::Ones();

  double dt = 0.0;
  bool enabled = false;
};

struct AdmittanceOutput
{
  bool valid = false;

  Vector6d filtered_wrench_error = Vector6d::Zero();
  Vector6d position_offset = Vector6d::Zero();
  Vector6d velocity_offset = Vector6d::Zero();
  Vector6d acceleration_offset = Vector6d::Zero();
};

struct ImpedanceConfig
{
  Matrix6d inertia = Matrix6d::Zero();
  Matrix6d damping = Matrix6d::Zero();
  Matrix6d stiffness = Matrix6d::Zero();

  Vector6d enabled_dof = Vector6d::Ones();
  Vector6d wrench_limit = Vector6d::Constant(1.0e6);

  bool use_inertia_feedforward = true;
};

struct ImpedanceInput
{
  Vector6d position_error = Vector6d::Zero();
  Vector6d velocity_error = Vector6d::Zero();
  Vector6d desired_acceleration = Vector6d::Zero();
  Vector6d feedforward_wrench = Vector6d::Zero();

  Vector6d active_dof = Vector6d::Ones();

  bool enabled = false;
};

struct ImpedanceOutput
{
  bool valid = false;
  Vector6d commanded_wrench = Vector6d::Zero();
};

/*
 * Generalized momentum-observer input.
 *
 * This class intentionally does not know whether the values belong to a whole
 * robot, a link, or a joint. The robot-specific wrapper must construct all
 * three vectors in one consistent coordinate convention.
 */
struct WrenchObserverConfig
{
  Matrix6d gain = Matrix6d::Identity();
  double maximum_dt = 0.1;
  bool reset_on_disable = true;
};

struct WrenchObserverInput
{
  Vector6d momentum = Vector6d::Zero();
  Vector6d commanded_effort = Vector6d::Zero();
  Vector6d model_bias = Vector6d::Zero();

  double dt = 0.0;
  bool enabled = false;
};

struct WrenchObserverOutput
{
  bool valid = false;
  Vector6d external_effort = Vector6d::Zero();
};

struct InteractionControllerConfig
{
  AdmittanceConfig admittance;
  ImpedanceConfig impedance;
  WrenchObserverConfig observer;
};

/*
 * Small robot-independent calculation module.
 *
 * It contains:
 *   - admittance: measured wrench -> motion offset
 *   - impedance: motion error -> commanded wrench
 *   - momentum observer: momentum/command/model -> estimated external effort
 *
 * It does not access ROS, a navigator, a robot model, or an estimator.
 */
class InteractionController
{
public:
  InteractionController();
  explicit InteractionController(const InteractionControllerConfig& config);

  void configure(const InteractionControllerConfig& config);
  const InteractionControllerConfig& getConfig() const;

  void reset();
  void resetAdmittance();
  void resetObserver();

  AdmittanceOutput calculateAdmittance(const AdmittanceInput& input);
  ImpedanceOutput calculateImpedance(const ImpedanceInput& input) const;
  WrenchObserverOutput estimateExternalWrench(const WrenchObserverInput& input);

private:
  static void validateConfig(const InteractionControllerConfig& config);
  static void validateFiniteMatrix(
      const Matrix6d& matrix,
      const std::string& name);
  static void validateNonNegativeVector(
      const Vector6d& vector,
      const std::string& name);
  static Vector6d binaryMask(const Vector6d& value);
  static Vector6d clampSymmetric(
      const Vector6d& value,
      const Vector6d& limit);

private:
  InteractionControllerConfig config_;
  bool configured_ = false;

  Eigen::LDLT<Matrix6d> admittance_mass_solver_;

  bool admittance_filter_initialized_ = false;
  Vector6d filtered_wrench_error_ = Vector6d::Zero();
  Vector6d position_offset_ = Vector6d::Zero();
  Vector6d velocity_offset_ = Vector6d::Zero();
  Vector6d acceleration_offset_ = Vector6d::Zero();

  bool observer_initialized_ = false;
  Vector6d observer_initial_momentum_ = Vector6d::Zero();
  Vector6d observer_integral_ = Vector6d::Zero();
  Vector6d observer_external_effort_ = Vector6d::Zero();
};

}  // namespace aerial_robot_control
