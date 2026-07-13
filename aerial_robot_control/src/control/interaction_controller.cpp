#include <aerial_robot_control/control/interaction_controller.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerial_robot_control
{

InteractionController::InteractionController()
{
  configure(InteractionControllerConfig());
}

InteractionController::InteractionController(const InteractionControllerConfig& config)
{
  configure(config);
}

void InteractionController::configure(const InteractionControllerConfig& config)
{
  validateConfig(config);

  config_ = config;
  config_.admittance.enabled_dof = binaryMask(config_.admittance.enabled_dof);
  config_.impedance.enabled_dof = binaryMask(config_.impedance.enabled_dof);

  admittance_mass_solver_.compute(config_.admittance.virtual_mass);
  if(admittance_mass_solver_.info() != Eigen::Success || !admittance_mass_solver_.isPositive())
  {
    throw std::invalid_argument("InteractionController: admittance virtual_mass must be positive definite");
  }

  configured_ = true;
  reset();
}

const InteractionControllerConfig&
InteractionController::getConfig() const
{
  return config_;
}

void InteractionController::reset()
{
  resetAdmittance();
  resetObserver();
}

void InteractionController::resetAdmittance()
{
  admittance_filter_initialized_ = false;
  filtered_wrench_error_.setZero();
  position_offset_.setZero();
  velocity_offset_.setZero();
  acceleration_offset_.setZero();
}

void InteractionController::resetObserver()
{
  observer_initialized_ = false;
  observer_initial_momentum_.setZero();
  observer_integral_.setZero();
  observer_external_effort_.setZero();
}

AdmittanceOutput InteractionController::calculateAdmittance(const AdmittanceInput& input)
{
  if(!configured_)
  {
    throw std::logic_error("InteractionController: configure() must be called first");
  }

  AdmittanceOutput output;

  if(!input.enabled)
  {
    if(config_.admittance.reset_on_disable)
    {
      resetAdmittance();
    }
    return output;
  }

  if(!std::isfinite(input.dt) ||
     input.dt <= 0.0 ||
     input.dt > config_.admittance.maximum_dt ||
     !input.measured_wrench.allFinite() ||
     !input.active_dof.allFinite())
  {
    return output;
  }

  const Vector6d active_dof = config_.admittance.enabled_dof.cwiseProduct(binaryMask(input.active_dof));

  Vector6d wrench_error = input.measured_wrench - config_.admittance.reference_wrench;
  wrench_error = clampSymmetric(wrench_error, config_.admittance.wrench_error_limit);
  wrench_error = wrench_error.cwiseProduct(active_dof);

  const double tau = config_.admittance.wrench_filter_time_constant;

  if(tau > 0.0)
  {
    const double alpha = input.dt / (tau + input.dt);

    if(!admittance_filter_initialized_)
    {
      filtered_wrench_error_ = wrench_error;
      admittance_filter_initialized_ = true;
    }
    else
    {
      filtered_wrench_error_ += alpha * (wrench_error - filtered_wrench_error_);
    }
  }
  else
  {
    filtered_wrench_error_ = wrench_error;
    admittance_filter_initialized_ = true;
  }

  // Disabled states are explicitly cleared so an old offset cannot leak into
  // a newly selected constraint mode.
  for(int i = 0; i < 6; ++i)
  {
    if(active_dof(i) < 0.5)
    {
      filtered_wrench_error_(i) = 0.0;
      position_offset_(i) = 0.0;
      velocity_offset_(i) = 0.0;
      acceleration_offset_(i) = 0.0;
    }
  }

  const Vector6d rhs = filtered_wrench_error_ - config_.admittance.damping * velocity_offset_ - config_.admittance.stiffness * position_offset_;

  acceleration_offset_ = admittance_mass_solver_.solve(rhs);
  if(admittance_mass_solver_.info() != Eigen::Success || !acceleration_offset_.allFinite())
  {
    return output;
  }

  acceleration_offset_ = acceleration_offset_.cwiseProduct(active_dof);
  acceleration_offset_ = clampSymmetric(acceleration_offset_, config_.admittance.acceleration_offset_limit);

  // Semi-implicit Euler: velocity first, then position.
  velocity_offset_ += acceleration_offset_ * input.dt;
  velocity_offset_ = velocity_offset_.cwiseProduct(active_dof);
  velocity_offset_ = clampSymmetric(velocity_offset_, config_.admittance.velocity_offset_limit);

  position_offset_ += velocity_offset_ * input.dt;
  position_offset_ = position_offset_.cwiseProduct(active_dof);

  for(int i = 0; i < 6; ++i)
  {
    const double limit = std::abs(config_.admittance.position_offset_limit(i));

    if(position_offset_(i) > limit)
    {
      position_offset_(i) = limit;
      if(velocity_offset_(i) > 0.0)
      {
        velocity_offset_(i) = 0.0;
      }
    }
    else if(position_offset_(i) < -limit)
    {
      position_offset_(i) = -limit;
      if(velocity_offset_(i) < 0.0)
      {
        velocity_offset_(i) = 0.0;
      }
    }
  }

  output.valid = true;
  output.filtered_wrench_error = filtered_wrench_error_;
  output.position_offset = position_offset_;
  output.velocity_offset = velocity_offset_;
  output.acceleration_offset = acceleration_offset_;
  return output;
}

ImpedanceOutput InteractionController::calculateImpedance(const ImpedanceInput& input) const
{
  if(!configured_)
  {
    throw std::logic_error("InteractionController: configure() must be called first");
  }

  ImpedanceOutput output;

  if(!input.enabled ||
     !input.position_error.allFinite() ||
     !input.velocity_error.allFinite() ||
     !input.desired_acceleration.allFinite() ||
     !input.feedforward_wrench.allFinite() ||
     !input.active_dof.allFinite())
  {
    return output;
  }

  const Vector6d active_dof = config_.impedance.enabled_dof.cwiseProduct(binaryMask(input.active_dof));

  Vector6d commanded_wrench = config_.impedance.stiffness * input.position_error + config_.impedance.damping * input.velocity_error + input.feedforward_wrench;

  if(config_.impedance.use_inertia_feedforward)
  {
    commanded_wrench += config_.impedance.inertia * input.desired_acceleration;
  }

  commanded_wrench = commanded_wrench.cwiseProduct(active_dof);
  commanded_wrench = clampSymmetric(commanded_wrench, config_.impedance.wrench_limit);

  if(!commanded_wrench.allFinite())
  {
    return output;
  }

  output.valid = true;
  output.commanded_wrench = commanded_wrench;
  return output;
}

WrenchObserverOutput InteractionController::estimateExternalWrench(const WrenchObserverInput& input)
{
  if(!configured_)
  {
    throw std::logic_error("InteractionController: configure() must be called first");
  }

  WrenchObserverOutput output;

  if(!input.enabled)
  {
    if(config_.observer.reset_on_disable)
    {
      resetObserver();
    }
    return output;
  }

if(!std::isfinite(input.dt) ||
   input.dt <= 0.0 ||
   input.dt > config_.observer.maximum_dt ||
   !input.momentum.allFinite() ||
   !input.commanded_effort.allFinite() ||
   !input.model_bias.allFinite())
{
  /*
   * Do not keep an observer trajectory across missing, delayed, or invalid
   * samples. The next valid sample will establish a fresh momentum reference.
   */
  resetObserver();
  return output;
}

  if(!observer_initialized_)
  {
    observer_initial_momentum_ = input.momentum;
    observer_integral_.setZero();
    observer_external_effort_.setZero();
    observer_initialized_ = true;

    output.valid = true;
    output.external_effort = observer_external_effort_;
    return output;
  }

  /*
   * Same generalized momentum-observer sign convention as the current JSK
   * PoseLinearController implementation:
   *
   *   integral += (commanded - model_bias + estimate) dt
   *   estimate  = gain * (momentum - initial_momentum - integral)
   */
  observer_integral_ += (input.commanded_effort - input.model_bias + observer_external_effort_) * input.dt;
  observer_external_effort_ = config_.observer.gain * (input.momentum - observer_initial_momentum_ - observer_integral_);

  if(!observer_external_effort_.allFinite())
  {
    resetObserver();
    return output;
  }

  output.valid = true;
  output.external_effort = observer_external_effort_;
  return output;
}

void InteractionController::validateConfig(const InteractionControllerConfig& config)
{
  validateFiniteMatrix(config.admittance.virtual_mass, "admittance.virtual_mass");
  validateFiniteMatrix(config.admittance.damping, "admittance.damping");
  validateFiniteMatrix(config.admittance.stiffness, "admittance.stiffness");

  validateFiniteMatrix(config.impedance.inertia, "impedance.inertia");
  validateFiniteMatrix(config.impedance.damping, "impedance.damping");
  validateFiniteMatrix(config.impedance.stiffness, "impedance.stiffness");

  validateFiniteMatrix(config.observer.gain, "observer.gain");

  validateNonNegativeVector(config.admittance.wrench_error_limit, "admittance.wrench_error_limit");
  validateNonNegativeVector(config.admittance.position_offset_limit, "admittance.position_offset_limit");
  validateNonNegativeVector(config.admittance.velocity_offset_limit, "admittance.velocity_offset_limit");
  validateNonNegativeVector(config.admittance.acceleration_offset_limit, "admittance.acceleration_offset_limit");
  validateNonNegativeVector(config.impedance.wrench_limit, "impedance.wrench_limit");

  if(!config.admittance.reference_wrench.allFinite() || !config.admittance.enabled_dof.allFinite() || !config.impedance.enabled_dof.allFinite())
  {
    throw std::invalid_argument("InteractionController: configuration contains NaN or infinity");
  }

  if(!std::isfinite(config.admittance.maximum_dt) ||
     config.admittance.maximum_dt <= 0.0 ||
     !std::isfinite(config.observer.maximum_dt) ||
     config.observer.maximum_dt <= 0.0 ||
     !std::isfinite(config.admittance.wrench_filter_time_constant) ||
     config.admittance.wrench_filter_time_constant < 0.0)
  {
    throw std::invalid_argument("InteractionController: invalid timing configuration");
  }
}

void InteractionController::validateFiniteMatrix(const Matrix6d& matrix, const std::string& name)
{
  if(!matrix.allFinite())
  {
    throw std::invalid_argument("InteractionController: " + name + " is not finite");
  }
}

void InteractionController::validateNonNegativeVector(const Vector6d& vector, const std::string& name)
{
  if(!vector.allFinite() || (vector.array() < 0.0).any())
  {
    throw std::invalid_argument("InteractionController: " + name + " must be finite and non-negative");
  }
}

Vector6d InteractionController::binaryMask(const Vector6d& value)
{
  Vector6d result = Vector6d::Zero();
  for(int i = 0; i < 6; ++i)
  {
    result(i) = value(i) > 0.5 ? 1.0 : 0.0;
  }
  return result;
}

Vector6d InteractionController::clampSymmetric(const Vector6d& value, const Vector6d& limit)
{
  Vector6d result = value;

  for(int i = 0; i < 6; ++i)
  {
    const double bound = std::abs(limit(i));
    result(i) = std::max(-bound, std::min(result(i), bound));
  }

  return result;
}

}  // namespace aerial_robot_control
