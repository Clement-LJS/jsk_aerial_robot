#include <aerial_robot_motion/constraint/acceleration_limit.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <pluginlib/class_list_macros.h>

namespace aerial_robot_motion { namespace constraint {
void AccelerationLimitConstraint::initialize(const ros::NodeHandle& nh, const ModelInfo& info)
{
  Base::initialize(nh, info);
  acceleration_.resize(info.dimension());
  acceleration_.head<3>() = vectorParam(nh, "root_linear_acceleration", 3, 1.0);
  acceleration_.segment<3>(3) = vectorParam(nh, "root_angular_acceleration", 3, 2.0);
  acceleration_.tail(info.joint_names.size()) = vectorParam(
      nh, "joint_acceleration", info.joint_names.size(),
      nonnegativeParam(nh, "joint_acceleration_default", 2.0));
  if ((acceleration_.array() <= 0).any())
    throw std::runtime_error("acceleration limits must be strictly positive");
  previous_ = Eigen::VectorXd::Zero(info.dimension());
  has_previous_ = false;
}

bool AccelerationLimitConstraint::bounds(const Eigen::VectorXd& previous, bool has_previous,
                                         double dt, const Eigen::VectorXd& acceleration,
                                         Eigen::VectorXd& lower, Eigen::VectorXd& upper)
{
  if (!validStep(dt) || acceleration.size() == 0 || !acceleration.allFinite() ||
      (acceleration.array() <= 0).any() ||
      (has_previous && (previous.size() != acceleration.size() || !previous.allFinite())))
    return false;
  Eigen::VectorXd center = Eigen::VectorXd::Zero(acceleration.size());
  if (has_previous) center = previous;
  const Eigen::VectorXd delta = dt * acceleration;
  lower = center - delta;
  upper = center + delta;
  return lower.allFinite() && upper.allFinite();
}

bool AccelerationLimitConstraint::update(const MotionContext& context, QPProblem& problem)
{
  if (problem.g.size() != acceleration_.size()) return false;
  Eigen::VectorXd lower, upper;
  if (!bounds(previous_, has_previous_, context.dt, acceleration_, lower, upper)) return false;
  problem.intersectBounds(lower, upper);
  return true;
}

void AccelerationLimitConstraint::solutionAccepted(const MotionContext&, const Eigen::VectorXd& solution)
{
  if (solution.size() != acceleration_.size() || !solution.allFinite())
    throw std::invalid_argument("cannot retain invalid acceleration-limit history");
  previous_ = solution;
  has_previous_ = true;
}

void AccelerationLimitConstraint::reset()
{
  previous_.setZero();
  has_previous_ = false;
}
}}
PLUGINLIB_EXPORT_CLASS(aerial_robot_motion::constraint::AccelerationLimitConstraint,
                       aerial_robot_motion::constraint::Base)
