#include <aerial_robot_motion/constraint/static_thrust.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <pluginlib/class_list_macros.h>
#include <limits>

namespace aerial_robot_motion { namespace constraint {
void StaticThrustConstraint::initialize(const ros::NodeHandle& nh, const ModelInfo& info)
{
  Base::initialize(nh, info);
  if (!info.supports_physical_constraints)
    throw std::runtime_error("StaticThrust requires a transformable robot model");
  nh.param("lower_limit", lower_, info.thrust_lower);
  nh.param("upper_limit", upper_, info.thrust_upper);
  if (!std::isfinite(lower_) || !std::isfinite(upper_) || lower_ > upper_)
    throw std::runtime_error("invalid static-thrust limits");
}

bool StaticThrustConstraint::update(const MotionContext& context, QPProblem& problem)
{
  if (context.contact_active) return true;  // Static model omits the support reaction.
  const auto& physical = context.state.physical;
  if (!physical.static_thrust_available || !validStep(context.dt) ||
      physical.static_thrust_jacobian.cols() != problem.g.size()) return false;
  const int rows = physical.static_thrust.size();
  const Eigen::VectorXd lower = Eigen::VectorXd::Constant(rows, lower_) - physical.static_thrust;
  const Eigen::VectorXd upper = Eigen::VectorXd::Constant(rows, upper_) - physical.static_thrust;
  problem.appendConstraints(context.dt * physical.static_thrust_jacobian, lower, upper);
  return true;
}
}}
PLUGINLIB_EXPORT_CLASS(aerial_robot_motion::constraint::StaticThrustConstraint,
                       aerial_robot_motion::constraint::Base)
