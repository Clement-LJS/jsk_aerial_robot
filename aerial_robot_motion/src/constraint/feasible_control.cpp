#include <aerial_robot_motion/constraint/feasible_control.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <pluginlib/class_list_macros.h>
#include <limits>

namespace aerial_robot_motion { namespace constraint {
void FeasibleControlConstraint::initialize(const ros::NodeHandle& nh, const ModelInfo& info)
{
  Base::initialize(nh, info);
  if (!info.supports_physical_constraints)
    throw std::runtime_error("FeasibleControl requires a transformable robot model");
  force_margin_ = nonnegativeParam(nh, "minimum_force_margin", 0.0);
  torque_margin_ = nonnegativeParam(nh, "minimum_torque_margin", 0.0);
}

bool FeasibleControlConstraint::update(const MotionContext& context, QPProblem& problem)
{
  if (context.contact_active) return true;  // Feasibility model omits support reaction.
  const auto& physical = context.state.physical;
  if (!physical.feasible_control_available || !validStep(context.dt) ||
      physical.feasible_force_jacobian.cols() != problem.g.size() ||
      physical.feasible_torque_jacobian.cols() != problem.g.size()) return false;
  const double infinity = std::numeric_limits<double>::infinity();
  problem.appendConstraints(context.dt * physical.feasible_force_jacobian,
      Eigen::VectorXd::Constant(physical.feasible_force_margin.size(), force_margin_) -
          physical.feasible_force_margin,
      Eigen::VectorXd::Constant(physical.feasible_force_margin.size(), infinity));
  problem.appendConstraints(context.dt * physical.feasible_torque_jacobian,
      Eigen::VectorXd::Constant(physical.feasible_torque_margin.size(), torque_margin_) -
          physical.feasible_torque_margin,
      Eigen::VectorXd::Constant(physical.feasible_torque_margin.size(), infinity));
  return true;
}
}}
PLUGINLIB_EXPORT_CLASS(aerial_robot_motion::constraint::FeasibleControlConstraint,
                       aerial_robot_motion::constraint::Base)
