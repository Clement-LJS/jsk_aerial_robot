#include <aerial_robot_motion/constraint/joint_torque.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <pluginlib/class_list_macros.h>

namespace aerial_robot_motion { namespace constraint {
void JointTorqueConstraint::initialize(const ros::NodeHandle& nh, const ModelInfo& info)
{
  Base::initialize(nh, info);
  if (!info.supports_physical_constraints || info.joint_names.empty())
    throw std::runtime_error("JointTorque requires a transformable model with controlled link joints");
  effort_ = info.effort;
  if (nh.hasParam("effort_limits"))
    effort_ = vectorParam(nh, "effort_limits", info.joint_names.size(), 0.0);
  if (effort_.size() != static_cast<int>(info.joint_names.size()) || !effort_.allFinite() ||
      (effort_.array() <= 0).any())
    throw std::runtime_error("JointTorque requires positive configured or URDF effort limits");
}

bool JointTorqueConstraint::update(const MotionContext& context, QPProblem& problem)
{
  if (context.contact_active) return true;  // Static model omits contact/cutting reactions.
  const auto& physical = context.state.physical;
  if (!physical.joint_torque_available || !validStep(context.dt) ||
      physical.joint_torque.size() != effort_.size() ||
      physical.joint_torque_jacobian.cols() != problem.g.size()) return false;
  problem.appendConstraints(context.dt * physical.joint_torque_jacobian,
                            -effort_ - physical.joint_torque,
                             effort_ - physical.joint_torque);
  return true;
}
}}
PLUGINLIB_EXPORT_CLASS(aerial_robot_motion::constraint::JointTorqueConstraint,
                       aerial_robot_motion::constraint::Base)
