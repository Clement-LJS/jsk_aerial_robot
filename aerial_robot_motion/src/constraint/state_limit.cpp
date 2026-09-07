#include <aerial_robot_motion/constraint/state_limit.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <pluginlib/class_list_macros.h>
#include <limits>
namespace aerial_robot_motion { namespace constraint {
void StateLimitConstraint::initialize(const ros::NodeHandle& nh, const ModelInfo& info)
{
  Base::initialize(nh, info);
  linear_ = vectorParam(nh, "root_linear_limits", 3, 0.1);
  angular_ = vectorParam(nh, "root_angular_limits", 3, 0.15);
  translation_step_ = nonnegativeParam(nh, "max_translation_step", 0.005);
  rotation_step_ = nonnegativeParam(nh, "max_rotation_step", 0.005);
}
bool StateLimitConstraint::update(const MotionContext& ctx, QPProblem& p)
{
  if (!validStep(ctx.dt) || p.g.size() < 6) return false;
  Eigen::VectorXd upper = Eigen::VectorXd::Constant(p.g.size(), std::numeric_limits<double>::infinity());
  upper.head<3>() = linear_.cwiseMin(Eigen::Vector3d::Constant(translation_step_ / ctx.dt));
  upper.segment<3>(3) = angular_.cwiseMin(Eigen::Vector3d::Constant(rotation_step_ / ctx.dt));
  p.intersectBounds(-upper, upper);
  return true;
}
}}
PLUGINLIB_EXPORT_CLASS(aerial_robot_motion::constraint::StateLimitConstraint, aerial_robot_motion::constraint::Base)
