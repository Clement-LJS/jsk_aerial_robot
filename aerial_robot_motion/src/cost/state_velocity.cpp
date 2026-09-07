#include <aerial_robot_motion/cost/state_velocity.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <pluginlib/class_list_macros.h>

namespace aerial_robot_motion { namespace cost {
void StateVelocityCost::initialize(const ros::NodeHandle& nh, const ModelInfo& info)
{
  Base::initialize(nh, info);
  weights_ = Eigen::VectorXd::Zero(info.dimension());
  weights_.head<3>().setConstant(nonnegativeParam(nh, "root_translation_weight", 0.01));
  weights_.segment<3>(3).setConstant(nonnegativeParam(nh, "root_rotation_weight", 0.01));
  weights_.tail(info.joint_names.size()) = vectorParam(nh, "joint_weights", info.joint_names.size(),
      nonnegativeParam(nh, "joint_weight", 0.01));
}
bool StateVelocityCost::update(const MotionContext&, QPProblem& problem)
{
  if (weights_.size() != problem.g.size()) return false;
  problem.addCost((2.0 * weights_).asDiagonal(), Eigen::VectorXd::Zero(weights_.size()));
  return true;
}
}}
PLUGINLIB_EXPORT_CLASS(aerial_robot_motion::cost::StateVelocityCost, aerial_robot_motion::cost::Base)
