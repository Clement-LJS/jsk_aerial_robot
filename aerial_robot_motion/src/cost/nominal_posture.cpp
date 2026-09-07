#include <aerial_robot_motion/cost/nominal_posture.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <pluginlib/class_list_macros.h>

namespace aerial_robot_motion { namespace cost {
void NominalPostureCost::initialize(const ros::NodeHandle& nh, const ModelInfo& info)
{
  Base::initialize(nh, info);
  gain_ = nonnegativeParam(nh, "gain", 0.5);
  joint_target_ = vectorParam(nh, "joint_positions", info.joint_names.size(), 0.0, false);
  weights_ = Eigen::VectorXd::Zero(info.dimension());
  weights_.head<6>() = vectorParam(nh, "root_weights", 6, 0.0);
  weights_.tail(info.joint_names.size()) = vectorParam(nh, "joint_weights", info.joint_names.size(), 0.01);
  root_target_.translation() = vectorParam(nh, "root_position", 3, 0.0, false);
  root_target_.linear() = rotationExp(vectorParam(nh, "root_rotation_vector", 3, 0.0, false));
  if (info.lower.size() != joint_target_.size() || info.upper.size() != joint_target_.size() ||
      (joint_target_.array() < info.lower.array()).any() || (joint_target_.array() > info.upper.array()).any())
    throw std::runtime_error("nominal joint posture exceeds model joint limits");
}
bool NominalPostureCost::update(const MotionContext& context, QPProblem& problem)
{
  if (weights_.size() != problem.g.size() || context.state.joints.size() != joint_target_.size() ||
      !context.state.root.matrix().allFinite() || !context.state.joints.allFinite()) return false;
  Eigen::VectorXd desired(problem.g.size());
  desired.head<6>() = poseError(root_target_, context.state.root);
  desired.tail(joint_target_.size()) = joint_target_ - context.state.joints;
  desired *= gain_;
  problem.addCost((2.0 * weights_).asDiagonal(), -2.0 * weights_.cwiseProduct(desired));
  return true;
}
}}
PLUGINLIB_EXPORT_CLASS(aerial_robot_motion::cost::NominalPostureCost, aerial_robot_motion::cost::Base)
