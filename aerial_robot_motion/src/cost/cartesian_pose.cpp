#include <aerial_robot_motion/cost/cartesian_pose.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <pluginlib/class_list_macros.h>

namespace aerial_robot_motion { namespace cost {
void CartesianPoseCost::initialize(const ros::NodeHandle& nh, const ModelInfo& info)
{
  Base::initialize(nh, info);
  gains_ = vectorParam(nh, "gains", 6, 2.0);
  weights_ = vectorParam(nh, "weights", 6, 1.0);
  const Eigen::VectorXd mask = vectorParam(nh, "axis_mask", 6, 1.0);
  for (int i = 0; i < 6; ++i)
    if (mask(i) != 0 && mask(i) != 1) throw std::runtime_error("Cartesian axis_mask values must be 0 or 1");
  weights_ = weights_.cwiseProduct(mask);
  max_twist_ = vectorParam(nh, "max_twist", 6, 0.2);
  std::string axes; nh.param<std::string>("axes_frame", axes, "world");
  if (axes != "world" && axes != "tool") throw std::runtime_error("Cartesian axes_frame must be world or tool");
  tool_axes_ = axes == "tool";
}
Eigen::Matrix<double, 6, 6> CartesianPoseCost::axesTransform(const MotionContext& context) const
{
  Eigen::Matrix<double, 6, 6> result = Eigen::Matrix<double, 6, 6>::Identity();
  if (tool_axes_)
  {
    result.topLeftCorner<3, 3>() = context.state.tool.linear().transpose();
    result.bottomRightCorner<3, 3>() = context.state.tool.linear().transpose();
  }
  return result;
}
Vector6 CartesianPoseCost::referenceTwist(const MotionContext& context) const
{
  const auto axes = axesTransform(context);
  Vector6 desired = gains_.cwiseProduct(axes * poseError(context.target, context.state.tool)) + axes * context.target_twist;
  return desired.cwiseMax(-max_twist_).cwiseMin(max_twist_);
}
Eigen::VectorXd CartesianPoseCost::residual(const MotionContext& context, const Eigen::VectorXd& velocity) const
{
  return axesTransform(context) * context.state.tool_jacobian * velocity - referenceTwist(context);
}
bool CartesianPoseCost::update(const MotionContext& context, QPProblem& problem)
{
  const auto& jacobian = context.state.tool_jacobian;
  if (jacobian.rows() != 6 || jacobian.cols() != problem.g.size() || !jacobian.allFinite() ||
      !context.target.matrix().allFinite() || !context.state.tool.matrix().allFinite() ||
      !context.target_twist.allFinite()) return false;
  const Vector6 desired = referenceTwist(context);
  const Eigen::MatrixXd task_jacobian = axesTransform(context) * jacobian;
  problem.addCost(2.0 * task_jacobian.transpose() * weights_.asDiagonal() * task_jacobian,
                  -2.0 * task_jacobian.transpose() * weights_.asDiagonal() * desired);
  return true;
}
}}
PLUGINLIB_EXPORT_CLASS(aerial_robot_motion::cost::CartesianPoseCost, aerial_robot_motion::cost::Base)
