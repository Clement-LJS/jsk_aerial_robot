#pragma once
#include <aerial_robot_motion/cost/base_plugin.h>

namespace aerial_robot_motion { namespace cost {
class CartesianPoseCost : public Base
{
public:
  void initialize(const ros::NodeHandle& nh, const ModelInfo& info) override;
  bool update(const MotionContext& context, QPProblem& problem) override;
  Eigen::VectorXd residual(const MotionContext&, const Eigen::VectorXd&) const override;
private:
  Vector6 referenceTwist(const MotionContext&) const;
  Eigen::Matrix<double, 6, 6> axesTransform(const MotionContext&) const;
  bool tool_axes_ = false;
  Vector6 gains_ = Vector6::Constant(2.0), weights_ = Vector6::Ones();
  Vector6 max_twist_ = Vector6::Constant(0.2);
};
}}
