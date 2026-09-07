#pragma once
#include <aerial_robot_motion/cost/base_plugin.h>

namespace aerial_robot_motion { namespace cost {
class StateVelocityCost : public Base
{
public:
  void initialize(const ros::NodeHandle& nh, const ModelInfo& info) override;
  bool update(const MotionContext& context, QPProblem& problem) override;
private:
  Eigen::VectorXd weights_ = Eigen::VectorXd::Constant(6, 0.01);
};
}}
