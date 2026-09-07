#pragma once
#include <aerial_robot_motion/cost/base_plugin.h>

namespace aerial_robot_motion { namespace cost {
class NominalPostureCost : public Base
{
public:
  void initialize(const ros::NodeHandle& nh, const ModelInfo& info) override;
  bool update(const MotionContext& context, QPProblem& problem) override;
private:
  Eigen::Isometry3d root_target_ = Eigen::Isometry3d::Identity();
  Eigen::VectorXd joint_target_, weights_;
  double gain_ = 0.5;
};
}}
