#pragma once
#include <aerial_robot_motion/constraint/base_plugin.h>
namespace aerial_robot_motion { namespace constraint {
class StateLimitConstraint : public Base
{
public:
  void initialize(const ros::NodeHandle&, const ModelInfo&) override;
  bool update(const MotionContext&, QPProblem&) override;
private:
  Eigen::Vector3d linear_ = Eigen::Vector3d::Constant(0.1);
  Eigen::Vector3d angular_ = Eigen::Vector3d::Constant(0.15);
  double translation_step_ = 0.005, rotation_step_ = 0.005;
};
}}
