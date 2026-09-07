#pragma once
#include <aerial_robot_motion/constraint/base_plugin.h>

namespace aerial_robot_motion { namespace constraint {
class JointTorqueConstraint : public Base
{
public:
  void initialize(const ros::NodeHandle&, const ModelInfo&) override;
  bool update(const MotionContext&, QPProblem&) override;
private:
  Eigen::VectorXd effort_;
};
}}
