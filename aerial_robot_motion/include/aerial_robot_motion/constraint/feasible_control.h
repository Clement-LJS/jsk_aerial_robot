#pragma once
#include <aerial_robot_motion/constraint/base_plugin.h>

namespace aerial_robot_motion { namespace constraint {
class FeasibleControlConstraint : public Base
{
public:
  void initialize(const ros::NodeHandle&, const ModelInfo&) override;
  bool update(const MotionContext&, QPProblem&) override;
private:
  double force_margin_ = 0.0, torque_margin_ = 0.0;
};
}}
