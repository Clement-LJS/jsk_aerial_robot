#pragma once
#include <aerial_robot_motion/constraint/base_plugin.h>

namespace aerial_robot_motion { namespace constraint {
class StaticThrustConstraint : public Base
{
public:
  void initialize(const ros::NodeHandle&, const ModelInfo&) override;
  bool update(const MotionContext&, QPProblem&) override;
private:
  double lower_ = 0.0, upper_ = 0.0;
};
}}
