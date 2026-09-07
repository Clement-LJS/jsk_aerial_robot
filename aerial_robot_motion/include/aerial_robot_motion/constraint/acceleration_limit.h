#pragma once
#include <aerial_robot_motion/constraint/base_plugin.h>

namespace aerial_robot_motion { namespace constraint {
class AccelerationLimitConstraint : public Base
{
public:
  void initialize(const ros::NodeHandle&, const ModelInfo&) override;
  bool update(const MotionContext&, QPProblem&) override;
  void solutionAccepted(const MotionContext&, const Eigen::VectorXd&) override;
  void reset() override;
  static bool bounds(const Eigen::VectorXd& previous, bool has_previous, double dt,
                     const Eigen::VectorXd& acceleration, Eigen::VectorXd& lower,
                     Eigen::VectorXd& upper);
private:
  Eigen::VectorXd acceleration_, previous_;
  bool has_previous_ = false;
};
}}
