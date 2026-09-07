#pragma once
#include <aerial_robot_motion/constraint/base_plugin.h>
namespace aerial_robot_motion { namespace constraint {
class JointLimitConstraint : public Base
{
public:
  void initialize(const ros::NodeHandle&, const ModelInfo&) override;
  bool update(const MotionContext&, QPProblem&) override;
  static bool bounds(const ModelInfo&, const Eigen::VectorXd& q, double dt,
                     double damping_range, double margin, Eigen::VectorXd& lower, Eigen::VectorXd& upper);
private:
  double damping_range_ = 0.15, margin_ = 0.01;
};
}}
