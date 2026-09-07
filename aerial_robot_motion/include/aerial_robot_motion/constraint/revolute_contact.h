#pragma once
#include <aerial_robot_motion/constraint/base_plugin.h>
namespace aerial_robot_motion { namespace constraint {
class RevoluteContactConstraint : public Base
{
public:
  void initialize(const ros::NodeHandle&, const ModelInfo&) override;
  bool update(const MotionContext&, QPProblem&) override;
  static bool equality(const MotionContext&, double position_gain, double orientation_gain,
                       double max_position_velocity, double max_orientation_velocity,
                       Eigen::MatrixXd& a, Eigen::VectorXd& b);
private:
  double position_gain_ = 3.0, orientation_gain_ = 3.0;
  double max_position_velocity_ = 0.05, max_orientation_velocity_ = 0.2;
};
}}
