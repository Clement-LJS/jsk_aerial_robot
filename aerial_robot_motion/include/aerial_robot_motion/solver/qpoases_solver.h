#pragma once
#include <aerial_robot_motion/core/solver_base.h>
#include <memory>
#include <ros/node_handle.h>

namespace aerial_robot_motion
{
class QpOasesSolver : public SolverBase
{
public:
  QpOasesSolver();
  ~QpOasesSolver() override;
  void configure(const ros::NodeHandle& nh);
  bool solve(const QPProblem& problem, Eigen::VectorXd& solution, std::string& status) override;
  void reset() override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
