#pragma once
#include <aerial_robot_motion/core/qp_problem.h>

namespace aerial_robot_motion
{
class SolverBase
{
public:
  virtual ~SolverBase() = default;
  virtual bool solve(const QPProblem& problem, Eigen::VectorXd& solution, std::string& status) = 0;
  virtual void reset() = 0;
};
}
