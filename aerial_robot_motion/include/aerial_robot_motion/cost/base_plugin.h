#pragma once
#include <aerial_robot_motion/core/motion_state.h>
#include <aerial_robot_motion/core/qp_problem.h>
#include <ros/node_handle.h>

namespace aerial_robot_motion { namespace cost {
class Base
{
public:
  virtual ~Base() = default;
  virtual void initialize(const ros::NodeHandle& nh, const ModelInfo& info)
    { info_ = info; name_ = nh.getNamespace(); }
  virtual bool update(const MotionContext& context, QPProblem& problem) = 0;
  virtual Eigen::VectorXd residual(const MotionContext&, const Eigen::VectorXd&) const
    { return Eigen::VectorXd{}; }
  const std::string& name() const { return name_; }
protected:
  ModelInfo info_;
  std::string name_;
};
}}
