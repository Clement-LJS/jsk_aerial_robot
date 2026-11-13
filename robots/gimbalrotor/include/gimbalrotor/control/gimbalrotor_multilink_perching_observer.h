// -*- mode: c++ -*-
#pragma once
#include <Eigen/Core>
#include <string>

namespace perching_dynamics
{
// n-dimensional generalized load, never a Cartesian wrench. Revolute entries
// are N.m; friction, compliance and modelling error also appear in the residual.
class MomentumObserver
{
public:
  void reset(const std::string& reason = "awaiting first attached sample");
  bool observe(const Eigen::VectorXd& momentum, const Eigen::VectorXd& predicted_rate,
               double stamp, double gain, double max_dt);
  bool ready() const { return ready_; }
  double stamp() const { return stamp_; }
  const std::string& reason() const { return reason_; }
  const Eigen::VectorXd& load() const { return residual_; }
private:
  Eigen::VectorXd initial_, integral_, residual_;
  double stamp_ = 0;
  bool ready_ = false;
  std::string reason_ = "awaiting first attached sample";
};
} // namespace perching_dynamics
