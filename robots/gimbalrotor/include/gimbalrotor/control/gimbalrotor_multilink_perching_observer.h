// -*- mode: c++ -*-
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

namespace perching_dynamics
{
// Observer-only conditioning. The flight attitude controller keeps its original
// gyro feedback. All inputs here refer to one interpolated measurement time.
class ObservationFilter
{
public:
  Eigen::VectorXd q, rate, effort;
  Eigen::Vector3d omega = Eigen::Vector3d::Zero(), gyro = omega;
  Eigen::Matrix<double, 6, 1> wrench = Eigen::Matrix<double, 6, 1>::Zero();
  // Condition measured coordinates and their derivatives at one bandwidth:
  // a 0.001534-rad roll encoder tick otherwise gives 0.077 rad/s at 50 Hz.
  // Body-rate disagreement is diagnostic only; flight feedback is unchanged.
  double time_constant = .4;
  bool observe(const Eigen::VectorXd& position, const Eigen::Matrix3d& rotation,
               const Eigen::Vector3d& measured_gyro, const Eigen::VectorXd& torque,
               const Eigen::Matrix<double, 6, 1>& input, double stamp, double max_interval);
  void reset();
private:
  double stamp_ = 0, started_ = 0;
  Eigen::VectorXd previous_;
  Eigen::Matrix3d previous_rotation_ = Eigen::Matrix3d::Identity();
};

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
