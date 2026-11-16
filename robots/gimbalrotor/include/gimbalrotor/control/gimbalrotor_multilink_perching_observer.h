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
  Eigen::Vector3d q = Eigen::Vector3d::Zero(), rate = q, omega = q, effort = q, gyro = q;
  Eigen::Matrix<double, 6, 1> wrench = Eigen::Matrix<double, 6, 1>::Zero();
  // A 0.001534-rad roll encoder tick must not become a 0.077-rad/s
  // impulse at 50 Hz. This observation bandwidth leaves margin below the
  // unchanged 0.01-rad/s consistency bound; it does not filter flight feedback.
  double time_constant = .4;
  bool observe(const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation,
               const Eigen::Vector3d& measured_gyro, const Eigen::Vector3d& torque,
               const Eigen::Matrix<double, 6, 1>& input, double stamp, double max_interval);
  void reset();
private:
  double stamp_ = 0, started_ = 0;
  Eigen::Vector3d previous_ = Eigen::Vector3d::Zero();
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
