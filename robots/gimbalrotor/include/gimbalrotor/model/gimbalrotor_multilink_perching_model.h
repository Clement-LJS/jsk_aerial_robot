// -*- mode: c++ -*-
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <functional>
#include <string>
#include <vector>

namespace perching_dynamics
{
// Pure reduced-coordinate dynamics. Adapters define the independent coordinates
// and must enforce the validity of any omitted/frozen coordinates.
// Rotor spin/angular momentum is omitted, as in RobotModel's inertia map.
class MultilinkModel
{
public:
  struct Link
  {
    double mass = 0;
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero(); // about own CoM, link axes
  };
  using Poses = std::vector<Eigen::Isometry3d>;
  // Return world poses at each link CoM, plus the chosen body reference B.
  using Geometry = std::function<bool(const Eigen::VectorXd&, Poses&, Eigen::Isometry3d&)>;
  // Optional analytic geometry, all Jacobians world-expressed at their own
  // material origins/CoMs. Callbacks must own snapshots, with no shared mutable
  // scratch: evaluate() may be called concurrently on a model or its copies.
  struct GeometryState
  {
    Poses centers;
    Eigen::Isometry3d body = Eigen::Isometry3d::Identity();
    std::vector<Eigen::MatrixXd> linear, angular;
    Eigen::MatrixXd body_linear, body_angular;
  };
  using DifferentialGeometry = std::function<bool(const Eigen::VectorXd&, GeometryState&)>;
  struct Evaluation
  {
    Eigen::MatrixXd mass;
    Eigen::VectorXd momentum, gravity, predicted_rate;
    double kinetic = 0, potential = 0;
  };

  bool configure(std::vector<std::string> coordinates, std::vector<Link> links, Geometry geometry,
                 std::string& reason, DifferentialGeometry differential = {});
  bool evaluate(const Eigen::VectorXd& q, const Eigen::VectorXd& rate,
                const Eigen::VectorXd& joint_load, const Eigen::Vector3d& force_world,
                const Eigen::Vector3d& moment_world_at_u, const Eigen::Vector3d& body_offset_u,
                Evaluation& result, std::string& reason) const;
  const std::vector<std::string>& coordinates() const { return coordinates_; }

private:
  bool state(const Eigen::VectorXd&, GeometryState&) const;
  bool energy(const GeometryState&, const Eigen::VectorXd&, double&, double&) const;
  std::vector<std::string> coordinates_;
  std::vector<Link> links_;
  Geometry geometry_;
  DifferentialGeometry differential_;
};

} // namespace perching_dynamics
