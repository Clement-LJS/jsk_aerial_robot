// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/gimbalrotor_perching.h>

namespace perching_dynamics
{
// Pure reduced-coordinate dynamics. The initial adapter freezes every joint
// except roll/yaw; its caller MUST verify those measured positions AND rates.
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
  struct Evaluation
  {
    Eigen::MatrixXd mass;
    Eigen::VectorXd momentum, gravity, predicted_rate;
    double kinetic = 0, potential = 0;
  };

  bool configure(std::vector<std::string> coordinates, std::vector<Link> links, Geometry geometry,
                 std::string& reason);
  bool evaluate(const Eigen::VectorXd& q, const Eigen::VectorXd& rate,
                const Eigen::VectorXd& joint_load, const Eigen::Vector3d& force_world,
                const Eigen::Vector3d& moment_world_at_u, const Eigen::Vector3d& body_offset_u,
                Evaluation& result, std::string& reason) const;
  const std::vector<std::string>& coordinates() const { return coordinates_; }

private:
  bool metric(const Eigen::VectorXd&, Eigen::MatrixXd&, double&) const;
  std::vector<std::string> coordinates_;
  std::vector<Link> links_;
  Geometry geometry_;
};

// Immutable KDL/inertia snapshot. Aggregated rigid links and extra modules are
// included once, using the same aggregation convention as articulatedCogModel.
bool rollYawModel(const roll_yaw::Kinematics& geometry, const KDL::Tree& tree,
                 const KDL::JntArray& joints, const std::map<std::string, KDL::RigidBodyInertia>& inertia,
                 const std::map<std::string, KDL::Segment>& extra, const std::string& body,
                 unsigned roll_index, unsigned yaw_index, const std::string& roll, const std::string& yaw,
                 MultilinkModel& result, std::string& reason);
} // namespace perching_dynamics
