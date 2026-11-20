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
// and which omitted dynamics are approximated versus physically constrained.
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
  // Wrench origin u may be the current physical CoG, independently of the
  // adapter's reduced inertial geometry. body_offset_u is B->u in body axes;
  // evaluate shifts the world moment to B exactly once, without differentiating u.
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

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <urdf/model.h>
#include <map>

namespace perching_dynamics
{
// Coordinates are [external hinge, configured servos...]. A held servo stays in
// this model and in feedback; the task mask, not the mechanism, holds it still.
class SerialModel
{
public:
  using Pose = Eigen::Isometry3d;
  using Vector = Eigen::VectorXd;
  struct State
  {
    Pose body = Pose::Identity(), cog = Pose::Identity();
    std::vector<Pose> centers;
  };
  bool configure(const urdf::ModelInterface& urdf, const KDL::Tree& tree, const KDL::JntArray& joints,
                 const std::map<std::string, KDL::RigidBodyInertia>& inertias,
                 const std::map<std::string, KDL::Segment>& extra,
                 const std::string& body, const std::string& contact,
                 const std::vector<std::string>& active, const Pose& contact_pivot,
                 const Eigen::Vector3d& axis, const Eigen::Matrix3d& cog_to_body, std::string& reason);
  bool lock(const Pose& body, const Vector& q);
  bool evaluate(const Vector& q, State& state) const;
  bool bodyContact(const Vector& q, Pose& contact, Eigen::Vector3d& cog) const;
  bool estimate(const Pose& body, Vector& q, double& pivot_error, double& hinge_error) const;
  bool jacobian(const Vector& q, Eigen::MatrixXd& angular_body) const;
  bool pitchTangent(const Vector& q, const std::vector<bool>& moving, Vector& tangent) const;
  bool orient(const Vector& start, const Eigen::Matrix3d& target, const std::vector<bool>& moving, Vector& result) const;
  bool pitch(const Vector& start, double angle, const std::vector<bool>& moving, Vector& result) const;
  bool dynamics(MultilinkModel& result, std::string& reason) const;
  bool allowed(const Vector& q, double max_tilt, double min_pitch, double max_pitch) const;
  const std::vector<std::string>& coordinates() const { return coordinates_; }
  const Pose& pivot() const { return pivot_; }
  const Pose& contactPivot() const { return contact_pivot_; }
  const Eigen::Vector3d& axis() const { return axis_; }
  Vector lower, upper, velocity, acceleration;
  bool locked() const { return locked_; }
private:
  struct Node { KDL::Segment segment; std::size_t parent; int coordinate; double held; };
  bool frames(const Vector& q, std::vector<Pose>& poses) const;
  bool differential(const Vector& q, MultilinkModel::GeometryState& state) const;
  std::vector<Node> nodes_;
  std::vector<std::vector<bool>> descendants_;
  std::vector<std::string> coordinates_;
  std::vector<std::size_t> inertia_nodes_;
  std::vector<Eigen::Vector3d> centers_;
  std::vector<MultilinkModel::Link> links_;
  std::size_t body_ = 0, contact_ = 0;
  double mass_ = 0;
  Pose contact_pivot_ = Pose::Identity(), pivot_ = Pose::Identity();
  Eigen::Vector3d axis_ = Eigen::Vector3d::UnitY();
  Eigen::Matrix3d cog_to_body_ = Eigen::Matrix3d::Identity();
  bool locked_ = false;
};
}
