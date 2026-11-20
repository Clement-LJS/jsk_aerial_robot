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
  // Optional differential geometry, all Jacobians world-expressed at their own
  // material origins/CoMs. The caller serializes evaluation and lock/snapshot
  // updates through the shared navigator session mutex.
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

#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <gimbalrotor/gimbalrotor_perching_navigation.h>

class GimbalrotorPerchingModel : public GimbalrotorRobotModel
{
public:
  using GimbalrotorRobotModel::GimbalrotorRobotModel;
  using Pose = Eigen::Isometry3d;
  using Vector = Eigen::VectorXd;
  struct State
  {
    Pose body = Pose::Identity(), cog = Pose::Identity();
    std::vector<Pose> centers;
  };
  virtual bool configure(const std::vector<std::string>& active, std::string& reason);
  bool bodyContact(const Vector& q, Pose& contact, Eigen::Vector3d& cog) const;
  Pose bodyPivot() const;
  bool lock(const Pose& body, const Vector& q, const Eigen::Vector3d* world_pivot = nullptr);
  void unlock() { locked_ = false; hinge_arc = perching_geometry::Lock(); }
  bool locked() const { return locked_; }
  bool evaluate(const Vector& q, State& state) const;
  bool estimate(const Pose& body, Vector& q, double& pivot_error, double& hinge_error) const;
  bool jacobian(const Vector& q, Eigen::MatrixXd& angular_body) const;
  virtual bool pitchTangent(const Vector& q, const std::vector<bool>& moving, Vector& tangent) const;
  virtual bool orient(const Vector& start, const Eigen::Matrix3d& target, const std::vector<bool>& moving, Vector& result) const;
  virtual bool pitch(const Vector& start, double angle, const std::vector<bool>& moving, Vector& result) const;
  bool dynamics(perching_dynamics::MultilinkModel& result, std::string& reason) const;
  bool allowed(const Vector& q, double max_tilt, double min_pitch, double max_pitch) const;
  // World wrench at the body-attached CoG, mapped by virtual work at q.
  bool effort(const Vector& q, const Vector& rate, const Vector& acceleration, const Vector& joint_effort,
              Eigen::Matrix<double, 6, 1>& wrench) const;
  const std::vector<std::string>& coordinates() const { return coordinates_; }
  const Pose& pivot() const { return pivot_; }
  Vector lower, upper, velocity, acceleration;
  perching_geometry::Lock hinge_arc;
protected:
  bool frames(const Vector& q, std::map<std::string, KDL::Frame>& poses) const;
  bool differential(const Vector& q, perching_dynamics::MultilinkModel::GeometryState& state) const;
  std::vector<std::string> coordinates_, inertia_links_;
  std::vector<Eigen::Vector3d> centers_;
  std::vector<perching_dynamics::MultilinkModel::Link> links_;
  KDL::JntArray held_joints_;
  double mass_ = 0;
  Pose pivot_ = Pose::Identity();
  Eigen::Matrix3d cog_to_body_ = Eigen::Matrix3d::Identity();
  bool locked_ = false;
};
