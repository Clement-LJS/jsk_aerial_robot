#pragma once
#include <gimbalrotor/perching_geometry.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <vector>

namespace perching_dynamics
{
// Rotate axes only; the wrench remains about the same physical point.
Eigen::Matrix<double, 6, 1> rotateWrench(const Eigen::Matrix3d& rotation, const Eigen::Matrix<double, 6, 1>& wrench);

// All vectors use the same axes. source_minus_target points from the new
// reference point to the old one: M_target = M_source + r x F.
Eigen::Vector3d shiftMoment(const Eigen::Vector3d& moment, const Eigen::Vector3d& source_minus_target,
                            const Eigen::Vector3d& force);
}  // namespace perching_dynamics

namespace roll_yaw
{
// Generalized work per radian of signed body-local pitch progress. Wrench and
// its reference point are world expressed; includes fixed-pivot translation.
bool pathInteractionTorque(const Kinematics&, const Eigen::Vector3d& eta, double direction, double min_singular,
                           const Eigen::Vector3d& force_world, const Eigen::Vector3d& moment_world,
                           const Eigen::Vector3d& point_world, double& torque);
struct DynamicsTerms
{
  bool ok = false;
  Eigen::Vector3d momentum = Eigen::Vector3d::Zero(), drift = Eigen::Vector3d::Zero(),
                  pitch_map = Eigen::Vector3d::Zero();
  Eigen::Matrix3d mass = Eigen::Matrix3d::Zero();
  double kinetic = 0, potential = 0;
  std::string reason = "no dynamics sample";
};
// Worker-owned copied model. The inertia map already aggregates fixed children;
// do not sum tree inertias a second time. Extra modules enter exactly once.
class ArticulatedDynamics
{
public:
  ArticulatedDynamics(Kinematics model, const KDL::Tree&, const std::map<std::string, KDL::RigidBodyInertia>&,
                      const std::map<std::string, KDL::Segment>&, std::string base, unsigned roll, unsigned yaw,
                      Eigen::Vector3d gravity);
  DynamicsTerms evaluate(const Eigen::Vector3d& eta, const Eigen::Vector3d& rate, const KDL::JntArray& joints,
                         const KDL::JntArray& rates, const Eigen::Vector3d& force_world,
                         const Eigen::Vector3d& torque_cog_world, const Eigen::Vector3d& servo, double min_singular,
                         const std::function<bool()>& cancelled);

private:
  struct Link
  {
    std::string name;
    Eigen::Isometry3d local;
    double mass;
    Eigen::Matrix3d inertia;
  };
  void addLink(const std::string& name, Eigen::Isometry3d local, KDL::RigidBodyInertia value);
  bool poses(const Eigen::Vector3d&, const KDL::JntArray&, std::vector<Eigen::Isometry3d>&);
  bool energy(const Eigen::Vector3d&, const Eigen::Vector3d&, const KDL::JntArray&, const KDL::JntArray&,
              DynamicsTerms&);
  Kinematics model_;
  KDL::TreeFkSolverPos_recursive fk_;
  std::vector<Link> links_;
  std::string base_;
  unsigned roll_, yaw_;
  Eigen::Vector3d gravity_;
};
class MomentumObserver
{
public:
  double gain = 10, torque = 0;
  Eigen::Vector3d residual = Eigen::Vector3d::Zero();
  bool update(const DynamicsTerms&, double stamp, double max_gap);
  void reset()
  {
    initialized_ = false;
    stamp_ = 0;
    residual.setZero();
    torque = 0;
  }

private:
  bool initialized_ = false;
  double stamp_ = 0;
  Eigen::Vector3d initial_ = Eigen::Vector3d::Zero(), integral_ = Eigen::Vector3d::Zero();
};
}  // namespace roll_yaw
