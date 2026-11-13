#include <gimbalrotor/model/gimbalrotor_multilink_perching_model.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

namespace perching_dynamics
{
namespace
{
constexpr double difference_step = 1e-5;
const Eigen::Vector3d gravity_world(0, 0, -9.80665);
}

bool MultilinkModel::configure(std::vector<std::string> coordinates, std::vector<Link> links,
                               Geometry geometry, std::string& reason)
{
  *this = MultilinkModel();
  reason = "invalid coordinates, geometry or link inertia";
  if(coordinates.empty() || links.empty() || !geometry)
    return false;
  double mass = 0;
  for(const auto& link : links)
  {
    if(!std::isfinite(link.mass) || link.mass < 0 || !link.inertia.allFinite() ||
       (link.inertia - link.inertia.transpose()).norm() > 1e-9)
      return false;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(link.inertia);
    if(eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() < -1e-10 ||
       eig.eigenvalues()[2] > eig.eigenvalues()[0] + eig.eigenvalues()[1] + 1e-9)
      return false;
    mass += link.mass;
  }
  if(mass <= 0)
    return false;
  coordinates_ = std::move(coordinates);
  links_ = std::move(links);
  geometry_ = std::move(geometry);
  reason.clear();
  return true;
}

bool MultilinkModel::metric(const Eigen::VectorXd& q, Eigen::MatrixXd& mass, double& potential) const
{
  if(!geometry_ || q.size() != static_cast<int>(coordinates_.size()) || !q.allFinite())
    return false;
  Poses center;
  Eigen::Isometry3d body;
  if(!geometry_(q, center, body) || !roll_yaw::poseValid(body) || center.size() != links_.size())
    return false;
  std::vector<Eigen::MatrixXd> linear(links_.size(), Eigen::MatrixXd(3, q.size()));
  auto angular = linear;
  for(int j = 0; j < q.size(); ++j)
  {
    Eigen::VectorXd plus = q, minus = q;
    plus[j] += difference_step;
    minus[j] -= difference_step;
    Poses p, m;
    if(!geometry_(plus, p, body) || !geometry_(minus, m, body) ||
       p.size() != center.size() || m.size() != center.size())
      return false;
    for(std::size_t i = 0; i < links_.size(); ++i)
    {
      if(!roll_yaw::poseValid(center[i]) || !roll_yaw::poseValid(p[i]) || !roll_yaw::poseValid(m[i]))
        return false;
      linear[i].col(j) = (p[i].translation() - m[i].translation()) / (2 * difference_step);
      angular[i].col(j) = roll_yaw::logSO3(p[i].linear() * m[i].linear().transpose()) / (2 * difference_step);
    }
  }
  mass = Eigen::MatrixXd::Zero(q.size(), q.size());
  potential = 0;
  for(std::size_t i = 0; i < links_.size(); ++i)
  {
    const auto& link = links_[i];
    const Eigen::Matrix3d inertia_world = center[i].linear() * link.inertia * center[i].linear().transpose();
    mass += link.mass * linear[i].transpose() * linear[i] + angular[i].transpose() * inertia_world * angular[i];
    potential -= link.mass * gravity_world.dot(center[i].translation());
  }
  mass = (.5 * (mass + mass.transpose())).eval();
  return mass.allFinite() && std::isfinite(potential);
}

bool MultilinkModel::evaluate(const Eigen::VectorXd& q, const Eigen::VectorXd& rate,
                              const Eigen::VectorXd& joint_load, const Eigen::Vector3d& force,
                              const Eigen::Vector3d& moment, const Eigen::Vector3d& offset,
                              Evaluation& out, std::string& reason) const
{
  out = Evaluation();
  reason = "invalid dynamics state/input or singular mass matrix";
  if(q.size() != rate.size() || q.size() != joint_load.size() || !rate.allFinite() ||
     !joint_load.allFinite() || !force.allFinite() || !moment.allFinite() || !offset.allFinite() ||
     !metric(q, out.mass, out.potential))
    return false;
  Eigen::LLT<Eigen::MatrixXd> factor(out.mass);
  if(factor.info() != Eigen::Success)
    return false;
  out.momentum = out.mass * rate;
  out.kinetic = .5 * rate.dot(out.momentum);
  out.gravity.resize(q.size());
  out.predicted_rate = joint_load;
  // Differentiate the complete Lagrangian with all other coordinates fixed.
  // No partial Coriolis matrix and no gravity counted in the input wrench.
  constexpr double h = 1e-4;
  for(int j = 0; j < q.size(); ++j)
  {
    Eigen::VectorXd plus = q, minus = q;
    plus[j] += h;
    minus[j] -= h;
    Eigen::MatrixXd mp, mm;
    double up, um;
    Poses unused;
    Eigen::Isometry3d bp, bm;
    if(!metric(plus, mp, up) || !metric(minus, mm, um) ||
       !geometry_(plus, unused, bp) || !geometry_(minus, unused, bm) ||
       !roll_yaw::poseValid(bp) || !roll_yaw::poseValid(bm))
      return false;
    out.gravity[j] = (up - um) / (2 * h);
    out.predicted_rate[j] += .5 * rate.dot((mp - mm) * rate) / (2 * h) - out.gravity[j];
    // u is a VIRTUAL MATERIAL point of B. Hold this instantaneous body offset
    // fixed in the virtual displacement, never differentiate aggregate CoG.
    const Eigen::Vector3d ju = (bp * offset - bm * offset) / (2 * h);
    const Eigen::Vector3d jw = roll_yaw::logSO3(bp.linear() * bm.linear().transpose()) / (2 * h);
    out.predicted_rate[j] += ju.dot(force) + jw.dot(moment);
  }
  if(!out.momentum.allFinite() || !out.predicted_rate.allFinite() || !out.gravity.allFinite() ||
     !std::isfinite(out.kinetic))
    return false;
  reason.clear();
  return true;
}

bool rollYawModel(const roll_yaw::Kinematics& geometry, const KDL::Tree& tree,
                 const KDL::JntArray& joints, const std::map<std::string, KDL::RigidBodyInertia>& inertia,
                 const std::map<std::string, KDL::Segment>& extra, const std::string& body,
                 unsigned ri, unsigned yi, const std::string& roll, const std::string& yaw,
                 MultilinkModel& result, std::string& reason)
{
  result = MultilinkModel();
  reason = "missing locked roll-yaw geometry, joint mapping or inertias";
  if(!geometry.locked() || ri == yi || ri >= joints.rows() || yi >= joints.rows() ||
     !joints.data.allFinite() || inertia.empty())
    return false;
  for(const auto& module : extra)
    if(inertia.count(module.second.getName()) != 1)
    {
      reason = "extra module parent is absent from inertia map";
      return false;
    }
  std::vector<std::string> names;
  std::vector<KDL::Vector> centers;
  std::vector<MultilinkModel::Link> links;
  for(const auto& item : inertia)
  {
    KDL::RigidBodyInertia aggregate = KDL::RigidBodyInertia::Zero();
    perching_geometry::accumulateLinkInertia(aggregate, KDL::Frame::Identity(), item.first, item.second, extra);
    MultilinkModel::Link link;
    link.mass = aggregate.getMass();
    // KDL getRotationalInertia() is about its reference origin, NOT its CoM.
    const auto ic = aggregate.RefPoint(aggregate.getCOG()).getRotationalInertia();
    for(int r = 0; r < 3; ++r)
      for(int c = 0; c < 3; ++c)
        link.inertia(r, c) = ic.data[3 * r + c];
    links.push_back(link);
    names.push_back(item.first);
    centers.push_back(aggregate.getCOG());
  }
  auto fk = [geometry, tree, joints, body, ri, yi, names, centers]
      (const Eigen::VectorXd& q, MultilinkModel::Poses& poses, Eigen::Isometry3d& wb)
  {
    if(q.size() != 3 || !geometry.evaluateBody(q, wb))
      return false;
    KDL::JntArray state = joints;
    state(ri) = q[1];
    state(yi) = q[2];
    KDL::TreeFkSolverPos_recursive solver(tree);
    KDL::Frame root_body;
    if(solver.JntToCart(state, root_body, body) < 0)
      return false;
    poses.clear();
    for(std::size_t i = 0; i < names.size(); ++i)
    {
      KDL::Frame root_link;
      if(solver.JntToCart(state, root_link, names[i]) < 0)
        return false;
      poses.push_back(wb * roll_yaw::fromKdl(root_body.Inverse() * root_link * KDL::Frame(centers[i])));
    }
    return true;
  };
  return result.configure({"alpha", roll, yaw}, std::move(links), std::move(fk), reason);
}
} // namespace perching_dynamics
