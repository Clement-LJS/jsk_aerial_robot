#include <gimbalrotor/perching_dynamics.h>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

namespace perching_dynamics
{
Eigen::Matrix<double, 6, 1> rotateWrench(const Eigen::Matrix3d& rotation, const Eigen::Matrix<double, 6, 1>& wrench)
{
  Eigen::Matrix<double, 6, 1> rotated;
  rotated.head<3>() = rotation * wrench.head<3>();
  rotated.tail<3>() = rotation * wrench.tail<3>();
  return rotated;
}

Eigen::Vector3d shiftMoment(const Eigen::Vector3d& moment, const Eigen::Vector3d& source_minus_target,
                            const Eigen::Vector3d& force)
{
  return moment + source_minus_target.cross(force);
}
}  // namespace perching_dynamics

namespace roll_yaw
{
bool pathInteractionTorque(const Kinematics& model, const Eigen::Vector3d& q, double direction, double min_singular,
                           const Eigen::Vector3d& force, const Eigen::Vector3d& moment, const Eigen::Vector3d& point,
                           double& torque)
{
  Eigen::Matrix3d j;
  Eigen::Isometry3d body, bp, bm;
  if((direction != 1 && direction != -1) || !force.allFinite() || !moment.allFinite() || !point.allFinite() ||
     !model.jacobian(q, j) || !model.evaluateBody(q, body))
    return false;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(j, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if(svd.singularValues().minCoeff() < min_singular)
    return false;
  const Eigen::Vector3d tangent = svd.solve(direction * Eigen::Vector3d::UnitY());
  constexpr double h = 1e-5;
  if(!model.evaluateBody(q + h * tangent, bp) || !model.evaluateBody(q - h * tangent, bm))
    return false;
  const Eigen::Vector3d linear = (bp.translation() - bm.translation()) / (2 * h);
  const Eigen::Vector3d angular = body.linear() * (direction * Eigen::Vector3d::UnitY());
  const Eigen::Vector3d about_body = perching_dynamics::shiftMoment(moment, point - body.translation(), force);
  torque = force.dot(linear) + about_body.dot(angular);
  return std::isfinite(torque);
}
ArticulatedDynamics::ArticulatedDynamics(Kinematics model, const KDL::Tree& tree,
                                         const std::map<std::string, KDL::RigidBodyInertia>& inertia,
                                         const std::map<std::string, KDL::Segment>& extra, std::string base,
                                         unsigned roll, unsigned yaw, Eigen::Vector3d gravity)
    : model_(std::move(model)), fk_(tree), base_(std::move(base)), roll_(roll), yaw_(yaw), gravity_(gravity)
{
  for(const auto& item : inertia)
  {
    addLink(item.first, Eigen::Isometry3d::Identity(), item.second);
    for(const auto& e : extra)
      if(e.second.getName() == item.first)
        addLink(item.first, fromKdl(e.second.getFrameToTip()), e.second.getInertia());
  }
}
void ArticulatedDynamics::addLink(const std::string& name, Eigen::Isometry3d local, KDL::RigidBodyInertia value)
{
  if(value.getMass() <= 0)
    return;
  const auto center = value.getCOG();
  const auto tensor = value.RefPoint(center).getRotationalInertia();
  local.translation() += local.linear() * Eigen::Vector3d(center.x(), center.y(), center.z());
  Eigen::Matrix3d i;
  for(int r = 0; r < 3; ++r)
    for(int c = 0; c < 3; ++c)
      i(r, c) = tensor.data[3 * r + c];
  links_.push_back({name, local, value.getMass(), i});
}
bool ArticulatedDynamics::poses(const Eigen::Vector3d& eta, const KDL::JntArray& input,
                                std::vector<Eigen::Isometry3d>& out)
{
  if(roll_ >= input.rows() || yaw_ >= input.rows() || !input.data.allFinite())
    return false;
  KDL::JntArray joints = input;
  joints(roll_) = eta[1];
  joints(yaw_) = eta[2];
  KDL::Frame base;
  Eigen::Isometry3d body, cog;
  if(fk_.JntToCart(joints, base, base_) < 0 || !model_.evaluate(eta, body, cog))
    return false;
  const Eigen::Isometry3d root = body * fromKdl(base).inverse();
  out.resize(links_.size());
  for(std::size_t i = 0; i < links_.size(); ++i)
  {
    KDL::Frame f;
    if(fk_.JntToCart(joints, f, links_[i].name) < 0)
      return false;
    out[i] = root * fromKdl(f) * links_[i].local;
    if(!poseValid(out[i]))
      return false;
  }
  return !out.empty();
}
bool ArticulatedDynamics::energy(const Eigen::Vector3d& q, const Eigen::Vector3d& v, const KDL::JntArray& joints,
                                 const KDL::JntArray& rates, DynamicsTerms& out)
{
  constexpr double h = 1e-5;
  std::vector<Eigen::Isometry3d> center, plus, minus;
  if(!poses(q, joints, center))
    return false;
  KDL::JntArray jp = joints, jm = joints;
  jp.data += h * rates.data;
  jm.data -= h * rates.data;
  if(!poses(q + h * v, jp, plus) || !poses(q - h * v, jm, minus))
    return false;
  std::vector<Eigen::Vector3d> linear(center.size()), angular(center.size());
  std::vector<Eigen::Matrix3d> jv(center.size(), Eigen::Matrix3d::Zero()), jw(center.size(), Eigen::Matrix3d::Zero());
  out.kinetic = out.potential = 0;
  out.momentum.setZero();
  out.mass.setZero();
  for(std::size_t i = 0; i < center.size(); ++i)
  {
    linear[i] = (plus[i].translation() - minus[i].translation()) / (2 * h);
    angular[i] = logSO3(plus[i].linear() * minus[i].linear().transpose()) / (2 * h);
    const Eigen::Matrix3d iw = center[i].linear() * links_[i].inertia * center[i].linear().transpose();
    out.kinetic += .5 * (links_[i].mass * linear[i].squaredNorm() + angular[i].dot(iw * angular[i]));
    out.potential -= links_[i].mass * gravity_.dot(center[i].translation());
  }
  for(int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    d[k] = h;
    if(!poses(q + d, joints, plus) || !poses(q - d, joints, minus))
      return false;
    for(std::size_t i = 0; i < center.size(); ++i)
    {
      jv[i].col(k) = (plus[i].translation() - minus[i].translation()) / (2 * h);
      jw[i].col(k) = logSO3(plus[i].linear() * minus[i].linear().transpose()) / (2 * h);
    }
  }
  for(std::size_t i = 0; i < center.size(); ++i)
  {
    const Eigen::Matrix3d iw = center[i].linear() * links_[i].inertia * center[i].linear().transpose();
    out.momentum += links_[i].mass * jv[i].transpose() * linear[i] + jw[i].transpose() * iw * angular[i];
    out.mass += links_[i].mass * jv[i].transpose() * jv[i] + jw[i].transpose() * iw * jw[i];
  }
  return out.mass.allFinite() && out.momentum.allFinite() && std::isfinite(out.kinetic) && std::isfinite(out.potential);
}
DynamicsTerms ArticulatedDynamics::evaluate(const Eigen::Vector3d& q, const Eigen::Vector3d& v,
                                            const KDL::JntArray& joints, const KDL::JntArray& rates,
                                            const Eigen::Vector3d& force, const Eigen::Vector3d& torque,
                                            const Eigen::Vector3d& servo, double min_singular,
                                            const std::function<bool()>& cancelled)
{
  DynamicsTerms out;
  Eigen::Matrix3d j;
  Eigen::Isometry3d body, cog;
  out.reason = "invalid dynamics input/kinematics";
  if(!q.allFinite() || !v.allFinite() || !force.allFinite() || !torque.allFinite() || !servo.allFinite() ||
     joints.rows() != rates.rows() || !rates.data.allFinite() || !model_.jacobian(q, j) ||
     !model_.evaluate(q, body, cog))
    return out;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(j, Eigen::ComputeFullU | Eigen::ComputeFullV);
  if(svd.singularValues().minCoeff() < min_singular)
  {
    out.reason = "singular task torque map";
    return out;
  }
  out.pitch_map = svd.solve(Eigen::Vector3d::UnitY());
  if((j * out.pitch_map - Eigen::Vector3d::UnitY()).norm() > 1e-8 || !energy(q, v, joints, rates, out))
    return out;
  if(Eigen::LDLT<Eigen::Matrix3d>(out.mass).vectorD().minCoeff() <= 0)
  {
    out.reason = "nonpositive generalized inertia";
    return out;
  }
  constexpr double h = 1e-4;
  // A resultant applied to the BODY at the instantaneous CoG location. Its
  // translational Jacobian is a body material-point Jacobian, NOT d(CoG)/deta.
  // This is equivalent to summing body-branch rotor forces with frozen gimbals.
  std::vector<Eigen::Isometry3d> centers;
  if(!poses(q, joints, centers))
    return out;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double mass = 0;
  for(std::size_t i = 0; i < centers.size(); ++i)
  {
    center += links_[i].mass * centers[i].translation();
    mass += links_[i].mass;
  }
  if(mass <= 0)
    return out;
  const Eigen::Vector3d material = body.inverse() * (center / mass);
  Eigen::Matrix3d jv;
  for(int k = 0; k < 3; ++k)
  {
    if(cancelled && cancelled())
    {
      out.reason = "dynamics cancelled/deadline";
      return out;
    }
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    d[k] = h;
    DynamicsTerms p, m;
    Eigen::Isometry3d bp, bm, g;
    if(!energy(q + d, v, joints, rates, p) || !energy(q - d, v, joints, rates, m) || !model_.evaluate(q + d, bp, g) ||
       !model_.evaluate(q - d, bm, g))
      return out;
    out.drift[k] = (p.kinetic - m.kinetic - p.potential + m.potential) / (2 * h);
    jv.col(k) = ((bp * material) - (bm * material)) / (2 * h);
  }
  out.drift += jv.transpose() * force + j.transpose() * body.linear().transpose() * torque +
               Eigen::Vector3d(0, servo[1], servo[2]);
  out.ok = out.drift.allFinite();
  out.reason =
      out.ok ? "articulated momentum; command-derived body wrench + scaled servo effort" : "nonfinite dynamics";
  return out;
}
bool MomentumObserver::update(const DynamicsTerms& terms, double stamp, double max_gap)
{
  if(!terms.ok || !std::isfinite(stamp) || stamp <= 0 || !std::isfinite(gain) || gain <= 0)
  {
    reset();
    return false;
  }
  const double dt = stamp - stamp_;
  if(initialized_ && dt == 0)
    return false;
  if(!initialized_ || dt < 0 || dt > max_gap)
  {
    initial_ = terms.momentum;
    integral_.setZero();
    residual.setZero();
    initialized_ = true;
    stamp_ = stamp;
    return false;
  }
  // Backward Euler for integral(b+r): stable for any positive gain and dt.
  residual = gain * (terms.momentum - initial_ - integral_ - dt * terms.drift) / (1 + gain * dt);
  integral_ += dt * (terms.drift + residual);
  stamp_ = stamp;
  torque = terms.pitch_map.dot(residual);
  if(!std::isfinite(torque))
  {
    reset();
    return false;
  }
  return true;
}
}  // namespace roll_yaw
