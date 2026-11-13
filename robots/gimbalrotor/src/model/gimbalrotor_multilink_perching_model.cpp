#include <gimbalrotor/model/gimbalrotor_multilink_perching_model.h>
#include <cmath>
#include <utility>
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>

namespace perching_dynamics
{
namespace
{
constexpr double difference_step = 1e-5;
const Eigen::Vector3d gravity_world(0, 0, -9.80665);
bool poseValid(const Eigen::Isometry3d& p)
{
  return p.matrix().allFinite() &&
         (p.linear().transpose() * p.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-7 &&
         std::abs(p.linear().determinant() - 1) < 1e-7;
}
Eigen::Vector3d logSO3(const Eigen::Matrix3d& rotation)
{
  const Eigen::AngleAxisd angle(rotation);
  return angle.angle() * angle.axis();
}
}

bool MultilinkModel::configure(std::vector<std::string> coordinates, std::vector<Link> links,
                               Geometry geometry, std::string& reason, DifferentialGeometry differential)
{
  *this = MultilinkModel();
  reason = "invalid coordinates, geometry or link inertia";
  if(coordinates.empty() || links.empty() || (!geometry && !differential))
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
  differential_ = std::move(differential);
  reason.clear();
  return true;
}

bool MultilinkModel::state(const Eigen::VectorXd& q, GeometryState& s) const
{
  const int n = coordinates_.size();
  if(n == 0 || q.size() != n || !q.allFinite())
    return false;
  if(differential_)
  {
    if(!differential_(q, s))
      return false;
  }
  else
  {
    // Geometry-only fallback for future adapters; roll-yaw uses analytic columns.
    if(!geometry_ || !geometry_(q, s.centers, s.body) || s.centers.size() != links_.size())
      return false;
    s.linear.assign(links_.size(), Eigen::MatrixXd(3, n));
    s.angular = s.linear;
    s.body_linear.resize(3, n);
    s.body_angular.resize(3, n);
    for(int j = 0; j < n; ++j)
    {
      Eigen::VectorXd plus = q, minus = q;
      plus[j] += difference_step;
      minus[j] -= difference_step;
      Poses p, m;
      Eigen::Isometry3d bp, bm;
      if(!geometry_(plus, p, bp) || !geometry_(minus, m, bm) ||
         p.size() != links_.size() || m.size() != links_.size() || !poseValid(bp) || !poseValid(bm))
        return false;
      s.body_linear.col(j) = (bp.translation() - bm.translation()) / (2 * difference_step);
      s.body_angular.col(j) = logSO3(bp.linear() * bm.linear().transpose()) / (2 * difference_step);
      for(std::size_t i = 0; i < links_.size(); ++i)
      {
        if(!poseValid(p[i]) || !poseValid(m[i]))
          return false;
        s.linear[i].col(j) = (p[i].translation() - m[i].translation()) / (2 * difference_step);
        s.angular[i].col(j) = logSO3(p[i].linear() * m[i].linear().transpose()) / (2 * difference_step);
      }
    }
  }
  auto valid = [n](const Eigen::MatrixXd& j) { return j.rows() == 3 && j.cols() == n && j.allFinite(); };
  if(!poseValid(s.body) || s.centers.size() != links_.size() || s.linear.size() != links_.size() ||
     s.angular.size() != links_.size() || !valid(s.body_linear) || !valid(s.body_angular))
    return false;
  for(std::size_t i = 0; i < links_.size(); ++i)
    if(!poseValid(s.centers[i]) || !valid(s.linear[i]) || !valid(s.angular[i]))
      return false;
  return true;
}

bool MultilinkModel::energy(const GeometryState& s, const Eigen::VectorXd& rate,
                            double& kinetic, double& potential) const
{
  kinetic = potential = 0;
  for(std::size_t i = 0; i < links_.size(); ++i)
  {
    const auto& link = links_[i];
    const Eigen::Vector3d v = s.linear[i] * rate;
    const Eigen::Vector3d omega = s.centers[i].linear().transpose() * (s.angular[i] * rate);
    kinetic += .5 * (link.mass * v.squaredNorm() + omega.dot(link.inertia * omega));
    potential -= link.mass * gravity_world.dot(s.centers[i].translation());
  }
  return std::isfinite(kinetic) && std::isfinite(potential);
}

bool MultilinkModel::evaluate(const Eigen::VectorXd& q, const Eigen::VectorXd& rate,
                              const Eigen::VectorXd& joint_load, const Eigen::Vector3d& force,
                              const Eigen::Vector3d& moment, const Eigen::Vector3d& offset,
                              Evaluation& out, std::string& reason) const
{
  out = Evaluation();
  reason = "invalid dynamics state/input or singular mass matrix";
  GeometryState center;
  if(q.size() != rate.size() || q.size() != joint_load.size() || !rate.allFinite() ||
     !joint_load.allFinite() || !force.allFinite() || !moment.allFinite() || !offset.allFinite() ||
     !state(q, center) || !energy(center, rate, out.kinetic, out.potential))
    return false;
  out.mass = Eigen::MatrixXd::Zero(q.size(), q.size());
  for(std::size_t i = 0; i < links_.size(); ++i)
  {
    const Eigen::Matrix3d inertia_world = center.centers[i].linear() * links_[i].inertia *
                                         center.centers[i].linear().transpose();
    out.mass += links_[i].mass * center.linear[i].transpose() * center.linear[i] +
                center.angular[i].transpose() * inertia_world * center.angular[i];
  }
  out.mass = (.5 * (out.mass + out.mass.transpose())).eval();
  if(!out.mass.allFinite() || Eigen::LLT<Eigen::MatrixXd>(out.mass).info() != Eigen::Success)
    return false;
  out.momentum = out.mass * rate;
  out.gravity.resize(q.size());
  // Moment at body material origin B: tau_B = tau_u + (p_u - p_B) x f.
  // offset is frozen ONLY for virtual displacement, never a migrating-CoG Jacobian.
  const Eigen::Vector3d moment_body = moment + (center.body.linear() * offset).cross(force);
  out.predicted_rate = joint_load + center.body_linear.transpose() * force +
                       center.body_angular.transpose() * moment_body;
  // Direct partial(T-U)/partial q at fixed qdot. Differentiate scalar energy,
  // not whole mass matrices. This retains the configuration-dependent kinetic
  // term; gravity is counted only here and no separate C matrix is needed.
  constexpr double h = 1e-4;
  GeometryState displaced;
  for(int j = 0; j < q.size(); ++j)
  {
    Eigen::VectorXd plus = q, minus = q;
    plus[j] += h;
    minus[j] -= h;
    double tp, tm, up, um;
    if(!state(plus, displaced) || !energy(displaced, rate, tp, up) ||
       !state(minus, displaced) || !energy(displaced, rate, tm, um))
      return false;
    out.gravity[j] = (up - um) / (2 * h);
    out.predicted_rate[j] += (tp - tm) / (2 * h) - out.gravity[j];
  }
  if(!out.momentum.allFinite() || !out.predicted_rate.allFinite() || !out.gravity.allFinite())
    return false;
  reason.clear();
  return true;
}
} // namespace perching_dynamics
