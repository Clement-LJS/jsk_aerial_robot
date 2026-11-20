#include <gimbalrotor/control/gimbalrotor_multilink_perching_observer.h>
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
using perching_geometry::logSO3;
using perching_geometry::poseValid;
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
    // Geometry fallback differentiates every configured generalized coordinate.
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

namespace perching_dynamics
{
void ObservationFilter::reset()
{
  const double tau = time_constant;
  *this = ObservationFilter();
  time_constant = tau;
}

bool ObservationFilter::observe(const Eigen::VectorXd& position, const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& measured_gyro, const Eigen::VectorXd& torque,
    const Eigen::Matrix<double, 6, 1>& input, double stamp, double max_interval)
{
  if(position.size() == 0 || position.size() != torque.size() || !position.allFinite() || !rotation.allFinite() || !measured_gyro.allFinite() || !torque.allFinite() ||
     !input.allFinite() || !std::isfinite(stamp) || stamp <= 0 || !std::isfinite(time_constant) ||
     time_constant <= 0 || !std::isfinite(max_interval) || max_interval <= 0) return false;
  if(stamp_ > 0 && stamp <= stamp_) return false;
  const double dt = stamp - stamp_;
  if(stamp_ == 0 || dt > max_interval || q.size() != position.size())
  {
    q = previous_ = position;
    rate = Eigen::VectorXd::Zero(position.size()); omega.setZero(); gyro = measured_gyro; effort = torque; wrench = input;
    previous_rotation_ = rotation;
    stamp_ = started_ = stamp;
    return false;
  }
  const double weight = -std::expm1(-dt / time_constant);
  rate += weight * ((position - previous_) / dt - rate);
  q += weight * (position - q);
  // Independent body-rate diagnostic from measured pose history. Generalized
  // momentum uses only q/rate above, including the measured passive derivative.
  // Transport pose/gyro rates to the current measured body axes; neither replaces
  // generalized rates nor the unconditioned gyro used by flight feedback.
  const Eigen::Matrix3d transport = rotation.transpose() * previous_rotation_;
  const Eigen::Vector3d measured_rate = transport *
      perching_geometry::logSO3(previous_rotation_.transpose() * rotation) / dt;
  omega = (1 - weight) * transport * omega + weight * measured_rate;
  gyro = (1 - weight) * transport * gyro + weight * measured_gyro;
  effort += weight * (torque - effort);
  wrench += weight * (input - wrench);
  previous_ = position; previous_rotation_ = rotation; stamp_ = stamp;
  return stamp - started_ >= 2 * time_constant;
}

void MomentumObserver::reset(const std::string& reason)
{
  initial_.resize(0);
  integral_.resize(0);
  residual_.resize(0);
  stamp_ = 0;
  ready_ = false;
  reason_ = reason;
}

bool MomentumObserver::observe(const Eigen::VectorXd& p, const Eigen::VectorXd& b,
                               double stamp, double gain, double max_dt)
{
  if(p.size() == 0 || p.size() != b.size() || !p.allFinite() || !b.allFinite() ||
     !std::isfinite(stamp) || stamp <= 0 || !std::isfinite(gain) || gain <= 0 ||
     !std::isfinite(max_dt) || max_dt <= 0)
  {
    reset("invalid momentum observer input/gain");
    return false;
  }
  if(stamp_ == 0)
  {
    initial_ = p;
    integral_ = residual_ = Eigen::VectorXd::Zero(p.size());
    stamp_ = stamp;
    ready_ = false;
    reason_ = "rebased; need an advancing synchronized sample";
    return false;
  }
  const double dt = stamp - stamp_;
  if(dt <= 0 || dt > max_dt || p.size() != initial_.size())
  {
    reset("invalid observer dt/dimension; rebase required");
    return false;
  }
  residual_ = (gain / (1 + gain * dt)) * (p - initial_ - integral_ - dt * b);
  integral_ += dt * (b + residual_);
  if(!residual_.allFinite() || !integral_.allFinite())
  {
    reset("nonfinite momentum residual");
    return false;
  }
  stamp_ = stamp;
  ready_ = true;
  reason_ = "ready: generalized external load including unmodelled loads";
  return true;
}
} // namespace perching_dynamics

#include <Eigen/SVD>
#include <pluginlib/class_list_macros.h>
using namespace perching_geometry;

bool GimbalrotorMultilinkPerchingModel::configure(const std::vector<std::string>& active, std::string& reason)
{
  if(active.empty()) { reason = "articulated model requires named active joints"; return false; }
  return GimbalrotorPerchingModel::configure(active, reason);
}

bool GimbalrotorMultilinkPerchingModel::pitchTangent(const Vector& q, const std::vector<bool>& moving, Vector& tangent) const
{
  Eigen::MatrixXd angular;
  if(moving.size() != coordinates_.size() || !jacobian(q, angular)) return false;
  std::vector<int> selected;
  for(int i = 0; i < q.size(); ++i) if(moving[i]) selected.push_back(i);
  if(selected.empty()) return false;
  Eigen::MatrixXd task(3, selected.size());
  for(std::size_t i = 0; i < selected.size(); ++i) task.col(i) = angular.col(selected[i]);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(task, Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(1e-6);
  const Vector solution = svd.solve(Eigen::Vector3d::UnitY());
  // Rank is not a feasibility test: a single pitch axis is sufficient.
  if(!solution.allFinite() || (task * solution - Eigen::Vector3d::UnitY()).norm() > 1e-5) return false;
  tangent = Vector::Zero(q.size());
  for(std::size_t i = 0; i < selected.size(); ++i) tangent[selected[i]] = solution[i];
  return true;
}

bool GimbalrotorMultilinkPerchingModel::orient(const Vector& start, const Eigen::Matrix3d& target, const std::vector<bool>& moving, Vector& result) const
{
  if(!rotationValid(target) || moving.size() != coordinates_.size()) return false;
  result = start;
  std::vector<int> selected;
  for(int i = 0; i < start.size(); ++i) if(moving[i]) selected.push_back(i);
  if(selected.empty()) return false;
  for(int iteration = 0; iteration < 40; ++iteration)
  {
    State state; Eigen::MatrixXd angular;
    if(!evaluate(result, state) || !jacobian(result, angular)) return false;
    const Eigen::Vector3d error = logSO3(state.body.linear().transpose() * target);
    if(error.norm() < 1e-6) return true;
    Eigen::MatrixXd task(3, selected.size());
    for(std::size_t j = 0; j < selected.size(); ++j) task.col(j) = angular.col(selected[j]);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(task, Eigen::ComputeThinU | Eigen::ComputeThinV);
    svd.setThreshold(1e-6);
    Vector delta = svd.solve(error);
    if(!delta.allFinite() || (task * delta - error).norm() > 1e-5) return false;
    if(delta.norm() > .1) delta *= .1 / delta.norm();
    for(std::size_t j = 0; j < selected.size(); ++j) result[selected[j]] += delta[j];
    if((result.array() < lower.array()).any() || (result.array() > upper.array()).any()) return false;
  }
  return false;
}

bool GimbalrotorMultilinkPerchingModel::pitch(const Vector& start, double angle, const std::vector<bool>& moving, Vector& result) const
{
  if(!std::isfinite(angle) || std::abs(angle) > M_PI) return false;
  result = start;
  if(angle == 0) return start.size() == lower.size() && start.allFinite();
  // Integrate the very same tangent used for virtual work. This fixes the
  // null-space policy for parallel axes and preserves held coordinates exactly.
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(angle) / .002)));
  const double h = angle / steps;
  for(int i = 0; i < steps; ++i)
  {
    Vector tangent, middle;
    if(!pitchTangent(result, moving, tangent) || !pitchTangent(result + .5 * h * tangent, moving, middle)) return false;
    result += h * middle;
    if((result.array() < lower.array()).any() || (result.array() > upper.array()).any()) return false;
  }
  State a, b;
  return evaluate(start, a) && evaluate(result, b) &&
      logSO3((a.body.linear() * expSO3(Eigen::Vector3d::UnitY() * angle)).transpose() * b.body.linear()).norm() < 1e-4;
}


PLUGINLIB_EXPORT_CLASS(GimbalrotorMultilinkPerchingModel, aerial_robot_model::RobotModel);
