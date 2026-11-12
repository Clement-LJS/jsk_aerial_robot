#include <gimbalrotor/perching_trajectory.h>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <gimbalrotor/roll_yaw_worker.h>

namespace roll_yaw
{
bool taskPitch(const Eigen::Matrix3d& pre, const Eigen::Matrix3d& body, double previous, double& pitch,
               double& cross_track)
{
  if(!rotationValid(pre) || !rotationValid(body) || !std::isfinite(previous))
    return false;
  Eigen::Quaterniond q(pre.transpose() * body);
  q.normalize();
  if(std::hypot(q.w(), q.y()) < 1e-8)
    return false;
  pitch = previous + std::remainder(2 * std::atan2(q.y(), q.w()) - previous, 2 * M_PI);
  cross_track = logSO3(expSO3(-Eigen::Vector3d::UnitY() * pitch) * pre.transpose() * body).norm();
  return std::isfinite(pitch) && std::isfinite(cross_track);
}

void TaskProgress::reset(double start)
{
  *this = TaskProgress();
  raw = filtered = start;
}

bool TaskProgress::observe(const Eigen::Matrix3d& pre, const Eigen::Matrix3d& body, double time, double start,
                           double end, double filter_time, double tolerance)
{
  if(time <= stamp)
    return true;  // no new observation; preserve validity without inventing progress
  double pitch, residual;
  if(!taskPitch(pre, body, raw, pitch, residual))
  {
    valid = false;
    return false;
  }
  const double dt = time - stamp;
  raw = pitch;
  cross_track = residual;
  filtered = stamp > 0 ? filtered + (1 - std::exp(-dt / filter_time)) * (raw - filtered) : raw;
  stamp = time;
  valid = residual <= tolerance;
  const double sign = end >= start ? 1 : -1;
  // Current conservative agreement drives both forward and reverse lead.
  // Historical best is reporting/timeout data only, never feed permission.
  current = std::min(sign * (raw - start), sign * (filtered - start));
  upper_distance = std::max(sign * (raw - start), sign * (filtered - start));
  if(valid)
    distance = std::max(distance, std::clamp(current, 0.0, std::abs(end - start)));
  return true;
}

bool SettlingStatistics::qualifies(double body_limit, double joint_limit, double mean_limit) const
{
  return complete && std::isfinite(body_limit) && body_limit > 0 && std::isfinite(joint_limit) && joint_limit > 0 &&
         std::isfinite(mean_limit) && mean_limit > 0 && body_rms < body_limit && joint_rms < joint_limit &&
         body_mean < mean_limit && joint_mean < mean_limit && body_current < body_limit && joint_current < joint_limit;
}

void SettlingWindow::reset()
{
  samples_.clear();
  window_ = 0;
  last_ = SettlingStatistics();
}

SettlingStatistics SettlingWindow::observe(double now, const std::array<double, 3>& body,
                                           const std::array<double, 2>& joint, double window, double max_gap)
{
  SettlingStatistics out;
  if(!std::isfinite(now) || !std::isfinite(window) || window <= 0 || !std::isfinite(max_gap) || max_gap <= 0)
  {
    reset();
    out.status = "invalid window clock/configuration";
    return out;
  }
  Sample sample{now, {{body[0], body[1], body[2], joint[0], joint[1]}}};
  for(double x : sample.rate)
    if(!std::isfinite(x))
    {
      reset();
      out.status = "invalid angular rates";
      return out;
    }
  if(window_ != 0 && window_ != window)
  {
    reset();
    out.status = "window length changed; collecting fresh window";
  }
  if(!samples_.empty())
  {
    const double dt = now - samples_.back().time;
    if(dt == 0)
    {
      // No additional measurement time; still check the faster source NOW.
      out = last_;
      out.body_current = std::hypot(std::hypot(body[0], body[1]), body[2]);
      out.joint_current = std::hypot(joint[0], joint[1]);
      return out;
    }
    if(dt < 0 || dt > max_gap)
    {
      reset();
      out.status = dt < 0 ? "backwards clock; collecting fresh window" : "sampling gap; collecting fresh window";
    }
  }
  window_ = window;
  if(samples_.size() >= 4096)
  {
    reset();
    out.status = "settling sample capacity exceeded; collecting fresh window";
    return out;
  }
  samples_.push_back(sample);
  const double begin = now - window;
  // Retain the sample just before the left edge for exact linear clipping.
  while(samples_.size() > 1 && samples_[1].time <= begin)
    samples_.pop_front();
  out.observed = std::min(window, now - samples_.front().time);
  out.body_current = std::hypot(std::hypot(body[0], body[1]), body[2]);
  out.joint_current = std::hypot(joint[0], joint[1]);
  if(samples_.size() < 2 || samples_.front().time > begin)
  {
    last_ = out;
    return out;
  }

  std::array<double, 5> integral{{}}, energy{{}};
  for(std::size_t i = 1; i < samples_.size(); ++i)
  {
    const auto& a = samples_[i - 1];
    const auto& b = samples_[i];
    const double left = std::max(begin, a.time), dt = b.time - left;
    const double fraction = (left - a.time) / (b.time - a.time);
    for(std::size_t j = 0; j < integral.size(); ++j)
    {
      const double x = (1 - fraction) * a.rate[j] + fraction * b.rate[j], y = b.rate[j];
      integral[j] += (x + y) * .5 * dt;
      // Exact integral of squared linearly interpolated rate, not trapezoidal energy.
      energy[j] += (x * x + x * y + y * y) * dt / 3;
    }
  }
  // Subtract timestamps before dividing: epoch-valued clocks have finite precision.
  const double duration = now - begin;
  if(!std::isfinite(duration) || duration <= 0)
  {
    reset();
    out.status = "unrepresentable window duration";
    return out;
  }
  out.body_rms = std::sqrt((energy[0] + energy[1] + energy[2]) / duration);
  out.joint_rms = std::sqrt((energy[3] + energy[4]) / duration);
  out.body_mean = std::hypot(std::hypot(integral[0], integral[1]), integral[2]) / duration;
  out.joint_mean = std::hypot(integral[3], integral[4]) / duration;
  for(double x : {out.body_rms, out.joint_rms, out.body_mean, out.joint_mean, out.body_current, out.joint_current})
    if(!std::isfinite(x))
    {
      reset();
      out.status = "nonfinite window statistics";
      return out;
    }
  out.complete = true;
  out.status = "complete window";
  last_ = out;
  return out;
}

bool Limits::valid() const
{
  return lower.allFinite() && upper.allFinite() && velocity.allFinite() && acceleration.allFinite() &&
         (lower.array() < upper.array()).all() && (velocity.array() > 0).all() && (acceleration.array() > 0).all();
}
bool Request::valid() const
{
  for(double x : {yaw, roll, pitch_start, pitch_end, rate, acceleration})
    if(!std::isfinite(x))
      return false;
  return rate > 0 && acceleration > 0 && std::abs(yaw) <= M_PI && std::abs(roll) < M_PI / 2 &&
         std::abs(pitch_start) < M_PI && std::abs(pitch_end) < M_PI &&
         (align_only || std::abs(pitch_end - pitch_start) > 1e-8) &&
         (!body_pitch_jog || (align_only && yaw == 0 && roll == 0 && pitch_start != 0 && pitch_end == 0));
}
bool PlanningConfig::mechanismValid() const
{
  return limits.lower.allFinite() && limits.upper.allFinite() && (limits.lower.array() < limits.upper.array()).all() &&
         limits.lower[0] > -M_PI && limits.upper[0] < M_PI && limits.lower[0] <= 0 && limits.upper[0] >= 0 &&
         std::isfinite(max_tilt) && max_tilt > 0 && max_tilt < M_PI / 2;
}
bool PlanningConfig::valid() const
{
  for(double x : {dt, max_tilt, orientation_tolerance, rate_tolerance, alignment_rate, alignment_acceleration,
                  max_duration, cog_speed_limit, cog_acceleration_limit})
    if(!std::isfinite(x) || x <= 0)
      return false;
  return mechanismValid() && limits.valid() && limits.lower[0] > -M_PI && limits.upper[0] < M_PI &&
         limits.lower[0] <= 0 && limits.upper[0] >= 0 && std::isfinite(min_singular_value) && min_singular_value > 0 &&
         dt >= .001 && dt <= .1 && max_tilt < M_PI / 2 && scaling_attempts >= 1 && scaling_attempts <= 10 &&
         max_duration / dt < 200000;
}
Stroke::Stroke(double distance, double rate, double acc) : distance_(std::abs(distance)), sign_(distance >= 0 ? 1 : -1)
{
  speed_ = std::min(rate, std::sqrt(distance_ * 2 * acc / M_PI));
  ramp_ = speed_ > 0 ? M_PI * speed_ / (2 * acc) : 0;
  feed_ = speed_ > 0 ? std::max(0.0, distance_ / speed_ - ramp_) : 0;
}
void Stroke::sample(double t, double& angle, double& rate, double& acc) const
{
  angle = rate = acc = 0;
  if(t <= 0 || duration() == 0)
    return;
  if(t >= duration())
  {
    angle = sign_ * distance_;
    return;
  }
  if(t < ramp_)
  {
    angle = speed_ * .5 * (t - ramp_ / M_PI * std::sin(M_PI * t / ramp_));
    rate = speed_ * .5 * (1 - std::cos(M_PI * t / ramp_));
    acc = speed_ * M_PI / (2 * ramp_) * std::sin(M_PI * t / ramp_);
  }
  else if(t < ramp_ + feed_)
  {
    angle = speed_ * (.5 * ramp_ + t - ramp_);
    rate = speed_;
  }
  else
  {
    const double d = duration() - t;
    angle = distance_ - speed_ * .5 * (d - ramp_ / M_PI * std::sin(M_PI * d / ramp_));
    rate = speed_ * .5 * (1 - std::cos(M_PI * d / ramp_));
    acc = -speed_ * M_PI / (2 * ramp_) * std::sin(M_PI * d / ramp_);
  }
  angle *= sign_;
  rate *= sign_;
  acc *= sign_;
}
Desired cuttingOrientation(const Eigen::Matrix3d& n, const Request& r, double theta, double rate, double acc)
{
  return {n * expSO3(Eigen::Vector3d::UnitZ() * r.yaw) * expSO3(Eigen::Vector3d::UnitX() * r.roll) *
              expSO3(Eigen::Vector3d::UnitY() * theta),
          Eigen::Vector3d(0, rate, 0), Eigen::Vector3d(0, acc, 0)};
}
namespace
{
bool fcAttitudeAllowed(const Eigen::Matrix3d& r)
{
  // Spinal attitude_control.h MAX_TILT_ANGLE: roll/pitch of the CoG frame,
  // matching GimbalrotorController::sendFourAxisCommand. Do not widen it here.
  constexpr double limit = 1.0;
  return rotationValid(r) && std::abs(std::atan2(r(2, 1), r(2, 2))) <= limit &&
         std::abs(std::atan2(-r(2, 0), std::hypot(r(2, 1), r(2, 2)))) <= limit;
}
using Controls = std::array<Eigen::Vector3d, 6>;
Controls controls(const Knot& a, const Knot& b)
{
  const double dt = b.t - a.t;
  return {{a.eta, a.eta + a.velocity * dt / 5, a.eta + 2 * a.velocity * dt / 5 + a.acceleration * dt * dt / 20,
           b.eta - 2 * b.velocity * dt / 5 + b.acceleration * dt * dt / 20, b.eta - b.velocity * dt / 5, b.eta}};
}
template <std::size_t N>
Eigen::Vector3d bezier(std::array<Eigen::Vector3d, N> p, double s)
{
  for(std::size_t n = N - 1; n > 0; --n)
    for(std::size_t i = 0; i < n; ++i)
      p[i] = (1 - s) * p[i] + s * p[i + 1];
  return p[0];
}
bool coordinates(const Path& path, double t, Eigen::Vector3d& q, Eigen::Vector3d& v, Eigen::Vector3d& acc)
{
  if(path.knots.size() < 2 || !std::isfinite(t))
    return false;
  t = std::max(0.0, std::min(t, path.duration()));
  auto it = std::upper_bound(path.knots.begin(), path.knots.end(), t, [](double x, const Knot& k) { return x < k.t; });
  if(it == path.knots.end())
    --it;
  if(it == path.knots.begin())
    ++it;
  const auto& b = *it;
  const auto& a = *(it - 1);
  const double dt = b.t - a.t, s = (t - a.t) / dt;
  const auto p = controls(a, b);
  std::array<Eigen::Vector3d, 5> dv;
  std::array<Eigen::Vector3d, 4> da;
  for(int i = 0; i < 5; ++i)
    dv[i] = 5 * (p[i + 1] - p[i]) / dt;
  for(int i = 0; i < 4; ++i)
    da[i] = 4 * (dv[i + 1] - dv[i]) / dt;
  q = bezier(p, s);
  v = bezier(dv, s);
  acc = bezier(da, s);
  return q.allFinite() && v.allFinite() && acc.allFinite();
}
bool configurationAllowed(const Kinematics& m, const Eigen::Vector3d& q, const Desired& des, const PlanningConfig& c,
                          std::string& error)
{
  Eigen::Isometry3d b, g;
  Eigen::Matrix3d j;
  if(!q.allFinite() || !rotationValid(des.rotation) || !m.evaluate(q, b, g) || !m.jacobian(q, j))
  {
    error = "invalid FK/reference";
    return false;
  }
  if((q.array() < c.limits.lower.array() - 1e-9).any() || (q.array() > c.limits.upper.array() + 1e-9).any())
  {
    error = "mechanism position limit [alpha, roll, yaw]: q=" + std::to_string(q[0]) + "," + std::to_string(q[1]) +
            "," + std::to_string(q[2]);
    return false;
  }
  if(b.linear()(2, 2) < std::cos(c.max_tilt) || des.rotation(2, 2) < std::cos(c.max_tilt))
  {
    error = "body world tilt exceeds configured operating range";
    return false;
  }
  if(!fcAttitudeAllowed(g.linear()))
  {
    error = "CoG attitude exceeds Spinal roll/pitch command limits";
    return false;
  }
  if(Eigen::JacobiSVD<Eigen::Matrix3d>(j).singularValues().minCoeff() < c.min_singular_value)
  {
    error = "singular/ill-conditioned mechanism";
    return false;
  }
  if(logSO3(b.linear().transpose() * des.rotation).norm() > c.orientation_tolerance)
  {
    error = "FK orientation tracking tolerance exceeded";
    return false;
  }
  return true;
}
bool jointBounds(const Path& path, const PlanningConfig& c, std::string& error)
{
  if(path.knots.size() < 2)
  {
    error = "empty polynomial path";
    return false;
  }
  for(std::size_t k = 1; k < path.knots.size(); ++k)
  {
    const double dt = path.knots[k].t - path.knots[k - 1].t;
    if(!std::isfinite(dt) || dt <= 0)
    {
      error = "invalid polynomial interval";
      return false;
    }
    auto failed = [&](const std::string& why, int axis = -1)
    {
      error = "interval=" + std::to_string(k) + " t=" + std::to_string(path.knots[k - 1].t) + " s " +
              (axis < 0 ? ""
                        : std::string(axis == 0   ? "alpha"
                                      : axis == 1 ? "roll"
                                                  : "yaw") +
                              ": ") +
              why;
      return false;
    };
    const auto p = controls(path.knots[k - 1], path.knots[k]);
    std::array<Eigen::Vector3d, 5> v;
    for(const auto& q : p)
      if(!q.allFinite())
        return failed("nonfinite polynomial controls");
    for(const auto& q : p)
      for(int axis = 0; axis < 3; ++axis)
        if(q[axis] < c.limits.lower[axis] - 1e-12 || q[axis] > c.limits.upper[axis] + 1e-12)
          return failed("interpolated position limit", axis);
    for(int i = 0; i < 5; ++i)
    {
      v[i] = 5 * (p[i + 1] - p[i]) / dt;
      for(int axis = 0; axis < 3; ++axis)
        if(!std::isfinite(v[i][axis]) || std::abs(v[i][axis]) > c.limits.velocity[axis] * .5 + 1e-9)
          return failed("interpolated velocity reserve exceeded; time scaling required", axis);
    }
    for(int i = 0; i < 4; ++i)
      if(((4 * (v[i + 1] - v[i]) / dt).cwiseAbs().array() > c.limits.acceleration.array() * .5 + 1e-9).any())
      {
        std::ostringstream why;
        why << "interpolated acceleration reserve exceeded; interval=" << k << " coefficient=" << i
            << " acceleration=" << (4 * (v[i + 1] - v[i]) / dt).transpose()
            << " limit=" << (c.limits.acceleration * .5).transpose();
        error = why.str();
        return false;
      }
  }
  return true;
}
bool validatePath(const Kinematics& m, const Path& path, const PlanningConfig& c, std::string& error)
{
  if(!jointBounds(path, c, error))
    return false;
  for(std::size_t k = 1; k < path.knots.size(); ++k)
  {
    if(c.cancelled && c.cancelled())
    {
      error = "validation cancelled/deadline exceeded";
      return false;
    }
    if(c.report)
      c.report(k, path.knots.size() - 1, true);
    const double dt = path.knots[k].t - path.knots[k - 1].t;
    const int samples = c.min_singular_value == 0 ? 24 : std::max(4, static_cast<int>(std::ceil(dt / (c.dt / 4))));
    for(int i = 0; i <= samples; ++i)
    {
      if(c.cancelled && c.cancelled())
      {
        error = "validation cancelled/deadline exceeded";
        return false;
      }
      Reference ref;
      const double t = path.knots[k - 1].t + dt * i / samples;
      if(!samplePath(m, path, t, ref) || !configurationAllowed(m, ref.eta, ref.desired, c, error))
      {
        error = "interval=" + std::to_string(k) + " sample=" + std::to_string(i) + ": " + error;
        return false;
      }
      if((ref.body_omega - ref.body.linear().transpose() * ref.desired.rotation * ref.desired.omega).norm() >
         c.rate_tolerance)
      {
        error = "body angular velocity tracking tolerance exceeded";
        return false;
      }
      if(ref.cog_velocity.norm() > c.cog_speed_limit * .5 ||
         ref.cog_acceleration.norm() > c.cog_acceleration_limit * .5)
      {
        error = "CoG speed/acceleration limit";
        return false;
      }
    }
  }
  return true;
}
bool buildPath(const Kinematics& m, const Eigen::Vector3d& initial, double duration,
               const std::function<Desired(double)>& desired, const PlanningConfig& c, Path& path, std::string& error)
{
  if(duration > c.max_duration)
  {
    error = "phase duration exceeds limit";
    return false;
  }
  const int steps = std::max(2, static_cast<int>(std::ceil(duration / c.dt)));
  const double dt = duration / steps;
  path.knots.clear();
  path.desired = desired;
  Eigen::Vector3d eta = initial;
  for(int i = 0; i <= steps; ++i)
  {
    if(c.cancelled && c.cancelled())
    {
      error = "planning cancelled/deadline exceeded";
      return false;
    }
    if(c.report)
      c.report(i, steps, false);
    const auto d = desired(i * dt);
    const Eigen::Vector3d seed = eta;
    Eigen::Matrix3d j;
    if(!inverseOrientation(m, d.rotation, seed, c, eta, error) || !configurationAllowed(m, eta, d, c, error) ||
       (eta - seed).norm() > .1 || (i == 0 && (eta - initial).norm() > 1e-7) || !m.jacobian(eta, j))
    {
      error = "sample=" + std::to_string(i) + " t=" + std::to_string(i * dt) +
              " s: " + (error.empty() ? "IK branch/start consistency" : error);
      return false;
    }
    // Differentiate the same orientation constraint: J qdot = omega,
    // J qddot + Jdot qdot = omegadot. Endpoints are zero speed/acceleration.
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(j, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3d velocity = svd.solve(d.omega);
    Eigen::Matrix3d jp, jm;
    if(!m.jacobian(eta + 1e-4 * velocity, jp) || !m.jacobian(eta - 1e-4 * velocity, jm))
      return false;
    const Eigen::Vector3d acceleration = svd.solve(d.acceleration - (jp - jm) / (2e-4) * velocity);
    path.knots.push_back({i * dt, eta, velocity, acceleration});
  }
  if(!validatePath(m, path, c, error))
  {
    error = "interpolation: " + error;
    return false;
  }
  return cachePathTravel(path);
}
}  // namespace
bool inverseOrientation(const Kinematics& model, const Eigen::Matrix3d& desired, const Eigen::Vector3d& seed,
                        const PlanningConfig& config, Eigen::Vector3d& q, std::string& error)
{
  error.clear();
  q = seed;
  if(!rotationValid(desired) || !seed.allFinite())
  {
    error = "invalid IK input";
    return false;
  }
  for(int iteration = 0; iteration < 40; ++iteration)
  {
    Eigen::Isometry3d body;
    Eigen::Matrix3d j;
    if(!model.evaluateBody(q, body) || !model.jacobian(q, j))
    {
      error = "IK model failure";
      return false;
    }
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(j, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if(svd.singularValues().minCoeff() < config.min_singular_value)
    {
      error = "singular orientation IK";
      return false;
    }
    const Eigen::Vector3d residual = logSO3(body.linear().transpose() * desired);
    if(residual.norm() < 1e-9)
      return mechanismAllowed(model, q, config, error);
    // Damped inverse with bounded iteration length and nearest-seed branches.
    const auto singular = svd.singularValues();
    const Eigen::Vector3d inverse = singular.array() / (singular.array().square() + 1e-10);
    Eigen::Vector3d delta = svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose() * residual;
    if(delta.norm() > .08)
      delta *= .08 / delta.norm();
    q += delta;
    for(int axis = 0; axis < 3; ++axis)
      q[axis] = seed[axis] + std::remainder(q[axis] - seed[axis], 2 * M_PI);
  }
  error = "orientation IK did not converge in 40 iterations";
  return false;
}
bool localPitchSample(const Kinematics& model, const Eigen::Vector3d& q, const Eigen::Isometry3d& body,
                      const Eigen::Isometry3d& start, const Eigen::Isometry3d& previous, double delta,
                      double previous_pitch, const PlanningConfig& c, double& pitch, std::string& error)
{
  Eigen::Matrix3d j;
  double cross;
  if(!q.allFinite() || !poseValid(body) || !poseValid(start) || !poseValid(previous) || !c.saw_point_valid ||
     !c.saw_point.allFinite() || !model.jacobian(q, j) ||
     !taskPitch(start.linear(), body.linear(), previous_pitch, pitch, cross))
  {
    error = "local pitch geometry/proxy unavailable";
    return false;
  }
  if((q.array() < c.limits.lower.array() - 1e-9).any() || (q.array() > c.limits.upper.array() + 1e-9).any() ||
     body.linear()(2, 2) < std::cos(c.max_tilt) ||
     Eigen::JacobiSVD<Eigen::Matrix3d>(j).singularValues().minCoeff() < c.min_singular_value)
  {
    error = "local pitch mechanism/tilt/singularity limit";
    return false;
  }
  const double direction = delta > 0 ? 1 : -1;
  if(direction * pitch < -1e-9 || direction * pitch > std::abs(delta) + 1e-9 ||
     direction * (pitch - previous_pitch) < -1e-9 ||
     cross > std::min(c.orientation_tolerance, .1 * std::abs(delta)) + 1e-9)
  {
    error = "local pitch intermediate direction/off-axis bound";
    return false;
  }
  if(direction * ((body * c.saw_point).z() - (previous * c.saw_point).z()) > 1e-8)
  {
    error = "local pitch saw-origin direction contradicts request";
    return false;
  }
  return true;
}
bool mechanismAllowed(const Kinematics& m, const Eigen::Vector3d& q, const PlanningConfig& c, std::string& error)
{
  Eigen::Isometry3d b, g;
  if(!c.mechanismValid() || !q.allFinite() || !m.evaluate(q, b, g))
  {
    error = "invalid mechanism configuration/FK";
    return false;
  }
  if((q.array() < c.limits.lower.array() - 1e-9).any() || (q.array() > c.limits.upper.array() + 1e-9).any())
  {
    error = "mechanism position limit [alpha, roll, yaw]: q=" + std::to_string(q[0]) + "," + std::to_string(q[1]) +
            "," + std::to_string(q[2]);
    return false;
  }
  if(b.linear()(2, 2) < std::cos(c.max_tilt) || !fcAttitudeAllowed(g.linear()))
  {
    error = "body/CoG attitude limit";
    return false;
  }
  return true;
}
bool mechanismReference(const Kinematics& m, const Eigen::Vector3d& q, const Eigen::Vector3d& v,
                        const Eigen::Vector3d& a, Reference& r)
{
  r = Reference();
  r.eta = q;
  r.velocity = v;
  r.acceleration = a;
  if(!q.allFinite() || !v.allFinite() || !a.allFinite() || !m.evaluate(q, r.body, r.cog))
    return false;
  r.desired.rotation = r.body.linear();
  // Differentiate FK along the interpolated configuration, including articulated
  // CoG translation and any configuration-dependent CoG orientation callback.
  const double h = 1e-4;
  Eigen::Isometry3d bp, gp, bm, gm;
  if(!m.evaluate(r.eta + h * r.velocity + .5 * h * h * r.acceleration, bp, gp) ||
     !m.evaluate(r.eta - h * r.velocity + .5 * h * h * r.acceleration, bm, gm))
    return false;
  const Eigen::Vector3d wb_p = logSO3(r.body.linear().transpose() * bp.linear());
  const Eigen::Vector3d wb_m = logSO3(r.body.linear().transpose() * bm.linear());
  const Eigen::Vector3d wg_p = logSO3(r.cog.linear().transpose() * gp.linear());
  const Eigen::Vector3d wg_m = logSO3(r.cog.linear().transpose() * gm.linear());
  r.body_omega = (wb_p - wb_m) / (2 * h);
  r.body_acceleration = (wb_p + wb_m) / (h * h);
  r.cog_omega = (wg_p - wg_m) / (2 * h);
  r.cog_angular_acceleration = (wg_p + wg_m) / (h * h);
  r.cog_velocity = (gp.translation() - gm.translation()) / (2 * h);
  r.cog_acceleration = (gp.translation() + gm.translation() - 2 * r.cog.translation()) / (h * h);
  r.desired = {r.body.linear(), r.body_omega, r.body_acceleration};
  return r.body_omega.allFinite() && r.body_acceleration.allFinite() && r.cog_velocity.allFinite() &&
         r.cog_acceleration.allFinite() && r.cog_omega.allFinite() && r.cog_angular_acceleration.allFinite();
}

bool cachePathTravel(Path& path)
{
  path.travel.assign(path.knots.size(), Eigen::Vector3d::Zero());
  for(std::size_t k = 1; k < path.knots.size(); ++k)
  {
    const double dt = path.knots[k].t - path.knots[k - 1].t;
    if(!std::isfinite(dt) || dt <= 0)
      return false;
    const auto p = controls(path.knots[k - 1], path.knots[k]);
    Eigen::Vector3d bound = Eigen::Vector3d::Zero();
    for(int i = 0; i < 5; ++i)
      bound = bound.cwiseMax((5 * (p[i + 1] - p[i])).cwiseAbs());
    path.travel[k] = path.travel[k - 1] + bound;
  }
  return path.knots.size() >= 2;
}
Eigen::Vector3d pathTravel(const Path& p, double t)
{
  if(p.travel.size() != p.knots.size() || p.knots.size() < 2)
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  t = std::clamp(t, 0.0, p.duration());
  auto it = std::upper_bound(p.knots.begin(), p.knots.end(), t, [](double x, const Knot& k) { return x < k.t; });
  std::size_t k = std::min<std::size_t>(p.knots.size() - 1, std::max<std::size_t>(1, it - p.knots.begin()));
  const double u = (t - p.knots[k - 1].t) / (p.knots[k].t - p.knots[k - 1].t);
  return p.travel[k - 1] + u * (p.travel[k] - p.travel[k - 1]);
}
void scaleReference(Reference& r, double v, double a)
{
  r.acceleration = r.acceleration * v * v + r.velocity * a;
  r.body_acceleration = r.body_acceleration * v * v + r.body_omega * a;
  r.cog_acceleration = r.cog_acceleration * v * v + r.cog_velocity * a;
  r.cog_angular_acceleration = r.cog_angular_acceleration * v * v + r.cog_omega * a;
  r.desired.acceleration = r.desired.acceleration * v * v + r.desired.omega * a;
  r.velocity *= v;
  r.body_omega *= v;
  r.cog_velocity *= v;
  r.cog_omega *= v;
  r.desired.omega *= v;
}
bool samplePath(const Kinematics& m, const Path& p, double t, Reference& r)
{
  Eigen::Vector3d q, v, a;
  if(!coordinates(p, t, q, v, a) || !mechanismReference(m, q, v, a, r))
    return false;
  r.t = std::max(0.0, std::min(t, p.duration()));
  if(p.desired)
    r.desired = p.desired(r.t);
  r.orientation_error = logSO3(r.body.linear().transpose() * r.desired.rotation).norm();
  return true;
}
bool referenceAllowed(const Reference& r, const PlanningConfig& c, std::string& error)
{
  if(!poseValid(r.body) || !poseValid(r.cog) || !r.eta.allFinite() || !r.velocity.allFinite() ||
     !r.acceleration.allFinite() || !r.body_omega.allFinite() || !r.body_acceleration.allFinite() ||
     !r.cog_velocity.allFinite() || !r.cog_acceleration.allFinite() || !r.cog_omega.allFinite() ||
     !r.cog_angular_acceleration.allFinite())
  {
    error = "invalid generated reference";
    return false;
  }
  if((r.eta.array() < c.limits.lower.array() - 1e-9).any() || (r.eta.array() > c.limits.upper.array() + 1e-9).any() ||
     (r.velocity.cwiseAbs().array() > c.limits.velocity.array() + 1e-9).any() ||
     (r.acceleration.cwiseAbs().array() > c.limits.acceleration.array() + 1e-9).any())
  {
    error = "candidate joint position/rate/acceleration limit";
    return false;
  }
  if(r.body.linear()(2, 2) < std::cos(c.max_tilt) || !fcAttitudeAllowed(r.cog.linear()))
  {
    error = "candidate body/CoG attitude limit";
    return false;
  }
  if(r.cog_velocity.norm() > c.cog_speed_limit || r.cog_acceleration.norm() > c.cog_acceleration_limit)
  {
    error = "candidate CoG speed/acceleration limit";
    return false;
  }
  return true;
}
bool mechanismPath(const Kinematics& model, const Reference& from, const Eigen::Vector3d& goal,
                   const PlanningConfig& config, Path& path, std::string& error)
{
  path = Path();
  if(!config.limits.valid() || !from.eta.allFinite() || !from.velocity.allFinite() || !from.acceleration.allFinite())
  {
    error = "invalid manual state/rate bounds";
    return false;
  }
  if(!mechanismAllowed(model, goal, config, error))
    return false;
  // Every Bernstein inequality is quadratic in positive duration: position
  // controls directly, velocity controls multiplied by T, acceleration by T^2.
  // Their real roots partition T >= 0.1 into constant-feasibility intervals.
  // Test boundaries and one interior point per interval, including the final
  // unbounded interval. This is finite work even when feasibility is nonmonotone.
  using Polynomial = std::array<long double, 3>;  // constant, linear, quadratic
  std::vector<double> boundaries{.1};
  auto roots = [&](Polynomial p)
  {
    auto add = [&](long double t)
    {
      if(std::isfinite(t) && t > .1L && t < std::numeric_limits<double>::max() / 4)
        boundaries.push_back(static_cast<double>(t));
    };
    if(p[2] == 0)
    {
      if(p[1] != 0)
        add(-p[0] / p[1]);
      return;
    }
    const long double discriminant = p[1] * p[1] - 4 * p[2] * p[0];
    if(discriminant < 0)
      return;
    const long double q = -.5L * (p[1] + std::copysign(std::sqrt(discriminant), p[1]));
    if(q == 0)
      add(-p[1] / (2 * p[2]));
    else
    {
      add(q / p[2]);
      add(p[0] / q);
    }
  };
  for(int axis = 0; axis < 3; ++axis)
  {
    const long double q = from.eta[axis], v = from.velocity[axis], a = from.acceleration[axis], g = goal[axis];
    const std::array<Polynomial, 6> p{
        {{q, 0, 0}, {q, v / 5, 0}, {q, 2 * v / 5, a / 20}, {g, 0, 0}, {g, 0, 0}, {g, 0, 0}}};
    for(const auto& b : p)
    {
      roots({b[0] - config.limits.lower[axis], b[1], b[2]});
      roots({b[0] - config.limits.upper[axis], b[1], b[2]});
    }
    for(int i = 0; i < 5; ++i)
      for(int sign : {-1, 1})
        roots({5 * (p[i + 1][0] - p[i][0]), 5 * (p[i + 1][1] - p[i][1]) + sign * .5L * config.limits.velocity[axis],
               5 * (p[i + 1][2] - p[i][2])});
    for(int i = 0; i < 4; ++i)
      for(int sign : {-1, 1})
        roots({20 * (p[i + 2][0] - 2 * p[i + 1][0] + p[i][0]), 20 * (p[i + 2][1] - 2 * p[i + 1][1] + p[i][1]),
               20 * (p[i + 2][2] - 2 * p[i + 1][2] + p[i][2]) + sign * .5L * config.limits.acceleration[axis]});
  }
  std::sort(boundaries.begin(), boundaries.end());
  boundaries.erase(std::unique(boundaries.begin(), boundaries.end()), boundaries.end());
  auto feasible = [&](double duration)
  {
    path.knots = {{0, from.eta, from.velocity, from.acceleration},
                  {duration, goal, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
    return jointBounds(path, config, error) && cachePathTravel(path);
  };
  // No FK sampling, worker, or offline-planning settings on this manual route.
  for(std::size_t i = 0; i < boundaries.size(); ++i)
  {
    if(feasible(boundaries[i]))
    {
      error.clear();
      return true;
    }
    const double interior = i + 1 < boundaries.size() ? boundaries[i] + .5 * (boundaries[i + 1] - boundaries[i])
                                                      : boundaries[i] + std::max(1.0, boundaries[i]);
    if(feasible(interior))
    {
      error.clear();
      return true;
    }
  }
  error = "no feasible smooth manual duration within Bernstein position/rate/acceleration reserves";
  path = Path();
  return false;
}
bool validatedMechanismPath(const Kinematics& model, const Reference& from, const Eigen::Vector3d& goal,
                            const PlanningConfig& config, Path& path, std::string& error)
{
  if(!mechanismPath(model, from, goal, config, path, error))
    return false;
  // Offline cut-start connections retain full nonlinear validation and time
  // scaling. Only the automatic executor needs a separate desired closure.
  for(int attempt = 0; attempt < 6; ++attempt, path.knots.back().t *= 1.5)
  {
    if(config.cancelled && config.cancelled())
    {
      error = "validation cancelled/deadline exceeded";
      break;
    }
    if(path.duration() > config.max_duration)
    {
      error = "connection duration exceeds limit";
      break;
    }
    if(!validatePath(model, path, config, error))
      continue;
    const auto points = path.knots;
    path.desired = [model, points](double t)
    {
      Path p;
      p.knots = points;
      Eigen::Vector3d q, v, a;
      Reference ref;
      if(!coordinates(p, t, q, v, a) || !mechanismReference(model, q, v, a, ref))
      {
        Desired bad;
        bad.rotation.setConstant(std::numeric_limits<double>::quiet_NaN());
        return bad;
      }
      return ref.desired;
    };
    return cachePathTravel(path);
  }
  path = Path();
  return false;
}
namespace
{
bool signedProxyMotion(const Kinematics& model, const Path& path, const PlanningConfig& c, double direction,
                       double& drop, std::string& error)
{
  Reference ref;
  if(!samplePath(model, path, 0, ref))
  {
    error = "invalid saw-origin proxy start";
    return false;
  }
  const double start_z = (ref.body * c.saw_point).z();
  double previous_z = start_z;
  const int samples = std::max(2, static_cast<int>(std::ceil(path.duration() / (c.dt / 4))));
  for(int i = 1; i <= samples; ++i)
  {
    if(c.cancelled && c.cancelled())
    {
      error = "proxy validation cancelled";
      return false;
    }
    if(!samplePath(model, path, path.duration() * i / samples, ref))
    {
      error = "invalid saw-origin proxy sample";
      return false;
    }
    const double z = (ref.body * c.saw_point).z();
    if(direction * (z - previous_z) > 1e-8)
    {
      error = std::string(direction > 0 ? "upward" : "downward") +
              " saw-origin proxy motion opposes request at sample=" + std::to_string(i) +
              "; requested sign retained; choose another extent/alignment";
      return false;
    }
    previous_z = z;
  }
  drop = start_z - previous_z;
  if(direction * drop <= 1e-6)
  {
    error = "no meaningful requested-direction saw-origin proxy displacement";
    return false;
  }
  return true;
}
}  // namespace
Plan prepare(const Kinematics& model, const Eigen::Vector3d& initial, const Eigen::Matrix3d& neutral,
             const Request& request, const PlanningConfig& c)
{
  Plan out;
  out.model = model;
  out.goal = request;
  out.neutral = neutral;
  out.reason = "invalid request/configuration";
  if(!request.valid() || !c.valid() || !rotationValid(neutral) || !initial.allFinite())
    return out;
  Eigen::Isometry3d body, cog;
  if(!model.evaluate(initial, body, cog))
  {
    out.reason = "invalid initial FK";
    return out;
  }
  if(!request.align_only && (request.yaw != 0 || request.roll != 0 || request.pitch_start != 0 ||
                             logSO3(body.linear().transpose() * neutral).norm() > c.orientation_tolerance))
  {
    out.reason = "cut must start at measured R0; no implicit alignment allowed";
    return out;
  }
  if((!request.align_only || request.body_pitch_jog) && !c.saw_point_valid)
  {
    out.reason = "signed feed UNVERIFIED: missing rigid saw-link origin in controlled body frame";
    return out;
  }
  if(request.body_pitch_jog && logSO3(body.linear().transpose() * neutral).norm() > 1e-9)
  {
    out.reason = "local pitch jog must start at applied body orientation";
    return out;
  }
  const Eigen::Matrix3d end = cuttingOrientation(neutral, request, request.pitch_start, 0, 0).rotation;
  const Eigen::Vector3d delta = logSO3(body.linear().transpose() * end);
  for(int attempt = 0; attempt < c.scaling_attempts; ++attempt)
  {
    if(c.cancelled && c.cancelled())
    {
      out.reason = "planning cancelled/deadline exceeded";
      return out;
    }
    const double scale = std::pow(2.0, attempt);
    out.scale_attempt = attempt;
    const double time = std::max(1.0, std::max(1.875 * delta.norm() / c.alignment_rate,
                                               std::sqrt(5.8 * delta.norm() / c.alignment_acceleration))) *
                        scale;
    auto align = [start = Eigen::Matrix3d(body.linear()), delta, time](double t)
    {
      const double s = std::max(0.0, std::min(1.0, t / time));
      const double a = 10 * s * s * s - 15 * std::pow(s, 4) + 6 * std::pow(s, 5);
      const double v = 30 * s * s * (1 - s) * (1 - s) / time, acc = 60 * s * (1 - s) * (1 - 2 * s) / (time * time);
      return Desired{start * expSO3(delta * a), delta * v, delta * acc};
    };
    if(!buildPath(model, initial, time, align, c, out.align, out.reason))
      continue;
    if(request.align_only)
    {
      if(request.body_pitch_jog &&
         !signedProxyMotion(model, out.align, c, request.pitch_start > 0 ? 1 : -1, out.proxy_drop, out.reason))
      {
        out.align = Path();
        return out;
      }
      out.ok = true;
      out.reason =
          request.body_pitch_jog ? "body-local pitch jog geometry prepared" : "body alignment geometry prepared";
      return out;
    }
    Stroke stroke(request.pitch_end - request.pitch_start, request.rate / scale,
                  request.acceleration / (scale * scale));
    auto cut = [neutral, request, stroke](double t)
    {
      double theta, v, a;
      stroke.sample(t, theta, v, a);
      return cuttingOrientation(neutral, request, request.pitch_start + theta, v, a);
    };
    if(!buildPath(model, out.align.knots.back().eta, stroke.duration(), cut, c, out.cut, out.reason))
      continue;
    // Ordinary cuts remain downward-only, independently of their angle sign.
    if(!signedProxyMotion(model, out.cut, c, 1, out.proxy_drop, out.reason))
    {
      out.align.knots.clear();
      out.cut.knots.clear();
      return out;
    }
    out.ok = true;
    out.reason = "geometry prepared: R0 * Ry(d); saw-origin drop=" + std::to_string(out.proxy_drop) +
                 " m; physical tooth contact unverified";
    return out;
  }
  out.align.knots.clear();
  out.cut.knots.clear();
  return out;
}
const char* stageName(Stage s)
{
  static const char* names[] = {"FREE_FLIGHT", "CONTACT_LOCKED", "PLANNING",       "PRE_ALIGN", "READY",
                                "EXECUTING",   "PAUSED",         "COMPLETED_HOLD", "FAULT_HOLD"};
  return names[static_cast<int>(s)];
}
bool ExecutionConfig::feedbackConfigValid() const
{
  for(double x : {synchronization_tolerance, timeout, pivot_tolerance, hinge_tolerance})
    if(!std::isfinite(x) || x <= 0)
      return false;
  return std::isfinite(future_tolerance) && future_tolerance >= 0 && future_tolerance < timeout;
}
bool ExecutionConfig::valid() const
{
  for(double x : {tracking_tolerance, orientation_tolerance, reference_lead, progress_filter_time, goal_tolerance,
                  progress_timeout, require_settled_on_enable ? settle_rate : 1.0})
    if(!std::isfinite(x) || x <= 0)
      return false;
  return feedbackConfigValid() && std::isfinite(soft_tracking_fraction) && soft_tracking_fraction > 0 &&
         soft_tracking_fraction < .8 && std::isfinite(resume_fraction) && resume_fraction > 0 && resume_fraction < 1 &&
         reference_lead < soft_tracking_fraction * orientation_tolerance && std::isfinite(preparation_tolerance) &&
         preparation_tolerance > 0 && preparation_tolerance <= hinge_tolerance &&
         preparation_tolerance < tracking_tolerance && preparation_tolerance < orientation_tolerance &&
         std::isfinite(preparation_quiet_time) && preparation_quiet_time > 0;
}
bool ExecutionConfig::governorValid(const PlanningConfig& p) const
{
  if(!valid() || !p.valid() || !std::isfinite(max_execution_step) || max_execution_step <= 0 ||
     max_execution_step > timeout)
    return false;
  // One maximum cycle of the path's reserved rates must fit between soft and
  // hard bounds. Braking distance itself is reserved by the cached path horizon.
  const Eigen::Vector3d cycle = .5 * p.limits.velocity * max_execution_step;
  return cycle.maxCoeff() < (1 - soft_tracking_fraction) * tracking_tolerance &&
         cycle.sum() < (1 - soft_tracking_fraction) * orientation_tolerance;
}
bool ExecutionConfig::settlingValid() const
{
  for(double x : {settle_time, settle_timeout})
    if(!std::isfinite(x) || x <= 0)
      return false;
  if(settle_use_window)
  {
    for(double x : {settle_body_rate, settle_joint_rate, settle_mean_rate})
      if(!std::isfinite(x) || x <= 0)
        return false;
  }
  else if(!std::isfinite(settle_rate) || settle_rate <= 0)
    return false;
  return settle_timeout > settle_time;
}
FeedbackStatus feedbackStatus(const Feedback& s, const ExecutionConfig& config, const PlanningConfig& planning)
{
  if(!config.feedbackConfigValid() || !planning.mechanismValid())
    return FeedbackStatus::FAULT;
  if(!s.hover || !std::isfinite(s.now))
    return FeedbackStatus::FAULT;
  if(!s.valid)
    return FeedbackStatus::WAITING;
  auto fresh = [&](double t)
  { return std::isfinite(t) && t > 0 && s.now - t >= -config.future_tolerance && s.now - t <= config.timeout; };
  if(!fresh(s.joint_stamp) || !fresh(s.body_stamp) ||
     std::abs(s.joint_stamp - s.body_stamp) > config.synchronization_tolerance)
    return FeedbackStatus::WAITING;
  if(!poseValid(s.body) || !s.eta.allFinite() || !s.joint_rate.allFinite() || !s.body_omega.allFinite())
    return FeedbackStatus::FAULT;
  return FeedbackStatus::READY;
}
std::string feedbackRejection(const Feedback& s, const ExecutionConfig& config, const PlanningConfig& planning)
{
  if(!config.feedbackConfigValid() || !planning.mechanismValid())
    return "invalid feedback/mechanism configuration";
  if(!s.hover)
    return "contact control requires HOVER without forced landing";
  if(!std::isfinite(s.now))
    return "invalid feedback clock";
  if(!s.valid)
    return s.invalid_reason.empty() ? "missing/incoherent feedback" : s.invalid_reason;
  auto fresh = [&](double t)
  { return std::isfinite(t) && t > 0 && s.now - t >= -config.future_tolerance && s.now - t <= config.timeout; };
  auto age_reason = [&](const char* source, double stamp)
  {
    std::ostringstream out;
    out << "stale/future " << source << " feedback: age=" << s.now - stamp << " s; allowed=["
        << -config.future_tolerance << "," << config.timeout << "] s";
    return out.str();
  };
  if(!fresh(s.joint_stamp))
    return age_reason("joint", s.joint_stamp);
  if(!fresh(s.body_stamp))
    return age_reason("body", s.body_stamp);
  if(std::abs(s.joint_stamp - s.body_stamp) > config.synchronization_tolerance)
    return "joint/body feedback timestamp skew";
  if(!poseValid(s.body))
    return "invalid body pose/rotation";
  if(!s.eta.allFinite())
    return "nonfinite joint positions";
  if(!s.joint_rate.allFinite() || !s.body_omega.allFinite())
    return "invalid angular-rate feedback";
  return "";
}
std::string Executor::feedbackRejection(const Feedback& s) const
{
  return roll_yaw::feedbackRejection(s, config, planning);
}
bool Executor::attach(const Kinematics& captured, const Reference& held, std::uint64_t identity, double now)
{
  std::string error;
  if(!planning.valid() || !config.governorValid(planning) || !config.settlingValid() || !std::isfinite(now) ||
     now <= 0 || !identity || !mechanismAllowed(captured, held.eta, planning, error))
  {
    reason = "invalid explicit planner attachment: " + error;
    return false;
  }
  model = captured;
  target = held;
  session = identity;
  stage = Stage::CONTACT_LOCKED;
  hold("explicit body-orientation planning requested");
  last_time_ = now;
  progress = 0;
  resetSettling();
  speed_ = 1;
  return true;
}
bool Executor::feedbackValid(const Feedback& s) const
{
  return feedbackRejection(s).empty();
}
bool Executor::capture(const Feedback& s, const Kinematics& m)
{
  if(owns())
  {
    reason = "existing contact retained; disable before capture";
    return false;
  }
  const auto rejection = feedbackRejection(s);
  if(!rejection.empty())
  {
    reason = rejection;
    return false;
  }
  if(config.require_settled_on_enable &&
     (s.joint_rate.tail<2>().norm() > config.settle_rate || s.body_omega.norm() > config.settle_rate))
  {
    reason = "optional settled-on-enable rate check failed";
    return false;
  }
  Kinematics next = m;
  Eigen::Vector3d q = s.eta;
  q[0] = 0;
  if(!next.lock(s.body, q))
  {
    reason = "invalid fixed-pivot model";
    return false;
  }
  std::string why;
  if(!mechanismAllowed(next, q, planning, why))
  {
    reason = why;
    return false;
  }
  Reference held;
  held.eta = q;
  if(!next.evaluate(q, held.body, held.cog))
  {
    reason = "invalid captured body/CoG FK";
    return false;
  }
  held.desired.rotation = held.body.linear();
  model = std::move(next);
  target = held;
  ++session;
  plan = Plan();
  first_fault.clear();
  stage = Stage::CONTACT_LOCKED;
  reason = "measured contact captured; N is locked body orientation";
  last_time_ = s.now;
  progress = 0;
  resetSettling();
  stopping_ = cancel_ = false;
  speed_ = 1;
  return true;
}
bool Executor::submissionMatches(const Plan& p, const Feedback& f, const Reference& held) const
{
  // Identity is the held applied reference, never two noisy measured snapshots.
  // Live readiness is checked separately by the navigator against this start.
  (void)f;
  return p.session == session && p.submission_valid && (held.eta - p.applied_start.eta).norm() < 1e-9 &&
         (held.body.matrix() - p.applied_start.body.matrix()).norm() < 1e-9 &&
         (held.cog.matrix() - p.applied_start.cog.matrix()).norm() < 1e-9 && held.velocity.norm() < 1e-6 &&
         held.acceleration.norm() < 1e-6 && held.body_omega.norm() < 1e-6 && held.body_acceleration.norm() < 1e-6 &&
         held.cog_velocity.norm() < 1e-6 && held.cog_acceleration.norm() < 1e-6 && held.cog_omega.norm() < 1e-6 &&
         held.cog_angular_acceleration.norm() < 1e-6;
}
bool Executor::accept(Plan p)
{
  if(stage != Stage::PLANNING || p.session != session)
    return false;
  resetSettling();
  if(!config.governorValid(planning))
  {
    p.ok = false;
    p.reason = "invalid soft/hard tracking or trajectory limits";
  }
  if(!p.ok)
  {
    stage = Stage::CONTACT_LOCKED;
    reason = p.reason;
    return false;
  }
  if(p.align.knots.size() < 2 || (p.align.knots.front().eta - target.eta).norm() > 1e-9)
  {
    fault("plan start no longer matches held target");
    return false;
  }
  Reference first;
  if(!samplePath(model, p.align, 0, first) || (first.body.matrix() - target.body.matrix()).norm() > 1e-9 ||
     (first.cog.matrix() - target.cog.matrix()).norm() > 1e-9 || first.velocity.norm() > 1e-6 ||
     first.acceleration.norm() > 1e-6)
  {
    fault("connection first reference differs from coherent held start");
    return false;
  }
  if(!cachePathTravel(p.align) || (!p.goal.align_only && !cachePathTravel(p.cut)))
  {
    fault("invalid path braking envelope");
    return false;
  }
  plan = std::move(p);
  progress = nominal_progress = 0;
  speed_ = 0;
  governor_hold_ = false;
  stopping_ = cancel_ = false;
  task.reset(plan.goal.pitch_start);
  planned_pitch = plan.goal.pitch_start;
  path_end_commanded = goal_reached = progress_lag = false;
  lag_since_ = 0;
  best_forward_ = 0;
  applied_relief = 0;
  stage = Stage::CONTACT_LOCKED;  // Every connection, including a zero-distance one, settles before READY.
  reason = plan.reason;
  return true;
}
bool Executor::command(const std::string& cmd, std::uint64_t id)
{
  if(id != session || !owns())
  {
    reason = "stale or missing lock session";
    return false;
  }
  if(cmd == "prepare" && (!planning.valid() || !config.governorValid(planning) || !config.settlingValid()))
  {
    reason = "invalid offline planning configuration";
    return false;
  }
  if(cmd == "prepare" && (stage == Stage::CONTACT_LOCKED || stage == Stage::COMPLETED_HOLD))
  {
    plan = Plan();
    resetSettling();
    stage = Stage::PLANNING;
    reason = "preparing from held configuration";
    return true;
  }
  if(cmd == "align" && stage == Stage::CONTACT_LOCKED && plan.ok)
  {
    stage = Stage::PRE_ALIGN;
    motion_clock_pending_ = true;
    progress = nominal_progress = 0;
    speed_ = 0;
    resetSettling();
    reason = "aligning along geometrically validated path";
    return true;
  }
  if(cmd == "start" && stage == Stage::READY)
  {
    if(plan.cut.knots.empty())
    {
      reason = "alignment-only plan; prepare a pitch stroke to start";
      return false;
    }
    stage = Stage::EXECUTING;
    motion_clock_pending_ = true;
    progress = nominal_progress = 0;
    speed_ = 0;
    governor_hold_ = false;
    resetSettling();
    task.reset(plan.goal.pitch_start);
    lag_since_ = 0;
    best_forward_ = 0;
    goal_reached = false;
    reason = "executing geometrically validated pitch stroke";
    return true;
  }
  if((cmd == "pause" || cmd == "cancel") && (stage == Stage::PRE_ALIGN || stage == Stage::EXECUTING))
  {
    stopping_ = true;
    cancel_ = cmd == "cancel";
    resetSettling();
    reason = "decelerating along validated path";
    return true;
  }
  if(cmd == "resume" && stage == Stage::PAUSED && !feedback_waiting_)
  {
    stage = paused_align_ ? Stage::PRE_ALIGN : Stage::EXECUTING;
    motion_clock_pending_ = true;
    stopping_ = false;
    lag_since_ = 0;
    best_forward_ = task.current;
    governor_hold_ = false;
    resetSettling();
    reason = paused_align_ ? "resuming validated alignment" : "resuming validated pitch stroke";
    return true;
  }
  if(cmd == "cancel" && (stage == Stage::PAUSED || stage == Stage::PLANNING || stage == Stage::CONTACT_LOCKED ||
                         stage == Stage::READY || stage == Stage::COMPLETED_HOLD))
  {
    plan = Plan();
    resetSettling();
    stage = Stage::COMPLETED_HOLD;
    reason = "cancelled; holding contact";
    return true;
  }
  reason = "command rejected in current trajectory state";
  return false;
}
void Executor::fault(const std::string& why)
{
  const auto last_settling = settling;
  resetSettling();
  settling = last_settling;
  if(owns())
  {
    // Preserve the initiating cause for state/UI; diagnostics retain the latest
    // observation in reason even if subsequent tracking checks also fail.
    if(stage != Stage::FAULT_HOLD)
      first_fault = why;
    stage = Stage::FAULT_HOLD;
  }
  reason = why;
  stopping_ = cancel_ = false;
  target.velocity.setZero();
  target.acceleration.setZero();
  target.body_omega.setZero();
  target.body_acceleration.setZero();
  target.cog_velocity.setZero();
  target.cog_acceleration.setZero();
  target.cog_omega.setZero();
  target.cog_angular_acceleration.setZero();
  target.desired = {target.body.linear(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
}
void Executor::hold(const std::string& why)
{
  if(!owns() || stage == Stage::FAULT_HOLD)
    return;
  // Freeze at the applied reference, retaining the contact and lock identity.
  fault(why);
  first_fault.clear();
  stage = Stage::CONTACT_LOCKED;
}
void Executor::waitForFeedback(double now, const std::string& why)
{
  if(!owns() || stage == Stage::FAULT_HOLD)
    return;
  if(stage == Stage::PRE_ALIGN || stage == Stage::EXECUTING)
  {
    paused_align_ = stage == Stage::PRE_ALIGN;
    stage = cancel_ ? Stage::COMPLETED_HOLD : Stage::PAUSED;
    if(cancel_)
      plan = Plan();
  }
  if(stage == Stage::PLANNING)
  {
    plan = Plan();
    stage = Stage::CONTACT_LOCKED;
  }
  if(applied_valid)
    target = applied;
  scaleReference(target, 0, 0);
  applied = target;
  speed_ = 0;
  last_time_ = now;
  motion_clock_pending_ = true;
  lag_since_ = 0;
  governor_hold_ = false;
  stopping_ = cancel_ = false;
  feedback_waiting_ = true;
  resetSettling();
  reason = "waiting for coherent feedback: " + why;
}
void Executor::reset()
{
  stage = Stage::FREE_FLIGHT;
  feedback_waiting_ = false;
  ++session;
  plan = Plan();
  first_fault.clear();
  reason = "perching disabled";
  last_time_ = 0;
  resetSettling();
}
void Executor::resetSettling()
{
  stable_since_ = ready_since_ = settling_stamp_ = settling_elapsed = 0;
  settling = SettlingStatistics();
  settling_window_.reset();
}
void Executor::tick(const Feedback& s)
{
  reference_updated = false;
  if(!owns())
    return;
  if(!std::isfinite(s.now) || s.now < last_time_)
  {
    fault("execution clock moved backwards/invalid");
    return;
  }
  const auto status = roll_yaw::feedbackStatus(s, config, planning);
  if(status == FeedbackStatus::WAITING)
  {
    waitForFeedback(s.now, feedbackRejection(s));
    return;
  }
  if(status == FeedbackStatus::FAULT)
  {
    fault(feedbackRejection(s));
    return;
  }
  if(feedback_waiting_)
  {
    feedback_waiting_ = false;
    last_time_ = s.now;
    motion_clock_pending_ = true;
    reason = "feedback restored; explicit resume required for paused motion";
  }
  if(!model.estimate(s.body, s.eta, measured))
  {
    fault("passive hinge estimation failed");
    return;
  }
  Eigen::Vector3d measured_q = s.eta;
  measured_q[0] = measured.alpha;
  if((measured_q.array() < planning.limits.lower.array()).any() ||
     (measured_q.array() > planning.limits.upper.array()).any() || s.body.linear()(2, 2) < std::cos(planning.max_tilt))
  {
    fault("measured mechanism/body outside configured operating range");
    return;
  }
  if(measured.pivot_error > config.pivot_tolerance || measured.hinge_residual > config.hinge_tolerance)
  {
    std::ostringstream why;
    why << "pivot=" << measured.pivot_error << "/" << config.pivot_tolerance << " m; hinge=" << measured.hinge_residual
        << "/" << config.hinge_tolerance << " rad";
    fault(why.str());
    return;
  }
  const Reference& tracking = applied_valid ? applied : target;
  const double joint_error = (measured_q - tracking.eta).cwiseAbs().maxCoeff();
  const double body_error = logSO3(s.body.linear().transpose() * tracking.body.linear()).norm();
  if(joint_error > config.tracking_tolerance || body_error > config.orientation_tolerance)
  {
    std::ostringstream why;
    why << "effective tracking: mechanism=" << joint_error << "/" << config.tracking_tolerance
        << " rad; body=" << body_error << "/" << config.orientation_tolerance << " rad";
    fault(why.str());
    return;
  }
  const double dt = s.now - last_time_;
  last_time_ = s.now;
  const bool moving = stage == Stage::PRE_ALIGN || stage == Stage::EXECUTING;
  if(dt < 0)
  {
    fault("execution clock moved backwards");
    return;
  }
  if(moving && motion_clock_pending_)
  {
    motion_clock_pending_ = false;
    return;
  }
  if(moving && (!std::isfinite(dt) || dt < 0 || dt > config.max_execution_step))
  {
    fault("invalid execution clock step");
    return;
  }
  if(moving && dt == 0)
    return;
  if(stage == Stage::FAULT_HOLD)
    return;
  if(plan.ok && !plan.goal.align_only)
  {
    const Eigen::Matrix3d pre = cuttingOrientation(plan.neutral, plan.goal, 0, 0, 0).rotation;
    if(!task.observe(pre, s.body.linear(), s.body_stamp, plan.goal.pitch_start, plan.goal.pitch_end,
                     config.progress_filter_time, config.orientation_tolerance))
    {
      fault("degenerate measured task twist");
      return;
    }
    goal_error = plan.goal.pitch_end - task.raw;
    goal_orientation_error = logSO3(s.body.linear().transpose() *
                                    cuttingOrientation(plan.neutral, plan.goal, plan.goal.pitch_end, 0, 0).rotation)
                                 .norm();
  }
  if(plan.ok && stage == Stage::READY)
  {
    Reference start;
    if(!samplePath(model, plan.align, plan.align.duration(), start) || (tracking.eta - start.eta).norm() > 1e-9 ||
       (tracking.body.matrix() - start.body.matrix()).norm() > 1e-9 ||
       (tracking.cog.matrix() - start.cog.matrix()).norm() > 1e-9)
    {
      plan = Plan();
      hold("applied prepared start changed; prepare again");
      return;
    }
    if(joint_error > config.preparation_tolerance || body_error > config.preparation_tolerance)
      reason = "prepared, waiting for live start tracking";
    else if(reason == "prepared, waiting for live start tracking")
      reason = "READY: prepared start tracking restored; explicit start required";
  }
  if(stage != Stage::PRE_ALIGN && stage != Stage::EXECUTING)
    return;
  const auto& path = stage == Stage::EXECUTING ? plan.cut : plan.align;
  if(!plan.ok || plan.session != session)
  {
    fault("trajectory ownership/session mismatch");
    return;
  }
  const bool cut = stage == Stage::EXECUTING;
  double speed_acc = std::min((planning.limits.acceleration.array() / planning.limits.velocity.array()).minCoeff(),
                              planning.cog_acceleration_limit / planning.cog_speed_limit) *
                     .5;
  double speed_cap = 1;
  const bool correcting = cut && (relief_enabled || std::abs(relief) > 1e-10 || speed_ < 0);
  if(correcting)
  {
    if(!std::isfinite(relief) || !std::isfinite(relief_angle_limit) || relief_angle_limit <= 0 ||
       !std::isfinite(relief_rate_limit) || relief_rate_limit <= 0 || !std::isfinite(relief_acceleration_limit) ||
       relief_acceleration_limit <= 0)
    {
      fault("invalid scalar relief limits/state");
      return;
    }
    // Stroke derivatives are bounded by the requested magnitudes (time scaling
    // only reduces them). Reserve half the angular acceleration for each chain term.
    speed_cap = std::min(
        {1.0, relief_rate_limit / plan.goal.rate, std::sqrt(.5 * relief_acceleration_limit / plan.goal.acceleration)});
    speed_acc = std::min(speed_acc, .5 * relief_acceleration_limit / plan.goal.rate);
  }
  const double direction = plan.goal.pitch_end >= plan.goal.pitch_start ? 1 : -1;
  const Eigen::Matrix3d pre = cuttingOrientation(plan.neutral, plan.goal, 0, 0, 0).rotation;
  auto pitch = [&](double t)
  {
    double p = 0, c = 0;
    if(!taskPitch(pre, path.desired(t).rotation, plan.goal.pitch_start, p, c))
      return std::numeric_limits<double>::quiet_NaN();
    return direction * (p - plan.goal.pitch_start);
  };
  const double length = std::abs(plan.goal.pitch_end - plan.goal.pitch_start);
  auto phaseAt = [&](double angle)
  {
    if(angle <= 0)
      return 0.0;
    if(angle >= length - 1e-10)
      return path.duration();
    double lo = 0, hi = path.duration();
    for(int i = 0; i < 36; ++i)
    {
      const double mid = (lo + hi) * .5;
      if(pitch(mid) > angle)
        hi = mid;
      else
        lo = mid;
    }
    return (lo + hi) * .5;
  };
  double lower = 0, upper = path.duration();
  bool invalid_band = false;
  if(cut)
  {
    if(!std::isfinite(pitch(progress)))
    {
      fault("invalid prepared pitch map");
      return;
    }
    const double low_angle = std::max(0.0, task.upper_distance - config.reference_lead);
    const double high_angle = std::min(length, task.current + config.reference_lead);
    invalid_band = !task.valid || low_angle > high_angle;
    if(!invalid_band)
    {
      lower = phaseAt(low_angle);
      upper = phaseAt(high_angle);
    }
    else
      lower = upper = progress;
    // Retreat remains within the configured angle from the frozen nominal point.
    if(correcting)
      lower = std::max(lower, phaseAt(std::max(0.0, pitch(nominal_progress) - relief_angle_limit)));
    else
      lower = std::max(lower, progress);  // ordinary execution is forward/hold
  }
  const Eigen::Vector3d origin = pathTravel(path, progress);
  const Eigen::Vector3d joint_errors = (measured_q - tracking.eta).cwiseAbs();
  double soft_joint = config.tracking_tolerance * config.soft_tracking_fraction;
  double soft_body = config.orientation_tolerance * config.soft_tracking_fraction;
  if(!cut)
  {
    const double endpoint_joint = (measured_q - path.knots.back().eta).cwiseAbs().maxCoeff();
    const double endpoint_body = logSO3(s.body.linear().transpose() * path.desired(path.duration()).rotation).norm();
    if(endpoint_joint < joint_error && joint_error > soft_joint)
      soft_joint =
          std::max(soft_joint, config.tracking_tolerance -
                                   .5 * planning.limits.velocity.maxCoeff() * config.max_execution_step - 1e-6);
    if(endpoint_body < body_error && body_error > soft_body)
      soft_body = std::max(soft_body, config.orientation_tolerance -
                                          .5 * planning.limits.velocity.sum() * config.max_execution_step - 1e-6);
  }
  auto travelAllowed = [&](double t)
  {
    const Eigen::Vector3d travel = (pathTravel(path, t) - origin).cwiseAbs();
    return (travel.array() + joint_errors.array() <= soft_joint + 1e-12).all() &&
           travel.sum() + body_error <= soft_body + 1e-12;
  };
  auto horizon = [&](double end)
  {
    double near = progress, far = end;
    for(int i = 0; i < 32; ++i)
    {
      const double mid = (near + far) * .5;
      if(travelAllowed(mid))
        near = mid;
      else
        far = mid;
    }
    return near;
  };
  // Do not erase a shrinking measured boundary by silently clamping to progress.
  const bool outside_band = lower > progress + 1e-8 || upper < progress - 1e-8;
  if(!outside_band)
  {
    lower = horizon(lower);
    upper = horizon(upper);
  }
  if(governor_hold_ && (!cut || task.valid) && joint_error < soft_joint * config.resume_fraction &&
     body_error < soft_body * config.resume_fraction)
    governor_hold_ = false;
  if(invalid_band || joint_error >= soft_joint || body_error >= soft_body ||
     (std::abs(speed_) < .01 && !resisting_load && relief >= -1e-10 && upper - progress < 1e-4 &&
      upper < path.duration() - 1e-7))
    governor_hold_ = true;
  if(interaction_hold && !stopping_)
  {
    stopping_ = true;
    cancel_ = false;
    reason = "interaction input unavailable; braking to PAUSED; explicit resume required";
  }
  double next_nominal = nominal_progress;
  double destination = upper;
  if(cut)
  {
    // Freeze the nominal schedule during relief, tracking holds and recovery.
    // Release the correction before normal feeding; no wall-clock debt.
    if(!stopping_ && !interaction_hold && !resisting_load && !governor_hold_ && std::abs(relief) < 1e-8)
      next_nominal = std::max(nominal_progress, std::min(upper, nominal_progress + dt));
    const double correction = correcting ? std::clamp(relief, -relief_angle_limit, 0.0) : 0;
    const double wanted = std::clamp(pitch(next_nominal) + correction, 0.0, length);
    destination = phaseAt(wanted);
    relief_available = correcting && !outside_band && !invalid_band
                           ? std::clamp(pitch(next_nominal) - pitch(lower), 0.0, relief_angle_limit)
                           : 0;
    relief_limited = resisting_load && (wanted <= 1e-10 || -relief >= relief_available - 1e-8);
    if(!outside_band)
      destination = std::clamp(destination, lower, upper);
    // A resisting load may authorize reverse relief, never additional feed.
    if(resisting_load)
      destination = std::min(destination, progress);
  }
  const bool reverse_requested = correcting && destination < progress - 1e-8 && lower < progress - 1e-8;
  // A forward-feed latch must not forbid safe retreat inside the same symmetric
  // tracking allowance. It still freezes the nominal schedule.
  const bool brake = stopping_ || (governor_hold_ && !reverse_requested) || invalid_band || outside_band;
  const double desired_speed =
      brake ? 0
            : (destination > progress + 1e-8 ? speed_cap
                                             : (correcting && destination < progress - 1e-8 ? -speed_cap : 0));
  const double dv = speed_acc * dt;
  double new_speed = std::clamp(desired_speed, speed_ - dv, speed_ + dv);
  // Do not change direction inside one trapezoidal integration step.
  if(speed_ * new_speed < 0)
    new_speed = 0;
  const double moving_direction =
      speed_ != 0 ? std::copysign(1.0, speed_) : (new_speed != 0 ? std::copysign(1.0, new_speed) : 0);
  if(moving_direction != 0)
  {
    const double boundary = moving_direction > 0 ? upper : lower;
    const double gap = moving_direction * (boundary - progress), old = std::abs(speed_);
    auto safeSpeed = [&](double remaining)
    {
      const double b = speed_acc * (dt + config.max_execution_step);
      return std::max(
          0.0, (-b + std::sqrt(std::max(0.0, b * b - 4 * dv * old + 8 * speed_acc * std::max(0.0, remaining)))) * .5);
    };
    const double safe = safeSpeed(gap), minimum = std::max(0.0, old - dv);
    // Only the admissible boundary is a protective constraint. A moving scalar
    // destination can cross the braking envelope: brake through zero, then return.
    if(gap < -1e-8 || (safe + 1e-8 < minimum && old > 1e-8))
    {
      fault("tracking/relief boundary contracted inside braking envelope; protective reference hold");
      return;
    }
    double commanded = std::min(std::abs(new_speed), safe);
    if(!brake)
      commanded = std::min(commanded, safeSpeed(moving_direction * (destination - progress)));
    new_speed = moving_direction * std::max(minimum, commanded);
  }
  const double next_progress = progress + (speed_ + new_speed) * .5 * dt;
  const double phase_acc = (new_speed - speed_) / dt;
  if(next_progress < -1e-9 || next_progress > path.duration() + 1e-9 ||
     (!outside_band && !invalid_band && (next_progress < lower - 1e-8 || next_progress > upper + 1e-8)))
  {
    fault("signed phase step exceeds prepared/tracking bounds");
    return;
  }
  Reference next;
  std::string sample_error;
  if(!samplePath(model, path, next_progress, next))
  {
    fault("invalid trajectory sample");
    return;
  }
  // Preserve the acceleration term at zero crossing; only a completed hold zeros it.
  scaleReference(next, new_speed, phase_acc);
  if(!referenceAllowed(next, planning, sample_error) ||
     (next.eta - measured_q).cwiseAbs().maxCoeff() > config.tracking_tolerance ||
     logSO3(s.body.linear().transpose() * next.body.linear()).norm() > config.orientation_tolerance ||
     (correcting && (next.desired.omega.norm() > relief_rate_limit + 1e-8 ||
                     next.desired.acceleration.norm() > relief_acceleration_limit + 1e-8)))
  {
    fault("signed reference rejected: " + sample_error);
    return;
  }
  // Commit scalar path state only after the same coherent reference passed bounds.
  progress = next_progress;
  nominal_progress = next_nominal;
  speed_ = new_speed;
  target = next;
  reference_updated = true;
  const bool restricted = governor_hold_ || resisting_load || interaction_hold || (cut && relief < -1e-8);
  progress_lag = !stopping_ && restricted;
  const bool pending_progress = progress < path.duration() - 1e-7 || (cut && restricted);
  if(!stopping_)
  {
    if(cut && task.valid && task.current > best_forward_ + std::max(1e-4, .1 * config.goal_tolerance))
    {
      best_forward_ = task.current;
      lag_since_ = s.now;
    }
    if(lag_since_ == 0)
      lag_since_ = s.now;
    if((cut || progress_lag) && pending_progress && s.now - lag_since_ > config.progress_timeout)
    {
      stopping_ = true;
      cancel_ = false;
      reason = "no meaningful measured forward progress; braking to PAUSED, original goal pending";
    }
    else if(pending_progress)
    {
      reason = governor_hold_ && !reverse_requested    ? "tracking hold; original goal pending"
               : speed_ < -1e-8                        ? "bounded on-path relief retreat; original goal pending"
               : resisting_load                        ? (relief_limited ? "relief limit reached; original goal pending"
                                                                         : "resisting-load hold; original goal pending")
               : correcting && std::abs(relief) > 1e-8 ? "bounded correction recovery; original goal pending"
               : cut                                   ? "feeding prepared stroke"
                                                       : "advancing prepared connection";
    }
  }
  if(!cut && !progress_lag)
    lag_since_ = s.now;
  if(cut)
  {
    double cross;
    taskPitch(pre, next.desired.rotation, planned_pitch, planned_pitch, cross);
    applied_relief = pitch(progress) - pitch(nominal_progress);
    path_end_commanded = progress >= path.duration() - 1e-7;
  }
  // Reaching the path's zero-rate endpoint also completes a stop. Otherwise an
  // automatic alignment could enter READY before deceleration finished and leave
  // a pending cancel/disable stranded there.
  if(stopping_ && (std::abs(speed_) < 1e-10 || progress >= path.duration() || progress <= 0))
  {
    resetSettling();
    paused_align_ = stage == Stage::PRE_ALIGN;
    stage = cancel_ ? Stage::COMPLETED_HOLD : Stage::PAUSED;
    // A hold must not retain the final deceleration feed-forward indefinitely.
    target.velocity.setZero();
    target.acceleration.setZero();
    target.body_omega.setZero();
    target.body_acceleration.setZero();
    target.cog_velocity.setZero();
    target.cog_acceleration.setZero();
    target.cog_omega.setZero();
    target.cog_angular_acceleration.setZero();
    target.desired = {target.body.linear(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    reason = cancel_ ? "cancelled; stopped at contact-consistent reference"
                     : "PAUSED: " + reason + "; coherent reference held; explicit resume required";
    if(cancel_)
      plan = Plan();
    stopping_ = cancel_ = false;
    return;
  }
  if(progress < path.duration() - 1e-7 || stopping_ ||
     (cut && (resisting_load || interaction_hold || std::abs(relief) >= 1e-8)))
    resetSettling();
  if(progress >= path.duration() - 1e-7 && !stopping_ &&
     (!cut || (!resisting_load && !interaction_hold && std::abs(relief) < 1e-8)))
  {
    if(ready_since_ == 0)
      ready_since_ = s.now;
    bool qualified = false;
    std::ostringstream detail;
    if(config.settle_use_window)
    {
      // Do not lose faster-body excursions between distinct joint timestamps.
      if(s.body_omega.norm() >= config.settle_body_rate || s.joint_rate.tail<2>().norm() >= config.settle_joint_rate)
        settling_window_.reset();
      const double sample_stamp = std::min(s.body_stamp, s.joint_stamp);
      const auto stats =
          settling_window_.observe(sample_stamp, {{s.body_omega.x(), s.body_omega.y(), s.body_omega.z()}},
                                   {{s.joint_rate[1], s.joint_rate[2]}}, config.settle_time, config.timeout);
      settling_stamp_ = sample_stamp;
      settling = stats;
      qualified = stats.qualifies(config.settle_body_rate, config.settle_joint_rate, config.settle_mean_rate);
      detail << "window " << stats.observed << "/" << config.settle_time << " s (" << stats.status << ")";
      if(stats.complete)
      {
        detail << "; body RMS=" << stats.body_rms << " joint RMS=" << stats.joint_rms
               << "; body mean=" << stats.body_mean << " joint mean=" << stats.joint_mean;
        if(stats.body_rms >= config.settle_body_rate)
          detail << "; body RMS exceeds bound";
        if(stats.joint_rms >= config.settle_joint_rate)
          detail << "; joint RMS exceeds bound";
        if(stats.body_mean >= config.settle_mean_rate)
          detail << "; body mean drift exceeds bound";
        if(stats.joint_mean >= config.settle_mean_rate)
          detail << "; joint mean drift exceeds bound";
      }
      detail << "; current body=" << stats.body_current << " joint=" << stats.joint_current
             << "; bounds body=" << config.settle_body_rate << " joint=" << config.settle_joint_rate
             << " mean=" << config.settle_mean_rate << " rad/s";
      if(stats.body_current >= config.settle_body_rate)
        detail << "; current body rate exceeds bound";
      if(stats.joint_current >= config.settle_joint_rate)
        detail << "; current joint rate exceeds bound";
    }
    else
    {
      const double body_rate = s.body_omega.norm(), joint_rate = s.joint_rate.tail<2>().norm();
      const bool settled = joint_rate < config.settle_rate && body_rate < config.settle_rate;
      const double sample_stamp = std::min(s.body_stamp, s.joint_stamp);
      if(sample_stamp < settling_stamp_ || sample_stamp - settling_stamp_ > config.timeout)
        stable_since_ = 0;
      settling_stamp_ = sample_stamp;
      if(!settled)
        stable_since_ = 0;
      else if(stable_since_ == 0)
        stable_since_ = sample_stamp;
      qualified = stable_since_ > 0 && sample_stamp - stable_since_ >= config.settle_time;
      detail << "strict quiet time " << (stable_since_ > 0 ? sample_stamp - stable_since_ : 0) << "/"
             << config.settle_time << " s; current body=" << body_rate << " joint=" << joint_rate
             << "; bound=" << config.settle_rate << " rad/s";
    }
    const bool at_goal =
        stage == Stage::PRE_ALIGN
            ? (logSO3(s.body.linear().transpose() * path.desired(path.duration()).rotation).norm() <=
                   config.goal_tolerance &&
               (measured_q - path.knots.back().eta).cwiseAbs().maxCoeff() <= config.goal_tolerance)
            : (task.valid && std::abs(goal_error) <= config.goal_tolerance &&
               goal_orientation_error <= config.goal_tolerance &&
               (measured_q - plan.cut.knots.back().eta).cwiseAbs().maxCoeff() <= config.tracking_tolerance);
    if(!at_goal)
    {
      qualified = false;
      settling_window_.reset();
      settling = SettlingStatistics();
      settling_stamp_ = 0;
      stable_since_ = 0;
      detail << "; original measured goal pending";
      if(stage == Stage::PRE_ALIGN)
        detail << "; alignment SO(3) error="
               << logSO3(s.body.linear().transpose() * path.desired(path.duration()).rotation).norm();
      else
        detail << "; pitch error=" << goal_error << " full SO(3) error=" << goal_orientation_error;
    }
    settling_elapsed = s.now - ready_since_;
    if(settling_elapsed > config.settle_timeout)
    {
      fault("settling timed out: " + detail.str());
      return;
    }
    if(qualified)
    {
      if(stage == Stage::PRE_ALIGN && !plan.goal.align_only)
      {
        plan.measured_start.eta = measured_q;
        plan.measured_start.body = s.body;
        plan.measured_start_valid = true;
      }
      stage = stage == Stage::PRE_ALIGN ? Stage::READY : Stage::COMPLETED_HOLD;
      goal_reached = stage == Stage::COMPLETED_HOLD;
      reason = stage == Stage::READY ? "MOTION_COMPLETE: alignment settled; READY for next command"
                                     : "GOAL_REACHED: measured original endpoint settled; holding contact";
      resetSettling();
    }
    else
      reason = "waiting for measured settling: " + detail.str();
  }
}
}  // namespace roll_yaw
