#pragma once

// Single-pitch mechanism geometry and state. The ROS adapter serializes calls
// with the same perching mutex as the multilink navigator.
#include <kdl/frames.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>

namespace pitch_only
{
constexpr double unknown = std::numeric_limits<double>::quiet_NaN();
enum class Stage
{
  FREE_FLIGHT,
  CONTACT_LOCKED_PRE_REZERO,
  PITCH_REZERO_ACTIVE,
  PITCH_REZERO_READY_HOLD,
  SERVO_ADJUSTMENT,
  PASSIVE_CUTTING,
  UNPERCH_PREP,
  DISENGAGED,
  FAULT_HOLD
};
inline const char* name(Stage s)
{
  static const char* names[] = {
      "FREE_FLIGHT",      "CONTACT_LOCKED_PRE_REZERO", "PITCH_REZERO_ACTIVE", "PITCH_REZERO_READY_HOLD",
      "SERVO_ADJUSTMENT", "PASSIVE_CUTTING",           "UNPERCH_PREP",        "DISENGAGED",
      "FAULT_HOLD"};
  return names[static_cast<int>(s)];
}
inline bool finite(const KDL::Vector& p)
{
  return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
}
inline bool finite(const KDL::Frame& f)
{
  if(!finite(f.p))
    return false;
  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
      if(!std::isfinite(f.M(i, j)))
        return false;
  return std::abs(f.M.UnitX().Norm() - 1) < 1e-6 && std::abs(f.M.UnitY().Norm() - 1) < 1e-6 &&
         std::abs(f.M.UnitZ().Norm() - 1) < 1e-6 && std::abs(KDL::dot(f.M.UnitX(), f.M.UnitY())) < 1e-6 &&
         (f.M.UnitX() * f.M.UnitY() - f.M.UnitZ()).Norm() < 1e-6;
}
inline double pitch(const KDL::Frame& f)
{
  double r, p, y;
  f.M.GetRPY(r, p, y);
  return p;
}
// ZYX Euler pitch derivative from BASELINK angular velocity. Using the
// estimator's rate avoids amplifying pose measurement noise by differentiation.
inline double pitchRate(const KDL::Frame& body, const KDL::Vector& omega)
{
  double roll, p, y;
  body.M.GetRPY(roll, p, y);
  return std::cos(roll) * omega.y() - std::sin(roll) * omega.z();
}
inline double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(x, hi));
}
inline double wrap(double x)
{
  return std::atan2(std::sin(x), std::cos(x));
}
inline KDL::Frame arc(const KDL::Frame& f, const KDL::Vector& pivot, const KDL::Vector& axis, double alpha)
{
  const auto r = KDL::Rotation::Rot(axis, alpha);
  return {r * f.M, pivot + r * (f.p - pivot)};
}

struct Config
{
  // Joint limits/rate are resolved from the URDF by the adapter. YAML may narrow
  // the range; a continuous joint requires explicit finite limits.
  double q_min = unknown, q_max = unknown, q_rate = .1;
  double q_zero_tolerance = .02, q_hold_error = .02, joint_settle_rate = .05;
  double settle_duration = .30;
  double feedback_timeout = .15, future_tolerance = .02;
  double rezero_pitch = 0, rezero_sign = -1, rezero_kp = 1;
  double rezero_rate = .1745329, rezero_range = .5235988;
  double rezero_pitch_tolerance = .0349066, rezero_rate_tolerance = .05, rezero_timeout = 5;
  double passive_min = -.5235988, passive_max = .5235988, passive_rate = .1745329;
  double body_pitch_min = -.5235988, body_pitch_max = .5235988;
  KDL::Vector pivot_offset{.21049, -.014, .0205}, hinge_axis{0, 1, 0};

  bool valid() const;
};
struct Sample
{
  double now = 0, joint_stamp = 0;
  double q = unknown, qdot = unknown, angular_speed = unknown, pitch_rate = unknown;
  bool hover = false, model_valid = false;
  KDL::Frame body;
};
using Model = std::function<bool(double, KDL::Frame&, KDL::Vector&)>;
class Core
{
public:
  Config config;
  Model model;
  Stage stage = Stage::FREE_FLIGHT;
  std::string status = "free flight; enable perching in hover to capture the measured pitch joint";
  std::uint64_t generation = 0;
  bool zero_hold_armed = false, rezero_completed = false;
  double q_target = 0, q_goal = 0, alpha = 0, alpha_goal = 0, alpha_min = 0, alpha_max = 0;
  KDL::Frame target, contact, reference;
  KDL::Vector pivot, axis, cog;

  bool attached() const
  {
    return stage != Stage::FREE_FLIGHT && stage != Stage::DISENGAGED;
  }
  bool ownsPose() const
  {
    return stage != Stage::FREE_FLIGHT;
  }
  bool bodyPitchAllowed(const KDL::Frame& body) const
  {
    const double p = pitch(body);
    return std::isfinite(p) && p >= config.body_pitch_min && p <= config.body_pitch_max;
  }
  bool fresh(double now, double stamp) const
  {
    return std::isfinite(now) && std::isfinite(stamp) && stamp > 0 && now - stamp >= -config.future_tolerance &&
           now - stamp <= config.feedback_timeout;
  }
  const char* feedbackError(const Sample& s) const;
  bool feedback(const Sample& s) const
  {
    return feedbackError(s) == nullptr;
  }
  // Match multilink joint settling: joint position/velocity, not body rotation.
  bool stopped(const Sample& s) const
  {
    return std::abs(s.qdot) <= config.joint_settle_rate;
  }
  bool steady(const Sample& s) const;
  bool rezeroReady(const Sample& s) const;
  bool reject(const char* reason)
  {
    status = reason;
    return false;
  }
  void fault(const char* reason);
  void reset();
  bool armZero(const Sample& s);
  bool enable(const Sample& s);
  bool rezero(const Sample& s);
  bool rebase(const Sample& s, Stage required);
  bool relock(const Sample& s);
  bool servo(double q, const Sample& s);
  bool freeze(const Sample& s);
  bool cut(double a, const Sample& s);
  bool unfreeze(const Sample& s)
  {
    return rebase(s, Stage::PASSIVE_CUTTING);
  }
  bool prepare(const Sample& s);
  bool released(const Sample& s);
  bool recover(const Sample& s);
  double measuredAlpha(const Sample& s) const;
  void tick(const Sample& s, double dt);

private:
  double stable_since = 0, start = 0;
  double rezero_stable_since = 0;
};
}  // namespace pitch_only
