// -*- mode: c++ -*-
#pragma once

#include <algorithm>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <map>
#include <functional>
#include <string>
#include <kdl/frames.hpp>
#include <cstdint>
#include <limits>
#include <kdl/treefksolverpos_recursive.hpp>
#include <vector>
#include <array>
#include <deque>
#include <chrono>
#include <memory>
#include <mutex>

// Geometry and target-provider contracts

namespace perching_geometry
{
enum class Mode
{
  DISABLED,
  NORMAL,
  SLANTED
};

struct Session
{
  Mode mode;
  ros::Time lock_stamp;
};

inline bool finite(const tf::Vector3& v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

struct Pose
{
  tf::Vector3 position;
  tf::Quaternion orientation;
  double delta = 0.0;  // Bounded logical angle, before arc sign.
};

struct Lock
{
  tf::Vector3 position{0, 0, 0}, pivot{0, 0, 0}, radius{0, 0, 0}, axis{0, 0, 0};
  tf::Quaternion orientation{0, 0, 0, 1};
  bool valid = false;

  bool initialize(const tf::Vector3& p, const tf::Vector3& c, const tf::Quaternion& q, double min_radius)
  {
    valid = false;
    if(!finite(p) || !finite(c) || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
       !std::isfinite(q.w()) || !std::isfinite(q.length2()) || q.length2() < 1.0e-12 || !std::isfinite(min_radius) ||
       min_radius <= 0.0)
      return false;
    position = p;
    pivot = c;
    orientation = q.normalized();
    radius = p - c;
    axis = tf::Matrix3x3(orientation).getColumn(1).normalized();
    valid = finite(radius) && finite(axis) && std::isfinite(radial().length()) && radial().length() >= min_radius;
    return valid;
  }

  tf::Vector3 radial() const
  {
    return radius - axis * axis.dot(radius);
  }

  // Legacy absolute-attitude input projected onto the captured local-Y arc.
  bool targetOrientation(const tf::Quaternion& desired, double limit, double sign, Pose& pose) const;

  bool target(double delta, double limit, double sign, Pose& pose) const
  {
    if(!valid || !std::isfinite(delta) || !std::isfinite(limit) || limit <= 0.0 || !std::isfinite(sign) ||
       std::abs(sign) != 1.0 || !finite(position) || !finite(pivot) || !finite(radius) || !finite(axis) ||
       std::abs(axis.length2() - 1.0) > 1.0e-6 || !std::isfinite(orientation.length2()) ||
       std::abs(orientation.length2() - 1.0) > 1.0e-6)
      return false;
    pose.delta = std::max(-limit, std::min(limit, delta));
    // Preserve the lock exactly for zero; avoid subtract/add roundoff.
    pose.position = position;
    pose.orientation = orientation;
    if(pose.delta != 0.0)
    {
      const tf::Quaternion rotation(axis, sign * pose.delta);
      pose.position = pivot + tf::Matrix3x3(rotation) * radius;
      pose.orientation = (rotation * orientation).normalized();
    }
    return finite(pose.position);
  }

  bool project(const tf::Vector3& desired, double limit, double sign, Pose& pose) const
  {
    if(!valid || !finite(desired) || !std::isfinite(sign) || std::abs(sign) != 1.0)
      return false;
    const tf::Vector3 relative = desired - pivot;
    const tf::Vector3 perpendicular = relative - axis * axis.dot(relative);
    if(!finite(perpendicular) || perpendicular.length2() < 1.0e-12)
      return false;
    const tf::Vector3 r = radial().normalized();
    const tf::Vector3 d = perpendicular.normalized();
    const double physical_angle = std::atan2(axis.dot(r.cross(d)), r.dot(d));
    return target(physical_angle / sign, limit, sign, pose);
  }

  tf::Vector3 tangentVelocity(const tf::Vector3& current, const tf::Vector3& velocity) const
  {
    if(!valid || !finite(current) || !finite(velocity))
      return tf::Vector3(0, 0, 0);
    const tf::Vector3 tangent = axis.cross(current - pivot);
    if(!finite(tangent) || tangent.length2() < 1.0e-12)
      return tf::Vector3(0, 0, 0);
    const tf::Vector3 unit_tangent = tangent.normalized();
    return unit_tangent * unit_tangent.dot(velocity);
  }
};

// Shared interface avoids asynchronous delta topics in the control loop and
// ensures admittance uses the navigator's current lock, limit and arc sign.
class TargetProvider
{
public:
  virtual ~TargetProvider() = default;
  // Read mode and lock identity together across asynchronous callbacks.
  virtual Session perchingSession() const = 0;
  Mode perchingMode() const
  {
    return perchingSession().mode;
  }
  virtual bool perchingAdmittanceTarget(const ros::Time& lock_stamp, double physical_offset,
                                        const tf::Vector3& nominal_position, Pose& pose) const = 0;
};
}  // namespace perching_geometry

namespace perching_geometry
{
Eigen::Vector3d logSO3(const Eigen::Matrix3d& r);
Eigen::Matrix3d expSO3(const Eigen::Vector3d& v);
bool rotationValid(const Eigen::Matrix3d& r);
bool poseValid(const Eigen::Isometry3d& p);
Eigen::Isometry3d fromKdl(const KDL::Frame& f);
KDL::Frame toKdl(const Eigen::Isometry3d& p);
void accumulateLinkInertia(KDL::RigidBodyInertia& total, const KDL::Frame& frame, const std::string& link,
                           const KDL::RigidBodyInertia& inertia, const std::map<std::string, KDL::Segment>& extra);
struct MotionIdentity
{
  std::uint64_t epoch = 1, sequence = 0;
  std::string source;
  std::string rejection(std::uint64_t e, const std::string& s, std::uint64_t n, bool claim = false) const
  {
    if(e != epoch) return "cancelled motion epoch";
    if(s.empty() || s.size() > 128 || n == 0 ||
       s.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos)
      return "invalid motion identity";
    if(!claim && !source.empty() && source != s) return "another source owns motion; claim after nominal motion finishes";
    if(source == s && n <= sequence) return "old motion sequence";
    return "";
  }
  void accept(const std::string& s, std::uint64_t n) { source = s; sequence = n; }
  void revoke() { ++epoch; source.clear(); sequence = 0; }
};

}
namespace perching_servo
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
  KDL::Vector pivot_offset{0, 0, 0}, hinge_axis{0, 1, 0};

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
}  // namespace perching_servo

// Wrench operations

namespace perching_dynamics
{
// Rotate axes only; the wrench remains about the same physical point.
Eigen::Matrix<double, 6, 1> rotateWrench(const Eigen::Matrix3d& rotation, const Eigen::Matrix<double, 6, 1>& wrench);

// All vectors use the same axes. source_minus_target points from the new
// reference point to the old one: M_target = M_source + r x F.
Eigen::Vector3d shiftMoment(const Eigen::Vector3d& moment, const Eigen::Vector3d& source_minus_target,
                            const Eigen::Vector3d& force);
}  // namespace perching_dynamics
