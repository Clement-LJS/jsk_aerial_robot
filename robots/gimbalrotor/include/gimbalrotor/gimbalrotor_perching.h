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
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <exception>

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

// Specialized passive-alpha / active-roll / active-yaw geometry.

namespace perching_geometry
{
// Add an already aggregated link and its attached extra modules in root axes.
// Callers retain their own FK validation and minimum-mass policy.
void accumulateLinkInertia(KDL::RigidBodyInertia& total, const KDL::Frame& frame, const std::string& link,
                           const KDL::RigidBodyInertia& inertia, const std::map<std::string, KDL::Segment>& extra);
}  // namespace perching_geometry

namespace roll_yaw
{
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
using Pose = Eigen::Isometry3d;
Vec logSO3(const Mat& r);
Mat expSO3(const Vec& v);
bool rotationValid(const Mat& r);
bool poseValid(const Pose& p);
Pose fromKdl(const KDL::Frame& f);

// Inertia-weighted CoG reconstruction from a copied articulated model. relative
// is R_GB, exactly RobotModel::getCogDesireOrientation(), not principal axes.
std::function<bool(const Vec&, Pose&)> articulatedCogModel(KDL::Tree tree, KDL::JntArray joints,
                                                           std::map<std::string, KDL::RigidBodyInertia> inertia,
                                                           std::map<std::string, KDL::Segment> extra, std::string base,
                                                           KDL::Rotation relative, unsigned roll_index,
                                                           unsigned yaw_index);

struct Estimate
{
  double alpha = 0, pivot_error = 0, hinge_residual = 0;
};
// B=base_link, C=contact link, P=calibrated physical hinge, G=controller CoG.
// The callback supplies articulated T_BG, including the model's CoG orientation
// convention (not an inertia eigenvector frame). It must own an immutable snapshot.
class Kinematics
{
public:
  bool configure(const KDL::Chain& body_contact, const std::string& roll, const std::string& yaw,
                 const Pose& contact_pivot, const Vec& axis, std::function<bool(const Vec&, Pose&)> body_cog,
                 std::string& error);
  bool bodyPivot(const Vec& eta, Pose& bp) const;
  bool lock(const Pose& measured_body, const Vec& measured_eta);
  bool evaluateBody(const Vec& eta, Pose& body) const;
  bool evaluate(const Vec& eta, Pose& body, Pose& cog) const;
  bool jacobian(const Vec& eta, Mat& j) const;
  bool numericalJacobian(const Vec& eta, Mat& j, double h = 1e-6) const;
  bool estimate(const Pose& body, const Vec& measured_eta, Estimate& result) const;
  const Pose& pivotLock() const
  {
    return lock_;
  }
  bool locked() const
  {
    return locked_;
  }

private:
  KDL::Chain chain_;
  Pose cp_ = Pose::Identity(), lock_ = Pose::Identity();
  Vec axis_ = Vec::UnitY();
  std::function<bool(const Vec&, Pose&)> cog_;
  bool configured_ = false, locked_ = false;
};
}  // namespace roll_yaw

// Pitch-only geometry and state machine

// Single-pitch mechanism geometry and state. The navigator serializes calls
// with its common perching mutex.

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

namespace roll_yaw
{
// Generalized work per radian of signed body-local pitch progress. Wrench and
// its reference point are world expressed; includes fixed-pivot translation.
bool pathInteractionTorque(const Kinematics&, const Eigen::Vector3d& eta, double direction, double min_singular,
                           const Eigen::Vector3d& force_world, const Eigen::Vector3d& moment_world,
                           const Eigen::Vector3d& point_world, double& torque);
}  // namespace roll_yaw

// Trajectory execution and settling

namespace roll_yaw
{
// Signed continuous Y twist, with a full swing residual. Quaternion hemisphere
// changes disappear through nearest-2pi unwrapping; a 180-degree swing is invalid.
bool taskPitch(const Eigen::Matrix3d& pre, const Eigen::Matrix3d& body, double previous, double& pitch,
               double& cross_track);

struct TaskProgress
{
  // distance retains the historical reporting API; current/upper_distance govern motion.
  double raw = 0, filtered = 0, distance = 0, current = 0, upper_distance = 0, cross_track = 0, stamp = 0;
  bool valid = false;
  void reset(double start);
  bool observe(const Eigen::Matrix3d& pre, const Eigen::Matrix3d& body, double time, double start, double end,
               double filter_time, double tolerance);
};

struct SettlingStatistics
{
  bool complete = false;
  double observed = 0, body_rms = 0, joint_rms = 0, body_mean = 0, joint_mean = 0;
  double body_current = 0, joint_current = 0;
  std::string status = "collecting fresh window";

  bool qualifies(double body_limit, double joint_limit, double mean_limit) const;
};

// Executor-local numerical history; no ROS callbacks or planner-worker state.
// Body XYZ and active roll/yaw are separate vector groups; passive alpha is excluded.
class SettlingWindow
{
public:
  void reset();

  SettlingStatistics observe(double now, const std::array<double, 3>& body, const std::array<double, 2>& joint,
                             double window, double max_gap);

private:
  struct Sample
  {
    double time;
    std::array<double, 5> rate;
  };
  std::deque<Sample> samples_;
  double window_ = 0;
  SettlingStatistics last_;
};

struct Limits
{
  Eigen::Vector3d lower = Eigen::Vector3d::Constant(-1.57079632679), upper = Eigen::Vector3d::Constant(1.57079632679);
  Eigen::Vector3d velocity = Eigen::Vector3d(.15, .1, .1), acceleration = Eigen::Vector3d(.1, .1, .1);
  bool valid() const;
};
struct Request
{
  double yaw = 0, roll = 0, pitch_start = 0, pitch_end = 0, rate = .03, acceleration = .02;
  bool align_only = false;
  bool body_pitch_jog = false;  // legacy offline library request; navigator small jogs bypass prepare()
  bool valid() const;
};
struct PlanningConfig
{
  Limits limits;
  double min_singular_value = .05;
  Eigen::Vector3d saw_point = Eigen::Vector3d::Zero();  // fixed point in controlled body frame, from URDF
  bool saw_point_valid = false;
  double dt = .02, max_tilt = .2, orientation_tolerance = .003, rate_tolerance = .01;
  double alignment_rate = .04, alignment_acceleration = .02, max_duration = 180;
  double cog_speed_limit = .1, cog_acceleration_limit = .2;
  int scaling_attempts = 5;
  // Worker-local cooperative budget/progress hooks; never call into navigation.
  std::function<bool()> cancelled;
  std::function<void(int, int, bool)> report;
  bool mechanismValid() const;
  bool valid() const;
};
struct Desired
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d omega = Eigen::Vector3d::Zero(), acceleration = Eigen::Vector3d::Zero();
};
// Cosine velocity ramps, constant feed, cosine deceleration; C2 angle profile.
class Stroke
{
public:
  Stroke(double distance, double rate, double acceleration);
  double duration() const
  {
    return 2 * ramp_ + feed_;
  }
  void sample(double t, double& angle, double& rate, double& acceleration) const;

private:
  double distance_, sign_, speed_, ramp_, feed_;
};
Desired cuttingOrientation(const Eigen::Matrix3d& neutral, const Request&, double theta, double rate,
                           double acceleration);
struct Knot
{
  double t = 0;
  Eigen::Vector3d eta = Eigen::Vector3d::Zero(), velocity = Eigen::Vector3d::Zero(),
                  acceleration = Eigen::Vector3d::Zero();
};
struct Reference
{
  double t = 0, orientation_error = 0;
  Eigen::Vector3d eta = Eigen::Vector3d::Zero(), velocity = Eigen::Vector3d::Zero(),
                  acceleration = Eigen::Vector3d::Zero();
  Eigen::Isometry3d body = Eigen::Isometry3d::Identity(), cog = Eigen::Isometry3d::Identity();
  Desired desired;
  Eigen::Vector3d body_omega = Eigen::Vector3d::Zero(), body_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d cog_velocity = Eigen::Vector3d::Zero(), cog_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d cog_omega = Eigen::Vector3d::Zero(), cog_angular_acceleration = Eigen::Vector3d::Zero();
};
struct Path
{
  std::vector<Knot> knots;
  std::function<Desired(double)> desired;
  // Conservative cumulative joint variation from Bernstein derivative hulls.
  std::vector<Eigen::Vector3d> travel;
  double duration() const
  {
    return knots.empty() ? 0 : knots.back().t;
  }
};
struct Plan
{
  bool ok = false;
  std::uint64_t session = 0;
  int scale_attempt = 0;
  std::string reason;
  Request goal;
  Eigen::Matrix3d neutral = Eigen::Matrix3d::Identity();
  double wall_seconds = 0;
  Kinematics model;
  Path align, cut;
  Reference submission, applied_start;
  bool submission_valid = false;
  Reference measured_start;
  bool measured_start_valid = false;  // measured READY for diagnostics, not plan identity
  double proxy_drop = 0;              // world Z start minus end, metres; origin is not tooth contact
};
bool cachePathTravel(Path&);
Eigen::Vector3d pathTravel(const Path&, double);
void scaleReference(Reference&, double speed, double acceleration);
bool samplePath(const Kinematics&, const Path&, double time, Reference&);
// Shared mechanism validation and articulated FK.
bool mechanismAllowed(const Kinematics&, const Eigen::Vector3d&, const PlanningConfig&, std::string&);
bool mechanismReference(const Kinematics&, const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector3d&,
                        Reference&);
bool referenceAllowed(const Reference&, const PlanningConfig&, std::string&);
// Cheap manual construction; automatic connections must use the validated helper.
bool validatedMechanismPath(const Kinematics&, const Reference&, const Eigen::Vector3d&, const PlanningConfig&, Path&,
                            std::string&);
bool mechanismPath(const Kinematics&, const Reference&, const Eigen::Vector3d&, const PlanningConfig&, Path&,
                   std::string&);
// Fixed-size manual jog geometry check; caller supplies already evaluated body FK.
bool localPitchSample(const Kinematics&, const Eigen::Vector3d&, const Eigen::Isometry3d& body,
                      const Eigen::Isometry3d& start, const Eigen::Isometry3d& previous, double delta,
                      double previous_pitch, const PlanningConfig&, double& pitch, std::string&);
bool inverseOrientation(const Kinematics&, const Eigen::Matrix3d&, const Eigen::Vector3d& seed, const PlanningConfig&,
                        Eigen::Vector3d&, std::string&);
Plan prepare(const Kinematics&, const Eigen::Vector3d& initial, const Eigen::Matrix3d& neutral, const Request&,
             const PlanningConfig&);

enum class Stage
{
  FREE_FLIGHT,
  CONTACT_LOCKED,
  PLANNING,
  PRE_ALIGN,
  READY,
  EXECUTING,
  PAUSED,
  COMPLETED_HOLD,
  FAULT_HOLD
};
const char* stageName(Stage);
struct Feedback
{
  double now = 0, joint_stamp = 0, body_stamp = 0;
  bool hover = false, valid = false;
  std::string invalid_reason;
  Eigen::Isometry3d body = Eigen::Isometry3d::Identity();
  Eigen::Vector3d eta = Eigen::Vector3d::Zero(), joint_rate = Eigen::Vector3d::Zero(),
                  body_omega = Eigen::Vector3d::Zero();
};
struct ExecutionConfig
{
  double synchronization_tolerance = .05;
  double timeout = .15, future_tolerance = .02, pivot_tolerance = .01, hinge_tolerance = .03;
  double tracking_tolerance = .04, orientation_tolerance = .04, settle_rate = .01, settle_time = .5,
         settle_timeout = 10;
  bool settle_use_window = false;
  double settle_body_rate = .05, settle_joint_rate = .03, settle_mean_rate = .02;
  bool require_settled_on_enable = false;
  double max_execution_step = .05;  // 40 Hz nominal, bounded jitter allowance
  double soft_tracking_fraction = .6, resume_fraction = .5;
  double preparation_tolerance = .03, preparation_quiet_time = .3;
  double reference_lead = .018, progress_filter_time = .15, goal_tolerance = .0087;
  double progress_timeout = 30;
  bool governorValid(const PlanningConfig&) const;
  bool feedbackConfigValid() const;
  bool settlingValid() const;
  bool valid() const;
};
enum class FeedbackStatus
{
  READY,
  WAITING,
  FAULT
};
FeedbackStatus feedbackStatus(const Feedback&, const ExecutionConfig&, const PlanningConfig&);
// Shared onboard feedback validation; manual navigation does not instantiate an executor.
std::string feedbackRejection(const Feedback&, const ExecutionConfig&, const PlanningConfig&);
class Executor
{
public:
  Stage stage = Stage::FREE_FLIGHT;
  std::uint64_t session = 0;
  std::string reason = "free flight";
  std::string first_fault;
  ExecutionConfig config;
  PlanningConfig planning;
  Kinematics model;
  Reference target;
  Estimate measured;
  Plan plan;
  double progress = 0, nominal_progress = 0;
  // Negative scalar relief, radians increasing into the cut. No off-path IK.
  double relief = 0;
  bool interaction_hold = false, resisting_load = false;
  bool relief_enabled = false, relief_limited = false;
  double relief_angle_limit = .0383972, relief_rate_limit = .15, relief_acceleration_limit = .1;
  double applied_relief = 0, relief_available = 0;
  double phaseSpeed() const
  {
    return speed_;
  }
  bool stopping() const
  {
    return stopping_;
  }
  TaskProgress task;
  double planned_pitch = 0, goal_error = 0, goal_orientation_error = 0;
  bool path_end_commanded = false, goal_reached = false, progress_lag = false;
  bool reference_updated = false;  // this tick accepted a coherent scalar/reference proposal
  Reference applied;
  bool applied_valid = false;
  SettlingStatistics settling;
  double settling_elapsed = 0;
  bool owns() const
  {
    return stage != Stage::FREE_FLIGHT;
  }
  bool feedbackValid(const Feedback&) const;
  std::string feedbackRejection(const Feedback&) const;
  bool capture(const Feedback&, const Kinematics&);
  // Attach an explicit planner to an already captured base lock; do not recapture contact.
  bool attach(const Kinematics&, const Reference&, std::uint64_t, double now);
  bool accept(Plan);
  bool submissionMatches(const Plan&, const Feedback&, const Reference&) const;
  bool command(const std::string&, std::uint64_t);
  void tick(const Feedback&);
  void waitForFeedback(double now, const std::string&);
  void fault(const std::string&);
  void reset();
  void hold(const std::string&);

private:
  void resetSettling();
  SettlingWindow settling_window_;
  double last_time_ = 0, stable_since_ = 0, ready_since_ = 0, speed_ = 1;
  double lag_since_ = 0, settling_stamp_ = 0, best_forward_ = 0;
  bool governor_hold_ = false, feedback_waiting_ = false;
  bool stopping_ = false, cancel_ = false, paused_align_ = false, motion_clock_pending_ = false;
};
}  // namespace roll_yaw

// Scalar admittance

namespace roll_yaw
{
struct AdmittanceConfig
{
  double inertia = .40, damping = 5, stiffness = 4.5, torque_limit = .18, angle_limit = .0383972;
  double rate_limit = .15, acceleration_limit = .1, reference_torque = 0;
  double filter_time = .1, on_threshold = .04, off_threshold = .02, on_dwell = .1, off_dwell = .2;
  int baseline_samples = 80;
  bool valid() const;
};
class PitchAdmittance
{
public:
  AdmittanceConfig config;
  bool enabled = false, contact = false, baseline_ready = false;
  double bias = 0, filtered = 0, offset = 0, rate = 0, acceleration = 0;
  int samples = 0;
  void reset();
  void observe(double torque, double stamp, bool no_cut);
  bool step(double dt, bool input_fresh, bool relief_only = false, double available_relief = -1);

private:
  double sum_ = 0, stamp_ = 0, on_ = 0, off_ = 0;
};
// Local constrained reference synthesis; never edits a nominal Path or goal.
bool compliantReference(const Kinematics&, const Reference& nominal, const Reference& previous, double offset,
                        double rate, double acceleration, double dt, const PlanningConfig&, Reference&, std::string&);
bool advanceCompliance(const Kinematics&, const Reference& nominal, const Reference& previous, double dt, bool fresh,
                       const PlanningConfig&, PitchAdmittance&, Reference&, std::string&);
}  // namespace roll_yaw

// Background worker support

namespace roll_yaw
{
inline double steadySeconds()
{ return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

// One persistent thread, at most one job/result. Callback methods never wait.
// Cancellation is cooperative; destruction joins ONLY during plugin shutdown.
template<class Result> class Worker
{
public:
  using Job=std::function<Result(const std::function<bool()>&)>;
  Worker():thread_([this]{run();}) {}
  ~Worker()
  {
    // Protect the wait predicate to prevent a lost shutdown notification.
    { std::lock_guard<std::mutex> l(mutex_); quit_=true; cancel(); }
    wake_.notify_one(); thread_.join();
  }
  Worker(const Worker&)=delete;
  Worker& operator=(const Worker&)=delete;
  void cancel() { ++generation_; }
  bool submit(Job job)
  {
    std::unique_lock<std::mutex> l(mutex_,std::try_to_lock);
    if(!l||busy_||job_||result_) return false;
    job_=std::move(job); job_generation_=generation_.load(); wake_.notify_one(); return true;
  }
  std::unique_ptr<Result> take()
  {
    std::unique_lock<std::mutex> l(mutex_,std::try_to_lock);
    return l?std::move(result_):nullptr;
  }
private:
  void run()
  {
    while(!quit_) {
      Job job; unsigned long generation;
      {
        std::unique_lock<std::mutex> l(mutex_);
        wake_.wait(l,[this]{return quit_||bool(job_);});
        if(quit_) return;
        job=std::move(job_); generation=job_generation_; busy_=true;
      }
      auto cancelled=[this,generation]{return quit_||generation_.load()!=generation;};
      std::unique_ptr<Result> result;
      try { result.reset(new Result(job(cancelled))); }
      catch(const std::exception& e) { result.reset(new Result()); result->reason=std::string("worker exception: ")+e.what(); }
      catch(...) { result.reset(new Result()); result->reason="unknown worker exception"; }
      { std::lock_guard<std::mutex> l(mutex_); busy_=false; if(!cancelled()) result_=std::move(result); }
    }
  }
  std::atomic<bool> quit_{false};
  std::atomic<unsigned long> generation_{0};
  unsigned long job_generation_=0;
  std::mutex mutex_; std::condition_variable wake_; Job job_;
  std::unique_ptr<Result> result_; bool busy_=false;
  std::thread thread_;
};
}
