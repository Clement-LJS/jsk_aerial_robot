#include <gimbalrotor/gimbalrotor_perching.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <cmath>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <algorithm>
#include <limits>
#include <sstream>

// Geometry and kinematics

namespace perching_geometry
{
bool Lock::targetOrientation(const tf::Quaternion& desired, double limit, double sign, Pose& pose) const
{
  const tf::Quaternion relative = desired * orientation.inverse();
  const double angle = 2 * std::atan2(tf::Vector3(relative.x(), relative.y(), relative.z()).dot(axis), relative.w());
  return target(std::atan2(std::sin(angle), std::cos(angle)) / sign, limit, sign, pose);
}

void accumulateLinkInertia(KDL::RigidBodyInertia& total, const KDL::Frame& frame, const std::string& link,
                           const KDL::RigidBodyInertia& inertia, const std::map<std::string, KDL::Segment>& extra)
{
  total = total + frame * inertia;
  for(const auto& module : extra)
  {
    if(module.second.getName() == link)
      total = total + frame * (module.second.getFrameToTip() * module.second.getInertia());
  }
}
}  // namespace perching_geometry

namespace perching_geometry
{
Eigen::Vector3d logSO3(const Eigen::Matrix3d& r)
{
  Eigen::AngleAxisd a(r);
  return a.angle() * a.axis();
}
Eigen::Matrix3d expSO3(const Eigen::Vector3d& v)
{
  const double a = v.norm();
  return a < 1e-14 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(a, v / a).toRotationMatrix();
}
bool rotationValid(const Eigen::Matrix3d& r)
{
  return r.allFinite() && (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-7 &&
         std::abs(r.determinant() - 1) < 1e-7;
}
bool poseValid(const Eigen::Isometry3d& p)
{
  return p.matrix().allFinite() && rotationValid(p.linear());
}
Eigen::Isometry3d fromKdl(const KDL::Frame& f)
{
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  for(int i = 0; i < 3; ++i)
  {
    p.translation()[i] = f.p[i];
    for(int j = 0; j < 3; ++j)
      p.linear()(i, j) = f.M(i, j);
  }
  return p;
}
KDL::Frame toKdl(const Eigen::Isometry3d& p)
{
  Eigen::Quaterniond q(p.linear());
  return {KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w()),
          KDL::Vector(p.translation().x(), p.translation().y(), p.translation().z())};
}
}
namespace perching_servo
{
bool Config::valid() const
{
  const double positive[] = {
      q_rate,      q_zero_tolerance, q_hold_error,           joint_settle_rate,     feedback_timeout, rezero_kp,
      rezero_rate, rezero_range,     rezero_pitch_tolerance, rezero_rate_tolerance, rezero_timeout,   passive_rate};
  for(double x : positive)
    if(!std::isfinite(x) || x <= 0)
      return false;
  return std::isfinite(q_min) && std::isfinite(q_max) && q_min < q_max && std::isfinite(settle_duration) &&
         settle_duration >= 0 && std::isfinite(future_tolerance) && future_tolerance >= 0 &&
         std::isfinite(rezero_pitch) && std::isfinite(rezero_sign) && std::abs(rezero_sign) == 1 &&
         std::isfinite(body_pitch_min) && std::isfinite(body_pitch_max) && body_pitch_min >= -M_PI / 2 &&
         body_pitch_max <= M_PI / 2 && body_pitch_min < body_pitch_max && rezero_pitch >= body_pitch_min &&
         rezero_pitch <= body_pitch_max && std::isfinite(passive_min) && std::isfinite(passive_max) &&
         passive_min < 0 && passive_max > 0;
}

const char* Core::feedbackError(const Sample& s) const
{
  if(!config.valid())
    return "invalid positioning parameters or unresolved joint limits";
  if(!s.model_valid)
    return "mechanism model or joint feedback unavailable";
  if(!s.hover)
    return "navigation state is not HOVER_STATE";
  if(!fresh(s.now, s.joint_stamp))
    return "joint state is missing or stale";
  if(!std::isfinite(s.q) || s.q < config.q_min - 1e-6 || s.q > config.q_max + 1e-6)
    return "measured pitch joint outside configured limits";
  if(!std::isfinite(s.qdot))
    return "waiting for measured/derived joint velocity";
  if(!finite(s.body) || !std::isfinite(s.angular_speed) || !std::isfinite(s.pitch_rate))
    return "invalid estimator body pose/rate";
  return nullptr;
}

bool Core::steady(const Sample& s) const
{
  return feedback(s) && stopped(s) && std::abs(s.q - q_target) <= config.q_hold_error && stable_since > 0 &&
         s.now - stable_since >= config.settle_duration && std::abs(q_goal - q_target) < 1e-9 &&
         std::abs(alpha_goal - alpha) < 1e-9;
}

bool Core::rezeroReady(const Sample& s) const
{
  return stage == Stage::PITCH_REZERO_READY_HOLD && feedback(s) && std::abs(s.q - q_target) <= config.q_hold_error &&
         std::abs(wrap(config.rezero_pitch - pitch(s.body))) <= config.rezero_pitch_tolerance &&
         std::abs(s.pitch_rate) <= config.rezero_rate_tolerance;
}

void Core::fault(const char* reason)
{
  if(ownsPose())
    stage = Stage::FAULT_HOLD;
  status = reason;
  q_goal = q_target;
  alpha_goal = alpha;
  stable_since = 0;
}

void Core::reset()
{
  stage = Stage::FREE_FLIGHT;
  zero_hold_armed = false;
  rezero_completed = false;
  q_target = q_goal = alpha = alpha_goal = alpha_min = alpha_max = 0;
  stable_since = rezero_stable_since = 0;
  ++generation;
  status = "free flight; perching disabled";
}

bool Core::armZero(const Sample& s)
{
  if(stage != Stage::FREE_FLIGHT || !feedback(s) || !stopped(s) || config.q_min > 0 || config.q_max < 0 ||
     std::abs(s.q) > config.q_zero_tolerance)
    return reject("optional zero hold requires hover and a settled joint already near zero");
  zero_hold_armed = true;
  q_target = q_goal = 0;
  status = "holding pitch at zero before contact";
  return true;
}

bool Core::enable(const Sample& s)
{
  if(!feedback(s))
    return reject("enable requires hover, valid model and fresh joint/estimator feedback");
  if(!bodyPitchAllowed(s.body))
    return reject("enable rejected: BASELINK world pitch outside body limits");
  KDL::Frame bc;
  KDL::Vector g;
  if(!model || !model(s.q, bc, g) || !finite(bc) || !finite(g))
    return reject("enable FK invalid");
  contact = s.body * bc;
  target = reference = s.body;
  cog = g;
  pivot = contact.p;
  axis = contact.M.UnitY();
  q_target = q_goal = s.q;
  alpha = alpha_goal = 0;
  rezero_completed = false;
  zero_hold_armed = false;
  stage = Stage::CONTACT_LOCKED_PRE_REZERO;
  ++generation;
  stable_since = 0;
  status = "contact locked; holding measured pitch joint";
  return true;
}

bool Core::rezero(const Sample& s)
{
  if(stage != Stage::CONTACT_LOCKED_PRE_REZERO)
    return reject("rezero requires an enabled provisional perching lock");
  if(const auto error = feedbackError(s))
    return reject(error);
  if(std::abs(s.q - q_target) > config.q_hold_error)
    return reject("pitch joint moved outside hold tolerance before rezero");
  // Start from valid feedback; convergence is checked during motion.
  // Convergence is checked while rezero is active, using world body pitch rate.
  reference = target;
  alpha = alpha_goal = 0;
  start = s.now;
  stable_since = 0;
  rezero_stable_since = 0;
  stage = Stage::PITCH_REZERO_ACTIVE;
  status = "rezero active; pitch servo frozen";
  return true;
}

bool Core::rebase(const Sample& s, Stage required)
{
  const bool ready = required == Stage::PITCH_REZERO_READY_HOLD ? rezeroReady(s) : steady(s);
  if(stage != required || !ready)
    return reject("rebase requires fresh settled feedback for the current stage");
  if(!bodyPitchAllowed(s.body))
    return reject("rebase rejected: BASELINK world pitch outside body limits");
  KDL::Frame bc;
  KDL::Vector g;
  if(!model || !model(s.q, bc, g) || !finite(bc) || !finite(g))
    return reject("rebase FK invalid");
  // Use measured contact orientation but keep the original world pivot.
  contact = s.body * bc;
  contact.p = pivot;
  target = contact * bc.Inverse();
  cog = g;
  q_target = q_goal = s.q;
  if(required == Stage::PITCH_REZERO_READY_HOLD)
    alpha = 0;
  alpha_goal = alpha;
  // Preserve the post-rezero passive coordinate through servo changes and
  // freeze/unfreeze. Reconstruct the zero pose for the newly captured servo.
  reference = arc(target, pivot, axis, -alpha);
  stage = Stage::SERVO_ADJUSTMENT;
  ++generation;
  stable_since = 0;
  status = "servo adjustment; original pivot preserved";
  return true;
}

bool Core::relock(const Sample& s)
{
  if(stage != Stage::PITCH_REZERO_READY_HOLD ||
     std::abs(wrap(config.rezero_pitch - pitch(s.body))) > config.rezero_pitch_tolerance)
    return reject("relock requires rezero READY at the requested body pitch");
  if(!rebase(s, Stage::PITCH_REZERO_READY_HOLD))
    return false;
  rezero_completed = true;
  return true;
}

bool Core::servo(double q, const Sample& s)
{
  if(stage != Stage::SERVO_ADJUSTMENT || !feedback(s) || !std::isfinite(q))
    return reject("servo command requires SERVO_ADJUSTMENT and valid feedback");
  q_goal = clamp(q, config.q_min, config.q_max);
  stable_since = 0;
  status = "servo adjustment commanded";
  return true;
}

bool Core::freeze(const Sample& s)
{
  if(!rebase(s, Stage::SERVO_ADJUSTMENT))
    return false;
  alpha_min = config.passive_min;
  alpha_max = config.passive_max;
  stage = Stage::PASSIVE_CUTTING;
  status = "servo frozen at measurement; passive navigation enabled";
  return true;
}

bool Core::cut(double a, const Sample& s)
{
  if(stage != Stage::PASSIVE_CUTTING || !feedback(s) || !std::isfinite(a))
    return reject("passive command requires frozen servo and valid feedback");
  alpha_goal = clamp(a, alpha_min, alpha_max);
  stable_since = 0;
  status = "passive arc commanded; servo unchanged";
  return true;
}

bool Core::prepare(const Sample& s)
{
  if(stage != Stage::SERVO_ADJUSTMENT || !steady(s))
    return reject("release preparation requires settled servo adjustment");
  stage = Stage::UNPERCH_PREP;
  status = "holding current pose for manual release";
  return true;
}

bool Core::released(const Sample& s)
{
  if(stage != Stage::UNPERCH_PREP || !feedback(s))
    return reject("release confirmation requires release preparation");
  stage = Stage::DISENGAGED;
  ++generation;
  status = "release acknowledged; disable to resume free flight";
  return true;
}

bool Core::recover(const Sample& s)
{
  if(stage != Stage::FAULT_HOLD || !steady(s))
    return reject("recovery requires fresh settled feedback at the held joint angle");
  if(rezero_completed)
    return rebase(s, Stage::FAULT_HOLD);
  reference = target;
  alpha = alpha_goal = 0;
  stage = Stage::CONTACT_LOCKED_PRE_REZERO;
  stable_since = 0;
  status = "fault acknowledged; repeat rezero and relock";
  return true;
}

double Core::measuredAlpha(const Sample& s) const
{
  KDL::Vector v;
  const double a = (s.body.M * reference.M.Inverse()).GetRotAngle(v);
  return a * KDL::dot(v, axis);
}

void Core::tick(const Sample& s, double dt)
{
  if(!ownsPose())
    return;
  if(const auto error = feedbackError(s))
  {
    fault(error);
    return;
  }
  if(!std::isfinite(dt) || dt <= 0 || dt > config.feedback_timeout)
  {
    fault("invalid control time step");
    return;
  }
  if(stage != Stage::SERVO_ADJUSTMENT && std::abs(s.q - q_target) > config.q_hold_error)
  {
    fault("frozen pitch joint moved outside hold tolerance");
    return;
  }
  const bool settling = stopped(s) && std::abs(s.q - q_target) <= config.q_hold_error;
  if(!settling)
    stable_since = 0;
  else if(stable_since == 0)
    stable_since = s.now;
  if(stage == Stage::FAULT_HOLD || stage == Stage::DISENGAGED)
    return;
  double next_q = q_target, next_a = alpha;
  if(stage == Stage::PITCH_REZERO_ACTIVE)
  {
    if(s.now - start > config.rezero_timeout)
    {
      fault("rezero timeout; holding last target");
      return;
    }
    const double error = wrap(config.rezero_pitch - pitch(s.body));
    const bool converged = std::abs(error) <= config.rezero_pitch_tolerance &&
                           std::abs(s.pitch_rate) <= config.rezero_rate_tolerance && dt <= .1;
    if(!converged)
      rezero_stable_since = 0;
    else if(rezero_stable_since == 0)
      rezero_stable_since = s.now;
    if(converged && s.now - rezero_stable_since >= config.settle_duration)
    {
      // Rezero travel is discarded. This pose is the new passive zero.
      reference = target;
      alpha = alpha_goal = 0;
      alpha_min = config.passive_min;
      alpha_max = config.passive_max;
      stage = Stage::PITCH_REZERO_READY_HOLD;
      status = "rezero READY; press relock";
      return;
    }
    if(std::abs(error) <= config.rezero_pitch_tolerance)
    {
      alpha_goal = alpha;
    }
    else
    {
      stable_since = 0;
      const double rate = config.rezero_sign * clamp(config.rezero_kp * error, -config.rezero_rate, config.rezero_rate);
      next_a = alpha_goal = clamp(alpha + rate * dt, -config.rezero_range, config.rezero_range);
    }
  }
  if(stage == Stage::SERVO_ADJUSTMENT)
    next_q = q_target + clamp(q_goal - q_target, -config.q_rate * dt, config.q_rate * dt);
  if(stage == Stage::PASSIVE_CUTTING)
    next_a = alpha + clamp(alpha_goal - alpha, -config.passive_rate * dt, config.passive_rate * dt);
  auto evaluate = [&](double q, double a, KDL::Frame& b, KDL::Vector& g)
  {
    b = target;
    g = cog;
    if(stage == Stage::SERVO_ADJUSTMENT)
    {
      KDL::Frame bc;
      if(!model || !model(q, bc, g))
        return false;
      b = contact * bc.Inverse();
    }
    else if(stage == Stage::PASSIVE_CUTTING || stage == Stage::PITCH_REZERO_ACTIVE)
      b = arc(reference, pivot, axis, a);
    return finite(b) && finite(g);
  };
  KDL::Frame b;
  KDL::Vector g;
  if(!evaluate(next_q, next_a, b, g))
  {
    fault("target FK invalid or nonfinite");
    return;
  }
  if(!bodyPitchAllowed(b))
  {
    // Stop along the existing FK/arc path, preserving the pivot. Clamping
    // Euler pitch alone would break the contact geometry.
    double lo = 0, hi = 1;
    const double dq = next_q - q_target, da = next_a - alpha;
    for(int i = 0; i < 32; ++i)
    {
      const double t = (lo + hi) / 2;
      if(!evaluate(q_target + t * dq, alpha + t * da, b, g))
      {
        fault("target FK invalid or nonfinite");
        return;
      }
      if(bodyPitchAllowed(b))
        lo = t;
      else
        hi = t;
    }
    next_q = q_target + lo * dq;
    next_a = alpha + lo * da;
    if(!evaluate(next_q, next_a, b, g))
    {
      fault("target FK invalid or nonfinite");
      return;
    }
    q_goal = next_q;
    alpha_goal = next_a;
    status = "BASELINK world-pitch limit reached; reverse command to move inward";
  }
  q_target = next_q;
  alpha = next_a;
  target = b;
  cog = g;
}
}  // namespace perching_servo


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
