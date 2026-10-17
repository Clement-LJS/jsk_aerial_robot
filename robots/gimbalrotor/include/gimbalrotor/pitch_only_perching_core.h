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
enum class Stage { FREE_FLIGHT, CONTACT_LOCKED_PRE_REZERO, PITCH_REZERO_ACTIVE,
  PITCH_REZERO_READY_HOLD, SERVO_ADJUSTMENT, PASSIVE_CUTTING, UNPERCH_PREP,
  DISENGAGED, FAULT_HOLD };
inline const char* name(Stage s)
{
  static const char* names[] = {"FREE_FLIGHT", "CONTACT_LOCKED_PRE_REZERO", "PITCH_REZERO_ACTIVE",
    "PITCH_REZERO_READY_HOLD", "SERVO_ADJUSTMENT", "PASSIVE_CUTTING", "UNPERCH_PREP",
    "DISENGAGED", "FAULT_HOLD"};
  return names[static_cast<int>(s)];
}
inline bool finite(const KDL::Vector& p)
{ return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()); }
inline bool finite(const KDL::Frame& f)
{
  if(!finite(f.p)) return false;
  for(int i=0;i<3;++i) for(int j=0;j<3;++j) if(!std::isfinite(f.M(i,j))) return false;
  return std::abs(f.M.UnitX().Norm()-1)<1e-6 && std::abs(f.M.UnitY().Norm()-1)<1e-6 &&
    std::abs(f.M.UnitZ().Norm()-1)<1e-6 && std::abs(KDL::dot(f.M.UnitX(),f.M.UnitY()))<1e-6 &&
    (f.M.UnitX()*f.M.UnitY()-f.M.UnitZ()).Norm()<1e-6;
}
inline double pitch(const KDL::Frame& f) { double r,p,y; f.M.GetRPY(r,p,y); return p; }
// ZYX Euler pitch derivative from BASELINK angular velocity. Using the
// estimator's rate avoids amplifying pose measurement noise by differentiation.
inline double pitchRate(const KDL::Frame& body,const KDL::Vector& omega)
{
  double roll,p,y; body.M.GetRPY(roll,p,y);
  return std::cos(roll)*omega.y()-std::sin(roll)*omega.z();
}
inline double clamp(double x,double lo,double hi) { return std::max(lo,std::min(x,hi)); }
inline double wrap(double x) { return std::atan2(std::sin(x),std::cos(x)); }
inline KDL::Frame arc(const KDL::Frame& f,const KDL::Vector& pivot,const KDL::Vector& axis,double alpha)
{
  const auto r=KDL::Rotation::Rot(axis,alpha);
  return {r*f.M,pivot+r*(f.p-pivot)};
}

struct Config
{
  // Joint limits/rate are resolved from the URDF by the adapter. YAML may narrow
  // the range; a continuous joint requires explicit finite limits.
  double q_min=unknown, q_max=unknown, q_rate=.1;
  double q_zero_tolerance=.02, q_hold_error=.02, joint_settle_rate=.05;
  double settle_duration=.30;
  double feedback_timeout=.15, future_tolerance=.02;
  double rezero_pitch=0, rezero_sign=-1, rezero_kp=1;
  double rezero_rate=.1745329, rezero_range=.5235988;
  double rezero_pitch_tolerance=.0349066, rezero_rate_tolerance=.05, rezero_timeout=5;
  double passive_min=-.5235988, passive_max=.5235988, passive_rate=.1745329;
  double body_pitch_min=-.5235988, body_pitch_max=.5235988;
  KDL::Vector pivot_offset{.21049,-.014,.0205}, hinge_axis{0,1,0};

  bool valid() const
  {
    const double positive[]={q_rate,q_zero_tolerance,q_hold_error,joint_settle_rate,
      feedback_timeout,rezero_kp,rezero_rate,rezero_range,
      rezero_pitch_tolerance,rezero_rate_tolerance,rezero_timeout,passive_rate};
    for(double x:positive) if(!std::isfinite(x)||x<=0) return false;
    return std::isfinite(q_min)&&std::isfinite(q_max)&&q_min<q_max &&
      std::isfinite(settle_duration)&&settle_duration>=0 &&
      std::isfinite(future_tolerance)&&future_tolerance>=0 &&
      std::isfinite(rezero_pitch)&&std::isfinite(rezero_sign)&&std::abs(rezero_sign)==1 &&
      std::isfinite(body_pitch_min)&&std::isfinite(body_pitch_max)&&
      body_pitch_min>=-M_PI/2&&body_pitch_max<=M_PI/2&&body_pitch_min<body_pitch_max &&
      rezero_pitch>=body_pitch_min&&rezero_pitch<=body_pitch_max &&
      std::isfinite(passive_min)&&std::isfinite(passive_max)&&passive_min<0&&passive_max>0 &&
      finite(pivot_offset)&&finite(hinge_axis)&&std::abs(hinge_axis.Norm()-1)<1e-6;
  }
};
struct Sample
{
  double now=0, joint_stamp=0;
  double q=unknown, qdot=unknown, angular_speed=unknown, pitch_rate=unknown;
  bool hover=false, model_valid=false;
  KDL::Frame body;
};
using Model = std::function<bool(double,KDL::Frame&,KDL::Vector&)>;
class Core
{
public:
  Config config;
  Model model;
  Stage stage=Stage::FREE_FLIGHT;
  std::string status="free flight; enable perching in hover to capture the measured pitch joint";
  std::uint64_t generation=0;
  bool zero_hold_armed=false, rezero_completed=false;
  double q_target=0, q_goal=0, alpha=0, alpha_goal=0, alpha_min=0, alpha_max=0;
  KDL::Frame target, contact, reference;
  KDL::Vector pivot, axis, cog;

  bool attached() const { return stage!=Stage::FREE_FLIGHT && stage!=Stage::DISENGAGED; }
  bool ownsPose() const { return stage!=Stage::FREE_FLIGHT; }
  bool bodyPitchAllowed(const KDL::Frame& body) const
  { const double p=pitch(body); return std::isfinite(p)&&p>=config.body_pitch_min&&p<=config.body_pitch_max; }
  bool fresh(double now,double stamp) const
  { return std::isfinite(now)&&std::isfinite(stamp)&&stamp>0 &&
      now-stamp>=-config.future_tolerance && now-stamp<=config.feedback_timeout; }
  const char* feedbackError(const Sample& s) const
  {
    if(!config.valid()) return "invalid pitch-only parameters or unresolved joint limits";
    if(!s.model_valid) return "mechanism model or joint feedback unavailable";
    if(!s.hover) return "navigation state is not HOVER_STATE";
    if(!fresh(s.now,s.joint_stamp)) return "joint state is missing or stale";
    if(!std::isfinite(s.q)||s.q<config.q_min-1e-6||s.q>config.q_max+1e-6)
      return "measured pitch joint outside configured limits";
    if(!std::isfinite(s.qdot)) return "waiting for measured/derived joint velocity";
    if(!finite(s.body)||!std::isfinite(s.angular_speed)||!std::isfinite(s.pitch_rate)) return "invalid estimator body pose/rate";
    return nullptr;
  }
  bool feedback(const Sample& s) const { return feedbackError(s)==nullptr; }
  // Match multilink joint settling: joint position/velocity, not body rotation.
  bool stopped(const Sample& s) const
  { return std::abs(s.qdot)<=config.joint_settle_rate; }
  bool steady(const Sample& s) const
  {
    return feedback(s)&&stopped(s)&&std::abs(s.q-q_target)<=config.q_hold_error &&
      stable_since>0 && s.now-stable_since>=config.settle_duration &&
      std::abs(q_goal-q_target)<1e-9 && std::abs(alpha_goal-alpha)<1e-9;
  }
  bool rezeroReady(const Sample& s) const
  {
    return stage==Stage::PITCH_REZERO_READY_HOLD && feedback(s) &&
      std::abs(s.q-q_target)<=config.q_hold_error &&
      std::abs(wrap(config.rezero_pitch-pitch(s.body)))<=config.rezero_pitch_tolerance &&
      std::abs(s.pitch_rate)<=config.rezero_rate_tolerance;
  }
  bool reject(const char* reason) { status=reason; return false; }
  void fault(const char* reason)
  {
    if(ownsPose()) stage=Stage::FAULT_HOLD;
    status=reason; q_goal=q_target; alpha_goal=alpha; stable_since=0;
  }
  void reset()
  {
    stage=Stage::FREE_FLIGHT; zero_hold_armed=false; rezero_completed=false;
    q_target=q_goal=alpha=alpha_goal=alpha_min=alpha_max=0;
    stable_since=rezero_stable_since=0; ++generation; status="free flight; perching disabled";
  }
  bool armZero(const Sample& s)
  {
    if(stage!=Stage::FREE_FLIGHT||!feedback(s)||!stopped(s)||
       config.q_min>0||config.q_max<0||std::abs(s.q)>config.q_zero_tolerance)
      return reject("optional zero hold requires hover and a settled joint already near zero");
    zero_hold_armed=true; q_target=q_goal=0; status="holding pitch at zero before contact"; return true;
  }
  bool enable(const Sample& s)
  {
    if(!feedback(s)) return reject("enable requires hover, valid model and fresh joint/estimator feedback");
    if(!bodyPitchAllowed(s.body)) return reject("enable rejected: BASELINK world pitch outside body limits");
    KDL::Frame bc; KDL::Vector g;
    if(!model||!model(s.q,bc,g)||!finite(bc)||!finite(g)) return reject("enable FK invalid");
    contact=s.body*bc; target=reference=s.body; cog=g;
    pivot=contact*config.pivot_offset; axis=contact.M*config.hinge_axis;
    q_target=q_goal=s.q; alpha=alpha_goal=0; rezero_completed=false;
    zero_hold_armed=false; stage=Stage::CONTACT_LOCKED_PRE_REZERO;
    ++generation; stable_since=0; status="contact locked; holding measured pitch joint"; return true;
  }
  bool rezero(const Sample& s)
  {
    if(stage!=Stage::CONTACT_LOCKED_PRE_REZERO)
      return reject("rezero requires an enabled provisional perching lock");
    if(const auto error=feedbackError(s)) return reject(error);
    if(std::abs(s.q-q_target)>config.q_hold_error)
      return reject("pitch joint moved outside hold tolerance before rezero");
    // Like multilink, start from valid feedback without a pre-settling gate.
    // Convergence is checked while rezero is active, using world body pitch rate.
    reference=target; alpha=alpha_goal=0; start=s.now; stable_since=0;
    rezero_stable_since=0;
    stage=Stage::PITCH_REZERO_ACTIVE; status="rezero active; pitch servo frozen"; return true;
  }
  bool rebase(const Sample& s,Stage required)
  {
    const bool ready=required==Stage::PITCH_REZERO_READY_HOLD?rezeroReady(s):steady(s);
    if(stage!=required||!ready) return reject("rebase requires fresh settled feedback for the current stage");
    if(!bodyPitchAllowed(s.body)) return reject("rebase rejected: BASELINK world pitch outside body limits");
    KDL::Frame bc; KDL::Vector g;
    if(!model||!model(s.q,bc,g)||!finite(bc)||!finite(g)) return reject("rebase FK invalid");
    // Use measured contact orientation but keep the original world pivot.
    contact=s.body*bc; contact.p=pivot-contact.M*config.pivot_offset;
    target=contact*bc.Inverse(); cog=g; q_target=q_goal=s.q;
    if(required==Stage::PITCH_REZERO_READY_HOLD) alpha=0;
    alpha_goal=alpha;
    // Preserve the post-rezero passive coordinate through servo changes and
    // freeze/unfreeze. Reconstruct the zero pose for the newly captured servo.
    reference=arc(target,pivot,axis,-alpha);
    stage=Stage::SERVO_ADJUSTMENT; ++generation; stable_since=0;
    status="servo adjustment; original pivot preserved"; return true;
  }
  bool relock(const Sample& s)
  {
    if(stage!=Stage::PITCH_REZERO_READY_HOLD ||
       std::abs(wrap(config.rezero_pitch-pitch(s.body)))>config.rezero_pitch_tolerance)
      return reject("relock requires rezero READY at the requested body pitch");
    if(!rebase(s,Stage::PITCH_REZERO_READY_HOLD)) return false;
    rezero_completed=true; return true;
  }
  bool servo(double q,const Sample& s)
  {
    if(stage!=Stage::SERVO_ADJUSTMENT||!feedback(s)||!std::isfinite(q))
      return reject("servo command requires SERVO_ADJUSTMENT and valid feedback");
    q_goal=clamp(q,config.q_min,config.q_max); stable_since=0;
    status="servo adjustment commanded"; return true;
  }
  bool freeze(const Sample& s)
  {
    if(!rebase(s,Stage::SERVO_ADJUSTMENT)) return false;
    alpha_min=config.passive_min;
    alpha_max=config.passive_max;
    stage=Stage::PASSIVE_CUTTING;
    status="servo frozen at measurement; passive navigation enabled"; return true;
  }
  bool cut(double a,const Sample& s)
  {
    if(stage!=Stage::PASSIVE_CUTTING||!feedback(s)||!std::isfinite(a))
      return reject("passive command requires frozen servo and valid feedback");
    alpha_goal=clamp(a,alpha_min,alpha_max); stable_since=0;
    status="passive arc commanded; servo unchanged"; return true;
  }
  bool unfreeze(const Sample& s) { return rebase(s,Stage::PASSIVE_CUTTING); }
  bool prepare(const Sample& s)
  {
    if(stage!=Stage::SERVO_ADJUSTMENT||!steady(s)) return reject("release preparation requires settled servo adjustment");
    stage=Stage::UNPERCH_PREP; status="holding current pose for manual release"; return true;
  }
  bool released(const Sample& s)
  {
    if(stage!=Stage::UNPERCH_PREP||!feedback(s)) return reject("release confirmation requires release preparation");
    stage=Stage::DISENGAGED; ++generation; status="release acknowledged; disable to resume free flight"; return true;
  }
  bool recover(const Sample& s)
  {
    if(stage!=Stage::FAULT_HOLD||!steady(s)) return reject("recovery requires fresh settled feedback at the held joint angle");
    if(rezero_completed) return rebase(s,Stage::FAULT_HOLD);
    reference=target; alpha=alpha_goal=0;
    stage=Stage::CONTACT_LOCKED_PRE_REZERO; stable_since=0;
    status="fault acknowledged; repeat rezero and relock"; return true;
  }
  double measuredAlpha(const Sample& s) const
  {
    KDL::Vector v; const double a=(s.body.M*reference.M.Inverse()).GetRotAngle(v);
    return a*KDL::dot(v,axis);
  }
  void tick(const Sample& s,double dt)
  {
    if(!ownsPose()) return;
    if(const auto error=feedbackError(s)) { fault(error); return; }
    if(!std::isfinite(dt)||dt<=0||dt>config.feedback_timeout) { fault("invalid control time step"); return; }
    if(stage!=Stage::SERVO_ADJUSTMENT && std::abs(s.q-q_target)>config.q_hold_error)
    { fault("frozen pitch joint moved outside hold tolerance"); return; }
    const bool settling=stopped(s)&&std::abs(s.q-q_target)<=config.q_hold_error;
    if(!settling) stable_since=0; else if(stable_since==0) stable_since=s.now;
    if(stage==Stage::FAULT_HOLD||stage==Stage::DISENGAGED) return;
    double next_q=q_target, next_a=alpha;
    if(stage==Stage::PITCH_REZERO_ACTIVE) {
      if(s.now-start>config.rezero_timeout) { fault("rezero timeout; holding last target"); return; }
      const double error=wrap(config.rezero_pitch-pitch(s.body));
      const bool converged=std::abs(error)<=config.rezero_pitch_tolerance &&
        std::abs(s.pitch_rate)<=config.rezero_rate_tolerance && dt<=.1;
      if(!converged) rezero_stable_since=0;
      else if(rezero_stable_since==0) rezero_stable_since=s.now;
      if(converged&&s.now-rezero_stable_since>=config.settle_duration) {
        // Rezero travel is discarded. This pose is the new passive zero.
        reference=target; alpha=alpha_goal=0;
        alpha_min=config.passive_min; alpha_max=config.passive_max;
        stage=Stage::PITCH_REZERO_READY_HOLD;
        status="rezero READY; press relock"; return;
      }
      if(std::abs(error)<=config.rezero_pitch_tolerance) {
        alpha_goal=alpha;
      } else {
        stable_since=0;
        const double rate=config.rezero_sign*clamp(config.rezero_kp*error,-config.rezero_rate,config.rezero_rate);
        next_a=alpha_goal=clamp(alpha+rate*dt,-config.rezero_range,config.rezero_range);
      }
    }
    if(stage==Stage::SERVO_ADJUSTMENT)
      next_q=q_target+clamp(q_goal-q_target,-config.q_rate*dt,config.q_rate*dt);
    if(stage==Stage::PASSIVE_CUTTING)
      next_a=alpha+clamp(alpha_goal-alpha,-config.passive_rate*dt,config.passive_rate*dt);
    auto evaluate=[&](double q,double a,KDL::Frame& b,KDL::Vector& g) {
      b=target; g=cog;
      if(stage==Stage::SERVO_ADJUSTMENT) {
        KDL::Frame bc;
        if(!model||!model(q,bc,g)) return false;
        b=contact*bc.Inverse();
      } else if(stage==Stage::PASSIVE_CUTTING||stage==Stage::PITCH_REZERO_ACTIVE)
        b=arc(reference,pivot,axis,a);
      return finite(b)&&finite(g);
    };
    KDL::Frame b; KDL::Vector g;
    if(!evaluate(next_q,next_a,b,g)) { fault("target FK invalid or nonfinite"); return; }
    if(!bodyPitchAllowed(b)) {
      // Stop along the existing FK/arc path, preserving the pivot. Clamping
      // Euler pitch alone would break the contact geometry.
      double lo=0,hi=1;
      const double dq=next_q-q_target,da=next_a-alpha;
      for(int i=0;i<32;++i) {
        const double t=(lo+hi)/2;
        if(!evaluate(q_target+t*dq,alpha+t*da,b,g)) { fault("target FK invalid or nonfinite"); return; }
        if(bodyPitchAllowed(b)) lo=t; else hi=t;
      }
      next_q=q_target+lo*dq; next_a=alpha+lo*da;
      if(!evaluate(next_q,next_a,b,g)) { fault("target FK invalid or nonfinite"); return; }
      q_goal=next_q; alpha_goal=next_a;
      status="BASELINK world-pitch limit reached; reverse command to move inward";
    }
    q_target=next_q; alpha=next_a; target=b; cog=g;
  }
private:
  double stable_since=0,start=0;
  double rezero_stable_since=0;
};
} // namespace pitch_only
