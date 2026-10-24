#pragma once
#include <gimbalrotor/roll_yaw_orientation_qp.h>
#include <array>
#include <cstdint>
#include <vector>

namespace roll_yaw
{
struct Request
{
  double yaw=0, roll=0, pitch_start=0, pitch_end=0, rate=.03, acceleration=.02;
  bool valid() const;
};
struct PlanningConfig
{
  Limits limits;
  QpConfig qp;
  double dt=.02, max_tilt=.2, orientation_tolerance=.003, rate_tolerance=.01;
  double alignment_rate=.04, alignment_acceleration=.02, max_duration=180;
  double cog_speed_limit=.1, cog_acceleration_limit=.2;
  int scaling_attempts=5;
  bool valid() const;
};
struct Desired { Mat rotation=Mat::Identity(); Vec omega=Vec::Zero(), acceleration=Vec::Zero(); };
// Cosine velocity ramps, constant feed, cosine deceleration; C2 angle profile.
class Stroke
{
public:
  Stroke(double distance,double rate,double acceleration);
  double duration() const { return 2*ramp_+feed_; }
  void sample(double t,double& angle,double& rate,double& acceleration) const;
private:
  double distance_,sign_,speed_,ramp_,feed_;
};
Desired cuttingOrientation(const Mat& neutral,const Request&,double theta,double rate,double acceleration);
struct Knot { double t=0; Vec eta=Vec::Zero(), velocity=Vec::Zero(); };
struct Reference
{
  double t=0, orientation_error=0;
  Vec eta=Vec::Zero(), velocity=Vec::Zero(), acceleration=Vec::Zero();
  Pose body=Pose::Identity(), cog=Pose::Identity();
  Desired desired;
  Vec body_omega=Vec::Zero(), body_acceleration=Vec::Zero();
  Vec cog_velocity=Vec::Zero(), cog_acceleration=Vec::Zero();
  Vec cog_omega=Vec::Zero(), cog_angular_acceleration=Vec::Zero();
};
struct Path
{
  std::vector<Knot> knots;
  std::function<Desired(double)> desired;
  double duration() const { return knots.empty()?0:knots.back().t; }
};
struct Plan
{
  bool ok=false;
  std::uint64_t session=0;
  int scale_attempt=0;
  std::string reason;
  Kinematics model;
  Path align,cut;
};
bool samplePath(const Kinematics&,const Path&,double time,Reference&);
Plan prepare(const Kinematics&,const Vec& initial,const Mat& neutral,const Request&,const PlanningConfig&);

enum class Stage { FREE_FLIGHT, CONTACT_LOCKED, REZERO, PRE_ALIGN, READY, EXECUTING,
                   PAUSED, COMPLETED_HOLD, FAULT_HOLD };
const char* stageName(Stage);
struct Feedback
{
  double now=0, joint_stamp=0, body_stamp=0;
  bool hover=false, valid=false;
  Pose body=Pose::Identity(); Vec eta=Vec::Zero(), joint_rate=Vec::Zero(), body_omega=Vec::Zero();
};
struct ExecutionConfig
{
  double timeout=.15, future_tolerance=.02, pivot_tolerance=.01, hinge_tolerance=.03;
  double tracking_tolerance=.04, orientation_tolerance=.04, settle_rate=.01, settle_time=.5, settle_timeout=10;
  bool valid() const;
};
class Executor
{
public:
  Stage stage=Stage::FREE_FLIGHT;
  std::uint64_t session=0;
  std::string reason="free flight";
  ExecutionConfig config;
  PlanningConfig planning;
  Kinematics model;
  Reference target;
  Estimate measured;
  Plan plan;
  double progress=0;
  bool owns() const { return stage!=Stage::FREE_FLIGHT; }
  bool feedbackValid(const Feedback&) const;
  bool capture(const Feedback&,const Kinematics&);
  bool accept(Plan);
  bool command(const std::string&,std::uint64_t);
  void tick(const Feedback&);
  void fault(const std::string&);
  void reset();
private:
  double last_time_=0, stable_since_=0, ready_since_=0, speed_=1;
  bool stopping_=false, cancel_=false, paused_align_=false;
};
}
