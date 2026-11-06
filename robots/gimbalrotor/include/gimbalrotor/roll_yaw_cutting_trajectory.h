#pragma once
#include <gimbalrotor/roll_yaw_orientation_qp.h>
#include <gimbalrotor/roll_yaw_settling.h>
#include <gimbalrotor/roll_yaw_task.h>
#include <array>
#include <cstdint>
#include <vector>

namespace roll_yaw
{
struct Request
{
  double yaw=0, roll=0, pitch_start=0, pitch_end=0, rate=.03, acceleration=.02;
  bool align_only=false;
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
  // Worker-local cooperative budget/progress hooks; never call into navigation.
  std::function<bool()> cancelled;
  std::function<void(int,int,bool)> report;
  bool mechanismValid() const;
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
struct Knot { double t=0; Vec eta=Vec::Zero(), velocity=Vec::Zero(), acceleration=Vec::Zero(); };
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
  Request goal;
  Mat neutral=Mat::Identity();
  double wall_seconds=0;
  Kinematics model;
  Path align,cut;
};
bool samplePath(const Kinematics&,const Path&,double time,Reference&);
// Solver-free mechanism validation and FK shared by nominal and QP targets.
bool mechanismAllowed(const Kinematics&,const Vec&,const PlanningConfig&,std::string&);
bool mechanismReference(const Kinematics&,const Vec&,const Vec&,const Vec&,Reference&);
bool continuation(const Kinematics&,const Path&,double begin,double horizon,const PlanningConfig&,Path&,std::string&,
                  const Vec& measured_seed_error=Vec::Zero());
Plan prepare(const Kinematics&,const Vec& initial,const Mat& neutral,const Request&,const PlanningConfig&);

enum class Stage { FREE_FLIGHT, CONTACT_LOCKED, PLANNING, PRE_ALIGN, READY, EXECUTING,
                   PAUSED, COMPLETED_HOLD, FAULT_HOLD };
const char* stageName(Stage);
struct Feedback
{
  double now=0, joint_stamp=0, body_stamp=0;
  bool hover=false, valid=false;
  std::string invalid_reason;
  Pose body=Pose::Identity(); Vec eta=Vec::Zero(), joint_rate=Vec::Zero(), body_omega=Vec::Zero();
};
bool nominalMeasurementSeed(const Kinematics&,const Feedback&,const Reference& nominal,const Reference& applied,
                            const PlanningConfig&,Vec& error,std::string& reason);
struct ExecutionConfig
{
  double timeout=.15, future_tolerance=.02, pivot_tolerance=.01, hinge_tolerance=.03;
  double tracking_tolerance=.04, orientation_tolerance=.04, settle_rate=.01, settle_time=.5, settle_timeout=10;
  bool settle_use_window=false;
  double settle_body_rate=.05, settle_joint_rate=.03, settle_mean_rate=.02;
  bool require_settled_on_enable=false;
  double reference_lead=.06, reference_tracking_allowance=.01, progress_filter_time=.15, goal_tolerance=.0087;
  double progress_timeout=30;
  bool feedbackConfigValid() const;
  bool settlingValid() const;
  bool valid() const;
};
// Shared onboard feedback validation; manual navigation does not instantiate an executor.
std::string feedbackRejection(const Feedback&,const ExecutionConfig&,const PlanningConfig&);
class Executor
{
public:
  Stage stage=Stage::FREE_FLIGHT;
  std::uint64_t session=0;
  std::string reason="free flight";
  std::string first_fault;
  ExecutionConfig config;
  PlanningConfig planning;
  bool qp_planning=false;
  Kinematics model;
  Reference target;
  Estimate measured;
  Plan plan;
  Path online_path; double online_begin=0,online_stamp=0,online_activation_timeout=.5;
  bool online_started=false;
  unsigned online_expired=0;
  double progress=0;
  TaskProgress task;
  double planned_pitch=0, goal_error=0, goal_orientation_error=0;
  bool path_end_commanded=false, goal_reached=false, progress_lag=false;
  Reference applied; bool applied_valid=false;
  SettlingStatistics settling;
  double settling_elapsed=0;
  bool owns() const { return stage!=Stage::FREE_FLIGHT; }
  bool feedbackValid(const Feedback&) const;
  std::string feedbackRejection(const Feedback&) const;
  bool capture(const Feedback&,const Kinematics&);
  // Attach an explicit planner to an already captured base lock; do not recapture contact.
  bool attach(const Kinematics&,const Reference&,std::uint64_t,double now);
  bool accept(Plan);
  bool command(const std::string&,std::uint64_t);
  void tick(const Feedback&);
  void fault(const std::string&);
  void reset();
  void hold(const std::string&);
private:
  void resetSettling();
  SettlingWindow settling_window_;
  double last_time_=0, stable_since_=0, ready_since_=0, speed_=1;
  double lag_since_=0,settling_stamp_=0;
  bool stopping_=false, cancel_=false, paused_align_=false;
};
}
