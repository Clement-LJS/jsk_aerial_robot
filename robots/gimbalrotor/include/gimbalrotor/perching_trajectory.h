#pragma once

#include <gimbalrotor/perching_geometry.h>
#include <algorithm>
#include <cmath>
#include <array>
#include <deque>
#include <string>
#include <cstdint>
#include <vector>

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
