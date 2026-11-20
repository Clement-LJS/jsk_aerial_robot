// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/gimbalrotor_perching_navigation.h>
#include <gimbalrotor/RollYawMotion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt64.h>

#include <cstdint>
#include <deque>
#include <functional>
#include <limits>

namespace perching_geometry
{
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
}  // namespace perching_geometry

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
inline double pitch(const KDL::Frame& f)
{
  double r, p, y;
  f.M.GetRPY(r, p, y);
  return p;
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

namespace aerial_robot_navigation
{
class GimbalrotorMultilinkPerchingNavigator : public GimbalrotorPerchingNavigator
{
public:
  bool perchingAdmittanceTarget(const ros::Time&, double, const tf::Vector3&, perching_geometry::Pose&) const override;
  bool motionFeedbackValid() const override;
  bool observation(Sample&) const override;
  const Vector& appliedCoordinates() const { return applied_; }
  const Vector& appliedRates() const { return applied_rate_; }
  const Vector& appliedAccelerations() const { return applied_acceleration_; }
  bool jointCommand(sensor_msgs::JointState& command) const;
  const std::vector<bool>& movingCoordinates() const { return moving_; }
  bool nominalHeld() const override;
  bool trackingHeld() const override;
  bool applyPerchingReference(double pitch_offset, double rate, double acceleration) override;
  void publishDiagnostics() override;

protected:
  bool tryLockPerching(const std::string& reason) override;
  bool supportsPerchingMode(perching_geometry::Mode) const override;
  void readProfile() override;
  void initializeMechanism() override;
  bool resolveMechanismModel() override;
  void resetMotion() override;
  void applyManualPitchDelta(double delta) override;
  void relockCallback(const std_msgs::EmptyConstPtr& msg) override;
  void onNonHover() override;
  void onClockRewind() override;
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;

private:
  void jointCallback(const ros::MessageEvent<sensor_msgs::JointState const>&);
  void bodyCallback(const nav_msgs::OdometryConstPtr&);
  void motionCallback(const gimbalrotor::RollYawMotionConstPtr&);
  bool coordinateCommand(int, double);
  bool bodyPitchCommand(double angle, double rate, std::uint64_t frame);
  void stopMotion();
  void followCoordinates(const Vector&, double);
  void advanceNominal(double);
  void workflowCommand(const std::string&);
  void planCommand(const std::string&, std::uint64_t);
  perching_servo::Sample servoSample() const;
  bool servoModel(double, KDL::Frame&, KDL::Vector&) const;
  void publishApplied();
  bool candidateAllowed(const Vector&, double) const;
  bool evaluateReference(const Vector& q, Reference& out) const;
  bool angularJacobian(const Vector&, Eigen::MatrixXd&) const;
  bool pitchTangent(const Vector&, const std::vector<bool>&, Vector&) const;
  bool orient(const Vector&, const Eigen::Matrix3d&, const std::vector<bool>&, Vector&) const;
  bool pitch(const Vector&, double, const std::vector<bool>&, Vector&) const;
  bool allowed(const Vector&, double, double, double) const;

  Vector coordinate_lower_, coordinate_upper_, coordinate_velocity_, coordinate_acceleration_;
  std::vector<bool> moving_;
  std::map<std::string, int> coordinate_map_;
  std::map<int, std::string> wire_coordinates_;
  std::vector<std::string> wire_angles_;
  std::vector<ros::Subscriber> subscriptions_;
  std::map<std::string, ros::Publisher> bool_pubs_, number_pubs_;
  ros::Publisher joints_pub_;
  ros::Publisher reference_pub_, measured_pub_, cog_pub_;
  ros::Publisher session_pub_, epoch_pub_, ack_pub_, nominal_pub_, angles_pub_, lower_pub_, upper_pub_, body_pose_pub_;
  std::string body_frame_, joint_topic_, manual_coordinate_ = "alpha", positioning_joint_;
  bool positioning_ = false, body_target_valid_ = false, stopping_ = false;
  bool rezero_active_ = false, rezero_ready_ = false, rezero_failed_ = false;
  bool proposal_valid_ = false, prepared_motion_ = false, paused_ = false;
  Vector paused_destination_;
  std::string paused_phase_;
  double paused_body_destination_ = 0, prepared_increment_ = 0;
  std::string phase_ = "FREE_FLIGHT";
  Sample measured_;
  std::deque<Sample> joint_history_, body_history_;
  Vector nominal_, destination_, proposed_, nominal_rate_, proposed_rate_, applied_, applied_rate_, applied_acceleration_;
  Vector body_start_, locked_q_;
  Reference applied_state_;
  double body_destination_ = 0, body_progress_ = 0, proposed_progress_ = 0, body_rate_ = 0, proposed_body_rate_ = 0;
  double body_rate_limit_ = .03, body_tracking_tolerance_ = .04;
  double acceleration_limit_ = .02, sync_tolerance_ = .05;
  double max_tilt_ = .9, min_body_pitch_ = -M_PI / 2, max_body_pitch_ = M_PI / 2;
  double cog_speed_limit_ = .1, cog_acceleration_limit_ = .2;
  double rezero_started_ = 0, rezero_stable_since_ = 0, rezero_origin_ = 0, secondary_stable_since_ = 0;
  double rezero_target_ = 0, rezero_sign_ = -1, rezero_rate_ = .1745329, rezero_range_ = .5235988;
  double rezero_tolerance_ = .0349066, rezero_timeout_ = 5, hold_tolerance_ = .02;
  double rezero_kp_ = 1, rezero_stable_duration_ = .3;
  double jog_limit_ = .01745329252;
  gimbalrotor::RollYawMotionConstPtr last_motion_request_;
  std_msgs::String last_motion_ack_;
  std::uint64_t session_ = 0, body_frame_id_ = 0;
  perching_geometry::MotionIdentity motion_;
  perching_servo::Core servo_;
  perching_servo::Core previous_servo_;
  Eigen::Vector3d reference_velocity_ = Eigen::Vector3d::Zero(), reference_omega_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference_acceleration_ = Eigen::Vector3d::Zero(), reference_angular_acceleration_ = Eigen::Vector3d::Zero();
};
}
