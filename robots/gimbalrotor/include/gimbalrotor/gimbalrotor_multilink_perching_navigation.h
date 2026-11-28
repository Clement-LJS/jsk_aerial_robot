// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/gimbalrotor_perching_navigation.h>
#include <gimbalrotor/utils/perching_servo.h>
#include <gimbalrotor/RollYawMotion.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt64.h>

#include <cstdint>
#include <deque>
#include <mutex>

namespace perching_geometry
{
struct MotionIdentity
{
  std::uint64_t epoch = 1, sequence = 0;
  std::string source;
  std::map<std::string, std::uint64_t> processed;
  std::string rejection(std::uint64_t e, const std::string& s, std::uint64_t n) const
  {
    if(e != epoch) return "cancelled motion epoch";
    if(s.empty() || s.size() > 128 || n == 0 ||
       s.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos)
      return "invalid motion identity";
    const auto previous = processed.find(s);
    if(previous != processed.end() && n <= previous->second) return "old motion sequence";
    return "";
  }
  void accept(const std::string& s, std::uint64_t n) { source = s; sequence = n; }
  void revoke() { ++epoch; source.clear(); sequence = 0; processed.clear(); }
};
}  // namespace perching_geometry

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
  double pitchAccelerationLimit() const override { return acceleration_limit_; }
  bool nominalHeld() const override;
  bool trackingHeld() const override;
  std::optional<double> pitchFeedStep() const override;
  bool applyPerchingReference(double pitch_offset, double rate, double acceleration,
                             double forward_rate_limit = INFINITY, double recovery_rate_limit = INFINITY) override;
  void holdAppliedReference();
  void publishDiagnostics() override;

protected:
  bool tryLockPerching(const std::string& reason, bool relock = false) override;
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
  bool readSynchronizedSample(Sample&) const;
  void motionCallback(const gimbalrotor::RollYawMotionConstPtr&);
  bool jogReady() const;
  bool coordinateCommand(int, double);
  bool bodyPitchCommand(double angle, double rate, std::uint64_t frame);
  void stopMotion();
  void followCoordinates(const Vector&, double);
  void advanceNominal(double);
  void workflowCommand(const std::string&);
  void planCommand(const std::string&, std::uint64_t);
  perching_servo::Sample servoSample(const Sample* synchronized = nullptr) const;
  bool servoModel(double, KDL::Frame&, KDL::Vector&) const;
  void publishApplied();
  bool candidateAllowed(const Vector&, double) const;

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
  std::string effort_producer_;
  bool positioning_ = false, body_target_valid_ = false, stopping_ = false;
  bool rezero_active_ = false, rezero_ready_ = false, rezero_failed_ = false;
  bool proposal_valid_ = false, prepared_motion_ = false, paused_ = false;
  Vector paused_destination_;
  std::string paused_phase_;
  double paused_body_destination_ = 0, prepared_increment_ = 0;
  std::string phase_ = "FREE_FLIGHT";
  mutable Sample measured_;
  mutable double measurement_cycle_ = -1;
  mutable std::mutex feedback_mutex_;
  std::deque<Sample> joint_history_, body_history_;
  Vector nominal_, destination_, proposed_, nominal_rate_, proposed_rate_, applied_, applied_rate_, applied_acceleration_;
  Vector body_start_, locked_q_;
  Reference applied_state_;
  double body_destination_ = 0, body_progress_ = 0, proposed_progress_ = 0, body_rate_ = 0, proposed_body_rate_ = 0;
  double body_rate_limit_ = INFINITY, body_tracking_tolerance_ = .04;
  double acceleration_limit_ = .02, sync_tolerance_ = .05;
  double max_tilt_ = .9, min_body_pitch_ = -M_PI / 2, max_body_pitch_ = M_PI / 2;
  double cog_speed_limit_ = .1, cog_acceleration_limit_ = .2;
  double rezero_started_ = 0, rezero_stable_since_ = 0, rezero_origin_ = 0, secondary_stable_since_ = 0;
  double rezero_target_ = 0, rezero_sign_ = -1, rezero_rate_ = .1745329, rezero_range_ = .5235988;
  double rezero_tolerance_ = .0349066, rezero_timeout_ = 5, hold_tolerance_ = .02;
  double rezero_kp_ = 1, rezero_stable_duration_ = .3;
  double jog_limit_ = .01745329252;
  double jog_tolerance_ = .00174533;
  double forward_rate_limit_ = INFINITY, recovery_rate_limit_ = INFINITY;
  std::optional<Vector> relief_origin_;
  gimbalrotor::RollYawMotionConstPtr feed_request_;
  gimbalrotor::RollYawMotionConstPtr last_motion_request_;
  std_msgs::String last_motion_ack_;
  std::uint64_t session_ = 0, body_frame_id_ = 0;
  perching_geometry::MotionIdentity motion_;
  perching_servo::Core servo_;
  Eigen::Vector3d reference_velocity_ = Eigen::Vector3d::Zero(), reference_omega_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d reference_acceleration_ = Eigen::Vector3d::Zero(), reference_angular_acceleration_ = Eigen::Vector3d::Zero();
};
}
