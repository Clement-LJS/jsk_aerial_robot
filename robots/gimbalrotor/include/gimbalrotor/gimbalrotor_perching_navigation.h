// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/gimbalrotor_navigation.h>
#include <gimbalrotor/model/gimbalrotor_perching_model.h>
#include <gimbalrotor/utils/perching_geometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <ros/time.h>
#include <tf/transform_datatypes.h>

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

namespace aerial_robot_navigation
{
class GimbalrotorPerchingNavigator : public GimbalrotorNavigator, public perching_geometry::TargetProvider
{
public:
  using Vector = Eigen::VectorXd;
  using Pose = Eigen::Isometry3d;
  struct Sample
  {
    double stamp = 0, receipt = 0;
    Pose body = Pose::Identity();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    Vector q, rate, effort;
    bool effort_valid = false; // Named, finite samples; the producer must supply joint torque in N.m.
  };
  GimbalrotorPerchingNavigator();
  void initialize(ros::NodeHandle, ros::NodeHandle, boost::shared_ptr<aerial_robot_model::RobotModel>,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>, double) override;
  void update() override;
  perching_geometry::Session perchingSession() const override;
  bool perchingAdmittanceTarget(const ros::Time&, double, const tf::Vector3&, perching_geometry::Pose&) const override;
  std::recursive_mutex& perchingStateMutex() const { return perching_state_mutex_; }
  bool articulated() const { return !active_joints_.empty(); }
  virtual bool motionFeedbackValid() const;
  virtual bool observation(Sample&) const;
  double rigidAngle() const { return arc_pitch_sign_ * rigid_applied_pose_.delta; }
  double rigidRate() const { return rigid_applied_rate_; }
  double rigidAcceleration() const { return rigid_applied_acceleration_; }
  const perching_geometry::Lock& rigidLock() const { return hinge_arc_; }
  double pitchRateLimit() const { return rate_limit_; }
  virtual double pitchAccelerationLimit() const { return INFINITY; }
  void setAdmittanceDiagnostics(const std::map<std::string, std::string>& values) { admittance_diagnostics_ = values; }
  const std::string& prefix() const { return prefix_; }
  const std::string& worldFrame() const { return world_; }
  virtual bool nominalHeld() const;
  virtual bool trackingHeld() const;
  virtual std::optional<double> pitchFeedStep() const { return std::nullopt; }
  double appliedPitchOffset() const { return applied_pitch_offset_; }
  double appliedPitchOffsetRate() const { return applied_offset_rate_; }
  double referenceTimeStep() const { return cycle_dt_; }
  virtual bool applyPerchingReference(double pitch_offset, double rate, double acceleration,
                                     double forward_rate_limit = INFINITY, double recovery_rate_limit = INFINITY);
  virtual void publishDiagnostics();

protected:
  virtual bool tryLockPerching(const std::string& reason, bool relock = false);
  virtual bool supportsPerchingMode(perching_geometry::Mode) const;
  virtual void readProfile();
  virtual void initializeMechanism();
  virtual bool resolveMechanismModel();
  virtual void resetMotion();
  virtual void applyManualPitchDelta(double delta);
  virtual void relockCallback(const std_msgs::EmptyConstPtr& msg);
  virtual void onNonHover() {}
  virtual void onClockRewind() {}
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;
  bool hasPitchCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;
  void commitFixedContactLock(const tf::Vector3& cog_position, const tf::Quaternion& cog_orientation,
                             const tf::Vector3& contact_position);
  static double steadyTime();
  bool fresh(double stamp, double receipt) const;
  bool limitPitchOffset(double& offset, double& rate, double& acceleration,
                        double positive_rate_limit = INFINITY, double upper_offset = INFINITY) const;
  using Reference = GimbalrotorPerchingModel::State;
  bool lockGeometry(const Pose& body, const Vector& q, Reference& initial);
  Pose worldCogPose(const Pose& body, const Eigen::Vector3d& cog_body) const;

  boost::shared_ptr<GimbalrotorPerchingModel> model_;
  Pose pivot_ = Pose::Identity();
  std::vector<std::string> active_joints_;
  // Feedback, nominal commands and controller-visible session state share this lock.
  mutable std::recursive_mutex perching_state_mutex_;
  perching_geometry::Mode perching_mode_;
  bool perching_locked_;
  bool accept_uav_nav_pitch_command_;
  double command_pitch_sign_;
  ros::Time lock_stamp_;
  ros::Publisher status_pub_, state_pub_;
  std::string prefix_ = "perching", world_ = "world", status_name_ = "perching";
  std::string status_ = "free flight";
  std::map<std::string, std::string> admittance_diagnostics_;
  bool model_valid_ = false, velocity_active_ = false;
  double applied_pitch_offset_ = 0, applied_offset_rate_ = 0;
  double pitch_offset_limit_ = .15, pitch_offset_rate_limit_ = .5, pitch_offset_acceleration_limit_ = .1;
  double tracking_tolerance_ = .04, rezero_rate_tolerance_ = .05;
  double rate_limit_ = .03, velocity_command_ = 0, velocity_expires_ = 0, velocity_duration_ = 8;
  double timeout_ = .15, future_tolerance_ = .02, max_step_ = .05, max_gap_ = .15;
  double last_update_ = 0, cycle_dt_ = 0, applied_stamp_ = 0, pivot_error_ = 0, hinge_error_ = 0;

private:
  void rosParamInit() override;
  bool ownsNavigationTargets() const override;
  std::unique_lock<std::recursive_mutex> lockNavigationTargets() override;
  void navigationTargetConflict(const char*) override;
  void selectPerchingMode(perching_geometry::Mode mode, bool enable);
  void resetPerchingLock();
  void manualPitchDeltaCallback(const std_msgs::Float64ConstPtr& msg);
  void commitPerchingLockIdentity();
  bool applyPerchingConstraint(const aerial_robot_msgs::FlightNav& nav_msg);
  tf::Vector3 getCurrentRobotRPY() const;
  tf::Vector3 getDesiredPosition(const aerial_robot_msgs::FlightNav& nav_msg) const;
  tf::Vector3 getDesiredVelocity(const aerial_robot_msgs::FlightNav& nav_msg) const;
  tf::Vector3 getCurrentBaselinkPos() const;
  tf::Matrix3x3 getCurrentBaselinkRot() const;
  void publishLockedDebugPose();
  void publishLockedPivot();
  void publishCommandedDebugPose(const perching_geometry::Pose& pose);
  bool activePose(perching_geometry::Pose& pose) const;
  void publishApplied();

  ros::Subscriber perching_enable_sub_;
  ros::Subscriber perching_slanted_enable_sub_;
  ros::Subscriber relock_sub_;
  ros::Subscriber reset_sub_;
  ros::Subscriber manual_pitch_delta_sub_;
  ros::Publisher locked_pose_pub_;
  ros::Publisher locked_pivot_pub_;
  ros::Publisher commanded_pose_pub_;
  ros::Publisher commanded_pitch_delta_pub_;
  bool perching_lock_once_;
  bool command_pitch_as_delta_;
  bool use_pitch_command_for_arc_;
  bool active_perching_hold_enable_;
  double min_valid_radius_;
  double max_pitch_delta_;
  double arc_pitch_sign_;
  std::string perching_enable_topic_;
  std::string perching_slanted_enable_topic_;
  std::string locked_pivot_topic_;
  std::string relock_topic_;
  std::string reset_topic_;
  std::string manual_pitch_delta_topic_;
  double active_pitch_delta_;
  ros::Time last_lock_stamp_;  // Never reuse a lock identity, even with paused ROS time.
  tf::Vector3 locked_robot_pos_world_;
  tf::Vector3 reference_locked_rpy_;  // Reference Euler angles for NORMAL commands and logging.
  tf::Vector3 locked_pivot_world_;
  double locked_radius_;
  double rigid_applied_rate_ = 0, rigid_applied_acceleration_ = 0;
  KDL::JntArray held_joints_;
  Eigen::Matrix3d cog_to_body_ = Eigen::Matrix3d::Identity();
  perching_geometry::Lock hinge_arc_;
  perching_geometry::Pose rigid_applied_pose_;
  bool rigid_applied_valid_ = false;
  bool rigid_command_pending_ = false;
};
}
