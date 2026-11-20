// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/gimbalrotor_navigation.h>
#include <gimbalrotor/gimbalrotor_perching.h>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <gimbalrotor/RollYawMotion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt64.h>
#include <diagnostic_msgs/DiagnosticArray.h>

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
    Vector q, rate;
  };
  GimbalrotorPerchingNavigator();
  void initialize(ros::NodeHandle, ros::NodeHandle, boost::shared_ptr<aerial_robot_model::RobotModel>,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>, double) override;
  void update() override;
  perching_geometry::Session perchingSession() const override;
  bool perchingAdmittanceTarget(const ros::Time&, double, const tf::Vector3&, perching_geometry::Pose&) const override;
  std::recursive_mutex& perchingStateMutex() const { return perching_state_mutex_; }
  bool articulated() const { return !active_joints_.empty(); }
  bool motionFeedbackValid() const;
  bool observation(Sample&) const;
  const Vector& appliedCoordinates() const { return applied_; }
  const Vector& appliedRates() const { return applied_rate_; }
  const Vector& appliedAccelerations() const { return applied_acceleration_; }
  double rigidAngle() const { return arc_pitch_sign_ * rigid_applied_pose_.delta; }
  double rigidRate() const { return (velocity_active_ ? velocity_command_ : 0) + applied_offset_rate_; }
  bool jointCommand(sensor_msgs::JointState& command) const;
  const std::vector<bool>& movingCoordinates() const { return moving_; }
  const std::string& prefix() const { return prefix_; }
  const std::string& worldFrame() const { return world_; }
  bool nominalHeld() const;
  bool trackingHeld() const;
  double appliedPitchOffset() const { return applied_pitch_offset_; }
  bool applyPerchingReference(double pitch_offset, double rate, double acceleration);
  void publishDiagnostics();
private:
  bool tryLockPerching(const std::string& reason);
  void resetPerchingLock();
  bool applyPerchingConstraint(const aerial_robot_msgs::FlightNav& nav_msg);
  void applyManualPitchDelta(double delta);

  // Select the rigid contact mode or the configured articulated contact.
  void selectPerchingMode(perching_geometry::Mode mode, bool enable);
  void relockCallback(const std_msgs::EmptyConstPtr& msg);
  void resetCallback(const std_msgs::EmptyConstPtr& msg);

  // Feedback, nominal commands and controller-visible session state share this lock.

  void commitFixedContactLock(const tf::Vector3& cog_position,
                             const tf::Quaternion& cog_orientation,
                             const tf::Vector3& contact_position);

  void commitPerchingLockIdentity();
  void rosParamInit() override;
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;

  void perchingEnableCallback(const std_msgs::BoolConstPtr& msg);
  void perchingSlantedEnableCallback(const std_msgs::BoolConstPtr& msg);

  void manualPitchDeltaCallback(const std_msgs::Float64ConstPtr& msg);

  bool hasPitchCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;
  bool hasPositionCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;
  bool hasVelocityCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;

  tf::Vector3 getCurrentRobotPos() const;
  tf::Vector3 getCurrentRobotRPY() const;

  tf::Vector3 getDesiredPosition(const aerial_robot_msgs::FlightNav& nav_msg) const;
  tf::Vector3 getDesiredVelocity(const aerial_robot_msgs::FlightNav& nav_msg) const;

  tf::Vector3 getCurrentBaselinkPos() const;
  tf::Matrix3x3 getCurrentBaselinkRot() const;

  void publishLockedDebugPose();
  void publishLockedPivot();
  void publishCommandedDebugPose(const perching_geometry::Pose& pose);
  bool activePose(perching_geometry::Pose& pose) const;

  ros::Subscriber perching_enable_sub_;
  ros::Subscriber perching_slanted_enable_sub_;
  ros::Subscriber relock_sub_;
  ros::Subscriber reset_sub_;
  ros::Subscriber manual_pitch_delta_sub_;

  ros::Publisher locked_pose_pub_;
  ros::Publisher locked_pivot_pub_;
  ros::Publisher commanded_pose_pub_;
  ros::Publisher commanded_pitch_delta_pub_;

  mutable std::recursive_mutex perching_state_mutex_;
  perching_geometry::Mode perching_mode_;
  bool perching_locked_;
  bool perching_lock_once_;

  bool command_pitch_as_delta_;
  bool use_pitch_command_for_arc_;

  bool accept_uav_nav_pitch_command_;
  bool active_perching_hold_enable_;

  double min_valid_radius_;
  double max_pitch_delta_;
  double arc_pitch_sign_;
  double command_pitch_sign_;

  std::string perching_enable_topic_;
  std::string perching_slanted_enable_topic_;
  std::string locked_pivot_topic_;
  std::string relock_topic_;
  std::string reset_topic_;
  std::string manual_pitch_delta_topic_;

  double active_pitch_delta_;
  ros::Time lock_stamp_;
  ros::Time last_lock_stamp_;  // Never reuse a lock identity, even with paused ROS time.

  tf::Vector3 locked_robot_pos_world_;
  tf::Vector3 reference_locked_rpy_;  // Reference Euler angles for NORMAL commands and logging.
  tf::Vector3 locked_pivot_world_;

  double locked_radius_;


  bool ownsNavigationTargets() const override;
  std::unique_lock<std::recursive_mutex> lockNavigationTargets() override;
  void navigationTargetConflict(const char*) override;
  bool supportsPerchingMode(perching_geometry::Mode) const;
  bool tryLockRigid(const std::string&);
  void initializeMechanism();
  bool resolveMechanismModel();
  void jointCallback(const ros::MessageEvent<sensor_msgs::JointState const>&);
  void bodyCallback(const nav_msgs::OdometryConstPtr&);
  void motionCallback(const gimbalrotor::RollYawMotionConstPtr&);
  void coordinateCommand(int, double);
  void bodyPitchCommand(double, double, std::uint64_t);
  void stopMotion();
  void followCoordinates(const Vector&, double);
  void advanceNominal(double);
  void workflowCommand(const std::string&);
  void planCommand(const std::string&, std::uint64_t);
  perching_servo::Sample servoSample() const;
  bool servoModel(double, KDL::Frame&, KDL::Vector&) const;
  void publishApplied();
  void readProfile();
  void resetMotion();
  bool fresh(double stamp, double receipt) const;
  bool candidateAllowed(const Vector&, double) const;
  struct Reference
  {
    Pose body = Pose::Identity(), cog = Pose::Identity();
  };
  bool configureGeometry(std::string& reason);
  bool geometryFrames(const Vector&, std::map<std::string, KDL::Frame>&) const;
  bool bodyContact(const Vector&, Pose&, Eigen::Vector3d&) const;
  bool lockGeometry(const Pose&, const Vector&);
  bool evaluateReference(const Vector&, Reference&) const;
  bool estimateCoordinates(const Pose&, Vector&, double&, double&) const;
  bool angularJacobian(const Vector&, Eigen::MatrixXd&) const;
  bool pitchTangent(const Vector&, const std::vector<bool>&, Vector&) const;
  bool orient(const Vector&, const Eigen::Matrix3d&, const std::vector<bool>&, Vector&) const;
  bool pitch(const Vector&, double, const std::vector<bool>&, Vector&) const;
  bool allowed(const Vector&, double, double, double) const;
  boost::shared_ptr<GimbalrotorRobotModel> model_;
  KDL::Tree geometry_tree_;
  KDL::JntArray held_joints_;
  Pose pivot_ = Pose::Identity();
  Eigen::Matrix3d cog_to_body_ = Eigen::Matrix3d::Identity();
  perching_geometry::Lock hinge_arc_;
  Vector coordinate_lower_, coordinate_upper_, coordinate_velocity_, coordinate_acceleration_;
  std::vector<std::string> active_joints_;
  std::vector<bool> moving_;
  std::map<std::string, int> coordinate_map_;
  std::map<int, std::string> wire_coordinates_;
  std::vector<std::string> wire_angles_;
  std::vector<ros::Subscriber> subscriptions_;
  std::map<std::string, ros::Publisher> bool_pubs_, number_pubs_;
  ros::Publisher joints_pub_;
  ros::Publisher reference_pub_, measured_pub_, cog_pub_, status_pub_, state_pub_;
  ros::Publisher session_pub_, epoch_pub_, ack_pub_, nominal_pub_, angles_pub_, lower_pub_, upper_pub_, body_pose_pub_;
  std::string prefix_ = "perching", world_ = "world", body_frame_, status_name_ = "perching";
  std::string joint_topic_, manual_coordinate_ = "alpha", positioning_joint_;
  bool model_valid_ = false;
  bool positioning_ = false, body_target_valid_ = false, velocity_active_ = false, stopping_ = false;
  bool rezero_active_ = false, rezero_ready_ = false, rezero_failed_ = false;
  bool proposal_valid_ = false, prepared_motion_ = false, paused_ = false;
  Vector paused_destination_;
  std::string paused_phase_;
  double paused_body_destination_ = 0, prepared_increment_ = 0;
  std::string status_ = "free flight", phase_ = "FREE_FLIGHT";
  Sample measured_;
  std::deque<Sample> joint_history_, body_history_;
  Vector nominal_, destination_, proposed_, nominal_rate_, proposed_rate_, applied_, applied_rate_, applied_acceleration_;
  Vector body_start_, locked_q_;
  Reference applied_state_;
  perching_geometry::Pose rigid_applied_pose_;
  bool rigid_applied_valid_ = false;
  bool rigid_command_pending_ = false;
  double applied_pitch_offset_ = 0, applied_offset_rate_ = 0;
  double body_destination_ = 0, body_progress_ = 0, proposed_progress_ = 0, body_rate_ = 0, proposed_body_rate_ = 0;
  double body_rate_limit_ = .03;
  double tracking_tolerance_ = .04, body_tracking_tolerance_ = .04;
  double rate_limit_ = .03, acceleration_limit_ = .02, velocity_command_ = 0, velocity_expires_ = 0;
  double timeout_ = .15, future_tolerance_ = .02, sync_tolerance_ = .05, max_step_ = .05, max_gap_ = .15;
  double max_tilt_ = .9, min_body_pitch_ = -M_PI / 2, max_body_pitch_ = M_PI / 2;
  double cog_speed_limit_ = .1, cog_acceleration_limit_ = .2;
  double last_update_ = 0, cycle_dt_ = 0, applied_stamp_ = 0, pivot_error_ = 0, hinge_error_ = 0;
  double rezero_started_ = 0, rezero_stable_since_ = 0, rezero_origin_ = 0, secondary_stable_since_ = 0;
  double rezero_target_ = 0, rezero_sign_ = -1, rezero_rate_ = .1745329, rezero_range_ = .5235988;
  double rezero_tolerance_ = .0349066, rezero_timeout_ = 5, hold_tolerance_ = .02;
  double rezero_kp_ = 1, rezero_rate_tolerance_ = .05, rezero_stable_duration_ = .3;
  double velocity_duration_ = 8, jog_limit_ = .01745329252;
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
