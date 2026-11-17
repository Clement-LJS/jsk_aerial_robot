// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/gimbalrotor_navigation.h>
#include <gimbalrotor/gimbalrotor_perching.h>
#include <gimbalrotor/model/gimbalrotor_multilink_perching_model.h>
#include <gimbalrotor/control/gimbalrotor_multilink_perching_admittance_controller.h>
#include <cmath>
#include <mutex>
#include <string>
#include <sstream>
#include <iomanip>
#include <aerial_robot_msgs/FlightNav.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <map>
#include <set>
#include <vector>
#include <kdl/frames.hpp>
#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt64.h>
#include <gimbalrotor/RollYawMotion.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <limits>

namespace aerial_robot_navigation
{

class GimbalrotorPerchingNavigator : public GimbalrotorNavigator, public perching_geometry::TargetProvider,
                                     public aerial_robot_control::RollYawInteractionHost
{
public:
  GimbalrotorPerchingNavigator();
  ~GimbalrotorPerchingNavigator() override;

  void initialize(
      ros::NodeHandle nh,
      ros::NodeHandle nhp,
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
      double loop_du) override;

  void update() override;
  perching_geometry::Session perchingSession() const override;
  bool perchingAdmittanceTarget(const ros::Time& lock_stamp, double physical_offset,
      const tf::Vector3& nominal_position, perching_geometry::Pose& pose) const override;

  bool attachRollYawInteraction(const std::shared_ptr<aerial_robot_control::RollYawInteraction>&,
                               const std::function<void(const aerial_robot_control::RollYawInteractionInput&)>&) override;
  bool rollYawReferenceActive() const override;
  void withRollYawInteraction(const std::function<void(const aerial_robot_control::RollYawInteractionInput&)>&,
                             bool publish_diagnostics = false, const std::string& key = "interaction") override;

private:

  bool tryLockPerching(const std::string& reason);
  void resetPerchingLock();
  void applyActivePerchingTarget();
  bool applyPerchingConstraint(aerial_robot_msgs::FlightNav& nav_msg);
  void applyManualPitchDelta(double delta);

  // Dispatch contact commands to the selected startup configuration.
  void selectPerchingMode(perching_geometry::Mode mode, bool enable);
  void relockCallback(const std_msgs::EmptyConstPtr& msg);
  void resetCallback(const std_msgs::EmptyConstPtr& msg);

  // Controller-visible session state is serialized here. Roll-yaw feedback and
  // pending commands use input_mutex_, never this calculation/reader mutex.
  std::recursive_mutex& perchingStateMutex() const { return perching_state_mutex_; }
  void commitFixedContactLock(const tf::Vector3& cog_position,
                             const tf::Quaternion& cog_orientation,
                             const tf::Vector3& contact_position);

  void commitPerchingLockIdentity();
  void rosParamInit() override;
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;

  void perchingEnableCallback(const std_msgs::BoolConstPtr& msg);
  void perchingSlantedEnableCallback(const std_msgs::BoolConstPtr& msg);

  void branchPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void perchingPointCallback(const geometry_msgs::PointStampedConstPtr& msg);

  void manualPitchDeltaCallback(const std_msgs::Float64ConstPtr& msg);

  aerial_robot_msgs::FlightNav buildActivePerchingNavCommand();

  bool hasPitchCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;
  bool hasPositionCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;
  bool hasVelocityCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;

  tf::Vector3 getCurrentRobotPos() const;
  tf::Vector3 getCurrentRobotRPY() const;

  tf::Vector3 getDesiredPosition(const aerial_robot_msgs::FlightNav& nav_msg) const;
  tf::Vector3 getDesiredVelocity(const aerial_robot_msgs::FlightNav& nav_msg) const;

  tf::Vector3 getCurrentBaselinkPos() const;
  tf::Matrix3x3 getCurrentBaselinkRot() const;

  tf::Vector3 computeHandPerchingCenterWorldFromBaselink() const;

  bool isManualPivotMode() const;
  bool isBranchPivotMode() const;
  bool hasBranchPivotSource() const;

  tf::Vector3 computeLockPivotWorld() const;

  double clamp(double value, double min_value, double max_value) const;

  void publishLockedDebugPose();
  void publishLockedPivot();
  void publishCommandedDebugPose(const perching_geometry::Pose& pose);
  bool activePose(perching_geometry::Pose& pose) const;
  void applyAxialCompliance(perching_geometry::Pose& pose) const;
  void setPoseCommand(aerial_robot_msgs::FlightNav& msg, const perching_geometry::Pose& pose);

  ros::Subscriber perching_enable_sub_;
  ros::Subscriber perching_slanted_enable_sub_;
  ros::Subscriber branch_pose_sub_;
  ros::Subscriber perching_point_sub_;
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

  bool require_branch_point_;
  bool command_pitch_as_delta_;
  bool constrain_position_command_;
  bool constrain_velocity_command_;
  bool use_pitch_command_for_arc_;
  bool hold_locked_pose_without_pitch_command_;

  bool accept_uav_nav_pitch_command_;
  bool active_perching_hold_enable_;

  double min_valid_radius_;
  double max_pitch_delta_;
  double arc_pitch_sign_;
  double command_pitch_sign_;
  double y_compliance_deadband_;

  std::string pivot_source_;

  tf::Vector3 hand_perching_center_offset_baselink_; 
  
  std::string perching_enable_topic_;
  std::string perching_slanted_enable_topic_;
  std::string branch_pose_topic_;
  std::string perching_point_topic_;
  std::string locked_pivot_topic_;
  std::string relock_topic_;
  std::string reset_topic_;
  std::string manual_pitch_delta_topic_;

  bool has_branch_pose_;
  bool has_perching_point_;

  double active_pitch_delta_;
  perching_geometry::Lock geometry_;  // Authoritative reference quaternion for both modes.
  ros::Time lock_stamp_;
  ros::Time last_lock_stamp_;  // Never reuse a lock identity, even with paused ROS time.

  tf::Vector3 branch_pos_world_;
  tf::Vector3 perching_point_world_;

  tf::Vector3 locked_robot_pos_world_;
  tf::Vector3 reference_locked_rpy_;  // Reference Euler angles for NORMAL commands and logging.
  tf::Vector3 locked_pivot_world_;

  double locked_radius_;

  enum class PerchingConfiguration { BASELINE, PITCH_ONLY, MULTILINK, ROLL_YAW };
  PerchingConfiguration perching_configuration_ = PerchingConfiguration::BASELINE;

  bool supportsPerchingMode(perching_geometry::Mode mode) const;
  bool ownsNavigationTargets() const override;
  std::unique_lock<std::recursive_mutex> lockNavigationTargets() override;
  void navigationTargetConflict(const char* source) override;

  void updateCommonPerching();
  void selectCommonPerchingMode(perching_geometry::Mode mode, bool enable);
  void resetCommonPerchingLock();
  bool tryLockBaseline(const std::string& reason);
  void applyBaselineTarget();
  bool applyBaselineConstraint(aerial_robot_msgs::FlightNav& msg);
  void applyBaselinePitchDelta(double delta);
  void baselineNaviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg);
  void relockCommonCallback(const std_msgs::EmptyConstPtr& msg);

  // Pitch-only workflow
  void initializePitchOnly(ros::NodeHandle nh, ros::NodeHandle nhp);
  void updatePitchOnly();

  void selectPitchOnlyMode(perching_geometry::Mode mode,bool enable);
  void relockPitchOnlyCallback(const std_msgs::EmptyConstPtr&);
  bool tryLockPitchOnly(const std::string&);
  void resetPitchOnlyLock();
  void applyPitchOnlyTarget();

  void applyPitchOnlyDelta(double delta);
  void pitchOnlyNaviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg);
  bool resolveModel();
  bool modelAt(double q,KDL::Frame& bc,KDL::Vector& cog) const;
  pitch_only::Sample sample() const;
  void pitchOnlyJointCallback(const sensor_msgs::JointStateConstPtr& msg);
  void requestPitchOnly(const std::string& action);
  void commitLock();
  void publishPitchOnlyDiagnostics();
  bool commandPose();
  pitch_only::Core core_;
  pitch_only::Sample feedback_;
  bool model_valid_=false, pitch_only_model_resolution_attempted_=false;
  double pitch_only_pitch_command_sign_=1, pitch_only_pitch_axis_alignment_threshold_=.90;
  double configured_lower_=pitch_only::unknown, configured_upper_=pitch_only::unknown;
  double configured_rate_=pitch_only::unknown;
  std::size_t joint_index_=0;
  double joint_receive_stamp_=0;
  std::string joint_name_,contact_link_,world_frame_,pitch_only_prefix_;
  ros::Time last_update_;
  std::vector<ros::Subscriber> subscribers_;
  ros::Publisher pitch_only_joints_pub_,pose_pub_,pivot_pub_,pitch_only_state_pub_;
  std::map<std::string,ros::Publisher> bool_pubs_,number_pubs_;

  // Multilink workflow

  void initializeMultilink(ros::NodeHandle nh, ros::NodeHandle nhp);

  void updateMultilink();

  bool multilinkModelValid() const;
  bool multilinkLockValid() const;
  bool secondaryJointSettled() const;
  std::uint64_t mechanismTargetGeneration() const;

  bool tryLockMultilink(const std::string& reason);
  void resetMultilinkLock();
  void applyMultilinkTarget();
  bool applyMultilinkConstraint(
      aerial_robot_msgs::FlightNav& nav_msg);
  void applyMultilinkPitchDelta(double delta);

  enum class MultilinkPitchRezeroState { IDLE = 0, ACTIVE, HOLD };

  void multilinkPitchRezeroCallback(const std_msgs::EmptyConstPtr& msg);
  void resetMultilinkPitchRezeroState();
  void failMultilinkPitchRezero(const char* reason);
  bool verifyMultilinkPitchRezeroFrozenJoints();
  void updateMultilinkBodyPitchFeedback();
  void updateMultilinkPitchRezeroCommand();
  void applyMultilinkPitchRezeroTarget();
  bool readBaselinkPose(KDL::Frame& pose) const;
  double currentBaselinkPitch() const;
  static double wrapAngle(double angle);
  bool commandMultilinkBaselinkPose(const KDL::Frame& T_W_B_des,
                                 const KDL::Vector& p_B_G);

  // This hinge is an external contact DOF, never an additional model joint.
  MultilinkPitchRezeroState multilink_pitch_rezero_state_ = MultilinkPitchRezeroState::IDLE;
  bool multilink_pitch_rezero_ready_ = false;
  bool multilink_pitch_rezero_failed_ = false;
  bool pitch_rezero_config_valid_ = false;
  bool rezero_target_applied_this_update_ = false;
  std::string pitch_rezero_topic_;
  KDL::Vector pitch_rezero_pivot_offset_;
  KDL::Vector pitch_rezero_axis_;
  double pitch_rezero_target_pitch_ = 0.0;
  double pitch_rezero_kp_ = 1.0;
  double pitch_rezero_rate_limit_ = 0.1745329;
  double pitch_rezero_max_delta_ = 0.5235988;
  double pitch_rezero_command_sign_ = 1.0;
  double pitch_rezero_pitch_tolerance_ = 0.0349066;
  double pitch_rezero_rate_tolerance_ = 0.05;
  double pitch_rezero_stable_duration_ = 0.30;
  double pitch_rezero_timeout_ = 5.0;
  double pitch_rezero_pitch_joint_hold_tolerance_ = 0.02;
  double pitch_rezero_secondary_joint_hold_tolerance_ = 0.02;

  double rezero_frozen_pitch_joint_ = 0.0;
  double rezero_frozen_secondary_joint_ = 0.0;
  double rezero_pitch_joint_hold_error_ = 0.0;
  double rezero_secondary_joint_hold_error_ = 0.0;
  KDL::Vector rezero_pivot_world_;
  KDL::Vector rezero_axis_world_;
  KDL::Vector rezero_p_B_G_;
  KDL::Frame rezero_reference_body_world_;
  KDL::Frame rezero_last_body_target_;
  double rezero_alpha_cmd_ = 0.0;
  ros::Time rezero_start_stamp_;
  ros::Time rezero_stable_start_stamp_;
  double rezero_previous_body_pitch_ = 0.0;
  ros::Time rezero_previous_body_pitch_stamp_;
  double rezero_measured_pitch_rate_ = 0.0;
  double multilink_rezero_feedback_dt_ = 0.0;
  bool rezero_pitch_rate_valid_ = false;

  ros::Subscriber pitch_rezero_sub_;
  ros::Publisher pitch_rezero_active_pub_;
  ros::Publisher pitch_rezero_ready_pub_;
  ros::Publisher pitch_rezero_failed_pub_;
  ros::Publisher pitch_joint_hold_error_pub_;
  ros::Publisher secondary_joint_hold_error_pub_;
  ros::Publisher passive_pitch_delta_pub_;
  ros::Publisher body_pitch_pub_;
  ros::Publisher body_pitch_rate_pub_;

  enum SecondaryAxisType
  {
    SECONDARY_AXIS_INVALID = 0,
    SECONDARY_AXIS_ROLL,
    SECONDARY_AXIS_YAW
  };

  void multilinkRosParamInit();
  bool resolveMechanismModel();
  bool resolveJointLimits(
      const std::string& joint_name,
      bool configured_limits_valid,
      double configured_lower,
      double configured_upper,
      double& lower,
      double& upper) const;

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  void secondaryJointTargetCallback(const std_msgs::Float64ConstPtr& msg);

  bool readCurrentMechanismState(
      double& pitch_position,
      double& secondary_position,
      double& pitch_velocity,
      double& secondary_velocity) const;

  bool computeBaselinkToContactTransform(
      double pitch_position,
      double secondary_position,
      KDL::Frame& T_B_C) const;

  bool computeBaselinkToCogVector(
      double pitch_position,
      double secondary_position,
      KDL::Vector& p_B_G) const;

  bool buildFinalJointTarget(
      double& pitch_final,
      double& secondary_final) const;

  void updateSecondarySettledState();
  void publishMultilinkJointTarget(
      double pitch_final,
      double secondary_final);
  void publishDiagnostics() const;

  static bool frameFinite(const KDL::Frame& frame);
  static double clampMultilink(double value, double lower, double upper);

  ros::Subscriber joint_state_sub_;
  ros::Subscriber secondary_joint_target_sub_;

  ros::Publisher joint_control_pub_;
  ros::Publisher model_valid_pub_;
  ros::Publisher secondary_settled_pub_;
  ros::Publisher pitch_measured_pub_;
  ros::Publisher pitch_nominal_pub_;
  ros::Publisher pitch_final_pub_;
  ros::Publisher secondary_measured_pub_;
  ros::Publisher secondary_target_pub_;
  ros::Publisher target_body_pose_pub_;

  std::string pitch_joint_name_;
  std::string secondary_joint_name_;
  std::string contact_link_name_;
  std::string joint_command_topic_;
  std::string joint_state_topic_;
  std::string secondary_joint_target_topic_;

  double secondary_position_tolerance_;
  double secondary_velocity_tolerance_;
  double secondary_settle_duration_;

  double joint_state_timeout_;
  double joint_state_future_tolerance_;

  double multilink_pitch_axis_alignment_threshold_;
  double secondary_axis_alignment_threshold_;

  double multilink_pitch_command_sign_;
  double secondary_command_sign_;

  bool configured_pitch_limits_valid_;
  bool configured_secondary_limits_valid_;
  double configured_pitch_lower_;
  double configured_pitch_upper_;
  double configured_secondary_lower_;
  double configured_secondary_upper_;

  std::size_t pitch_joint_index_;
  std::size_t secondary_joint_index_;

  double pitch_joint_lower_;
  double pitch_joint_upper_;
  double secondary_joint_lower_;
  double secondary_joint_upper_;

  Eigen::Vector3d pitch_axis_local_;
  Eigen::Vector3d secondary_axis_local_;
  SecondaryAxisType secondary_axis_type_;

  bool multilink_model_resolution_attempted_;
  bool multilink_model_valid_;
  bool multilink_lock_valid_;
  KDL::Frame locked_contact_world_;

  double locked_pitch_joint_;
  double locked_secondary_joint_;
  double pitch_joint_nominal_target_;
  double secondary_joint_nominal_target_;
  double pitch_joint_final_target_;
  double secondary_joint_final_target_;

  bool joint_state_received_;
  ros::Time joint_state_measurement_stamp_;
  ros::Time joint_state_receive_stamp_;
  double measured_pitch_joint_;
  double measured_secondary_joint_;
  double measured_pitch_velocity_;
  double measured_secondary_velocity_;
  double previous_measured_pitch_joint_;
  double previous_measured_secondary_joint_;

  bool secondary_settled_;
  ros::Time secondary_settle_start_;

  std::uint64_t mechanism_target_generation_;

  // Roll-yaw workflow
  void initializeRollYaw(ros::NodeHandle nh, ros::NodeHandle nhp);
  void updateRollYaw();

  bool ownsRollYawTargets() const;

  void rollYawTargetConflict(const char*);
  bool tryLockRollYaw(const std::string&);
  void resetRollYawLock();
  void applyRollYawTarget();
  void applyRollYawPitchDelta(double);

  void rollYawNaviCallback(const aerial_robot_msgs::FlightNavConstPtr&);
  // Enable, disable, reset and relock use the common contact lifecycle.
  void rollYawRosParamInit(ros::NodeHandle nh, ros::NodeHandle nhp);
  void taskCommandCallback(const std_msgs::UInt64ConstPtr&, const std::string& action);
  bool snapshotModel(roll_yaw::Kinematics&, std::string&);
  roll_yaw::Feedback feedback() const;
  std::string contactRejection(const roll_yaw::Feedback&, bool* hard_fault = nullptr) const;
  mutable double contact_body_stamp_ = -1, contact_joint_stamp_ = -1;
  mutable std::uint64_t contact_session_ = 0;
  mutable bool contact_hard_fault_ = false;
  mutable std::string contact_reason_;
  void rollYawJointCallback(const ros::MessageEvent<sensor_msgs::JointState const>&);
  void bodyCallback(const nav_msgs::OdometryConstPtr&);
  void bodyAlignmentCallback(const std_msgs::Float64ConstPtr&, int axis);
  void bodyPitchCallback(const std_msgs::Float64ConstPtr&);
  void bodyPitchVelocityCallback(const std_msgs::Float64ConstPtr&);
  bool admitPitchVelocity(double, double receipt);
  bool advancePitchVelocity(roll_yaw::Reference&, double dt, double now);
  void motionCallback(const gimbalrotor::RollYawMotionConstPtr&);
  bool rejectUnscopedMotion(const std::string& action);
  bool admitPitchTarget(double angle, double rate, std::uint64_t frame);
  void revokeMotion(bool retain_reference = false);
  roll_yaw::MotionIdentity motion_identity_;
  roll_yaw::MotionIdentity input_motion_identity_; // input_mutex_; receipt ordering before coalescing
  std::atomic<std::uint64_t> input_motion_epoch_{1};
  bool motion_dispatch_ = false;
  ros::Publisher motion_epoch_pub_;
  struct PitchTarget
  {
    bool valid = false, frame_valid = false, complete = false;
    std::uint64_t frame = 0;
    Eigen::Matrix3d start = Eigen::Matrix3d::Identity();
    double angle = 0, rate = 0, progress = 0;
  } pitch_target_;
  struct PitchVelocity
  {
    bool active = false, pending = false, stopping = false;
    double requested = 0, expires = 0, clock = 0;
    std::string limit = "idle";
  } pitch_velocity_;
  roll_yaw::PitchPulse pitch_pulse_;
  struct LocalPitch
  {
    bool valid = false;
    Eigen::Matrix3d start = Eigen::Matrix3d::Identity();
    double angle = 0, rate = 0, offset = 0, rate_limit = 0, acceleration_limit = 0;
  } local_pitch_;
  bool local_interaction_hold_ = false, local_resisting_load_ = false;
  bool admitPitchPulse(double rate, double duration, std::uint64_t sequence);
  double body_pitch_velocity_command_duration_ = 8.0;
  bool admitPitchJog(double);
  void finishPitchJog(const std::string& phase, const std::string& why);
  std::string preparationTrackingRejection(const roll_yaw::Feedback&, const roll_yaw::Reference&) const;
  std::string preparationRejection(const roll_yaw::Feedback&, const roll_yaw::Reference&) const;
  void advancePreparedConnection();
  struct PitchJog
  {
    bool active = false;
    std::uint64_t session = 0;
    Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
    Eigen::Matrix3d endpoint = Eigen::Matrix3d::Identity();
    double delta = 0, progress = 0;
    std::string phase = "idle";
  } jog_;
  struct JogAdmission
  {
    bool ok = false;
    std::uint64_t session = 0;
    Eigen::Vector3d goal = Eigen::Vector3d::Zero();
    std::string reason;
  };
  std::unique_ptr<roll_yaw::Worker<JogAdmission>> jog_worker_;
  bool jog_pending_ = false;
  double body_pitch_jog_max_delta_ = 0.0174532925199433, jog_admission_wall_ = 0;
  bool connection_pending_ = false;
  double connection_wait_started_ = 0;
  roll_yaw::Request alignment_request_;
  bool body_alignment_valid_ = false;
  void mechanismTargetCallback(const std_msgs::Float64ConstPtr&, int axis);
  void manualCallback(const geometry_msgs::Vector3StampedConstPtr&);
  bool setNominalTarget(const Eigen::Vector3d&, int changed_axis = -1);
  bool buildFinalMechanismTarget(roll_yaw::Reference&, double now);
  bool advanceManualReference(roll_yaw::Reference&, double dt);
  roll_yaw::PlanningConfig onlineLimits() const;
  bool candidateTrackingAllowed(const roll_yaw::Reference&, const roll_yaw::PlanningConfig&, double dt, std::string&) const;
  Eigen::Vector3d feed_scale_ = Eigen::Vector3d::Ones();
  Eigen::Vector3d measured_coordinates_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d contact_rotation_residual_ = Eigen::Vector3d::Zero();
  double contact_hold_since_ = 0;
  void commandRollYawBaselinkPose(const roll_yaw::Reference&);
  void publishRollYawJointTarget();
  void invalidatePlanner();
  void stopMotion(const std::string&);
  void completeStop(const std::string&);
  void faultHold(const std::string&);
  void requestRollYaw(const std::string&, std::uint64_t, int axis = -1, double value = 0,
                     const roll_yaw::Request* prepared_request = nullptr);
  void publishRollYawDiagnostics();

  // Multilink lifecycle, adapted from pitch+secondary to passive alpha+roll+yaw.
  bool lock_valid_ = false, faulted_ = false;
  roll_yaw::Kinematics model_;
  std::shared_ptr<const roll_yaw::Kinematics> planning_model_;
  Eigen::Vector3d nominal_ = Eigen::Vector3d::Zero();  // independent accepted destinations
  bool manual_tracking_ = false, tracking_hold_ = false;
  enum class HoldReason { NONE, FEEDBACK, CONTACT, CLOCK, TRACKING, FEASIBILITY };
  HoldReason hold_reason_ = HoldReason::NONE;
  double recovery_body_stamp_ = 0, recovery_joint_stamp_ = 0;
  double manual_time_ = 0, manual_wall_ = 0;
  bool stop_pending_ = false, feedback_waiting_ = false, retarget_pending_ = false;
  roll_yaw::SettlingWindow preparation_window_;
  roll_yaw::SettlingStatistics preparation_stats_;
  bool preparation_ready_ = false;
  double preparation_clock_ = 0;
  roll_yaw::Reference final_target_;  // sole applied FK/controller reference
  roll_yaw::Estimate measured_contact_;
  std::string reason_ = "free flight", previous_failure_;
  ros::Time roll_yaw_lock_stamp_, manual_receipt_stamp_;

  enum class RollYawPitchRezeroState
  {
    IDLE,
    ACTIVE,
    HOLD
  };
  void rollYawPitchRezeroCallback(const std_msgs::EmptyConstPtr&);
  void resetRollYawPitchRezeroState();
  void updateRollYawBodyPitchFeedback();
  void updateRollYawPitchRezeroCommand();
  bool verifyRollYawPitchRezeroFrozenJoints();
  void applyRollYawPitchRezeroTarget();
  void failRollYawPitchRezero(const std::string&);
  RollYawPitchRezeroState roll_yaw_pitch_rezero_state_ = RollYawPitchRezeroState::IDLE;
  bool roll_yaw_pitch_rezero_ready_ = false, roll_yaw_pitch_rezero_failed_ = false, rezero_configuration_valid_ = false;
  bool target_applied_this_update_ = false;
  double rezero_pitch_ = 0, rezero_kp_ = 1, rezero_sign_ = -1, rezero_rate_ = .1745329, rezero_travel_ = .5235988;
  double rezero_tolerance_ = .0349066, rezero_rate_tolerance_ = .05;
  double rezero_stable_duration_ = .3, rezero_timeout_ = 5;
  double rezero_roll_tolerance_ = .02, rezero_yaw_tolerance_ = .02;
  double rezero_started_ = 0, rezero_stable_since_ = 0, rezero_alpha_ = 0;
  double rezero_previous_pitch_ = 0, rezero_previous_time_ = 0, rezero_measured_rate_ = 0, roll_yaw_rezero_feedback_dt_ = 0;
  bool rezero_rate_valid_ = false;
  Eigen::Vector3d rezero_frozen_ = Eigen::Vector3d::Zero();

  // Offline orientation paths share the manual FK/controller output.
  bool planning_configuration_valid_ = false;
  roll_yaw::PlanningConfig planning_config_;
  roll_yaw::ExecutionConfig feedback_config_;
  std::unique_ptr<roll_yaw::Executor> planner_;
  std::unique_ptr<roll_yaw::Worker<roll_yaw::Plan>> prepare_worker_;
  bool prepare_pending_ = false;
  std::uint64_t prepare_cancellations_ = 0;
  std::string prepare_result_ = "not requested";
  double prepare_started_ = 0, prepare_deadline_ = 120, prepare_wall_ = 0;
  struct PlanningStatus
  {
    std::atomic<int> interval{0}, total{0};
    std::atomic<bool> validating{false};
    std::atomic<double> solve_seconds{0}, validation_seconds{0};
  };
  static roll_yaw::Plan prepareTrajectory(const roll_yaw::Kinematics& model_snapshot, Eigen::Vector3d initial,
                                          const Eigen::Matrix3d& neutral, const roll_yaw::Request& request,
                                          const roll_yaw::Reference& measured_start,
                                          const roll_yaw::Reference& connection_start, roll_yaw::PlanningConfig& config,
                                          std::uint64_t session, const std::shared_ptr<PlanningStatus>& status,
                                          double deadline, double consistency, const std::function<bool()>& cancel);
  std::shared_ptr<PlanningStatus> planning_status_ = std::make_shared<PlanningStatus>();
  std::uint64_t producer_generation_ = 0, pending_generation_ = 0;
  Eigen::Matrix3d neutral_ = Eigen::Matrix3d::Identity();

  std::function<bool(const roll_yaw::Kinematics&, perching_dynamics::MultilinkModel&, std::string&)> dynamics_factory_;
  std::function<bool()> dynamics_snapshot_current_;
  // Rotor gimbals move physically; only the reduced model keeps their lock configuration.
  std::set<std::string> rotor_gimbal_joints_;
  bool last_command_accepted_ = false;
  double control_pitch_ = 0;
  std::uint64_t plan_id_ = 0;
  double body_receipt_ = 0, joint_receipt_ = 0;
  std::weak_ptr<aerial_robot_control::RollYawInteraction> interaction_;
  aerial_robot_control::RollYawInteractionInput interactionInput(const roll_yaw::Feedback&) const;
  void resetCompliance();
  void resetArticulatedSession(const std::string&);
  void updateInteraction(const roll_yaw::Feedback&);
  bool articulatedLoadValid(const roll_yaw::Feedback&) const;
  ros::Publisher status_pub_, task_pub_, compliance_pub_, ack_pub_;
  std::string last_status_;
  double last_diagnostics_ = 0, navigation_wall_ = 0;

  // Callback-owned ingestion. No planner, FK, observer or publication under
  // this mutex. Rates use consecutive producer samples, not control snapshots.
  struct RollYawInputs
  {
    roll_yaw::Feedback measured_;
    Eigen::Vector3d joint_bootstrap_ = Eigen::Vector3d::Zero();
    double joint_bootstrap_stamp_ = 0;
    double body_receipt_ = 0, joint_receipt_ = 0;
    std::string joint_failure_ = "no joint feedback", body_failure_ = "no body feedback";
    std::uint64_t joint_ignored_stamps_ = 0, body_ignored_stamps_ = 0,
                  joint_invalid_messages_ = 0, body_invalid_messages_ = 0;
    double last_joint_message_stamp_ = 0, last_body_message_stamp_ = 0;
    bool joint_valid_ = false, body_valid_ = false, clock_reset = false;
    sensor_msgs::JointState joints;
    std::string producer;
    double joint_wall = 0;
    double joint_callback_wait = 0, body_callback_wait = 0, joint_queue_age = 0;
    std::uint64_t joint_sequence = 0;
  } inputs_;
  mutable std::mutex input_mutex_;
  void captureRollYawInputs();
  struct PendingCommand
  {
    std::uint64_t sequence;
    std::string key;
    std::uint64_t session;
    bool urgent;
    std::function<void()> apply;
  };
  std::deque<PendingCommand> pending_commands_;
  std::map<std::string, PendingCommand> pending_observations_;
  std::atomic<std::uint64_t> input_session_{0};
  std::uint64_t input_sequence_ = 0, urgent_sequence_ = 0, calculation_sequence_ = 0;
  std::uint64_t consumed_joint_sequence_ = 0, coalesced_joint_samples_ = 0;
  bool deferRollYawCommand(const std::string&, std::function<void()>, bool urgent = false,
                          const gimbalrotor::RollYawMotion* motion = nullptr);
  void drainRollYawCommands();
  void cancelQueuedRollYawCommands(const std::string&, bool all = false);
  std::uint64_t dispatch_sequence_ = 0;
  double input_lock_wait_ = 0, input_lock_held_ = 0;
  double joint_callback_wait_ = 0, body_callback_wait_ = 0, joint_queue_age_ = 0;
  double reference_wall_ = 0, observer_wall_ = 0, diagnostic_wall_ = 0, publication_wall_ = 0;
  double completed_stamp_ = 0, publication_stamp_ = 0, publication_period_ = 0;
  std::uint64_t completed_updates_ = 0, overruns_ = 0, preempted_updates_ = 0;
  int subdivisions_ = 0;
  roll_yaw::WorkCounters completed_work_;
  double completed_reference_wall_ = 0, completed_observer_wall_ = 0, completed_diagnostic_wall_ = 0;
  double completed_publication_wall_ = 0, completed_lock_wait_ = 0, completed_lock_held_ = 0;
  double completed_state_lock_wait_ = 0, completed_state_lock_held_ = 0;
  int completed_subdivisions_ = 0;
  Eigen::Vector3d tracking_error_ = Eigen::Vector3d::Zero(), braking_reserve_ = Eigen::Vector3d::Zero();
  double body_tracking_error_ = 0;
  bool snapshot_feedback_valid_ = false, commit_feedback_valid_ = false;
  bool defer_diagnostics_ = false;
  std_msgs::String pending_reference_ack_;
  // Fixed-field comparison is retained: the base model has public module mutators
  // without a change generation. Check at snapshot/commit, without map copies.
  struct ModelCheck
  {
    bool valid = true, available = true;
    std::string category, name, captured, current;
    double difference = 0;
    std::string describe() const
    {
      std::ostringstream out;
      out << category << ":" << name << " captured=" << captured << " current=" << current
          << " difference=" << std::setprecision(17) << difference;
      return out.str();
    }
  } model_check_;
  std::function<ModelCheck()> dynamics_snapshot_check_;
  bool dynamics_snapshot_valid_ = false;

  roll_yaw::Feedback measured_;
  Eigen::Isometry3d cp_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d axis_ = Eigen::Vector3d::Zero();
  std::string roll_, yaw_, contact_, roll_yaw_prefix_, world_, body_frame_;
  std::string joint_failure_ = "no joint feedback", body_failure_ = "no body feedback";
  std::uint64_t joint_ignored_stamps_ = 0, body_ignored_stamps_ = 0, joint_invalid_messages_ = 0,
                body_invalid_messages_ = 0;
  double last_joint_message_stamp_ = 0, last_body_message_stamp_ = 0;
  bool configuration_valid_ = false, joint_valid_ = false, body_valid_ = false;
  double synchronization_tolerance_ = .05;
  std::vector<ros::Subscriber> subscriptions_;
  ros::Publisher roll_yaw_joints_pub_, roll_yaw_state_pub_, session_pub_, reference_pub_, measured_pub_, cog_pub_, details_pub_;
  ros::Publisher angles_pub_, nominal_pub_, rezero_active_pub_, rezero_ready_pub_, rezero_failed_pub_, pitch_pub_;
  ros::Publisher position_lower_pub_, position_upper_pub_;
};

}  // namespace aerial_robot_navigation
