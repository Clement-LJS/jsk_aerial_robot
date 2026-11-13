// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/gimbalrotor_navigation.h>
#include <gimbalrotor/gimbalrotor_perching.h>
#include <gimbalrotor/model/gimbalrotor_multilink_perching_model.h>
#include <gimbalrotor/control/gimbalrotor_multilink_perching_observer.h>
#include <cmath>
#include <mutex>
#include <string>
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
#include <vector>
#include <kdl/frames.hpp>
#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <limits>

namespace aerial_robot_navigation
{

class GimbalrotorPerchingNavigator : public GimbalrotorNavigator, public perching_geometry::TargetProvider
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

  // All selected workflows share contact-session serialization.
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
  bool observeInteractionSample(const roll_yaw::Feedback&, double stamp, bool quiet);
  bool snapshotModel(roll_yaw::Kinematics&, std::string&);
  roll_yaw::Feedback feedback() const;
  std::string contactRejection(const roll_yaw::Feedback&) const;
  void rollYawJointCallback(const sensor_msgs::JointStateConstPtr&);
  void bodyCallback(const nav_msgs::OdometryConstPtr&);
  void bodyAlignmentCallback(const std_msgs::Float64ConstPtr&, int axis);
  void bodyPitchCallback(const std_msgs::Float64ConstPtr&);
  bool admitPitchJog(double);
  void finishPitchJog(const std::string& phase, const std::string& why);
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
  double body_pitch_jog_max_delta_ = 0.0174532925199433, jog_admission_wall_ = 0;
  bool connection_pending_ = false;
  double connection_wait_started_ = 0;
  roll_yaw::Request alignment_request_;
  bool body_alignment_valid_ = false;
  void mechanismTargetCallback(const std_msgs::Float64ConstPtr&, int axis);
  void manualCallback(const geometry_msgs::Vector3StampedConstPtr&);
  bool setNominalTarget(const Eigen::Vector3d&, int changed_axis = -1);
  bool buildFinalMechanismTarget(roll_yaw::Reference&);
  void commandRollYawBaselinkPose(const roll_yaw::Reference&);
  void publishRollYawJointTarget();
  void invalidatePlanner();
  void stopMotion(const std::string&);
  void completeStop(const std::string&);
  void faultHold(const std::string&);
  void requestRollYaw(const std::string&, std::uint64_t, int axis = -1, double value = 0);
  void publishRollYawDiagnostics();

  // Multilink lifecycle, adapted from pitch+secondary to passive alpha+roll+yaw.
  bool lock_valid_ = false, faulted_ = false;
  roll_yaw::Kinematics model_;
  std::shared_ptr<const roll_yaw::Kinematics> planning_model_;
  Eigen::Vector3d nominal_ = Eigen::Vector3d::Zero();  // independent accepted destinations
  roll_yaw::Path manual_path_;
  double manual_time_ = 0, manual_phase_ = 0, manual_speed_ = 1, manual_wall_ = 0;
  bool stop_pending_ = false, feedback_waiting_ = false;
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

  roll_yaw::PitchAdmittance admittance_;
  // Source is selected at startup or a disarmed READY tare, never mid-motion.
  enum class InteractionSource { LEGACY_WRENCH, ARTICULATED_MOMENTUM, INVALID };
  InteractionSource interaction_source_ = InteractionSource::LEGACY_WRENCH;
  std::string interaction_source_name_ = "legacy_wrench", effort_contract_ = "unavailable";
  double observer_gain_ = 10;
  perching_dynamics::MultilinkModel articulated_model_;
  perching_dynamics::MomentumObserver momentum_observer_;
  bool articulated_model_valid_ = false;
  std::string articulated_reason_ = "no locked dynamics snapshot";
  std::function<bool(const roll_yaw::Kinematics&, perching_dynamics::MultilinkModel&, std::string&)> dynamics_factory_;
  std::function<bool()> dynamics_snapshot_current_;
  std::map<std::string, double> frozen_joints_;
  struct DynamicsJoint
  {
    double position = 0, rate = 0, effort = std::numeric_limits<double>::quiet_NaN();
    double stamp = 0, receipt = 0;
    bool rate_valid = false;
  };
  std::map<std::string, DynamicsJoint> dynamics_joints_;
  geometry_msgs::WrenchStamped command_wrench_;
  ros::Subscriber command_wrench_sub_;
  double command_wrench_receipt_ = 0, articulated_joint_stamp_ = 0, articulated_command_stamp_ = 0;
  void loadInteractionSource();
  bool interactionConfigurationValid() const;
  void captureDynamicsJoints(const sensor_msgs::JointState&);
  void commandWrenchCallback(const geometry_msgs::WrenchStampedConstPtr&);
  bool observeArticulatedSample(const roll_yaw::Feedback&, double direction);
  void invalidateArticulated(const std::string&);
  bool last_command_accepted_ = false;
  double control_pitch_ = 0;
  std::uint64_t plan_id_ = 0;
  std::string admittance_reason_ = "disarmed";
  double body_receipt_ = 0, joint_receipt_ = 0;
  void admittanceCallback(const std_msgs::BoolConstPtr&);
  void externalWrenchCallback(const geometry_msgs::WrenchStampedConstPtr&);
  void tareCallback(const std_msgs::EmptyConstPtr&);
  void updateInteraction(const roll_yaw::Feedback&);
  bool interaction_configured_ = false, tare_requested_ = false, interaction_valid_ = false;
  std::string interaction_frame_, interaction_point_, interaction_topic_;
  double interaction_sign_ = 0, interaction_scale_ = 0, interaction_torque_ = 0, interaction_stamp_ = 0,
         interaction_receipt_ = 0;
  double compliance_time_ = 0;
  geometry_msgs::WrenchStamped external_wrench_;
  void resetCompliance();
  ros::Publisher status_pub_, task_pub_, compliance_pub_, ack_pub_;
  std::string last_status_;
  double last_diagnostics_ = 0, navigation_wall_ = 0;

  roll_yaw::Feedback measured_;
  Eigen::Vector3d joint_bootstrap_ = Eigen::Vector3d::Zero();
  double joint_bootstrap_stamp_ = 0;
  Eigen::Isometry3d cp_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d axis_ = Eigen::Vector3d::Zero();
  std::string roll_, yaw_, contact_, roll_yaw_prefix_, world_, body_frame_;
  std::string joint_failure_ = "no joint feedback", body_failure_ = "no body feedback";
  std::uint64_t joint_ignored_stamps_ = 0, body_ignored_stamps_ = 0, joint_invalid_messages_ = 0,
                body_invalid_messages_ = 0;
  double last_joint_message_stamp_ = 0, last_body_message_stamp_ = 0;
  bool configuration_valid_ = false, joint_valid_ = false, body_valid_ = false;
  double synchronization_tolerance_ = .05, cog_model_tolerance_ = .005;
  std::vector<ros::Subscriber> subscriptions_;
  ros::Publisher roll_yaw_joints_pub_, roll_yaw_state_pub_, session_pub_, reference_pub_, measured_pub_, cog_pub_, details_pub_;
  ros::Publisher angles_pub_, nominal_pub_, rezero_active_pub_, rezero_ready_pub_, rezero_failed_pub_, pitch_pub_;
  ros::Publisher position_lower_pub_, position_upper_pub_;
};

}  // namespace aerial_robot_navigation
