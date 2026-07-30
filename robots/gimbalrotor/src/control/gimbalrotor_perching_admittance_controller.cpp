#include <gimbalrotor/control/gimbalrotor_perching_admittance_controller.h>

#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>

namespace
{
const double PI = 3.14159265358979323846;
}

namespace aerial_robot_control
{
GimbalrotorPerchingAdmittanceController::GimbalrotorPerchingAdmittanceController()
  : GimbalrotorAdmittanceController(),
    perching_enable_topic_for_constraint_("perching/enable"),
    perching_admittance_enable_topic_("perching/admittance_enable"),
    perching_point_topic_("perching/point"),
    perching_branch_pose_topic_("perching/branch_pose"),
    perching_locked_pose_topic_("perching/locked_pose"),
    perching_locked_pivot_topic_("perching/locked_pivot"),
    normal_admittance_enabled_(false),
    perching_admittance_enabled_(false),
    effective_admittance_enabled_(false),
    normal_admittance_reset_requested_(false),
    legacy_admittance_reset_requested_(false),
    perching_enabled_for_constraint_(false),
    has_perching_point_(false),
    has_branch_pose_(false),
    has_locked_pose_msg_(false),
    has_locked_pivot_(false),
    has_locked_pose_(false),
    use_branch_pose_if_no_point_(false),
    require_perching_lock_(true),
    min_valid_radius_(0.05),
    max_pitch_delta_(0.78539816339),
    arc_pitch_sign_(1.0),
    locked_radius_(0.0),
    locked_x_side_(1.0),
    maximum_lock_stamp_difference_(0.05),
    equilibrium_wrench_required_samples_(80),
    equilibrium_wrench_sample_count_(0),
    equilibrium_wrench_ready_(false),
    normal_pitch_i_limit_(0.0),
    pitch_i_limit_suppressed_(false),
    hybrid_enabled_cfg_(false),
    cutting_active_topic_("perching/cutting_active"),
    require_cutting_active_(true),
    cutting_active_(false),
    contact_filter_time_constant_(0.03),
    admittance_filter_time_constant_(0.08),
    contact_on_sigma_multiplier_(5.0),
    contact_off_sigma_multiplier_(2.5),
    contact_on_min_torque_(0.03),
    contact_off_min_torque_(0.015),
    contact_on_duration_(0.15),
    contact_off_duration_(0.30),
    authority_ramp_up_time_(0.30),
    relief_pitch_sign_(-1.0),
    penetration_torque_ratio_(0.0),
    directional_deadband_sigma_multiplier_(2.0),
    directional_deadband_min_torque_(0.01),
    recovery_angle_epsilon_(0.002),
    recovery_rate_epsilon_(0.01),
    hard_residual_pivot_torque_limit_(0.27),
    max_hybrid_control_dt_(0.10),
    recontact_on_duration_(0.15),
    max_tare_torque_stddev_(0.02),
    hold_relief_target_on_measurement_fault_(true),
    abort_request_topic_("perching/hybrid/abort_request"),
    hybrid_state_(HYBRID_DISABLED),
    hybrid_contact_(false),
    hybrid_fault_(false),
    tau_fast_(0.0),
    tau_slow_(0.0),
    contact_on_timer_(0.0),
    contact_off_timer_(0.0),
    recontact_on_timer_(0.0),
    authority_alpha_(0.0),
    hybrid_equilibrium_wrench_sample_count_(0),
    hybrid_tare_ready_(false),
    hybrid_tare_collection_active_(false),
    hybrid_tare_lock_generation_(0),
    active_admittance_mode_(AdmittanceOperatingMode::NONE)
{
  perching_point_world_.setValue(0.0, 0.0, 0.0);
  branch_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_rpy_.setValue(0.0, 0.0, 0.0);
  locked_pivot_world_.setValue(0.0, 0.0, 0.0);
  locked_radius_vec_world_.setValue(0.0, 0.0, 0.0);

  locked_pose_stamp_ = ros::Time(0);
  locked_pivot_stamp_ = ros::Time(0);

  accepted_locked_pose_stamp_ = ros::Time(0);
  accepted_locked_pivot_stamp_ = ros::Time(0);
  lock_generation_ = 0;

  R_world_constraint_.setIdentity();

  constraint_axis_world_ = Eigen::Vector3d::UnitY();

  equilibrium_wrench_pivot_world_.setZero();
  equilibrium_wrench_sum_.setZero();

  prev_hybrid_time_ = ros::Time(0);
  last_valid_hybrid_control_time_ = ros::Time(0);

  supervised_wrench_world_.setZero();

  hybrid_equilibrium_wrench_sum_.setZero();
  hybrid_equilibrium_wrench_pivot_world_.setZero();
  has_cutting_active_message_ = false;
  last_cutting_active_receive_time_ = ros::Time(0);
  hybrid_arm_command_ = false;
  hybrid_disarm_request_ = false;
  tare_reset_pending_ = false;
  hard_reset_request_ = HARD_RESET_NONE;
  pending_normal_enable_valid_ = false;
  pending_normal_enable_value_ = false;
  pending_perching_arm_valid_ = false;
  pending_perching_arm_value_ = false;
  pending_cutting_active_valid_ = false;
  pending_cutting_active_value_ = false;
  hybrid_fault_code_ = HYBRID_FAULT_NONE;
  last_safe_output_valid_ = false;
}

void GimbalrotorPerchingAdmittanceController::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate)
{
  /*
   * First initialize the normal gimbalrotor admittance controller.
   *
   * This preserves normal admittance behavior.
   * This subclass only changes target injection while perching is active.
   */
  GimbalrotorAdmittanceController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  perchingRosParamInit();
  const AdmittanceCoreConfig sanitized_base_config = admittance_core_.getConfig();
  normal_admittance_config_ = sanitized_base_config;
  legacy_perching_admittance_config_ = sanitized_base_config;
  hybrid_perching_admittance_config_ = sanitized_base_config;
  hybridRosParamInit();

  /*
   * Save the normal pitch integral-output limit.
   * With the current GimbalrotorControl.yaml, this should normally be 10.0.
   */
  normal_pitch_i_limit_ = pid_controllers_.at(PITCH).getLimitI();

  ROS_WARN("[GimbalrotorPerchingAdmittanceController] Normal pitch I limit: %.6f", normal_pitch_i_limit_);

  /*
   * The base admittance controller subscribed to admittance_enable_topic_.
   * For this mode-aware controller, replace that subscription with our own normal/perching trigger separation.
   */
  admittance_enable_sub_.shutdown();

  normal_admittance_enable_sub_ = nh_.subscribe(admittance_enable_topic_, 1, &GimbalrotorPerchingAdmittanceController::normalAdmittanceEnableCallback, this);
  perching_admittance_enable_sub_ = nh_.subscribe(perching_admittance_enable_topic_, 1, &GimbalrotorPerchingAdmittanceController::perchingAdmittanceEnableCallback, this);
  perching_enable_sub_for_constraint_ = nh_.subscribe(perching_enable_topic_for_constraint_, 1, &GimbalrotorPerchingAdmittanceController::perchingEnableCallback, this);
  perching_point_sub_ = nh_.subscribe(perching_point_topic_, 1, &GimbalrotorPerchingAdmittanceController::perchingPointCallback, this);
  branch_pose_sub_ = nh_.subscribe(perching_branch_pose_topic_, 1, &GimbalrotorPerchingAdmittanceController::branchPoseCallback, this);

  /*
   * GimbalrotorPerchingNavigator publishes this as latched.
   * It contains:
   *   - locked robot position
   *   - locked robot RPY
   *
   * We use this to reconstruct the same pitch arc in the admittance controller.
   */
  locked_pose_sub_ = nh_.subscribe(perching_locked_pose_topic_, 1, &GimbalrotorPerchingAdmittanceController::lockedPoseCallback, this);
  locked_pivot_sub_ = nh_.subscribe(perching_locked_pivot_topic_, 1, &GimbalrotorPerchingAdmittanceController::lockedPivotCallback, this);

  /*
   * Hybrid contact-gated admittance interfaces (section 7 and 14 of the
   * spec). These are wired up unconditionally so that toggling
   * hybrid.enabled at runtime does not require re-launching the node.
   */
  cutting_active_sub_ = nh_.subscribe(cutting_active_topic_, 1, &GimbalrotorPerchingAdmittanceController::cuttingActiveCallback, this);

  hybrid_state_pub_ = nh_.advertise<std_msgs::UInt8>("perching/hybrid/state", 1);
  hybrid_contact_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/contact", 1);
  hybrid_armed_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/armed", 1);
  hybrid_disarm_requested_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/disarm_requested", 1);
  hybrid_authority_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/authority", 1);
  hybrid_torque_residual_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/pivot_torque_residual", 1);
  hybrid_torque_filtered_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/pivot_torque_filtered", 1);
  hybrid_cutting_active_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/cutting_active", 1);
  hybrid_cutting_signal_fresh_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/cutting_signal_fresh", 1);
  hybrid_cutting_gate_valid_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/cutting_gate_valid", 1);
  hybrid_tare_sample_count_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/tare_sample_count", 1);
  hybrid_tare_torque_stddev_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/tare_torque_stddev", 1);
  hybrid_tare_collection_active_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/tare_collection_active", 1);
  hybrid_pivot_torque_raw_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/pivot_torque_raw", 1);
  hybrid_pivot_torque_fast_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/pivot_torque_fast", 1);
  hybrid_pivot_torque_slow_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/pivot_torque_slow", 1);
  hybrid_contact_on_threshold_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/contact_on_threshold", 1);
  hybrid_contact_off_threshold_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/contact_off_threshold", 1);
  hybrid_directional_deadband_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/directional_deadband", 1);
  hybrid_directional_torque_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/directional_torque", 1);
  hybrid_effective_tau_input_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/effective_tau_input", 1);
  hybrid_pitch_offset_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/pitch_offset", 1);
  hybrid_pitch_offset_rate_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/pitch_offset_rate", 1);
  hybrid_pitch_offset_acceleration_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/pitch_offset_acceleration", 1);
  hybrid_nominal_pitch_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/nominal_pitch", 1);
  hybrid_modified_pitch_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/modified_pitch", 1);
  hybrid_dt_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/dt", 1);
  hybrid_dt_valid_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/dt_valid", 1);
  hybrid_wrench_fresh_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/wrench_fresh", 1);
  hybrid_lock_valid_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/lock_valid", 1);
  hybrid_lock_generation_pub_ = nh_.advertise<std_msgs::Float64>("perching/hybrid/lock_generation", 1);
  hybrid_snapshot_generation_valid_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/snapshot_generation_valid", 1);
  hybrid_tare_ready_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/tare_ready", 1);
  hybrid_pid_i_frozen_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/pid_i_frozen", 1);
  hybrid_fault_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/fault", 1);
  hybrid_fault_code_pub_ = nh_.advertise<std_msgs::UInt8>("perching/hybrid/fault_code", 1);
  hybrid_fault_reason_pub_ = nh_.advertise<std_msgs::String>("perching/hybrid/fault_reason", 1);
  hybrid_fault_holds_target_pub_ = nh_.advertise<std_msgs::Bool>("perching/hybrid/fault_holds_target", 1);
  hybrid_abort_request_pub_ = nh_.advertise<std_msgs::Bool>(abort_request_topic_, 1);

  prev_hybrid_time_ = ros::Time::now();

  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] initialized.");
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] perching_enable_topic: " << perching_enable_topic_for_constraint_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] perching_point_topic: " << perching_point_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] branch_pose_topic: " << perching_branch_pose_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] locked_pose_topic: " << perching_locked_pose_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] cutting_active_topic: " << cutting_active_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] hybrid.enabled: " << hybrid_enabled_cfg_);
}

void GimbalrotorPerchingAdmittanceController::reset()
{
  /*
   * Restore the normal pitch integral limit before resetting
   * the parent controller.
   */
  if(pitch_i_limit_suppressed_ && pid_controllers_.size() > PITCH)
  {
    PID& pitch_pid = pid_controllers_.at(PITCH);
    pitch_pid.setLimitI(normal_pitch_i_limit_);
    pitch_pid.setErrI(0.0);
    pitch_i_limit_suppressed_ = false;
  }

  /*
   * Unfreezing here is safe even if hybrid mode was never active:
   * setIntegratorFrozen(false) on an already-unfrozen PID is a no-op.
   */
  if(pid_controllers_.size() > PITCH)
  {
    pid_controllers_.at(PITCH).setIntegratorFrozen(false);
  }

  GimbalrotorAdmittanceController::reset();

  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  normal_admittance_enabled_ = false;
  perching_admittance_enabled_ = false;
  effective_admittance_enabled_ = false;
  normal_admittance_reset_requested_ = false;
  legacy_admittance_reset_requested_ = false;

  perching_enabled_for_constraint_ = false;

  has_perching_point_ = false;
  has_branch_pose_ = false;
  has_locked_pose_msg_ = false;
  has_locked_pivot_ = false;
  has_locked_pose_ = false;

  perching_point_world_.setValue(0.0, 0.0, 0.0);
  branch_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_rpy_.setValue(0.0, 0.0, 0.0);
  locked_pivot_world_.setValue(0.0, 0.0, 0.0);
  locked_radius_vec_world_.setValue(0.0, 0.0, 0.0);

  locked_radius_ = 0.0;
  locked_x_side_ = 1.0;

  locked_pose_stamp_ = ros::Time(0);
  locked_pivot_stamp_ = ros::Time(0);
  accepted_locked_pose_stamp_ = ros::Time(0);
  accepted_locked_pivot_stamp_ = ros::Time(0);
  lock_generation_ = 0;

  R_world_constraint_.setIdentity();

  constraint_axis_world_ = Eigen::Vector3d::UnitY();

  resetEquilibriumWrenchUnsafe();

  cutting_active_ = false;
  has_cutting_active_message_ = false;
  last_cutting_active_receive_time_ = ros::Time(0);
  resetHybridTareUnsafe();
  hybrid_arm_command_ = false;
  hybrid_disarm_request_ = false;
  tare_reset_pending_ = false;
  requestHardResetUnsafe(HARD_RESET_CONTROLLER_RESET);
  pending_normal_enable_valid_ = false;
  pending_perching_arm_valid_ = false;
  pending_cutting_active_valid_ = false;

  ROS_WARN("[GimbalrotorPerchingAdmittanceController] Perching admittance state reset.");

  resetHybridDynamicsUnsafe();
}

void GimbalrotorPerchingAdmittanceController::perchingRosParamInit()
{
  ros::NodeHandle imp_perch_nh(nh_, "controller/admittance/perching");

  /*
   * Defaults are intentionally the same topic names used by GimbalrotorPerchingNavigator.
   */
  getParam<std::string>(imp_perch_nh, "perching_enable_topic", perching_enable_topic_for_constraint_, std::string("perching/enable"));
  getParam<std::string>(imp_perch_nh, "perching_point_topic", perching_point_topic_, std::string("perching/point"));
  getParam<std::string>(imp_perch_nh, "perching_branch_pose_topic", perching_branch_pose_topic_, std::string("perching/branch_pose"));
  getParam<std::string>(imp_perch_nh, "perching_locked_pose_topic", perching_locked_pose_topic_, std::string("perching/locked_pose"));
  getParam<bool>(imp_perch_nh, "use_branch_pose_if_no_point", use_branch_pose_if_no_point_, false);
  getParam<bool>(imp_perch_nh, "require_perching_lock", require_perching_lock_, true);

  /*
   * These must match the perching navigator parameters.
   *
   * If they are not explicitly set under controller/admittance/perching, read the existing navigation parameters.
   */
  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "perching_min_valid_radius", min_valid_radius_, 0.05);
  getParam<double>(navi_nh, "perching_max_pitch_delta", max_pitch_delta_, 0.78539816339);
  getParam<double>(navi_nh, "perching_arc_pitch_sign", arc_pitch_sign_, 1.0);
  getParam<double>(imp_perch_nh, "perching_min_valid_radius", min_valid_radius_, min_valid_radius_);
  getParam<double>(imp_perch_nh, "perching_max_pitch_delta", max_pitch_delta_, max_pitch_delta_);
  getParam<double>(imp_perch_nh, "perching_arc_pitch_sign", arc_pitch_sign_, arc_pitch_sign_);

  if(!std::isfinite(min_valid_radius_) || min_valid_radius_ <= 0.0)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid perching_min_valid_radius %.6f. "
        "Using 0.05 m.",
        min_valid_radius_);

    min_valid_radius_ = 0.05;
  }

  if(!std::isfinite(max_pitch_delta_) || max_pitch_delta_ <= 0.0 || max_pitch_delta_ > PI)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid perching_max_pitch_delta %.6f. "
        "Using 45 degrees.",
        max_pitch_delta_);

    max_pitch_delta_ = PI / 4.0;
  }

  if(!std::isfinite(arc_pitch_sign_) || std::abs(arc_pitch_sign_) < 1.0e-9)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid perching_arc_pitch_sign %.6f. "
        "Using +1.",
        arc_pitch_sign_);

    arc_pitch_sign_ = 1.0;
  }
  else
  {
    arc_pitch_sign_ = arc_pitch_sign_ > 0.0 ? 1.0 : -1.0;
  }

  getParam<std::string>(imp_perch_nh, "perching_admittance_enable_topic", perching_admittance_enable_topic_, std::string("perching/admittance_enable"));
  getParam<std::string>(imp_perch_nh, "perching_locked_pivot_topic", perching_locked_pivot_topic_, std::string("perching/locked_pivot"));
  getParam<double>(imp_perch_nh, "maximum_lock_stamp_difference", maximum_lock_stamp_difference_, 0.05);

  if(!std::isfinite(maximum_lock_stamp_difference_) || maximum_lock_stamp_difference_ <= 0.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid maximum_lock_stamp_difference %.6f. "
        "Using 0.05 seconds.",
        maximum_lock_stamp_difference_);

    maximum_lock_stamp_difference_ = 0.05;
  }

  getParam<int>(imp_perch_nh, "equilibrium_wrench_required_samples", equilibrium_wrench_required_samples_, 80);

  if(equilibrium_wrench_required_samples_ < 1)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid equilibrium_wrench_required_samples: %d. "
        "Using 80 samples.",
        equilibrium_wrench_required_samples_);

    equilibrium_wrench_required_samples_ = 80;
  }
  ROS_WARN("[GimbalrotorPerchingAdmittanceController] equilibrium_wrench_required_samples: %d", equilibrium_wrench_required_samples_);
  ROS_WARN("[GimbalrotorPerchingAdmittanceController] perching_min_valid_radius: %.4f", min_valid_radius_);
  ROS_WARN("[GimbalrotorPerchingAdmittanceController] perching_max_pitch_delta deg: %.2f", max_pitch_delta_ * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingAdmittanceController] perching_arc_pitch_sign: %.2f", arc_pitch_sign_);
}

void GimbalrotorPerchingAdmittanceController::hybridRosParamInit()
{
  ros::NodeHandle hybrid_nh(nh_, "controller/admittance/perching/hybrid");

  getParam<bool>(hybrid_nh, "enabled", hybrid_enabled_cfg_, false);

  getParam<std::string>(hybrid_nh, "cutting_active_topic", cutting_active_topic_, std::string("perching/cutting_active"));

  getParam<bool>(hybrid_nh, "require_cutting_active", require_cutting_active_, true);

  getParam<double>(hybrid_nh, "contact_filter_time_constant", contact_filter_time_constant_, 0.03);
  getParam<double>(hybrid_nh, "admittance_filter_time_constant", admittance_filter_time_constant_, 0.08);

  getParam<double>(hybrid_nh, "contact_on_sigma_multiplier", contact_on_sigma_multiplier_, 5.0);
  getParam<double>(hybrid_nh, "contact_off_sigma_multiplier", contact_off_sigma_multiplier_, 2.5);
  getParam<double>(hybrid_nh, "contact_on_min_torque", contact_on_min_torque_, 0.03);
  getParam<double>(hybrid_nh, "contact_off_min_torque", contact_off_min_torque_, 0.015);
  getParam<double>(hybrid_nh, "contact_on_duration", contact_on_duration_, 0.15);
  getParam<double>(hybrid_nh, "contact_off_duration", contact_off_duration_, 0.30);
  getParam<double>(hybrid_nh, "recontact_on_duration", recontact_on_duration_, 0.15);

  getParam<double>(hybrid_nh, "authority_ramp_up_time", authority_ramp_up_time_, 0.30);

  getParam<double>(hybrid_nh, "relief_pitch_sign", relief_pitch_sign_, -1.0);
  getParam<double>(hybrid_nh, "penetration_torque_ratio", penetration_torque_ratio_, 0.0);
  getParam<double>(hybrid_nh, "directional_deadband_sigma_multiplier", directional_deadband_sigma_multiplier_, 2.0);
  getParam<double>(hybrid_nh, "directional_deadband_min_torque", directional_deadband_min_torque_, 0.01);

  getParam<double>(hybrid_nh, "recovery_angle_epsilon", recovery_angle_epsilon_, 0.002);
  getParam<double>(hybrid_nh, "recovery_rate_epsilon", recovery_rate_epsilon_, 0.01);

  getParam<double>(hybrid_nh, "hard_residual_pivot_torque_limit", hard_residual_pivot_torque_limit_, 0.27);
  getParam<double>(hybrid_nh, "cutting_active_timeout", cutting_active_timeout_, 0.30);
  getParam<double>(hybrid_nh, "max_control_dt", max_hybrid_control_dt_, 0.10);
  getParam<double>(hybrid_nh, "max_tare_torque_stddev", max_tare_torque_stddev_, 0.02);
  getParam<std::string>(hybrid_nh, "abort_request_topic", abort_request_topic_, std::string("perching/hybrid/abort_request"));
  getParam<bool>(hybrid_nh, "hold_relief_target_on_measurement_fault", hold_relief_target_on_measurement_fault_, true);

  /*
   * relief_pitch_sign only ever means +1 or -1 (spec section 10.2).
   * Snap anything else to the nearest allowed value instead of silently
   * accepting an arbitrary magnitude.
   */
  if(!std::isfinite(relief_pitch_sign_) || std::abs(relief_pitch_sign_) < 1.0e-9)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid relief_pitch_sign %.6f. "
        "Disabling hybrid mode.",
        relief_pitch_sign_);

    hybrid_enabled_cfg_ = false;
    relief_pitch_sign_ = -1.0;
  }
  else
  {
    relief_pitch_sign_ = relief_pitch_sign_ > 0.0 ? 1.0 : -1.0;
  }

  /*
   * Only the perching pitch axis uses one-sided admittance bounds.
   * All other axes keep the symmetric default from AdmittanceCore.
   *
   * relief_pitch_sign = +1 -> allowed offset in [0, +limit]
   * relief_pitch_sign = -1 -> allowed offset in [-limit, 0]
   */
  const double pitch_limit = std::abs(hybrid_perching_admittance_config_.angle_offset_limit(1));

  if(relief_pitch_sign_ > 0.0)
  {
    hybrid_perching_admittance_config_.rot_angle_min(1) = 0.0;
    hybrid_perching_admittance_config_.rot_angle_max(1) = pitch_limit;
  }
  else
  {
    hybrid_perching_admittance_config_.rot_angle_min(1) = -pitch_limit;
    hybrid_perching_admittance_config_.rot_angle_max(1) = 0.0;
  }

  hybrid_perching_admittance_config_.rot_cancel_velocity_at_bound(1) = 1.0;
  hybrid_perching_admittance_config_.force_lpf_alpha = 1.0;
  hybrid_perching_admittance_config_.torque_lpf_alpha = 1.0;

  /*
  * Use the same timing limit in both the hybrid supervisor and the shared AdmittanceCore.
  */
  hybrid_perching_admittance_config_.max_dt = max_hybrid_control_dt_;

  if(!validateHybridParameters())
  {
    hybrid_enabled_cfg_ = false;
  }

  switchAdmittanceMode(AdmittanceOperatingMode::NORMAL_FLIGHT);

  ROS_WARN("[GimbalrotorPerchingAdmittanceController] hybrid.enabled: %d", static_cast<int>(hybrid_enabled_cfg_));
  ROS_WARN("[GimbalrotorPerchingAdmittanceController] relief_pitch_sign: %.1f", relief_pitch_sign_);
  ROS_WARN("[GimbalrotorPerchingAdmittanceController] require_cutting_active: %d", static_cast<int>(require_cutting_active_));
  ROS_WARN("[GimbalrotorPerchingAdmittanceController] hard_residual_pivot_torque_limit: %.4f", hard_residual_pivot_torque_limit_);
}

bool GimbalrotorPerchingAdmittanceController::validateHybridParameters()
{
  bool valid = true;

  const auto require = [&valid](bool condition, const char* name, double value)
  {
    if(!condition)
    {
      ROS_ERROR(
          "[GimbalrotorPerchingAdmittanceController] "
          "Invalid hybrid %s: %.6f",
          name,
          value);

      valid = false;
    }
  };

  require(
      std::isfinite(contact_filter_time_constant_) &&
      contact_filter_time_constant_ > 0.0,
      "contact_filter_time_constant",
      contact_filter_time_constant_);

  require(
      std::isfinite(admittance_filter_time_constant_) &&
      admittance_filter_time_constant_ > 0.0,
      "admittance_filter_time_constant",
      admittance_filter_time_constant_);

  require(
      std::isfinite(contact_on_sigma_multiplier_) &&
      std::isfinite(contact_off_sigma_multiplier_) &&
      contact_on_sigma_multiplier_ > contact_off_sigma_multiplier_,
      "contact_on_sigma_multiplier",
      contact_on_sigma_multiplier_);

  require(
      std::isfinite(contact_off_sigma_multiplier_) &&
      contact_off_sigma_multiplier_ >= 0.0,
      "contact_off_sigma_multiplier",
      contact_off_sigma_multiplier_);

  require(
      std::isfinite(contact_on_min_torque_) &&
      std::isfinite(contact_off_min_torque_) &&
      contact_on_min_torque_ > contact_off_min_torque_,
      "contact_on_min_torque",
      contact_on_min_torque_);

  require(
      std::isfinite(contact_off_min_torque_) &&
      contact_off_min_torque_ >= 0.0,
      "contact_off_min_torque",
      contact_off_min_torque_);

  require(
      std::isfinite(contact_on_duration_) &&
      contact_on_duration_ >= 0.0,
      "contact_on_duration",
      contact_on_duration_);

  require(
      std::isfinite(contact_off_duration_) &&
      contact_off_duration_ >= 0.0,
      "contact_off_duration",
      contact_off_duration_);

  require(
      std::isfinite(recontact_on_duration_) &&
      recontact_on_duration_ >= 0.0,
      "recontact_on_duration",
      recontact_on_duration_);

  require(
      std::isfinite(authority_ramp_up_time_) &&
      authority_ramp_up_time_ >= 0.0,
      "authority_ramp_up_time",
      authority_ramp_up_time_);

  require(
      std::isfinite(relief_pitch_sign_) &&
      std::abs(std::abs(relief_pitch_sign_) - 1.0) < 1.0e-9,
      "relief_pitch_sign",
      relief_pitch_sign_);

  require(
      std::isfinite(penetration_torque_ratio_) &&
      penetration_torque_ratio_ >= 0.0 &&
      penetration_torque_ratio_ <= 1.0,
      "penetration_torque_ratio",
      penetration_torque_ratio_);

  require(
      std::isfinite(directional_deadband_sigma_multiplier_) &&
      directional_deadband_sigma_multiplier_ >= 0.0,
      "directional_deadband_sigma_multiplier",
      directional_deadband_sigma_multiplier_);

  require(
      std::isfinite(directional_deadband_min_torque_) &&
      directional_deadband_min_torque_ >= 0.0,
      "directional_deadband_min_torque",
      directional_deadband_min_torque_);

  require(
      std::isfinite(recovery_angle_epsilon_) &&
      recovery_angle_epsilon_ >= 0.0,
      "recovery_angle_epsilon",
      recovery_angle_epsilon_);

  require(
      std::isfinite(recovery_rate_epsilon_) &&
      recovery_rate_epsilon_ >= 0.0,
      "recovery_rate_epsilon",
      recovery_rate_epsilon_);

  const double configured_torque_limit = hybrid_perching_admittance_config_.torque_limit(1);

  require(
      std::isfinite(configured_torque_limit) &&
      configured_torque_limit >= 0.0,
      "configured_admittance_torque_limit",
      configured_torque_limit);

  require(
      std::isfinite(hard_residual_pivot_torque_limit_) &&
      hard_residual_pivot_torque_limit_ > configured_torque_limit,
      "hard_residual_pivot_torque_limit",
      hard_residual_pivot_torque_limit_);

  require(
      std::isfinite(cutting_active_timeout_) &&
      cutting_active_timeout_ > 0.0,
      "cutting_active_timeout",
      cutting_active_timeout_);

  require(
      std::isfinite(max_hybrid_control_dt_) &&
      max_hybrid_control_dt_ > 0.0 &&
      max_hybrid_control_dt_ <= 0.1,
      "max_control_dt",
      max_hybrid_control_dt_);

  require(
      std::isfinite(max_tare_torque_stddev_) &&
      max_tare_torque_stddev_ > 0.0,
      "max_tare_torque_stddev",
      max_tare_torque_stddev_);

  const double configured_pitch_limit = hybrid_perching_admittance_config_.angle_offset_limit(1);

  require(
      std::isfinite(configured_pitch_limit) &&
      configured_pitch_limit > 0.0,
      "configured_pitch_offset_limit",
      configured_pitch_limit);

  if(cutting_active_topic_.empty())
  {
    ROS_ERROR("[GimbalrotorPerchingAdmittanceController] cutting_active_topic must not be empty.");
    valid = false;
  }

  if(abort_request_topic_.empty())
  {
    ROS_ERROR("[GimbalrotorPerchingAdmittanceController] abort_request_topic must not be empty.");
    valid = false;
  }

  if(!valid)
  {
    ROS_ERROR("[GimbalrotorPerchingAdmittanceController] Invalid hybrid parameter set. Disabling hybrid mode.");
  }

  return valid;
}

void GimbalrotorPerchingAdmittanceController::resetEquilibriumWrenchUnsafe() const
{
  /* The caller must already hold perching_state_mutex_. */
  equilibrium_wrench_pivot_world_.setZero();
  equilibrium_wrench_sum_.setZero();

  equilibrium_wrench_sample_count_ = 0;
  equilibrium_wrench_ready_ = false;
}

void GimbalrotorPerchingAdmittanceController::resetHybridTareUnsafe()
{
  /* The caller must already hold perching_state_mutex_. */
  hybrid_tare_stats_.reset();
  hybrid_equilibrium_wrench_sum_.setZero();
  hybrid_equilibrium_wrench_pivot_world_.setZero();
  hybrid_equilibrium_wrench_sample_count_ = 0;
  hybrid_tare_ready_ = false;
  hybrid_tare_collection_active_ = false;
  hybrid_tare_lock_generation_ = 0;
}

void GimbalrotorPerchingAdmittanceController::resetHybridTare()
{
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    resetHybridTareUnsafe();
  }

  /*
   * A new tare defines a new interaction session.
   * Do not allow a fault in the new session to reuse a target generated during an earlier cutting session.
   */
  last_safe_admittance_output_ = AdmittanceCoreOutput();

  last_safe_output_valid_ = false;
}

void GimbalrotorPerchingAdmittanceController::requestHardResetUnsafe(HybridHardResetReason reason)
{
  const auto priority = [](HybridHardResetReason reset_reason)
      {
        switch(reset_reason)
        {
          case HARD_RESET_CONTROLLER_RESET:
            return 50;

          case HARD_RESET_PERCHING_DISABLED:
            return 40;

          case HARD_RESET_INVALID_GEOMETRY:
            return 30;

          case HARD_RESET_INVALID_LOCK:
            return 20;

          case HARD_RESET_NEW_LOCK:
            return 10;

          case HARD_RESET_NONE:
          default:
            return 0;
        }
      };

  if(reason == HARD_RESET_NONE)
  {
    return;
  }

  /*
   * Do not allow a later normal new-lock notification to erase an already pending invalid-lock or invalid-geometry condition.
   *
   * Explicit controller reset and explicit perching disable remain higher-priority operator actions.
   */
  if(priority(reason) >= priority(hard_reset_request_))
  {
    hard_reset_request_ = reason;
  }
}

void GimbalrotorPerchingAdmittanceController::resetHybridDynamicsUnsafe()
{
  /* Control-thread-only fields; safe to touch without perching_state_mutex_. */
  hybrid_state_ = HYBRID_DISABLED;
  hybrid_contact_ = false;
  hybrid_fault_ = false;
  hybrid_fault_reason_.clear();
  hybrid_fault_code_ = HYBRID_FAULT_NONE;

  tau_fast_ = 0.0;
  tau_slow_ = 0.0;

  contact_on_timer_ = 0.0;
  contact_off_timer_ = 0.0;
  recontact_on_timer_ = 0.0;

  authority_alpha_ = 0.0;
  tare_reset_pending_ = false;

  supervised_wrench_world_.setZero();

  admittance_core_.reset();
  admittance_output_ = AdmittanceCoreOutput();

  prev_hybrid_time_ = ros::Time::now();

  if(pid_controllers_.size() > PITCH)
  {
    pid_controllers_.at(PITCH).setIntegratorFrozen(false);
  }
}

void GimbalrotorPerchingAdmittanceController::controlCore()
{
  /*
   * Capture one internally consistent geometry snapshot.
   *
   * Do not rebuild the complete snapshot after processing global
   * transitions. A relock callback could arrive between the transition
   * processing and a second snapshot, allowing unprocessed new geometry
   * to be used for one control cycle.
   */
  PerchingSnapshot snapshot = makePerchingSnapshot();

  processPendingGlobalTransitions(snapshot);

  /*
   * Global-transition processing may clear the arm and disarm states.
   *
   * Refresh only those command fields while preserving the original
   * pivot, constraint frame and lock generation. If the geometry changes
   * after the original snapshot, updatePivotWrenchAndTare() will detect
   * the generation mismatch and enter FAULT during active compliance.
   */
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    snapshot.arm_command = hybrid_arm_command_;

    snapshot.disarm_requested = hybrid_disarm_request_;
  }

  if(!snapshot.perching_active)
  {
    runNormalFlightAdmittance(snapshot);
    return;
  }

  if(!hybrid_enabled_cfg_)
  {
    runLegacyPerchingAdmittance(snapshot);
    return;
  }

  runHybridPerchingAdmittance(snapshot);
}

GimbalrotorPerchingAdmittanceController::PerchingSnapshot
GimbalrotorPerchingAdmittanceController::makePerchingSnapshot()
{
  PerchingSnapshot snapshot;

  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  consumePendingCommandsUnsafe(snapshot);

  snapshot.perching_active = perching_enabled_for_constraint_;
  snapshot.arm_command = hybrid_arm_command_;
  snapshot.disarm_requested = hybrid_disarm_request_;
  snapshot.cutting_active = cutting_active_;
  snapshot.cutting_signal_fresh =
      has_cutting_active_message_ &&
      std::isfinite((ros::Time::now() - last_cutting_active_receive_time_).toSec()) &&
      (ros::Time::now() - last_cutting_active_receive_time_).toSec() >= 0.0 &&
      (ros::Time::now() - last_cutting_active_receive_time_).toSec() <= cutting_active_timeout_;
  
  snapshot.pivot_world = Eigen::Vector3d(locked_pivot_world_.x(), locked_pivot_world_.y(), locked_pivot_world_.z());
  snapshot.locked_robot_pos_world = Eigen::Vector3d(locked_robot_pos_world_.x(), locked_robot_pos_world_.y(), locked_robot_pos_world_.z());
  snapshot.locked_robot_rpy = Eigen::Vector3d(locked_robot_rpy_.x(), locked_robot_rpy_.y(), locked_robot_rpy_.z());
  snapshot.locked_radius_vec_world = Eigen::Vector3d(locked_radius_vec_world_.x(), locked_radius_vec_world_.y(), locked_radius_vec_world_.z());
  snapshot.constraint_axis_world = constraint_axis_world_;
  snapshot.R_world_constraint = R_world_constraint_;
  snapshot.locked_radius = locked_radius_;
  snapshot.locked_x_side = locked_x_side_;
  snapshot.lock_generation = lock_generation_;
  snapshot.lock_valid = has_locked_pose_ && validateSnapshotGeometry(snapshot);

  return snapshot;
}

void GimbalrotorPerchingAdmittanceController::consumePendingCommandsUnsafe(PerchingSnapshot& snapshot)
{
  if(pending_normal_enable_valid_)
  {
    if(normal_admittance_enabled_ != pending_normal_enable_value_)
    {
      normal_admittance_enabled_ = pending_normal_enable_value_;
      normal_admittance_reset_requested_ = true;
    }

    pending_normal_enable_valid_ = false;
  }

  if(pending_perching_arm_valid_)
  {
    if(pending_perching_arm_value_)
    {
      hybrid_arm_command_ = true;
      hybrid_disarm_request_ = false;
      perching_admittance_enabled_ = true;
    }
    else
    {
      hybrid_arm_command_ = false;
      hybrid_disarm_request_ = true;
      perching_admittance_enabled_ = false;
    }

    pending_perching_arm_valid_ = false;
  }

  if(pending_cutting_active_valid_)
  {
    cutting_active_ = pending_cutting_active_value_;
    pending_cutting_active_valid_ = false;
  }

  snapshot.arm_command = hybrid_arm_command_;
  snapshot.disarm_requested = hybrid_disarm_request_;
  snapshot.cutting_active = cutting_active_;
}

void GimbalrotorPerchingAdmittanceController::processPendingGlobalTransitions(const PerchingSnapshot& snapshot)
{
  HybridHardResetReason pending_reset = HARD_RESET_NONE;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    pending_reset = hard_reset_request_;
    hard_reset_request_ = HARD_RESET_NONE;
  }

  if(pending_reset != HARD_RESET_NONE)
  {
    const HybridState previous_state = hybrid_state_;

    const bool active_or_recovering = previous_state == COMPLIANT_CUTTING || previous_state == RECOVERY;

    /*
     * Any lock/geometry replacement during active compliance is a discontinuity. Do not silently reset and continue.
     */
    if(active_or_recovering)
    {
      if(pending_reset == HARD_RESET_INVALID_LOCK)
      {
        enterFault(
            HYBRID_FAULT_LOCK_LOST,
            "Perching lock lost during active compliance/recovery");

        return;
      }

      if(pending_reset == HARD_RESET_INVALID_GEOMETRY)
      {
        enterFault(
            HYBRID_FAULT_INVALID_GEOMETRY,
            "Perching geometry became invalid during active compliance/recovery");

        return;
      }

      if(pending_reset == HARD_RESET_NEW_LOCK)
      {
        enterFault(
            HYBRID_FAULT_INVALID_GEOMETRY,
            "Perching lock changed during active compliance/recovery");

        return;
      }
    }

    /*
     * A FAULT is latched. A normal relock or geometry message may
     * clear stale tare and arm state, but it must not clear the
     * fault itself.
     *
     * Only explicit controller reset or perching disable clears the fault.
     */
    if(previous_state == FAULT && pending_reset != HARD_RESET_CONTROLLER_RESET && pending_reset != HARD_RESET_PERCHING_DISABLED)
    {
      resetHybridTare();

      last_safe_admittance_output_ = AdmittanceCoreOutput();

      last_safe_output_valid_ = false;

      {
        std::lock_guard<std::mutex> lock(perching_state_mutex_);

        hybrid_arm_command_ = false;
        hybrid_disarm_request_ = false;
        perching_admittance_enabled_ = false;
        tare_reset_pending_ = false;
      }

      return;
    }

    performHybridHardReset(pending_reset);

    resetHybridTare();
  }

  if(!snapshot.perching_active)
  {
    restorePitchPidForNormalFlight();
  }
}

void GimbalrotorPerchingAdmittanceController::restorePitchPidForNormalFlight()
{
  if(pid_controllers_.size() <= PITCH)
  {
    return;
  }

  PID& pitch_pid = pid_controllers_.at(PITCH);
  pitch_pid.setIntegratorFrozen(false);

  if(pitch_i_limit_suppressed_)
  {
    pitch_pid.setLimitI(normal_pitch_i_limit_);
    pitch_pid.setErrI(0.0);
    pitch_i_limit_suppressed_ = false;
  }
}

void GimbalrotorPerchingAdmittanceController::performHybridHardReset(HybridHardResetReason reason)
{
  switch(reason)
  {
    case HARD_RESET_CONTROLLER_RESET:
    case HARD_RESET_PERCHING_DISABLED:
    case HARD_RESET_NEW_LOCK:
    case HARD_RESET_INVALID_LOCK:
    case HARD_RESET_INVALID_GEOMETRY:
      break;
    case HARD_RESET_NONE:
    default:
      return;
  }

  resetHybridDynamicsUnsafe();

  last_safe_admittance_output_ = AdmittanceCoreOutput();
  last_safe_output_valid_ = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    hybrid_arm_command_ = false;
    hybrid_disarm_request_ = false;
    perching_admittance_enabled_ = false;
    tare_reset_pending_ = false;
  }

  hybrid_fault_ = false;
  hybrid_fault_code_ = HYBRID_FAULT_NONE;
  hybrid_fault_reason_.clear();

  prev_hybrid_time_ = ros::Time::now();
  last_valid_hybrid_control_time_ = ros::Time(0);
}

void GimbalrotorPerchingAdmittanceController::switchAdmittanceMode(AdmittanceOperatingMode requested_mode)
{
  if(active_admittance_mode_ == requested_mode)
  {
    return;
  }

  switch(requested_mode)
  {
    case AdmittanceOperatingMode::NORMAL_FLIGHT:
      admittance_core_.setConfig(normal_admittance_config_);
      break;
    case AdmittanceOperatingMode::LEGACY_PERCHING:
      admittance_core_.setConfig(legacy_perching_admittance_config_);
      break;
    case AdmittanceOperatingMode::HYBRID_PERCHING:
      admittance_core_.setConfig(hybrid_perching_admittance_config_);
      break;
    case AdmittanceOperatingMode::NONE:
    default:
      break;
  }

  admittance_core_.reset();
  admittance_output_ = AdmittanceCoreOutput();
  active_admittance_mode_ = requested_mode;
}

void GimbalrotorPerchingAdmittanceController::runNormalFlightAdmittance(const PerchingSnapshot& snapshot)
{
  (void)snapshot;
  switchAdmittanceMode(AdmittanceOperatingMode::NORMAL_FLIGHT);
  admittance_enabled_ = normal_admittance_enabled_;

  bool reset_requested = false;
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    reset_requested = normal_admittance_reset_requested_;
    normal_admittance_reset_requested_ = false;
  }

  if(reset_requested)
  {
    admittance_core_.reset();
    admittance_output_ = AdmittanceCoreOutput();
    prev_admittance_time_ = ros::Time::now();
  }

  GimbalrotorAdmittanceController::controlCore();
}

void GimbalrotorPerchingAdmittanceController::runLegacyPerchingAdmittance(const PerchingSnapshot& snapshot)
{
  (void)snapshot;
  switchAdmittanceMode(AdmittanceOperatingMode::LEGACY_PERCHING);
  legacyControlCore();
}

void GimbalrotorPerchingAdmittanceController::runHybridPerchingAdmittance(const PerchingSnapshot& snapshot)
{
  switchAdmittanceMode(AdmittanceOperatingMode::HYBRID_PERCHING);
  hybridControlCore(snapshot);
}

bool GimbalrotorPerchingAdmittanceController::runInnerLoopWithAdmittanceOverride(const PerchingSnapshot* snapshot, const AdmittanceCoreOutput* output_override)
{
  const tf::Vector3 original_target_pos = navigator_->getTargetPos();
  const tf::Vector3 original_target_rpy = navigator_->getTargetRPY();

  bool output_applied = false;

  if(output_override != nullptr && output_override->valid)
  {
    if(snapshot != nullptr && snapshot->perching_active)
    {
      output_applied = applyHybridAdmittanceOutputToNavigator(*snapshot, original_target_pos, original_target_rpy, *output_override);
    }
    else
    {
      applyAdmittanceOutputToNavigator(original_target_pos, original_target_rpy, *output_override);

      output_applied = true;
    }
  }

  /*
   * The pose PID and allocation must execute regardless of whether
   * the target override was accepted.
   */
  GimbalrotorController::controlCore();

  if(output_applied)
  {
    navigator_->setTargetPos(original_target_pos);
    navigator_->setTargetRPY(original_target_rpy);
  }

  return output_applied;
}

void GimbalrotorPerchingAdmittanceController::legacyControlCore()
{
  bool effective_enabled = false;
  bool reset_requested = false;

  /*
   * Suppress the ROS pitch integral term only while perching admittance is actually enabled.
   *
   * Perching mode by itself does not suppress the integral.
   */
  bool suppress_pitch_i_limit = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    suppress_pitch_i_limit = perching_enabled_for_constraint_ && perching_admittance_enabled_;

    if(perching_enabled_for_constraint_)
    {
      effective_enabled = perching_admittance_enabled_;
    }
    else
    {
      effective_enabled = normal_admittance_enabled_;
    }

    /*
     * Reset admittance when effective enable changes.
     */
    if(effective_enabled != effective_admittance_enabled_)
    {
      reset_requested = true;
    }

    effective_admittance_enabled_ = effective_enabled;

    /*
     * Reset when the perching mode, lock, pivot or constraint frame changes.
     */
    if(legacy_admittance_reset_requested_)
    {
      reset_requested = true;
      legacy_admittance_reset_requested_ = false;
    }
  }

  PID& pitch_pid = pid_controllers_.at(PITCH);

  if(suppress_pitch_i_limit)
  {
    /*
     * This runs once when perching admittance becomes enabled.
     */
    if(!pitch_i_limit_suppressed_)
    {
      /*
       * Save the current normal limit before changing it.
       *
       * Normally this is the YAML value 10.0.
       */
      normal_pitch_i_limit_ = pitch_pid.getLimitI();

      pitch_i_limit_suppressed_ = true;

      ROS_WARN("[GimbalrotorPerchingAdmittanceController] Perching admittance enabled: pitch limit_i changed from %.6f to 0.0.", normal_pitch_i_limit_);
    }

    /*
     * Force the integral output limit to zero.
     */
    pitch_pid.setLimitI(0.0);

    /*
     * limit_i = 0 only forces the resulting I term to zero.
     * The stored integral error would otherwise continue accumulating, so clear it before PID::update().
     */
    pitch_pid.setErrI(0.0);
  }
  else if(pitch_i_limit_suppressed_)
  {
    /*
     * Perching admittance was disabled.
     *
     * Restore the normal limit and restart the integrator from zero.
     */
    pitch_pid.setErrI(0.0);
    pitch_pid.setLimitI(normal_pitch_i_limit_);
    pitch_i_limit_suppressed_ = false;

    ROS_WARN("[GimbalrotorPerchingAdmittanceController] Perching admittance disabled: pitch limit_i restored to %.6f.", normal_pitch_i_limit_);
  }

  /*
   * This assignment remains in the main control thread.
   */
  admittance_enabled_ = effective_enabled;

  if(reset_requested)
  {
    admittance_core_.reset();
    admittance_output_ = AdmittanceCoreOutput();

    prev_admittance_time_ = ros::Time::now();

    ROS_WARN("[GimbalrotorPerchingAdmittanceController] Admittance state reset.");
  }

  /*
   * This eventually calls PID::update().
   *
   * When perching is enabled, pitch limit_i is already zero, so the calculated pitch I term will be clamped to zero.
   */
  GimbalrotorAdmittanceController::controlCore();

  /*
   * PID::update() still integrates err_i even when limit_i is zero. Clear the stored error after the update so it cannot build up while perching.
   */
  if(suppress_pitch_i_limit)
  {
    pitch_pid.setErrI(0.0);
  }
}

void GimbalrotorPerchingAdmittanceController::hybridControlCore(const PerchingSnapshot& snapshot)
{
  const ros::Time now = ros::Time::now();

  const double dt = (now - prev_hybrid_time_).toSec();

  prev_hybrid_time_ = now;

  const PivotWrenchResult pivot = updatePivotWrenchAndTare(snapshot, now, dt);

  if(pivot.dt_valid)
  {
    updateTorqueFilters(pivot.residual_pivot_torque_axis, dt);
    updateHybridStateMachine(pivot, dt);
    updateAuthority(dt);
  }
  else
  {
    handleInvalidHybridTiming(pivot, dt);
  }

  const double tau_directional = computeDirectionalTorque();

  double tau_input = 0.0;

  if(hybrid_state_ == COMPLIANT_CUTTING)
  {
    tau_input = authority_alpha_ * tau_directional;
  }

  updatePitchIntegratorMode();

  supervised_wrench_world_.setZero();

  if(snapshot.constraint_axis_world.allFinite())
  {
    supervised_wrench_world_.tail<3>() = snapshot.constraint_axis_world * tau_input;
  }

  const bool core_enabled = hybrid_state_ == COMPLIANT_CUTTING || hybrid_state_ == RECOVERY;

  /*
   * Keep the parent wrapper's clock synchronized even though the
   * hybrid path updates AdmittanceCore directly.
   */
  prev_admittance_time_ = now;

  if(hybrid_state_ == FAULT)
  {
    admittance_enabled_ = false;
    supervised_wrench_world_.setZero();

    const AdmittanceCoreOutput* held_output =
        faultAllowsHeldTarget() &&
        snapshot.lock_valid &&
        pivot.snapshot_generation_valid &&
        validateSnapshotGeometry(snapshot) &&
        last_safe_output_valid_
            ? &last_safe_admittance_output_
            : nullptr;

    const bool target_actually_held = runInnerLoopWithAdmittanceOverride(&snapshot, held_output);

    publishHybridDiagnostics(
        pivot,
        tau_directional,
        0.0,
        target_actually_held
            ? held_output
            : nullptr,
        target_actually_held);

    return;
  }

  if(pivot.dt_valid && core_enabled)
  {
    AdmittanceCoreInput input;

    input.external_wrench_world = supervised_wrench_world_;

    input.R_world_compliance = snapshot.R_world_constraint;

    input.dt = dt;
    input.enabled = true;

    admittance_enabled_ = true;

    admittance_output_ = admittance_core_.update(input);
  }
  else
  {
    admittance_enabled_ = false;
    admittance_output_ = AdmittanceCoreOutput();
  }

  if(admittance_output_.valid && snapshot.lock_valid && pivot.snapshot_generation_valid)
  {
    last_safe_admittance_output_ = admittance_output_;

    last_safe_output_valid_ = true;
  }

  const bool target_authorized = admittance_output_.valid && (hybrid_state_ == COMPLIANT_CUTTING || hybrid_state_ == RECOVERY);

  const AdmittanceCoreOutput* requested_output = target_authorized ? &admittance_output_ : nullptr;

  const bool target_applied = runInnerLoopWithAdmittanceOverride(&snapshot, requested_output);

  if(pivot.dt_valid)
  {
    last_valid_hybrid_control_time_ = now;
  }

  publishHybridDiagnostics(
      pivot,
      tau_directional,
      tau_input,
      target_applied
          ? requested_output
          : nullptr,
      false);
}

void GimbalrotorPerchingAdmittanceController::handleInvalidHybridTiming(const PivotWrenchResult& pivot, double dt)
{
  (void)pivot;

  const bool active_or_recovering = hybrid_state_ == COMPLIANT_CUTTING || hybrid_state_ == RECOVERY;

  if(active_or_recovering)
  {
    enterFault(
        HYBRID_FAULT_CONTROL_GAP,
        "Invalid hybrid control dt during active compliance/recovery");
    return;
  }

  if(hybrid_state_ == CONTACT_CANDIDATE)
  {
    hybrid_state_ = ARMED_PID;
  }

  hybrid_contact_ = false;
  contact_on_timer_ = 0.0;
  contact_off_timer_ = 0.0;
  recontact_on_timer_ = 0.0;

  ROS_WARN_THROTTLE(1.0, "[GimbalrotorPerchingAdmittanceController] Hybrid timing sample rejected: dt=%.6f", dt);
}

void GimbalrotorPerchingAdmittanceController::perchingEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  bool mode_changed = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    mode_changed = perching_enabled_for_constraint_ != msg->data;

    perching_enabled_for_constraint_ = msg->data;

    if(mode_changed)
    {
      resetEquilibriumWrenchUnsafe();
      legacy_admittance_reset_requested_ = true;
      requestHardResetUnsafe(msg->data ? HARD_RESET_NEW_LOCK : HARD_RESET_PERCHING_DISABLED);
    }
  }

  if(msg->data)
  {
    ROS_WARN_STREAM(
        "[GimbalrotorPerchingAdmittanceController] "
        "Perching navigation enabled. "
        "Collecting equilibrium wrench while admittance "
        "is disabled. Required samples: "
        << equilibrium_wrench_required_samples_);
  }
  else
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Perching navigation disabled. "
        "Equilibrium wrench has been cleared. "
        "Normal admittance trigger is active.");
  }
}

void GimbalrotorPerchingAdmittanceController::perchingPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  perching_point_world_.setValue(msg->point.x, msg->point.y, msg->point.z);

  has_perching_point_ = true;
}

void GimbalrotorPerchingAdmittanceController::branchPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  branch_pos_world_.setValue(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

  has_branch_pose_ = true;

  if(!has_perching_point_ && use_branch_pose_if_no_point_)
  {
    perching_point_world_ = branch_pos_world_;
  }
}

void GimbalrotorPerchingAdmittanceController::lockedPivotCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  if(!validateLockedPivotMessage(*msg))
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    has_locked_pivot_ = false;
    has_locked_pose_ = false;
    legacy_admittance_reset_requested_ = true;
    ++lock_generation_;
    requestHardResetUnsafe(HARD_RESET_INVALID_GEOMETRY);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    locked_pivot_world_.setValue(msg->point.x, msg->point.y, msg->point.z);

    if(msg->header.stamp.isZero())
    {
      locked_pivot_stamp_ = ros::Time::now();
    }
    else
    {
      locked_pivot_stamp_ = msg->header.stamp;
    }

    has_locked_pivot_ = true;
  }

  updateLockedConstraintFromLockedPoseAndPivot();
}

void GimbalrotorPerchingAdmittanceController::lockedPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if(!validateLockedPoseMessage(*msg))
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    has_locked_pose_msg_ = false;
    has_locked_pose_ = false;
    legacy_admittance_reset_requested_ = true;
    ++lock_generation_;
    requestHardResetUnsafe(HARD_RESET_INVALID_GEOMETRY);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    poseMsgToTfPosRpy(*msg, locked_robot_pos_world_, locked_robot_rpy_);

    /*
     * The navigator normally provides a valid stamp. Use callback time only as a fallback.
     */
    if(msg->header.stamp.isZero())
    {
      locked_pose_stamp_ = ros::Time::now();
    }
    else
    {
      locked_pose_stamp_ = msg->header.stamp;
    }

    has_locked_pose_msg_ = true;
  }

  /*
   * Do not hold the mutex before entering this function because it locks the same mutex itself.
   */
  updateLockedConstraintFromLockedPoseAndPivot();
}

void GimbalrotorPerchingAdmittanceController::cuttingActiveCallback(const std_msgs::Bool::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(perching_state_mutex_);
  pending_cutting_active_valid_ = true;
  pending_cutting_active_value_ = msg->data;
  has_cutting_active_message_ = true;
  last_cutting_active_receive_time_ = ros::Time::now();
}

void GimbalrotorPerchingAdmittanceController::updateLockedConstraintFromLockedPoseAndPivot()
{
  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  const bool previously_valid = has_locked_pose_;

  /*
   * A locked robot pose is always required.
   */
  if(!has_locked_pose_msg_)
  {
    has_locked_pose_ = false;

    if(previously_valid)
    {
      legacy_admittance_reset_requested_ = true;
      ++lock_generation_;
      requestHardResetUnsafe(HARD_RESET_INVALID_LOCK);
    }

    return;
  }

  /*
   * If require_perching_lock is enabled, accept only the pivot published by the navigator's lock logic.
   */
  if(!has_locked_pivot_)
  {
    if(require_perching_lock_)
    {
      has_locked_pose_ = false;

      if(previously_valid)
      {
        legacy_admittance_reset_requested_ = true;
        ++lock_generation_;
        requestHardResetUnsafe(HARD_RESET_INVALID_LOCK);
      }

      ROS_WARN_THROTTLE(1.0, "[GimbalrotorPerchingAdmittanceController] Waiting for locked pivot from navigator.");

      return;
    }

    /*
     * Fallback behavior is allowed only when require_perching_lock is false.
     */
    if(has_perching_point_)
    {
      locked_pivot_world_ = perching_point_world_;
    }
    else if(use_branch_pose_if_no_point_ && has_branch_pose_)
    {
      locked_pivot_world_ = branch_pos_world_;
    }
    else
    {
      has_locked_pose_ = false;

      if(previously_valid)
      {
        legacy_admittance_reset_requested_ = true;
        ++lock_generation_;
        requestHardResetUnsafe(HARD_RESET_INVALID_LOCK);
      }

      return;
    }
  }

  /*
   * When using navigator lock messages, ensure that pose and pivot belong to the same lock operation.
   */
  if(require_perching_lock_)
  {
    if(locked_pose_stamp_.isZero() || locked_pivot_stamp_.isZero())
    {
      has_locked_pose_ = false;

      if(previously_valid)
      {
        legacy_admittance_reset_requested_ = true;
        ++lock_generation_;
        requestHardResetUnsafe(HARD_RESET_INVALID_LOCK);
      }

      ROS_WARN_THROTTLE(1.0, "[GimbalrotorPerchingAdmittanceController] Locked pose or pivot has an invalid timestamp.");

      return;
    }

    const double stamp_difference = std::abs((locked_pose_stamp_ - locked_pivot_stamp_).toSec());

    if(stamp_difference > maximum_lock_stamp_difference_)
    {
      has_locked_pose_ = false;

      if(previously_valid)
      {
        legacy_admittance_reset_requested_ = true;
        ++lock_generation_;
        requestHardResetUnsafe(HARD_RESET_INVALID_LOCK);
      }

      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorPerchingAdmittanceController] "
          "Locked pose and pivot do not belong to "
          "the same lock operation. "
          "Timestamp difference: %.3f s.",
          stamp_difference);

      return;
    }
  }

  /*
   * Ignore duplicate delivery of the already accepted latched pose/pivot pair.
   */
  if(has_locked_pose_ && locked_pose_stamp_ == accepted_locked_pose_stamp_ && locked_pivot_stamp_ == accepted_locked_pivot_stamp_)
  {
    return;
  }

  perching_point_world_ = locked_pivot_world_;
  locked_radius_vec_world_ = locked_robot_pos_world_ - locked_pivot_world_;
  locked_radius_ = norm2D(locked_radius_vec_world_.x(), locked_radius_vec_world_.z());

  if(locked_robot_pos_world_.x() - locked_pivot_world_.x() >= 0.0)
  {
    locked_x_side_ = 1.0;
  }
  else
  {
    locked_x_side_ = -1.0;
  }

  if(!isFiniteTfVector(locked_robot_pos_world_) || !isFiniteTfVector(locked_pivot_world_) || !std::isfinite(locked_radius_) || locked_radius_ < min_valid_radius_)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
      legacy_admittance_reset_requested_ = true;
      ++lock_generation_;
      requestHardResetUnsafe(HARD_RESET_INVALID_GEOMETRY);
    }

    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Locked constraint is invalid: "
        "radius %.4f is too small.",
        locked_radius_);

    return;
  }

  /*
   * Minimum current implementation: physical branch axis is world Y.
   */
  constraint_axis_world_ = Eigen::Vector3d::UnitY();

  Eigen::Vector3d radial_world(locked_radius_vec_world_.x(), locked_radius_vec_world_.y(), locked_radius_vec_world_.z());

  /*
   * Remove the component parallel to the branch.
   */
  radial_world -= constraint_axis_world_ * constraint_axis_world_.dot(radial_world);

  const double radial_norm = radial_world.norm();

  if(!radial_world.allFinite() || !std::isfinite(radial_norm) || radial_norm < 1.0e-6)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
      legacy_admittance_reset_requested_ = true;
      ++lock_generation_;
      requestHardResetUnsafe(HARD_RESET_INVALID_GEOMETRY);
    }

    ROS_ERROR("[GimbalrotorPerchingAdmittanceController] Cannot build constraint frame: invalid radial vector.");

    return;
  }

  radial_world.normalize();

  /*
   * Use radial x branch axis so the resulting frame is right-handed:
   *
   * constraint X x constraint Y = constraint Z.
   */
  Eigen::Vector3d tangent_world = radial_world.cross(constraint_axis_world_);

  const double tangent_norm = tangent_world.norm();

  if(!tangent_world.allFinite() || !std::isfinite(tangent_norm) || tangent_norm < 1.0e-6)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
      legacy_admittance_reset_requested_ = true;
      ++lock_generation_;
      requestHardResetUnsafe(HARD_RESET_INVALID_GEOMETRY);
    }

    ROS_ERROR("[GimbalrotorPerchingAdmittanceController] Cannot build constraint frame: invalid tangent vector.");

    return;
  }

  tangent_world.normalize();

  /*
   * Columns transform constraint-frame vectors into world-frame vectors:
   *
   * constraint X = radial
   * constraint Y = branch axis
   * constraint Z = tangent
   */
  R_world_constraint_.col(0) = radial_world;
  R_world_constraint_.col(1) = constraint_axis_world_;
  R_world_constraint_.col(2) = tangent_world;

  const double determinant = R_world_constraint_.determinant();

  if(!isValidRotationMatrix(R_world_constraint_))
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
      legacy_admittance_reset_requested_ = true;
      ++lock_generation_;
      requestHardResetUnsafe(HARD_RESET_INVALID_GEOMETRY);
    }

    ROS_ERROR("[GimbalrotorPerchingAdmittanceController] Invalid constraint rotation determinant: %.6f", determinant);

    return;
  }


  has_locked_pose_ = true;
  accepted_locked_pose_stamp_ = locked_pose_stamp_;
  accepted_locked_pivot_stamp_ = locked_pivot_stamp_;
  ++lock_generation_;

  /*
  * A new lock means the pivot, radius, nominal pose or constraint frame may have changed.
  *
  * The previous equilibrium wrench must not be reused.
  */
  resetEquilibriumWrenchUnsafe();
  legacy_admittance_reset_requested_ = true;
  requestHardResetUnsafe(HARD_RESET_NEW_LOCK);

  ROS_WARN("[GimbalrotorPerchingAdmittanceController] Locked perching constraint accepted.");

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] Locked pivot: %.3f %.3f %.3f",
      locked_pivot_world_.x(),
      locked_pivot_world_.y(),
      locked_pivot_world_.z());

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] Locked position: %.3f %.3f %.3f",
      locked_robot_pos_world_.x(),
      locked_robot_pos_world_.y(),
      locked_robot_pos_world_.z());

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] Locked pitch: %.2f deg",
      locked_robot_rpy_.y() *
      180.0 / PI);

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] Locked radius: %.3f m",
      locked_radius_);

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] Constraint determinant: %.6f",
      determinant);
}

Eigen::Matrix3d GimbalrotorPerchingAdmittanceController::getComplianceToWorldRotation() const
{
  if(hybrid_enabled_cfg_)
  {
    return GimbalrotorAdmittanceController::getComplianceToWorldRotation();
  }

  bool use_constraint_frame = false;

  Eigen::Matrix3d R_world_constraint = Eigen::Matrix3d::Identity();

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    use_constraint_frame = perching_enabled_for_constraint_ && has_locked_pose_;
    R_world_constraint = R_world_constraint_;
  }

  if(use_constraint_frame)
  {
    return R_world_constraint;
  }

  return GimbalrotorAdmittanceController::getComplianceToWorldRotation();
}

void GimbalrotorPerchingAdmittanceController::perchingAdmittanceEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  bool enable_accepted = false;
  bool lock_valid = false;
  bool tare_ready = false;
  int collected_samples = 0;
  int required_samples = equilibrium_wrench_required_samples_;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    lock_valid = perching_enabled_for_constraint_ && has_locked_pose_;

    if(hybrid_enabled_cfg_)
    {
      tare_ready = hybrid_tare_ready_;
      collected_samples = hybrid_equilibrium_wrench_sample_count_;
    }
    else
    {
      tare_ready = equilibrium_wrench_ready_;
      collected_samples = equilibrium_wrench_sample_count_;
    }

    if(!msg->data)
    {
      if(hybrid_enabled_cfg_)
      {
        pending_perching_arm_valid_ = true;
        pending_perching_arm_value_ = false;
      }
      else
      {
        perching_admittance_enabled_ = false;
        resetEquilibriumWrenchUnsafe();
        legacy_admittance_reset_requested_ = true;
      }

      enable_accepted = true;
    }
    else if(!lock_valid)
    {
      /*
       * Never enable pivot admittance without a valid perching lock.
       */
      if(hybrid_enabled_cfg_)
      {
        pending_perching_arm_valid_ = true;
        pending_perching_arm_value_ = false;
      }
      else
      {
        perching_admittance_enabled_ = false;
      }
      enable_accepted = false;
    }
    else if(!tare_ready)
    {
      /*
       * Do not enable admittance before the no-contact equilibrium wrench has been collected.
       */
      if(hybrid_enabled_cfg_)
      {
        pending_perching_arm_valid_ = true;
        pending_perching_arm_value_ = false;
      }
      else
      {
        perching_admittance_enabled_ = false;
      }
      enable_accepted = false;
    }
    else
    {
      if(hybrid_enabled_cfg_)
      {
        pending_perching_arm_valid_ = true;
        pending_perching_arm_value_ = true;
      }
      else if(!perching_admittance_enabled_)
      {
        perching_admittance_enabled_ = true;
      }

      enable_accepted = true;
    }
  }

  if(!msg->data)
  {
    ROS_WARN("[GimbalrotorPerchingAdmittanceController] Perching admittance disarmed. Equilibrium-wrench collection restarted.");
  }
  else if(!lock_valid)
  {
    ROS_ERROR("[GimbalrotorPerchingAdmittanceController] Cannot enable perching admittance: no valid perching lock.");
  }
  else if(!tare_ready)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Cannot enable perching admittance: "
        "equilibrium wrench is not ready. "
        "Collected %d / %d samples. "
        "Keep the robot untouched and enable again after "
        "the ready message appears.",
        collected_samples, required_samples);
  }
  else if(enable_accepted)
  {
    if(hybrid_enabled_cfg_)
    {
      ROS_WARN("[GimbalrotorPerchingAdmittanceController] Hybrid supervisor armed (ARMED_PID). Pure PID continues until real cutting contact is confirmed.");
    }
    else
    {
      ROS_WARN("[GimbalrotorPerchingAdmittanceController] Perching admittance enabled using the collected equilibrium pivot wrench.");
    }
  }
}

void GimbalrotorPerchingAdmittanceController::normalAdmittanceEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    pending_normal_enable_valid_ = true;
    pending_normal_enable_value_ = msg->data;
  }

  ROS_WARN("[GimbalrotorPerchingAdmittanceController] normal_admittance_enabled: %d", static_cast<int>(msg->data));
}

Eigen::Matrix<double, 6, 1> GimbalrotorPerchingAdmittanceController::shiftWrenchToPivot(const Eigen::Matrix<double, 6, 1>& wrench_cog_world, const Eigen::Vector3d& cog_pos_world, const Eigen::Vector3d& pivot_pos_world) const
{
  const Eigen::Vector3d force_world = wrench_cog_world.head<3>();
  const Eigen::Vector3d torque_cog_world = wrench_cog_world.tail<3>();

  const Eigen::Vector3d pivot_to_cog_world = cog_pos_world - pivot_pos_world;

  /*
   * Shift the torque reference point:
   *
   * tau_pivot = tau_cog + (p_cog - p_pivot) x force
   */
  const Eigen::Vector3d torque_pivot_world = torque_cog_world + pivot_to_cog_world.cross(force_world);

  Eigen::Matrix<double, 6, 1> wrench_pivot_world = wrench_cog_world;
  wrench_pivot_world.tail<3>() = torque_pivot_world;

  return wrench_pivot_world;
}

double GimbalrotorPerchingAdmittanceController::projectTorqueOntoAxis(const Eigen::Matrix<double, 6, 1>& wrench_pivot_world, const Eigen::Matrix3d& R_world_constraint) const
{
  /*
   * tau_q = [R_WC^T * tau_pivot_world]_y
   *
   * Constraint Y is the branch/passive-joint axis (section 3.4).
   */
  const Eigen::Vector3d torque_world = wrench_pivot_world.tail<3>();
  const Eigen::Vector3d torque_constraint = R_world_constraint.transpose() * torque_world;

  return torque_constraint.y();
}

Eigen::Matrix<double, 6, 1> GimbalrotorPerchingAdmittanceController::getExternalWrenchWorld() const
{
  bool perching_active = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    perching_active = perching_enabled_for_constraint_;
  }

  if(!hybrid_enabled_cfg_)
  {
    return legacyGetExternalWrenchWorld();
  }

  if(!perching_active)
  {
    return GimbalrotorAdmittanceController::getExternalWrenchWorld();
  }

  /*
   * The hybrid path already prepared the supervised wrench in
   * hybridControlCore() via updatePivotWrenchAndTare() + the directional
   * gate, before GimbalrotorAdmittanceController::controlCore() calls
   * this getter. Returning it here (rather than recomputing) is what
   * avoids the duplicate-tare-update problem described in section 12.
   */
  return supervised_wrench_world_;
}

void GimbalrotorPerchingAdmittanceController::enterRecoveryFromDisarm()
{
  tare_reset_pending_ = true;
  hybrid_state_ = RECOVERY;
  hybrid_contact_ = false;
}

Eigen::Matrix<double, 6, 1> GimbalrotorPerchingAdmittanceController::legacyGetExternalWrenchWorld() const
{
  /*
   * Base-controller result:
   *
   * force: world coordinates
   * torque: world coordinates, moment about the COG
   */
  const Eigen::Matrix<double, 6, 1> wrench_cog_world = GimbalrotorAdmittanceController::getExternalWrenchWorld();

  /*
   * Do not build a tare from missing or stale wrench data.
   *
   * The base function returns zero for missing/stale data, but zero could also be a physically valid measurement.
   * Therefore, explicitly check the receive state here.
   */
  bool wrench_is_fresh = false;

  {
    std::lock_guard<std::mutex> lock(external_wrench_mutex_);

    if(has_external_wrench_)
    {
      const double wrench_age = (ros::Time::now() - last_external_wrench_receive_time_).toSec();

      wrench_is_fresh = wrench_age >= 0.0 && wrench_age <= external_wrench_timeout_;
    }
  }

  if(!wrench_is_fresh)
  {
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  bool perching_active = false;
  bool lock_valid = false;

  tf::Vector3 pivot_world_tf;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    perching_active = perching_enabled_for_constraint_;
    lock_valid = has_locked_pose_;
    pivot_world_tf = locked_pivot_world_;
  }

  /*
   * Normal-flight path: preserve the normal COG-referenced wrench.
   */
  if(!perching_active)
  {
    return wrench_cog_world;
  }

  /*
   * Never pass a COG-referenced wrench into the perching admittance calculation when the pivot lock is invalid.
   */
  if(!lock_valid)
  {
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  const tf::Vector3 cog_pos_world_tf = estimator_->getPos(Frame::COG, estimate_mode_);
  const Eigen::Vector3d cog_pos_world(cog_pos_world_tf.x(), cog_pos_world_tf.y(), cog_pos_world_tf.z());
  const Eigen::Vector3d pivot_pos_world(pivot_world_tf.x(), pivot_world_tf.y(), pivot_world_tf.z());
  const Eigen::Matrix<double, 6, 1> wrench_pivot_world = shiftWrenchToPivot(wrench_cog_world, cog_pos_world, pivot_pos_world);
  Eigen::Matrix<double, 6, 1> residual_wrench_pivot_world = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> accepted_equilibrium_wrench = Eigen::Matrix<double, 6, 1>::Zero();

  bool admittance_enabled_now = false;
  bool tare_ready_now = false;
  bool tare_completed_now = false;

  int collected_samples = 0;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    /*
     * Recheck the state because it may have changed while the wrench and current COG position were calculated.
     */
    if(!perching_enabled_for_constraint_ || !has_locked_pose_)
    {
      return Eigen::Matrix<double, 6, 1>::Zero();
    }

    admittance_enabled_now = perching_admittance_enabled_;

    /*
     * Collect the no-contact equilibrium only while perching admittance is disabled.
     *
     * During these samples:
     *   - do not touch the robot,
     *   - keep the nominal pitch fixed,
     *   - keep the robot in the intended steady pre-cut condition (the saw may be spinning if the experiment uses a saw-on tare; do not hard-code "saw off"),
     *   - keep the robot perched.
     */
    if(!admittance_enabled_now)
    {
      if(!equilibrium_wrench_ready_)
      {
        equilibrium_wrench_sum_ += wrench_pivot_world;

        ++equilibrium_wrench_sample_count_;

        if(equilibrium_wrench_sample_count_ >= equilibrium_wrench_required_samples_)
        {
          equilibrium_wrench_pivot_world_ = equilibrium_wrench_sum_ / static_cast<double>(equilibrium_wrench_sample_count_);

          equilibrium_wrench_ready_ = true;
          tare_completed_now = true;
        }
      }

      tare_ready_now = equilibrium_wrench_ready_;
      collected_samples = equilibrium_wrench_sample_count_;
      accepted_equilibrium_wrench = equilibrium_wrench_pivot_world_;
    }
    else
    {
      tare_ready_now = equilibrium_wrench_ready_;
      collected_samples = equilibrium_wrench_sample_count_;
      accepted_equilibrium_wrench = equilibrium_wrench_pivot_world_;

      if(tare_ready_now)
      {
        /*
         * This is the wrench that enters the admittance controller:
         * residual = current pivot wrench - no-contact equilibrium pivot wrench
         */
        residual_wrench_pivot_world = wrench_pivot_world - equilibrium_wrench_pivot_world_;
      }
    }
  }

  if(tare_completed_now)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Equilibrium pivot wrench ready after %d samples. "
        "F_eq = %.3f %.3f %.3f N, "
        "T_eq = %.3f %.3f %.3f Nm.",
        collected_samples,
        accepted_equilibrium_wrench(0),
        accepted_equilibrium_wrench(1),
        accepted_equilibrium_wrench(2),
        accepted_equilibrium_wrench(3),
        accepted_equilibrium_wrench(4),
        accepted_equilibrium_wrench(5));
  }

  if(!admittance_enabled_now)
  {
    ROS_INFO_THROTTLE(1.0, "[GimbalrotorPerchingAdmittanceController] Collecting equilibrium pivot wrench: %d / %d.", collected_samples, equilibrium_wrench_required_samples_);

    /*
     * Admittance is disabled, so do not pass the raw biased pivot wrench to the core.
     */
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  if(!tare_ready_now)
  {
    ROS_ERROR_THROTTLE(1.0, "[GimbalrotorPerchingAdmittanceController] Perching admittance has no valid equilibrium wrench. Returning zero wrench.");

    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  ROS_INFO_THROTTLE(
      0.5,
      "[GimbalrotorPerchingAdmittanceController] "
      "Pivot torque Y: current %.3f, "
      "equilibrium %.3f, residual %.3f Nm.",
      wrench_pivot_world(4),
      accepted_equilibrium_wrench(4),
      residual_wrench_pivot_world(4));

  return residual_wrench_pivot_world;
}

GimbalrotorPerchingAdmittanceController::PivotWrenchResult
GimbalrotorPerchingAdmittanceController::updatePivotWrenchAndTare(const PerchingSnapshot& snapshot, const ros::Time& now, double dt)
{
  PivotWrenchResult result;
  result.lock_valid = snapshot.perching_active && snapshot.lock_valid;
  result.armed = snapshot.arm_command;
  result.disarm_requested = snapshot.disarm_requested;
  result.lock_generation = snapshot.lock_generation;
  result.dt = dt;
  result.dt_valid = std::isfinite(dt) && dt > 0.0 && dt <= max_hybrid_control_dt_;
  result.control_gap_fault = snapshot.perching_active && (hybrid_state_ == COMPLIANT_CUTTING || hybrid_state_ == RECOVERY) && !result.dt_valid;
  result.cutting_signal_fresh = snapshot.cutting_signal_fresh;
  result.snapshot_generation_valid = true;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    result.snapshot_generation_valid = snapshot.lock_generation == lock_generation_;
  }

  const Eigen::Matrix<double, 6, 1> wrench_cog_world = GimbalrotorAdmittanceController::getExternalWrenchWorld();

  result.wrench_cog_world = wrench_cog_world;

  bool wrench_is_fresh = false;

  {
    std::lock_guard<std::mutex> lock(external_wrench_mutex_);

    if(has_external_wrench_)
    {
      const double wrench_age = (now - last_external_wrench_receive_time_).toSec();
      wrench_is_fresh = wrench_age >= 0.0 && wrench_age <= external_wrench_timeout_;
    }
  }

  result.fresh = wrench_is_fresh;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    result.tare_ready = hybrid_tare_ready_;
  }

  const bool snapshot_geometry_valid = validateSnapshotGeometry(snapshot);

  if(!wrench_is_fresh || !snapshot.perching_active || !snapshot.lock_valid || !snapshot_geometry_valid)
  {
    result.finite = wrench_cog_world.allFinite() && (!snapshot.lock_valid || snapshot_geometry_valid);
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    if(hybrid_tare_collection_active_ && !hybrid_tare_ready_)
    {
      resetHybridTareUnsafe();
    }
    hybrid_tare_collection_active_ = false;
    return result;
  }

  const tf::Vector3 cog_pos_world_tf = estimator_->getPos(Frame::COG, estimate_mode_);
  const Eigen::Vector3d cog_pos_world(cog_pos_world_tf.x(), cog_pos_world_tf.y(), cog_pos_world_tf.z());
  const Eigen::Vector3d pivot_pos_world = snapshot.pivot_world;
  const Eigen::Matrix<double, 6, 1> wrench_pivot_world = shiftWrenchToPivot(wrench_cog_world, cog_pos_world, pivot_pos_world);
  result.wrench_pivot_world = wrench_pivot_world;

  const bool finite_ok =
      wrench_cog_world.allFinite() &&
      wrench_pivot_world.allFinite() &&
      snapshot_geometry_valid &&
      std::isfinite(cog_pos_world.x()) && std::isfinite(cog_pos_world.y()) && std::isfinite(cog_pos_world.z()) &&
      std::isfinite(pivot_pos_world.x()) && std::isfinite(pivot_pos_world.y()) && std::isfinite(pivot_pos_world.z());

  result.finite = finite_ok;

  if(!finite_ok)
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    if(hybrid_tare_collection_active_ && !hybrid_tare_ready_)
    {
      resetHybridTareUnsafe();
    }
    hybrid_tare_collection_active_ = false;
    return result;
  }

  result.raw_pivot_torque_axis = projectTorqueOntoAxis(wrench_pivot_world, snapshot.R_world_constraint);

  if(!std::isfinite(result.raw_pivot_torque_axis))
  {
    result.finite = false;

    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    if(hybrid_tare_collection_active_ && !hybrid_tare_ready_)
    {
      resetHybridTareUnsafe();
    }

    hybrid_tare_collection_active_ = false;

    return result;
  }  

  const bool valid_tare_sample =
      snapshot.perching_active &&
      snapshot.lock_valid &&
      result.snapshot_generation_valid &&
      !snapshot.arm_command &&
      !snapshot.cutting_active &&
      (!require_cutting_active_ || snapshot.cutting_signal_fresh) &&
      result.fresh &&
      finite_ok &&
      hybrid_state_ != FAULT;

  if(valid_tare_sample)
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    if(!hybrid_tare_ready_)
    {
      if(!hybrid_tare_collection_active_ || hybrid_tare_lock_generation_ != snapshot.lock_generation)
      {
        resetHybridTareUnsafe();
        hybrid_tare_collection_active_ = true;
        hybrid_tare_lock_generation_ = snapshot.lock_generation;
      }

      hybrid_equilibrium_wrench_sum_ += wrench_pivot_world;
      ++hybrid_equilibrium_wrench_sample_count_;

      hybrid_tare_stats_.update(result.raw_pivot_torque_axis);

      if(hybrid_equilibrium_wrench_sample_count_ >= equilibrium_wrench_required_samples_)
      {
        const double tare_stddev = hybrid_tare_stats_.stddev();

        if(!std::isfinite(tare_stddev) || tare_stddev > max_tare_torque_stddev_)
        {
          ROS_ERROR("[GimbalrotorPerchingAdmittanceController] Rejected hybrid tare: pivot torque stddev %.6f exceeds %.6f", tare_stddev, max_tare_torque_stddev_);
          resetHybridTareUnsafe();
        }
        else
        {
          hybrid_equilibrium_wrench_pivot_world_ = hybrid_equilibrium_wrench_sum_ / static_cast<double>(hybrid_equilibrium_wrench_sample_count_);

          hybrid_tare_ready_ = true;
          hybrid_tare_collection_active_ = false;

          ROS_WARN("[GimbalrotorPerchingAdmittanceController] Hybrid equilibrium pivot wrench ready after %d samples. tau_eq stddev: %.5f Nm.", hybrid_equilibrium_wrench_sample_count_, tare_stddev);
        }
      }
      else
      {
        ROS_INFO_THROTTLE(1.0, "[GimbalrotorPerchingAdmittanceController] Collecting hybrid equilibrium pivot wrench: %d / %d.", hybrid_equilibrium_wrench_sample_count_, equilibrium_wrench_required_samples_);
      }
    }
  }
  else
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    if(hybrid_tare_collection_active_ && !hybrid_tare_ready_)
    {
      resetHybridTareUnsafe();
    }

    hybrid_tare_collection_active_ = false;
  }

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    result.tare_ready = hybrid_tare_ready_;
    result.tare_torque_stddev = hybrid_tare_stats_.stddev();
    if(hybrid_tare_ready_)
    {
      result.residual_pivot_world = wrench_pivot_world - hybrid_equilibrium_wrench_pivot_world_;
    }
  }

  result.cutting_gate_valid =
      snapshot.arm_command &&
      result.lock_valid &&
      result.snapshot_generation_valid &&
      result.tare_ready &&
      result.fresh &&
      result.finite &&
      (!require_cutting_active_ || (snapshot.cutting_active && snapshot.cutting_signal_fresh));

  if(result.tare_ready)
  {
    result.residual_pivot_torque_axis = projectTorqueOntoAxis(result.residual_pivot_world, snapshot.R_world_constraint);
  }

  return result;
}

void GimbalrotorPerchingAdmittanceController::updateTorqueFilters(double raw_tau_q, double dt)
{
  if(!std::isfinite(raw_tau_q) || !std::isfinite(dt) || !(dt > 0.0))
  {
    return;
  }

  /*
   * Rate-independent first-order filters (section 8.1):
   *   a = dt / (T + dt)
   *   y_k = y_{k-1} + a * (x_k - y_{k-1})
   *
   * This is the implicit (backward-Euler) discretization of the analog
   * low-pass T*dy/dt + y = x, so the effective cutoff does not drift with the controller rate.
   */
  const double a_fast = dt / (contact_filter_time_constant_ + dt);
  const double a_slow = dt / (admittance_filter_time_constant_ + dt);

  tau_fast_ = tau_fast_ + a_fast * (raw_tau_q - tau_fast_);
  tau_slow_ = tau_slow_ + a_slow * (raw_tau_q - tau_slow_);
}

double GimbalrotorPerchingAdmittanceController::contactOnThreshold() const
{
  return std::max(contact_on_min_torque_, contact_on_sigma_multiplier_ * hybrid_tare_stats_.stddev());
}

double GimbalrotorPerchingAdmittanceController::contactOffThreshold() const
{
  return std::max(contact_off_min_torque_, contact_off_sigma_multiplier_ * hybrid_tare_stats_.stddev());
}

double GimbalrotorPerchingAdmittanceController::directionalDeadband() const
{
  return std::max(directional_deadband_min_torque_, directional_deadband_sigma_multiplier_ * hybrid_tare_stats_.stddev());
}

GimbalrotorPerchingAdmittanceController::HybridState
GimbalrotorPerchingAdmittanceController::decideRestState(bool armed, bool tare_ready) const
{
  if(armed && tare_ready)
  {
    return ARMED_PID;
  }

  return TARE_WAIT;
}

void GimbalrotorPerchingAdmittanceController::enterFault(
    HybridFaultCode code, const std::string& reason)
{
  if(hybrid_state_ != FAULT)
  {
    ROS_ERROR_STREAM("[GimbalrotorPerchingAdmittanceController] HYBRID FAULT: " << reason);
  }

  hybrid_state_ = FAULT;
  hybrid_fault_ = true;
  hybrid_fault_reason_ = reason;
  hybrid_fault_code_ = code;
  hybrid_contact_ = false;

  contact_on_timer_ = 0.0;
  contact_off_timer_ = 0.0;
  recontact_on_timer_ = 0.0;
  supervised_wrench_world_.setZero();
  authority_alpha_ = 0.0;

  if(pid_controllers_.size() > PITCH)
  {
    pid_controllers_.at(PITCH).setIntegratorFrozen(true);
  }
}

void GimbalrotorPerchingAdmittanceController::updateHybridStateMachine(const PivotWrenchResult& pivot, double dt)
{
  const bool tare_ready = pivot.tare_ready;
  const bool wrench_fresh = pivot.fresh;
  const bool active_or_recovering = hybrid_state_ == COMPLIANT_CUTTING || hybrid_state_ == RECOVERY;

  /* ---- Fault checks: highest priority (section 6.7 / 16) ---- */

  const bool stale_during_contact = (hybrid_state_ == COMPLIANT_CUTTING) && !wrench_fresh;
  const bool stale_cutting_signal_during_contact = (hybrid_state_ == COMPLIANT_CUTTING) && require_cutting_active_ && !pivot.cutting_signal_fresh;
  const bool hard_limit_relevant = active_or_recovering || hybrid_state_ == CONTACT_CANDIDATE || (hybrid_state_ == ARMED_PID && pivot.cutting_gate_valid);
  const bool hard_limit_exceeded = hard_limit_relevant && tare_ready && std::isfinite(pivot.residual_pivot_torque_axis) && std::abs(pivot.residual_pivot_torque_axis) > hard_residual_pivot_torque_limit_;
  const bool lock_lost_during_compliance = !pivot.lock_valid && active_or_recovering;
  const bool invalid_geometry_during_compliance = !pivot.snapshot_generation_valid && active_or_recovering;
  const bool nonfinite_during_compliance = !pivot.finite && active_or_recovering;

  if(nonfinite_during_compliance || stale_during_contact || hard_limit_exceeded || lock_lost_during_compliance || invalid_geometry_during_compliance || pivot.control_gap_fault || stale_cutting_signal_during_contact)
  {
    std::string reason = "unspecified";
    HybridFaultCode code = HYBRID_FAULT_NONE;

    if(nonfinite_during_compliance && !pivot.lock_valid)
    {
      reason = "invalid perching geometry";
      code = HYBRID_FAULT_INVALID_GEOMETRY;
    }
    else if(nonfinite_during_compliance)
    {
      reason = "non-finite wrench or geometry state";
      code = HYBRID_FAULT_NONFINITE_STATE;
    }
    else if(stale_during_contact)
    {
      reason = "stale external wrench during compliant cutting";
      code = HYBRID_FAULT_STALE_WRENCH;
    }
    else if(hard_limit_exceeded)
    {
      reason = "hard residual pivot torque limit exceeded";
      code = HYBRID_FAULT_HARD_TORQUE;
    }
    else if(lock_lost_during_compliance)
    {
      reason = "perching lock lost during compliance or recovery";
      code = HYBRID_FAULT_LOCK_LOST;
    }
    else if(invalid_geometry_during_compliance)
    {
      reason = "snapshot lock generation changed during compliance or recovery";
      code = HYBRID_FAULT_INVALID_GEOMETRY;
    }
    else if(pivot.control_gap_fault)
    {
      reason = "invalid hybrid control interval during compliance or recovery";
      code = HYBRID_FAULT_CONTROL_GAP;
    }
    else if(stale_cutting_signal_during_contact)
    {
      reason = "cutting-active signal timed out during compliant cutting";
      code = HYBRID_FAULT_CUTTING_SIGNAL_TIMEOUT;
    }

    enterFault(code, reason);
  }

  if(hybrid_state_ == FAULT)
  {
    /*
     * FAULT remains latched.
     *
     * A normal relock or geometry update does not clear the fault.
     * Only an explicit controller reset or perching disable clears it.
     * The controller must not resume automatically when sensor values become healthy again.
     */
    return;
  }

  /* ---- Top-level disabled conditions ---- */

  if(!pivot.lock_valid)
  {
    hybrid_state_ = HYBRID_DISABLED;
    hybrid_contact_ = false;
    contact_on_timer_ = 0.0;
    contact_off_timer_ = 0.0;
    return;
  }

  switch(hybrid_state_)
  {
    /*
     * FAULT is handled by the early return above and can never reach
     * this switch; group it with the other "not currently compliant"
     * states as a defensive default in case of a future refactor.
     */
    default:
    case HYBRID_DISABLED:
    case TARE_WAIT:
    case ARMED_PID:
    {
      hybrid_contact_ = false;
      contact_on_timer_ = 0.0;
      contact_off_timer_ = 0.0;
      recontact_on_timer_ = 0.0;

      if(pivot.disarm_requested)
      {
        resetHybridTare();
        {
          std::lock_guard<std::mutex> lock(perching_state_mutex_);
          hybrid_arm_command_ = false;
          hybrid_disarm_request_ = false;
          perching_admittance_enabled_ = false;
        }
        hybrid_state_ = TARE_WAIT;
        break;
      }

      if(!pivot.armed || !tare_ready)
      {
        hybrid_state_ = TARE_WAIT;
        break;
      }

      hybrid_state_ = ARMED_PID;

      if(pivot.cutting_gate_valid && std::abs(tau_fast_) > contactOnThreshold())
      {
        hybrid_state_ = CONTACT_CANDIDATE;
        contact_on_timer_ = dt;
        recontact_on_timer_ = 0.0;
      }

      break;
    }

    case CONTACT_CANDIDATE:
    {
      if(pivot.disarm_requested)
      {
        resetHybridTare();
        {
          std::lock_guard<std::mutex> lock(perching_state_mutex_);
          hybrid_arm_command_ = false;
          hybrid_disarm_request_ = false;
          perching_admittance_enabled_ = false;
        }
        contact_on_timer_ = 0.0;
        recontact_on_timer_ = 0.0;
        hybrid_state_ = TARE_WAIT;
        break;
      }

      if(!pivot.armed || !tare_ready)
      {
        hybrid_state_ = TARE_WAIT;
        contact_on_timer_ = 0.0;
        break;
      }

      if(!pivot.cutting_gate_valid || std::abs(tau_fast_) <= contactOnThreshold())
      {
        /* Torque fell back below threshold before persistence time: no target jump ever occurred. */
        hybrid_state_ = ARMED_PID;
        contact_on_timer_ = 0.0;
        break;
      }

      contact_on_timer_ += dt;

      if(contact_on_timer_ >= contact_on_duration_)
      {
        hybrid_state_ = COMPLIANT_CUTTING;
        contact_off_timer_ = 0.0;
        hybrid_contact_ = true;
        recontact_on_timer_ = 0.0;
      }

      break;
    }

    case COMPLIANT_CUTTING:
    {
      hybrid_contact_ = true;

      const bool torque_below_off = std::abs(tau_fast_) < contactOffThreshold();

      if(torque_below_off)
      {
        contact_off_timer_ += dt;
      }
      else
      {
        contact_off_timer_ = 0.0;
      }

      const bool lost_by_torque = contact_off_timer_ >= contact_off_duration_;
      const bool lost_by_cutting_command = require_cutting_active_ && !pivot.cutting_gate_valid;
      const bool lost_by_arm = !pivot.armed || pivot.disarm_requested;

      if(lost_by_torque || lost_by_cutting_command || lost_by_arm)
      {
        if(lost_by_arm)
        {
          enterRecoveryFromDisarm();
        }
        else
        {
          hybrid_state_ = RECOVERY;
          tare_reset_pending_ = false;
        }
        contact_on_timer_ = 0.0;
        contact_off_timer_ = 0.0;
        recontact_on_timer_ = 0.0;
      }

      break;
    }

    case RECOVERY:
    {
      hybrid_contact_ = false;

      /*
      * An explicit disarm received while already recovering must still complete the recovery and invalidate the current tare.
      *
      * Once tare_reset_pending_ becomes true, a later arm command must not cancel the requested disarm/reset sequence.
      */
      if(pivot.disarm_requested || !pivot.armed)
      {
        tare_reset_pending_ = true;
      }

      /* Allow re-entering compliance if strong contact resumes while still armed. */
      if(!tare_reset_pending_ && pivot.armed && pivot.cutting_gate_valid && std::abs(tau_fast_) > contactOnThreshold())      
      {
        recontact_on_timer_ += dt;
        if(recontact_on_timer_ >= recontact_on_duration_)
        {
          hybrid_state_ = COMPLIANT_CUTTING;
          contact_off_timer_ = 0.0;
          hybrid_contact_ = true;
          recontact_on_timer_ = 0.0;
          break;
        }
      }
      else
      {
        recontact_on_timer_ = 0.0;
      }

      const double offset = admittance_core_.getLastOutput().angle_offset_compliance(1);
      const double rate = admittance_core_.getLastOutput().angular_vel_offset_compliance(1);

      const bool near_zero = std::abs(offset) <= recovery_angle_epsilon_ && std::abs(rate) <= recovery_rate_epsilon_;

      if(near_zero)
      {
        admittance_core_.reset();
        admittance_output_ = AdmittanceCoreOutput();
        supervised_wrench_world_.setZero();
        authority_alpha_ = 0.0;
        if(tare_reset_pending_)
        {
          resetHybridTare();
          {
            std::lock_guard<std::mutex> lock(perching_state_mutex_);
            hybrid_arm_command_ = false;
            hybrid_disarm_request_ = false;
            perching_admittance_enabled_ = false;
          }
          tare_reset_pending_ = false;
          hybrid_state_ = TARE_WAIT;
        }
        else
        {
          hybrid_state_ = decideRestState(pivot.armed, tare_ready);
        }
      }

      break;
    }
  }
}

void GimbalrotorPerchingAdmittanceController::updateAuthority(double dt)
{
  if(!std::isfinite(dt) || !(dt > 0.0))
  {
    return;
  }

  /*
   * Authority is applied only during confirmed compliant cutting.
   *
   * During RECOVERY, external admittance forcing is removed
   * immediately. The retained admittance angle/rate states then
   * return smoothly to zero according to the configured virtual
   * inertia, damping and stiffness.
   *
   * Resetting authority to zero here also ensures that a later
   * recontact starts with a new ramp-up instead of immediately
   * restoring full authority.
   */
  if(hybrid_state_ != COMPLIANT_CUTTING)
  {
    authority_alpha_ = 0.0;
    return;
  }

  if(authority_ramp_up_time_ < 1.0e-3)
  {
    authority_alpha_ = 1.0;
    return;
  }

  const double max_step = dt / authority_ramp_up_time_;

  authority_alpha_ = std::min(1.0, authority_alpha_ + max_step);
}

double GimbalrotorPerchingAdmittanceController::computeDirectionalTorque() const
{
  const double tau_db = directionalDeadband();

  /*
   * Positive tau_relief_coordinate means torque is acting in the configured relief direction.
   */
  const double tau_relief_coordinate = relief_pitch_sign_ * tau_slow_;

  const double relief_magnitude = std::max(0.0, tau_relief_coordinate - tau_db);
  const double penetration_magnitude = std::max(0.0, -tau_relief_coordinate - tau_db);

  return relief_pitch_sign_ * relief_magnitude - relief_pitch_sign_ * penetration_torque_ratio_ * penetration_magnitude;
}

void GimbalrotorPerchingAdmittanceController::updatePitchIntegratorMode()
{
  if(pid_controllers_.size() <= PITCH)
  {
    return;
  }

  const bool should_freeze = hybrid_state_ == COMPLIANT_CUTTING || hybrid_state_ == RECOVERY || hybrid_state_ == FAULT;

  pid_controllers_.at(PITCH).setIntegratorFrozen(should_freeze);
}

void GimbalrotorPerchingAdmittanceController::
publishHybridDiagnostics(const PivotWrenchResult& pivot, double tau_directional, double tau_input, const AdmittanceCoreOutput* applied_output, bool fault_target_actually_held)
{
  const AdmittanceCoreOutput zero_output;

  const AdmittanceCoreOutput& applied =
      applied_output != nullptr &&
      applied_output->valid
          ? *applied_output
          : zero_output;

  std_msgs::UInt8 state_msg;
  state_msg.data = static_cast<uint8_t>(hybrid_state_);
  hybrid_state_pub_.publish(state_msg);

  std_msgs::Bool contact_msg;
  contact_msg.data = hybrid_contact_;
  hybrid_contact_pub_.publish(contact_msg);

  std_msgs::Bool armed_msg;
  armed_msg.data = pivot.armed;
  hybrid_armed_pub_.publish(armed_msg);

  std_msgs::Bool disarm_msg;
  disarm_msg.data = pivot.disarm_requested;
  hybrid_disarm_requested_pub_.publish(disarm_msg);

  std_msgs::Float64 authority_msg;
  authority_msg.data = authority_alpha_;
  hybrid_authority_pub_.publish(authority_msg);

  std_msgs::Float64 residual_msg;
  residual_msg.data = pivot.residual_pivot_torque_axis;
  hybrid_torque_residual_pub_.publish(residual_msg);

  std_msgs::Float64 filtered_msg;
  filtered_msg.data = tau_fast_;
  hybrid_torque_filtered_pub_.publish(filtered_msg);

  std_msgs::Bool cutting_active_msg;
  cutting_active_msg.data = cutting_active_;
  hybrid_cutting_active_pub_.publish(cutting_active_msg);

  std_msgs::Bool cutting_fresh_msg;
  cutting_fresh_msg.data = pivot.cutting_signal_fresh;
  hybrid_cutting_signal_fresh_pub_.publish(cutting_fresh_msg);

  std_msgs::Bool cutting_gate_msg;
  cutting_gate_msg.data = pivot.cutting_gate_valid;
  hybrid_cutting_gate_valid_pub_.publish(cutting_gate_msg);

  std_msgs::Float64 tare_count_msg;
  tare_count_msg.data = static_cast<double>(hybrid_equilibrium_wrench_sample_count_);
  hybrid_tare_sample_count_pub_.publish(tare_count_msg);

  std_msgs::Float64 tare_stddev_msg;
  tare_stddev_msg.data = pivot.tare_torque_stddev;
  hybrid_tare_torque_stddev_pub_.publish(tare_stddev_msg);

  std_msgs::Bool tare_collection_msg;
  tare_collection_msg.data = hybrid_tare_collection_active_;
  hybrid_tare_collection_active_pub_.publish(tare_collection_msg);

  std_msgs::Float64 raw_tau_msg;
  raw_tau_msg.data = pivot.raw_pivot_torque_axis;
  hybrid_pivot_torque_raw_pub_.publish(raw_tau_msg);

  std_msgs::Float64 fast_tau_msg;
  fast_tau_msg.data = tau_fast_;
  hybrid_pivot_torque_fast_pub_.publish(fast_tau_msg);

  std_msgs::Float64 slow_tau_msg;
  slow_tau_msg.data = tau_slow_;
  hybrid_pivot_torque_slow_pub_.publish(slow_tau_msg);

  std_msgs::Float64 on_threshold_msg;
  on_threshold_msg.data = contactOnThreshold();
  hybrid_contact_on_threshold_pub_.publish(on_threshold_msg);

  std_msgs::Float64 off_threshold_msg;
  off_threshold_msg.data = contactOffThreshold();
  hybrid_contact_off_threshold_pub_.publish(off_threshold_msg);

  std_msgs::Float64 deadband_msg;
  deadband_msg.data = directionalDeadband();
  hybrid_directional_deadband_pub_.publish(deadband_msg);

  std_msgs::Float64 directional_msg;
  directional_msg.data = tau_directional;
  hybrid_directional_torque_pub_.publish(directional_msg);

  std_msgs::Float64 effective_tau_msg;
  effective_tau_msg.data = tau_input;
  hybrid_effective_tau_input_pub_.publish(effective_tau_msg);

  std_msgs::Float64 pitch_offset_msg;
  pitch_offset_msg.data = applied.angle_offset_compliance(1);
  hybrid_pitch_offset_pub_.publish(pitch_offset_msg);

  std_msgs::Float64 pitch_rate_msg;
  pitch_rate_msg.data = applied.angular_vel_offset_compliance(1);
  hybrid_pitch_offset_rate_pub_.publish(pitch_rate_msg);

  std_msgs::Float64 pitch_acc_msg;
  pitch_acc_msg.data = applied.angular_acc_offset_compliance(1);
  hybrid_pitch_offset_acceleration_pub_.publish(pitch_acc_msg);

  const double nominal_pitch =
      navigator_
          ? navigator_->getTargetRPY().y()
          : 0.0;

  std_msgs::Float64 nominal_pitch_msg;
  nominal_pitch_msg.data = nominal_pitch;
  hybrid_nominal_pitch_pub_.publish(nominal_pitch_msg);

  std_msgs::Float64 modified_pitch_msg;
  modified_pitch_msg.data = normalizeAngle(nominal_pitch + applied.angle_offset_compliance(1));
  hybrid_modified_pitch_pub_.publish(modified_pitch_msg);

  std_msgs::Float64 dt_msg;
  dt_msg.data = pivot.dt;
  hybrid_dt_pub_.publish(dt_msg);

  std_msgs::Bool dt_valid_msg;
  dt_valid_msg.data = pivot.dt_valid;
  hybrid_dt_valid_pub_.publish(dt_valid_msg);

  std_msgs::Bool wrench_fresh_msg;
  wrench_fresh_msg.data = pivot.fresh;
  hybrid_wrench_fresh_pub_.publish(wrench_fresh_msg);

  std_msgs::Bool lock_valid_msg;
  lock_valid_msg.data = pivot.lock_valid;
  hybrid_lock_valid_pub_.publish(lock_valid_msg);

  std_msgs::Float64 generation_msg;
  generation_msg.data = static_cast<double>(pivot.lock_generation);
  hybrid_lock_generation_pub_.publish(generation_msg);

  std_msgs::Bool generation_valid_msg;
  generation_valid_msg.data = pivot.snapshot_generation_valid;
  hybrid_snapshot_generation_valid_pub_.publish(generation_valid_msg);

  std_msgs::Bool tare_ready_msg;
  tare_ready_msg.data = hybrid_tare_ready_;
  hybrid_tare_ready_pub_.publish(tare_ready_msg);

  std_msgs::Bool i_frozen_msg;
  i_frozen_msg.data = pid_controllers_.size() > PITCH && pid_controllers_.at(PITCH).isIntegratorFrozen();
  hybrid_pid_i_frozen_pub_.publish(i_frozen_msg);

  std_msgs::Bool fault_msg;
  fault_msg.data = hybrid_fault_;
  hybrid_fault_pub_.publish(fault_msg);

  std_msgs::UInt8 fault_code_msg;
  fault_code_msg.data = static_cast<uint8_t>(hybrid_fault_code_);
  hybrid_fault_code_pub_.publish(fault_code_msg);

  std_msgs::String fault_reason_msg;
  fault_reason_msg.data = hybrid_fault_reason_;
  hybrid_fault_reason_pub_.publish(fault_reason_msg);

  std_msgs::Bool fault_holds_msg;
  fault_holds_msg.data = hybrid_state_ == FAULT && fault_target_actually_held;
  hybrid_fault_holds_target_pub_.publish(fault_holds_msg);

  std_msgs::Bool abort_request_msg;
  abort_request_msg.data = hybrid_state_ == FAULT;
  hybrid_abort_request_pub_.publish(abort_request_msg);

  ROS_WARN_THROTTLE(
      0.5,
      "[GimbalrotorPerchingAdmittanceController] "
      "HYBRID state=%d contact=%d alpha=%.2f "
      "tau_raw=%.4f tau_res=%.4f "
      "tau_fast=%.4f tau_dir=%.4f "
      "pitch_applied_deg=%.2f",
      static_cast<int>(hybrid_state_),
      static_cast<int>(hybrid_contact_),
      authority_alpha_,
      pivot.raw_pivot_torque_axis,
      pivot.residual_pivot_torque_axis,
      tau_fast_,
      tau_directional,
      applied.angle_offset_compliance(1) * 180.0 / PI);
}

bool GimbalrotorPerchingAdmittanceController::
applyHybridAdmittanceOutputToNavigator(const PerchingSnapshot& snapshot, const tf::Vector3& original_target_pos, const tf::Vector3& original_target_rpy, const AdmittanceCoreOutput& output)
{
  if(!output.valid || !output.pos_offset_compliance.allFinite() || !output.angle_offset_compliance.allFinite() || !output.rpy_offset_world.allFinite())
  {
    enterFault(
        HYBRID_FAULT_NONFINITE_STATE,
        "Hybrid admittance output is invalid");

    return false;
  }

  bool generation_matches = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    generation_matches = snapshot.lock_generation == lock_generation_;
  }

  if(!generation_matches || !snapshot.lock_valid || !validateSnapshotGeometry(snapshot))
  {
    enterFault(
        HYBRID_FAULT_INVALID_GEOMETRY,
        "Hybrid target snapshot no longer "
        "matches locked geometry");

    return false;
  }

const double nominal_pitch = original_target_rpy.y();
const double requested_admittance_pitch_offset = output.angle_offset_compliance(1);
const double requested_target_pitch = normalizeAngle(nominal_pitch + requested_admittance_pitch_offset);

  /*
  * The final compliant attitude and the arc position must use the same pitch delta relative to the locked pose.
  */
  double constrained_pitch_delta = normalizeAngle(requested_target_pitch - snapshot.locked_robot_rpy.y());
  constrained_pitch_delta = clamp(constrained_pitch_delta, -max_pitch_delta_, max_pitch_delta_);
  const double target_pitch = normalizeAngle(snapshot.locked_robot_rpy.y() + constrained_pitch_delta);
  
  const double applied_admittance_pitch_offset = normalizeAngle(target_pitch - nominal_pitch);

  tf::Vector3 modified_target_pos = computePerchingArcPositionFromPitch(snapshot, target_pitch, original_target_pos);

  const Eigen::Vector3d branch_offset_world = snapshot.constraint_axis_world * output.pos_offset_compliance(1);

  modified_target_pos += tf::Vector3(branch_offset_world.x(), branch_offset_world.y(), branch_offset_world.z());

  tf::Vector3 modified_target_rpy = original_target_rpy;

  modified_target_rpy.setX(modified_target_rpy.x() + output.rpy_offset_world(0));
  modified_target_rpy.setY(target_pitch);
  modified_target_rpy.setZ(modified_target_rpy.z() + output.rpy_offset_world(2));

  navigator_->setTargetPos(modified_target_pos);
  navigator_->setTargetRPY(modified_target_rpy);

  /*
   * Check once more after target construction and injection. If a relock occurred during this function, restore the nominal target before the inner controller runs.
   */
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    generation_matches = snapshot.lock_generation == lock_generation_;
  }

  if(!generation_matches)
  {
    navigator_->setTargetPos(original_target_pos);
    navigator_->setTargetRPY(original_target_rpy);

    enterFault(
        HYBRID_FAULT_INVALID_GEOMETRY,
        "Perching lock changed while applying hybrid target");

    return false;
  }

  ROS_WARN_THROTTLE(
      0.5,
      "[GimbalrotorPerchingAdmittanceController] "
      "HYBRID admittance | "
      "nominal_pitch %.2f deg | "
      "d_pitch %.2f deg | "
      "target_pitch %.2f deg | "
      "target_pos %.3f %.3f %.3f",
      nominal_pitch * 180.0 / PI,
      applied_admittance_pitch_offset * 180.0 / PI,
      target_pitch * 180.0 / PI,
      modified_target_pos.x(),
      modified_target_pos.y(),
      modified_target_pos.z());

  return true;
}

void GimbalrotorPerchingAdmittanceController::applyAdmittanceOutputToNavigator(const tf::Vector3& original_target_pos, const tf::Vector3& original_target_rpy, const AdmittanceCoreOutput& output)
{
  bool perching_active = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    perching_active = perching_enabled_for_constraint_;
  }

  /*
   * Normal-flight path.
   */
  if(!perching_active)
  {
    GimbalrotorAdmittanceController::applyAdmittanceOutputToNavigator(original_target_pos, original_target_rpy, output);

    return;
  }

  tf::Vector3 modified_target_pos;
  tf::Vector3 modified_target_rpy;

  double nominal_pitch = 0.0;
  double admittance_pitch_offset = 0.0;
  double target_pitch = 0.0;
  PerchingSnapshot snapshot;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    /*
     * Recheck the state while holding the lock.
     * It may have changed after the first snapshot.
     *
     * Important: in hybrid mode, gate on hybrid_state_ rather than on
     * perching_admittance_enabled_ ("armed"). Arm can go false exactly
     * when RECOVERY begins (section 6.6); if we gated on "armed" here,
     * the still-decaying admittance offset would stop being applied to
     * the target on that same cycle and the target would snap back to
     * theta_nom instantly - precisely the discontinuity problem 4 is
     * meant to eliminate.
     */
    const bool apply_target =
        hybrid_enabled_cfg_
            ? (hybrid_state_ == COMPLIANT_CUTTING || hybrid_state_ == RECOVERY)
            : perching_admittance_enabled_;

    snapshot.perching_active = perching_enabled_for_constraint_;
    snapshot.lock_valid = has_locked_pose_;
    snapshot.pivot_world = Eigen::Vector3d(locked_pivot_world_.x(), locked_pivot_world_.y(), locked_pivot_world_.z());
    snapshot.locked_robot_pos_world = Eigen::Vector3d(locked_robot_pos_world_.x(), locked_robot_pos_world_.y(), locked_robot_pos_world_.z());
    snapshot.locked_robot_rpy = Eigen::Vector3d(locked_robot_rpy_.x(), locked_robot_rpy_.y(), locked_robot_rpy_.z());
    snapshot.locked_radius_vec_world = Eigen::Vector3d(locked_radius_vec_world_.x(), locked_radius_vec_world_.y(), locked_radius_vec_world_.z());
    snapshot.constraint_axis_world = constraint_axis_world_;
    snapshot.R_world_constraint = R_world_constraint_;
    snapshot.locked_radius = locked_radius_;
    snapshot.locked_x_side = locked_x_side_;
    snapshot.lock_generation = lock_generation_;

    if(!perching_enabled_for_constraint_ || !apply_target || !has_locked_pose_ || !validateSnapshotGeometry(snapshot))
    {
      return;
    }

    nominal_pitch = original_target_rpy.y();

    /*
     * Constraint coordinate Y is the branch axis.
     * Rotational admittance Y therefore produces the perching pitch correction.
     */
    admittance_pitch_offset = output.angle_offset_compliance(1);

    target_pitch = normalizeAngle(nominal_pitch + admittance_pitch_offset);

    /*
     * This function reads locked geometry.
     * The perching-state mutex is held here.
     */
    modified_target_pos = computePerchingArcPositionFromPitch(snapshot, target_pitch, original_target_pos);

    /*
     * Translational constraint Y is motion along the branch axis.
     */
    const Eigen::Vector3d branch_offset_world = snapshot.constraint_axis_world * output.pos_offset_compliance(1);

    modified_target_pos += tf::Vector3(branch_offset_world.x(), branch_offset_world.y(), branch_offset_world.z());

    modified_target_rpy = original_target_rpy;

    /*
     * Roll and yaw remain zero with the current YAML because those admittance axes are disabled.
     */
    modified_target_rpy.setX(modified_target_rpy.x() + output.rpy_offset_world(0));
    modified_target_rpy.setY(target_pitch);
    modified_target_rpy.setZ(modified_target_rpy.z() + output.rpy_offset_world(2));
  }

  /*
   * Do not hold the state mutex while calling into navigator code.
   */
  navigator_->setTargetPos(modified_target_pos);
  navigator_->setTargetRPY(modified_target_rpy);

  ROS_WARN_THROTTLE(
      0.5,
      "[GimbalrotorPerchingAdmittanceController] "
      "PERCHING admittance | "
      "nominal_pitch %.2f deg | "
      "d_pitch %.2f deg | "
      "target_pitch %.2f deg | "
      "target_pos %.3f %.3f %.3f",
      nominal_pitch * 180.0 / PI,
      admittance_pitch_offset * 180.0 / PI,
      target_pitch * 180.0 / PI,
      modified_target_pos.x(),
      modified_target_pos.y(),
      modified_target_pos.z());
}

tf::Vector3
GimbalrotorPerchingAdmittanceController::computePerchingArcPositionFromPitch(const PerchingSnapshot& snapshot, double target_pitch, const tf::Vector3& original_target_pos) const
{
  /*
   * Same geometric idea as GimbalrotorPerchingNavigator::computeArcPositionFromPitch().
   *
   * Locked:
   *   pivot C
   *   robot position P0
   *   radius vector r0 = P0 - C
   *   pitch theta0
   *
   * Command:
   *   target pitch theta
   *
   * Compute:
   *   dtheta = theta - theta0
   *   r_des = RotY(dtheta) * r0
   *   P_des = C + r_des
   *
   * Use only X-Z radius because branch axis is world Y.
   */
  double delta_pitch = normalizeAngle(target_pitch - snapshot.locked_robot_rpy.y());
  delta_pitch = clamp(delta_pitch, -max_pitch_delta_, max_pitch_delta_);

  const double signed_delta = arc_pitch_sign_ * delta_pitch;

  tf::Vector3 locked_radius_xz(snapshot.locked_radius_vec_world.x(), 0.0, snapshot.locked_radius_vec_world.z());

  double length_xz = norm2D(locked_radius_xz.x(), locked_radius_xz.z());

  if(length_xz < 1.0e-6)
    {
      locked_radius_xz.setX(snapshot.locked_x_side * snapshot.locked_radius);
      locked_radius_xz.setY(0.0);
      locked_radius_xz.setZ(0.0);
      length_xz = snapshot.locked_radius;
    }

  if(length_xz < 1.0e-6)
    {
      locked_radius_xz.setX(snapshot.locked_x_side);
      locked_radius_xz.setY(0.0);
      locked_radius_xz.setZ(0.0);
      length_xz = 1.0;
    }

  locked_radius_xz.setX(locked_radius_xz.x() / length_xz * snapshot.locked_radius);
  locked_radius_xz.setY(0.0);
  locked_radius_xz.setZ(locked_radius_xz.z() / length_xz * snapshot.locked_radius);

  tf::Matrix3x3 rot(tf::createQuaternionFromRPY(0.0, signed_delta, 0.0));

  tf::Vector3 rotated_radius = rot * locked_radius_xz;

  double rotated_length_xz = norm2D(rotated_radius.x(), rotated_radius.z());

  if(rotated_length_xz < 1.0e-6)
  {
    rotated_radius.setX(snapshot.locked_x_side * snapshot.locked_radius);
    rotated_radius.setY(0.0);
    rotated_radius.setZ(0.0);
  }
  else
  {
    rotated_radius.setX(rotated_radius.x() / rotated_length_xz * snapshot.locked_radius);
    rotated_radius.setY(0.0);
    rotated_radius.setZ(rotated_radius.z() / rotated_length_xz * snapshot.locked_radius);
  }

  tf::Vector3 target_pos = tf::Vector3(snapshot.pivot_world.x(), snapshot.pivot_world.y(), snapshot.pivot_world.z()) + rotated_radius;

  /*
   * Preserve Y generated by GimbalrotorPerchingNavigator.
   *
   * The navigator already handles Y deadband/compliance. This controller should not duplicate that logic.
   */
  target_pos.setY(original_target_pos.y());

  return target_pos;
}

double GimbalrotorPerchingAdmittanceController::clamp(double value, double min_value, double max_value) const
{
  if(!std::isfinite(value))
  {
    return 0.0;
  }

  if(value < min_value)
    {
      return min_value;
    }

  if(value > max_value)
    {
      return max_value;
    }

  return value;
}

double GimbalrotorPerchingAdmittanceController::normalizeAngle(double angle) const
{
  if(!std::isfinite(angle))
  {
    return 0.0;
  }

  while(angle > PI)
    {
      angle -= 2.0 * PI;
    }

  while(angle < -PI)
    {
      angle += 2.0 * PI;
    }

  return angle;
}

double GimbalrotorPerchingAdmittanceController::norm2D(double x, double z) const
{
  return std::sqrt(x * x + z * z);
}

void GimbalrotorPerchingAdmittanceController::poseMsgToTfPosRpy(const geometry_msgs::PoseStamped& msg, tf::Vector3& pos, tf::Vector3& rpy) const
{
  pos.setValue(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

  const geometry_msgs::Quaternion& q_msg = msg.pose.orientation;
  tf::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  q.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  rpy.setValue(roll, pitch, yaw);
}

bool GimbalrotorPerchingAdmittanceController::isFiniteTfVector(const tf::Vector3& vector) const
{
  return std::isfinite(vector.x()) && std::isfinite(vector.y()) && std::isfinite(vector.z());
}

bool GimbalrotorPerchingAdmittanceController::isValidRotationMatrix(const Eigen::Matrix3d& rotation) const
{
  if(!rotation.allFinite())
  {
    return false;
  }

  const double determinant = rotation.determinant();

  if(!std::isfinite(determinant) || std::abs(determinant - 1.0) > 1.0e-3)
  {
    return false;
  }

  const Eigen::Matrix3d orthogonality_error = rotation.transpose() * rotation - Eigen::Matrix3d::Identity();

  return orthogonality_error.allFinite() && orthogonality_error.norm() <= 1.0e-3;
}

bool GimbalrotorPerchingAdmittanceController::validateLockedPivotMessage(const geometry_msgs::PointStamped& msg) const
{
  const bool finite_pivot = std::isfinite(msg.point.x) && std::isfinite(msg.point.y) && std::isfinite(msg.point.z);

  if(!finite_pivot)
  {
    ROS_ERROR("[GimbalrotorPerchingAdmittanceController] Rejected locked pivot with non-finite position.");
  }

  return finite_pivot;
}

bool GimbalrotorPerchingAdmittanceController::validateLockedPoseMessage(const geometry_msgs::PoseStamped& msg) const
{
  const bool finite_position = std::isfinite(msg.pose.position.x) && std::isfinite(msg.pose.position.y) && std::isfinite(msg.pose.position.z);

  const geometry_msgs::Quaternion& q = msg.pose.orientation;

  const bool finite_quaternion = std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w);
  const double quaternion_norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

  if(!finite_position || !finite_quaternion || !std::isfinite(quaternion_norm) || quaternion_norm < 1.0e-6)
  {
    ROS_ERROR("[GimbalrotorPerchingAdmittanceController] Rejected locked pose with invalid position or quaternion.");
    return false;
  }

  tf::Quaternion normalized_q(q.x, q.y, q.z, q.w);
  normalized_q.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf::Matrix3x3(normalized_q).getRPY(roll, pitch, yaw);

  return std::isfinite(roll) && std::isfinite(pitch) && std::isfinite(yaw);
}

bool GimbalrotorPerchingAdmittanceController::validateSnapshotGeometry(const PerchingSnapshot& snapshot) const
{
  const double axis_norm = snapshot.constraint_axis_world.norm();

  return snapshot.pivot_world.allFinite() &&
         snapshot.locked_robot_pos_world.allFinite() &&
         snapshot.locked_robot_rpy.allFinite() &&
         snapshot.locked_radius_vec_world.allFinite() &&
         snapshot.constraint_axis_world.allFinite() &&
         std::isfinite(axis_norm) &&
         std::abs(axis_norm - 1.0) <= 1.0e-3 &&
         isValidRotationMatrix(snapshot.R_world_constraint) &&
         std::isfinite(snapshot.locked_radius) &&
         snapshot.locked_radius >= min_valid_radius_ &&
         std::isfinite(snapshot.locked_x_side);
}

bool GimbalrotorPerchingAdmittanceController::faultAllowsHeldTarget() const
{
  if(!hold_relief_target_on_measurement_fault_)
  {
    return false;
  }

  switch(hybrid_fault_code_)
  {
    case HYBRID_FAULT_STALE_WRENCH:
    case HYBRID_FAULT_HARD_TORQUE:
    case HYBRID_FAULT_CONTROL_GAP:
    case HYBRID_FAULT_CUTTING_SIGNAL_TIMEOUT:
    case HYBRID_FAULT_NONFINITE_STATE:
      return true;
    default:
      return false;
  }
}

} // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(
    aerial_robot_control::GimbalrotorPerchingAdmittanceController,
    aerial_robot_control::ControlBase)
