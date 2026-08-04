#include <gimbalrotor/control/gimbalrotor_perching_admittance_controller.h>

#include <pluginlib/class_list_macros.h>

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
    perching_constraint_valid_topic_("perching/constraint_valid"),
    perching_pivot_wrench_frame_id_("perching_pivot"),
    normal_admittance_enabled_(false),
    perching_admittance_enabled_(false),
    effective_admittance_enabled_(false),
    perching_enabled_for_constraint_(false),
    has_perching_point_(false),
    has_branch_pose_(false),
    has_locked_pose_msg_(false),
    has_locked_pivot_(false),
    has_locked_pose_(false),
    perching_constraint_valid_(false),
    use_branch_pose_if_no_point_(false),
    require_perching_lock_(true),
    min_valid_radius_(0.05),
    max_pitch_delta_(0.78539816339),
    arc_pitch_sign_(1.0),
    contact_torque_filter_alpha_(0.10),
    contact_on_threshold_(0.04),
    contact_off_threshold_(0.02),
    contact_on_duration_(0.10),
    contact_off_duration_(0.20),
    recovery_angle_epsilon_(0.00174533),
    recovery_rate_epsilon_(0.01),
    perching_pitch_torque_sign_(1.0),
    locked_radius_(0.0),
    locked_x_side_(1.0), 
    maximum_lock_stamp_difference_(0.05),
    equilibrium_wrench_required_samples_(80),
    equilibrium_wrench_sample_count_(0),
    equilibrium_wrench_ready_(false),
    admittance_reset_requested_(false),
    contact_active_(false),
    recovery_active_(false),
    contact_on_timer_(0.0),
    contact_off_timer_(0.0),
    residual_pitch_torque_raw_(0.0),
    residual_pitch_torque_filtered_(0.0),
    normal_pitch_i_limit_(0.0),
    pitch_i_limit_suppressed_(false)
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
  previous_contact_gate_time_ = ros::Time(0);

  R_world_constraint_.setIdentity();

  constraint_axis_world_ = Eigen::Vector3d::UnitY();

  equilibrium_wrench_pivot_world_.setZero();
  equilibrium_wrench_sum_.setZero();
  prepared_perching_admittance_wrench_world_.setZero();
  prepared_R_world_constraint_.setIdentity();
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
  GimbalrotorAdmittanceController::initialize(
      nh,
      nhp,
      robot_model,
      estimator,
      navigator,
      ctrl_loop_rate);

  perchingRosParamInit();

  /*
   * Save the normal pitch integral-output limit.
   * With the current GimbalrotorControl.yaml, this should normally be 10.0.
   */
  normal_pitch_i_limit_ = pid_controllers_.at(PITCH).getLimitI();

  ROS_WARN("[GimbalrotorPerchingAdmittanceController] Normal pitch I limit: %.6f", normal_pitch_i_limit_);

  /*
   * The base admittance controller subscribed to admittance_enable_topic_.
   * For this mode-aware controller, replace that subscription with our own
   * normal/perching trigger separation.
   */
  admittance_enable_sub_.shutdown();

  normal_admittance_enable_sub_ =
      nh_.subscribe(
          admittance_enable_topic_,
          1,
          &GimbalrotorPerchingAdmittanceController::normalAdmittanceEnableCallback,
          this);

  perching_admittance_enable_sub_ =
      nh_.subscribe(
          perching_admittance_enable_topic_,
          1,
          &GimbalrotorPerchingAdmittanceController::perchingAdmittanceEnableCallback,
          this);

  perching_enable_sub_for_constraint_ =
      nh_.subscribe(
          perching_enable_topic_for_constraint_,
          1,
          &GimbalrotorPerchingAdmittanceController::perchingEnableCallback,
          this);

  perching_point_sub_ =
      nh_.subscribe(
          perching_point_topic_,
          1,
          &GimbalrotorPerchingAdmittanceController::perchingPointCallback,
          this);

  branch_pose_sub_ =
      nh_.subscribe(
          perching_branch_pose_topic_,
          1,
          &GimbalrotorPerchingAdmittanceController::branchPoseCallback,
          this);

  /*
   * GimbalrotorPerchingNavigator publishes this as latched.
   * It contains:
   *   - locked robot position
   *   - locked robot RPY
   *
   * We use this to reconstruct the same pitch arc in the admittance controller.
   */
  locked_pose_sub_ =
      nh_.subscribe(
          perching_locked_pose_topic_,
          1,
          &GimbalrotorPerchingAdmittanceController::lockedPoseCallback,
          this);

  locked_pivot_sub_ =
      nh_.subscribe(
          perching_locked_pivot_topic_,
          1,
          &GimbalrotorPerchingAdmittanceController::lockedPivotCallback,
          this);
  constraint_valid_sub_ =
      nh_.subscribe(
          perching_constraint_valid_topic_,
          1,
          &GimbalrotorPerchingAdmittanceController::constraintValidCallback,
          this);

  contact_active_pub_ =
      nh_.advertise<std_msgs::Bool>(
          "perching/contact_admittance/contact_active",
          1);

  recovery_active_pub_ =
      nh_.advertise<std_msgs::Bool>(
          "perching/contact_admittance/recovery_active",
          1);

  pivot_torque_raw_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/contact_admittance/pivot_torque_raw",
          1);

  pivot_torque_filtered_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/contact_admittance/pivot_torque_filtered",
          1);

  pitch_offset_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/contact_admittance/pitch_offset",
          1);

  pivot_external_wrench_est_pub_ =
      nh_.advertise<geometry_msgs::WrenchStamped>(
          "perching/estimated_external_wrench_pivot",
          1);

  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] initialized.");
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] perching_enable_topic: "
                  << perching_enable_topic_for_constraint_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] perching_point_topic: "
                  << perching_point_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] branch_pose_topic: "
                  << perching_branch_pose_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] locked_pose_topic: "
                  << perching_locked_pose_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] constraint_valid_topic: "
                  << perching_constraint_valid_topic_);
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

  GimbalrotorAdmittanceController::reset();

  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  last_published_pivot_wrench_sequence_ = 0;

  normal_admittance_enabled_ = false;
  perching_admittance_enabled_ = false;
  effective_admittance_enabled_ = false;

  perching_enabled_for_constraint_ = false;

  has_perching_point_ = false;
  has_branch_pose_ = false;
  has_locked_pose_msg_ = false;
  has_locked_pivot_ = false;
  has_locked_pose_ = false;
  perching_constraint_valid_ = false;

  perching_point_world_.setValue(
      0.0, 0.0, 0.0);

  branch_pos_world_.setValue(
      0.0, 0.0, 0.0);

  locked_robot_pos_world_.setValue(
      0.0, 0.0, 0.0);

  locked_robot_rpy_.setValue(
      0.0, 0.0, 0.0);

  locked_pivot_world_.setValue(
      0.0, 0.0, 0.0);

  locked_radius_vec_world_.setValue(
      0.0, 0.0, 0.0);

  locked_radius_ = 0.0;
  locked_x_side_ = 1.0;

  locked_pose_stamp_ = ros::Time(0);
  locked_pivot_stamp_ = ros::Time(0);
  accepted_locked_pose_stamp_ = ros::Time(0);
  accepted_locked_pivot_stamp_ = ros::Time(0);

  R_world_constraint_.setIdentity();
  prepared_R_world_constraint_.setIdentity();

  constraint_axis_world_ = Eigen::Vector3d::UnitY();

  resetEquilibriumWrenchUnsafe();
  resetContactGateUnsafe();

  admittance_reset_requested_ = false;

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "Perching admittance state reset.");
}

void GimbalrotorPerchingAdmittanceController::perchingRosParamInit()
{
  ros::NodeHandle imp_perch_nh(nh_, "controller/admittance/perching");

  /*
   * Defaults are intentionally the same topic names used by
   * GimbalrotorPerchingNavigator.
   */
  getParam<std::string>(
      imp_perch_nh,
      "perching_enable_topic",
      perching_enable_topic_for_constraint_,
      std::string("perching/enable"));

  getParam<std::string>(
      imp_perch_nh,
      "perching_point_topic",
      perching_point_topic_,
      std::string("perching/point"));

  getParam<std::string>(
      imp_perch_nh,
      "perching_branch_pose_topic",
      perching_branch_pose_topic_,
      std::string("perching/branch_pose"));

  getParam<std::string>(
      imp_perch_nh,
      "perching_locked_pose_topic",
      perching_locked_pose_topic_,
      std::string("perching/locked_pose"));

  getParam<bool>(
      imp_perch_nh,
      "use_branch_pose_if_no_point",
      use_branch_pose_if_no_point_,
      false);

  getParam<bool>(
      imp_perch_nh,
      "require_perching_lock",
      require_perching_lock_,
      true);

  /*
   * These must match the perching navigator parameters.
   *
   * If they are not explicitly set under controller/admittance/perching,
   * read the existing navigation parameters.
   */
  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(
      navi_nh,
      "perching_min_valid_radius",
      min_valid_radius_,
      0.05);

  getParam<double>(
      navi_nh,
      "perching_max_pitch_delta",
      max_pitch_delta_,
      0.78539816339);

  getParam<double>(
      navi_nh,
      "perching_arc_pitch_sign",
      arc_pitch_sign_,
      1.0);

  getParam<double>(
      imp_perch_nh,
      "perching_min_valid_radius",
      min_valid_radius_,
      min_valid_radius_);

  getParam<double>(
      imp_perch_nh,
      "perching_max_pitch_delta",
      max_pitch_delta_,
      max_pitch_delta_);

  getParam<double>(
      imp_perch_nh,
      "perching_arc_pitch_sign",
      arc_pitch_sign_,
      arc_pitch_sign_);

  if(!std::isfinite(min_valid_radius_) ||
     min_valid_radius_ <= 1.0e-6)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid perching_min_valid_radius %.6f. "
        "Using 0.05 m.",
        min_valid_radius_);

    min_valid_radius_ = 0.05;
  }

  if(!std::isfinite(max_pitch_delta_) ||
     max_pitch_delta_ <= 0.0 ||
     max_pitch_delta_ > PI)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid perching_max_pitch_delta %.6f. "
        "Using 0.5235987756 rad.",
        max_pitch_delta_);

    max_pitch_delta_ = 0.5235987756;
  }

  if(!std::isfinite(arc_pitch_sign_) ||
     arc_pitch_sign_ == 0.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid perching_arc_pitch_sign %.6f. "
        "Using 1.0.",
        arc_pitch_sign_);

    arc_pitch_sign_ = 1.0;
  }

  arc_pitch_sign_ = arc_pitch_sign_ >= 0.0 ? 1.0 : -1.0;

  if(!std::isfinite(external_wrench_timeout_) ||
     external_wrench_timeout_ <= 0.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid external_wrench_timeout %.6f. "
        "Using 0.15 s.",
        external_wrench_timeout_);

    external_wrench_timeout_ = 0.15;
  }

  getParam<std::string>(
      imp_perch_nh,
      "perching_admittance_enable_topic",
      perching_admittance_enable_topic_,
      std::string("perching/admittance_enable"));

  getParam<std::string>(
      imp_perch_nh,
      "perching_locked_pivot_topic",
      perching_locked_pivot_topic_,
      std::string("perching/locked_pivot"));
  getParam<std::string>(
      imp_perch_nh,
      "perching_constraint_valid_topic",
      perching_constraint_valid_topic_,
      std::string("perching/constraint_valid"));

  getParam<std::string>(
      imp_perch_nh,
      "perching_pivot_wrench_frame_id",
      perching_pivot_wrench_frame_id_,
      std::string("perching_pivot"));

  if(perching_pivot_wrench_frame_id_.empty())
    perching_pivot_wrench_frame_id_ = "perching_pivot";

  getParam<double>(
      imp_perch_nh,
      "maximum_lock_stamp_difference",
      maximum_lock_stamp_difference_,
      0.05);

  if(!std::isfinite(maximum_lock_stamp_difference_) || maximum_lock_stamp_difference_ <= 0.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid maximum_lock_stamp_difference %.6f. "
        "Using 0.05 seconds.",
        maximum_lock_stamp_difference_);

    maximum_lock_stamp_difference_ = 0.05;
  }

  getParam<int>(
    imp_perch_nh,
    "equilibrium_wrench_required_samples",
    equilibrium_wrench_required_samples_,
    80);

  if(equilibrium_wrench_required_samples_ < 1)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid equilibrium_wrench_required_samples: %d. "
        "Using 80 samples.",
        equilibrium_wrench_required_samples_);

    equilibrium_wrench_required_samples_ = 80;
  }

  getParam<double>(
      imp_perch_nh,
      "contact_torque_filter_alpha",
      contact_torque_filter_alpha_,
      0.10);

  if(!std::isfinite(contact_torque_filter_alpha_) ||
     contact_torque_filter_alpha_ <= 0.0 ||
     contact_torque_filter_alpha_ > 1.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid contact_torque_filter_alpha %.6f. "
        "Using 0.10.",
        contact_torque_filter_alpha_);

    contact_torque_filter_alpha_ = 0.10;
  }

  getParam<double>(
      imp_perch_nh,
      "contact_on_threshold",
      contact_on_threshold_,
      0.04);

  getParam<double>(
      imp_perch_nh,
      "contact_off_threshold",
      contact_off_threshold_,
      0.02);

  if(!std::isfinite(contact_on_threshold_) ||
     !std::isfinite(contact_off_threshold_) ||
     contact_on_threshold_ <= contact_off_threshold_ ||
     contact_off_threshold_ < 0.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid contact thresholds on %.6f off %.6f. "
        "Using 0.04 / 0.02 Nm.",
        contact_on_threshold_,
        contact_off_threshold_);

    contact_on_threshold_ = 0.04;
    contact_off_threshold_ = 0.02;
  }

  getParam<double>(
      imp_perch_nh,
      "contact_on_duration",
      contact_on_duration_,
      0.10);

  getParam<double>(
      imp_perch_nh,
      "contact_off_duration",
      contact_off_duration_,
      0.20);

  if(!std::isfinite(contact_on_duration_) ||
     contact_on_duration_ < 0.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid contact_on_duration %.6f. "
        "Using 0.10 s.",
        contact_on_duration_);

    contact_on_duration_ = 0.10;
  }

  if(!std::isfinite(contact_off_duration_) ||
     contact_off_duration_ < 0.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid contact_off_duration %.6f. "
        "Using 0.20 s.",
        contact_off_duration_);

    contact_off_duration_ = 0.20;
  }

  getParam<double>(
      imp_perch_nh,
      "recovery_angle_epsilon",
      recovery_angle_epsilon_,
      0.00174533);

  if(!std::isfinite(recovery_angle_epsilon_) ||
     recovery_angle_epsilon_ <= 0.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid recovery_angle_epsilon %.6f. "
        "Using 0.00174533 rad.",
        recovery_angle_epsilon_);

    recovery_angle_epsilon_ = 0.00174533;
  }

  getParam<double>(
      imp_perch_nh,
      "recovery_rate_epsilon",
      recovery_rate_epsilon_,
      0.01);

  if(!std::isfinite(recovery_rate_epsilon_) ||
     recovery_rate_epsilon_ <= 0.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid recovery_rate_epsilon %.6f. "
        "Using 0.01 rad/s.",
        recovery_rate_epsilon_);

    recovery_rate_epsilon_ = 0.01;
  }

  getParam<double>(
      imp_perch_nh,
      "perching_pitch_torque_sign",
      perching_pitch_torque_sign_,
      1.0);

  if(!std::isfinite(perching_pitch_torque_sign_) ||
     perching_pitch_torque_sign_ == 0.0)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid perching_pitch_torque_sign %.6f. "
        "Using 1.0.",
        perching_pitch_torque_sign_);

    perching_pitch_torque_sign_ = 1.0;
  }

  perching_pitch_torque_sign_ =
      perching_pitch_torque_sign_ >= 0.0 ? 1.0 : -1.0;

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "equilibrium_wrench_required_samples: %d",
      equilibrium_wrench_required_samples_);

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "contact gate: alpha %.3f, on %.4f Nm for %.3f s, "
      "off %.4f Nm for %.3f s, recovery angle %.6f rad, "
      "rate %.6f rad/s, torque sign %.1f",
      contact_torque_filter_alpha_,
      contact_on_threshold_,
      contact_on_duration_,
      contact_off_threshold_,
      contact_off_duration_,
      recovery_angle_epsilon_,
      recovery_rate_epsilon_,
      perching_pitch_torque_sign_);


  ROS_WARN("[GimbalrotorPerchingAdmittanceController] perching_min_valid_radius: %.4f",
           min_valid_radius_);
  ROS_WARN("[GimbalrotorPerchingAdmittanceController] perching_max_pitch_delta deg: %.2f",
           max_pitch_delta_ * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingAdmittanceController] perching_arc_pitch_sign: %.2f",
           arc_pitch_sign_);
}

void GimbalrotorPerchingAdmittanceController::resetEquilibriumWrenchUnsafe() const
{
  /* The caller must already hold perching_state_mutex_. */
  equilibrium_wrench_pivot_world_.setZero();
  equilibrium_wrench_sum_.setZero();

  equilibrium_wrench_sample_count_ = 0;
  equilibrium_wrench_ready_ = false;
}

void GimbalrotorPerchingAdmittanceController::resetContactGateUnsafe()
{
  /* The caller must already hold perching_state_mutex_. */
  contact_active_ = false;
  recovery_active_ = false;

  contact_on_timer_ = 0.0;
  contact_off_timer_ = 0.0;

  residual_pitch_torque_raw_ = 0.0;
  residual_pitch_torque_filtered_ = 0.0;

  previous_contact_gate_time_ = ros::Time(0);

  prepared_perching_admittance_wrench_world_.setZero();
}

void GimbalrotorPerchingAdmittanceController::
enterZeroInputRecoveryUnsafe()
{
  /*
   * The caller must already hold perching_state_mutex_.
   */

  const double pitch_offset =
      admittance_output_
          .angle_offset_compliance(1);

  const double pitch_offset_rate =
      admittance_output_
          .angular_vel_offset_compliance(1);

  const bool finite_output =
      std::isfinite(pitch_offset) &&
      std::isfinite(pitch_offset_rate);

  const bool nonzero_admittance_state =
      finite_output &&
      (std::abs(pitch_offset) >
           recovery_angle_epsilon_ ||
       std::abs(pitch_offset_rate) >
           recovery_rate_epsilon_);

  /*
   * Preserve recovery when:
   *
   * - contact was active;
   * - recovery had already started; or
   * - the admittance core still contains a nonzero correction.
   */
  const bool should_recover =
      finite_output &&
      perching_admittance_enabled_ &&
      has_locked_pose_ &&
      equilibrium_wrench_ready_ &&
      (contact_active_ ||
       recovery_active_ ||
       nonzero_admittance_state);

  contact_active_ = false;
  recovery_active_ = should_recover;

  contact_on_timer_ = 0.0;
  contact_off_timer_ = 0.0;

  residual_pitch_torque_raw_ = 0.0;
  residual_pitch_torque_filtered_ = 0.0;

  previous_contact_gate_time_ =
      ros::Time(0);

  /*
   * Recovery means zero external torque enters the admittance core.
   * The existing virtual damping and stiffness return the stored
   * angle/rate states smoothly toward zero.
   */
  prepared_perching_admittance_wrench_world_
      .setZero();

  /*
   * A non-finite admittance state cannot safely recover.
   * Request a complete core reset instead.
   */
  if(!finite_output)
  {
    recovery_active_ = false;
    admittance_reset_requested_ = true;

    ROS_ERROR_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Non-finite admittance output detected. "
        "Requesting admittance reset.");
  }
}

void GimbalrotorPerchingAdmittanceController::updateContactGate(
    double residual_pitch_torque,
    double dt)
{
  const double magnitude = std::abs(residual_pitch_torque);

  if(!contact_active_)
  {
    contact_off_timer_ = 0.0;

    if(magnitude > contact_on_threshold_)
    {
      contact_on_timer_ += dt;

      if(contact_on_timer_ >= contact_on_duration_)
      {
        contact_active_ = true;
        recovery_active_ = false;
        contact_on_timer_ = 0.0;

        ROS_WARN(
            "[GimbalrotorPerchingAdmittanceController] "
            "Contact admittance active: filtered residual pitch torque %.4f Nm.",
            residual_pitch_torque);
      }
    }
    else
    {
      contact_on_timer_ = 0.0;
    }

    return;
  }

  contact_on_timer_ = 0.0;

  if(magnitude < contact_off_threshold_)
  {
    contact_off_timer_ += dt;

    if(contact_off_timer_ >= contact_off_duration_)
    {
      contact_active_ = false;
      recovery_active_ = true;
      contact_off_timer_ = 0.0;

      ROS_WARN(
          "[GimbalrotorPerchingAdmittanceController] "
          "Contact released: recovering pitch admittance offset.");
    }
  }
  else
  {
    contact_off_timer_ = 0.0;
  }
}

bool GimbalrotorPerchingAdmittanceController::recoveryComplete() const
{
  if(!perchingPitchOutputFinite(admittance_output_))
  {
    return false;
  }

  return std::abs(admittance_output_.angle_offset_compliance(1)) <=
             recovery_angle_epsilon_ &&
         std::abs(admittance_output_.angular_vel_offset_compliance(1)) <=
             recovery_rate_epsilon_;
}

bool GimbalrotorPerchingAdmittanceController::
perchingPitchAdmittanceConfigValid() const
{
  const double torque_lpf_alpha =
      admittance_config_.torque_lpf_alpha;

  const double inertia =
      admittance_config_.rot_virtual_inertia(1);

  const double damping =
      admittance_config_.rot_damping(1);

  const double stiffness =
      admittance_config_.rot_stiffness(1);

  const double torque_ref =
      admittance_config_.torque_ref(1);

  const double torque_limit =
      admittance_config_.torque_limit(1);

  const double angle_limit =
      admittance_config_.angle_offset_limit(1);

  const double rate_limit =
      admittance_config_.angular_vel_offset_limit(1);

  return admittance_config_.use_admittance &&
         std::isfinite(admittance_config_.rot_enable(1)) &&
         admittance_config_.rot_enable(1) >= 0.5 &&
         std::isfinite(torque_lpf_alpha) &&
         torque_lpf_alpha > 0.0 &&
         torque_lpf_alpha <= 1.0 &&
         std::isfinite(inertia) &&
         inertia > 0.0 &&
         std::isfinite(damping) &&
         damping > 0.0 &&
         std::isfinite(stiffness) &&
         stiffness > 0.0 &&
         std::isfinite(torque_ref) &&
         std::abs(torque_ref) <= 1.0e-9 &&
         std::isfinite(torque_limit) &&
         torque_limit > 0.0 &&
         std::isfinite(angle_limit) &&
         angle_limit > recovery_angle_epsilon_ &&
         std::isfinite(rate_limit) &&
         rate_limit > recovery_rate_epsilon_;
}

bool GimbalrotorPerchingAdmittanceController::
perchingPitchOutputFinite(
    const AdmittanceCoreOutput& output) const
{
  return output.valid &&
         std::isfinite(output.torque_compliance(1)) &&
         std::isfinite(output.angle_offset_compliance(1)) &&
         std::isfinite(output.angular_vel_offset_compliance(1)) &&
         std::isfinite(output.angular_acc_offset_compliance(1));
}

double GimbalrotorPerchingAdmittanceController::safeAdmittanceDt() const
{
  if(std::isfinite(ctrl_loop_du_) &&
     ctrl_loop_du_ > 0.0 &&
     ctrl_loop_du_ <= 0.1)
  {
    return ctrl_loop_du_;
  }

  return 0.025;
}

void GimbalrotorPerchingAdmittanceController::
publishContactAdmittanceDiagnostics() const
{
  bool contact_active = false;
  bool recovery_active = false;
  double raw_torque = 0.0;
  double filtered_torque = 0.0;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    contact_active = contact_active_;
    recovery_active = recovery_active_;
    raw_torque = residual_pitch_torque_raw_;
    filtered_torque = residual_pitch_torque_filtered_;
  }

  double pitch_offset = 0.0;

  if(admittance_output_.valid &&
     std::isfinite(admittance_output_.angle_offset_compliance(1)))
  {
    pitch_offset = admittance_output_.angle_offset_compliance(1);
  }

  std_msgs::Bool bool_msg;
  std_msgs::Float64 float_msg;

  bool_msg.data = contact_active;
  contact_active_pub_.publish(bool_msg);

  bool_msg.data = recovery_active;
  recovery_active_pub_.publish(bool_msg);

  float_msg.data = raw_torque;
  pivot_torque_raw_pub_.publish(float_msg);

  float_msg.data = filtered_torque;
  pivot_torque_filtered_pub_.publish(float_msg);

  float_msg.data = pitch_offset;
  pitch_offset_pub_.publish(float_msg);
}

void GimbalrotorPerchingAdmittanceController::
preparePerchingAdmittanceInput()
{
  bool perching_active = false;
  bool arm_enabled = false;
  bool lock_valid = false;
  bool tare_ready = false;

  tf::Vector3 pivot_world_tf;
  Eigen::Matrix3d R_world_constraint = Eigen::Matrix3d::Identity();
  Eigen::Vector3d constraint_axis_world = Eigen::Vector3d::UnitY();
  ros::Time accepted_pose_stamp_snapshot;
  ros::Time accepted_pivot_stamp_snapshot;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    prepared_perching_admittance_wrench_world_.setZero();

    perching_active = perching_enabled_for_constraint_;
    arm_enabled = perching_admittance_enabled_;
    lock_valid = has_locked_pose_ && perching_constraint_valid_;
    tare_ready = equilibrium_wrench_ready_;
    pivot_world_tf = locked_pivot_world_;
    R_world_constraint = R_world_constraint_;
    constraint_axis_world = constraint_axis_world_;
    accepted_pose_stamp_snapshot = accepted_locked_pose_stamp_;
    accepted_pivot_stamp_snapshot = accepted_locked_pivot_stamp_;

    prepared_R_world_constraint_ = R_world_constraint;

    if(!perching_active || !lock_valid)
    {
      resetContactGateUnsafe();
      return;
    }
  }


  const ExternalWrenchSnapshot wrench_snapshot = getExternalWrenchSnapshot();

  if(!wrench_snapshot.available)
  {
    {
      std::lock_guard<std::mutex> lock(perching_state_mutex_);

      /*
      * Remove unknown external forcing while preserving zero-input recovery of an existing correction.
      */
      enterZeroInputRecoveryUnsafe();
    }

    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "No estimated external wrench has been received. "
        "Contact forcing is zero.");

    return;
  }

  const double wrench_age = (ros::Time::now() - wrench_snapshot.receive_stamp).toSec();

  if(!std::isfinite(wrench_age) || wrench_age < 0.0 || wrench_age > external_wrench_timeout_)
  {
    {
      std::lock_guard<std::mutex> lock(perching_state_mutex_);

      enterZeroInputRecoveryUnsafe();
    }

    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Estimated external wrench is stale: %.3f s. "
        "Contact forcing is zero; an existing correction "
        "is returning through zero-input recovery.",
        wrench_age);

    return;
  }

  const Eigen::Matrix<double, 6, 1> wrench_cog_world = transformExternalWrenchToWorld(wrench_snapshot.raw_wrench);

  const ros::Time measurement_stamp =
      wrench_snapshot.measurement_stamp.isZero() ?
          wrench_snapshot.receive_stamp :
          wrench_snapshot.measurement_stamp;

  const std::uint64_t wrench_sequence = wrench_snapshot.sequence;
      
  const tf::Vector3 cog_pos_world_tf = estimator_->getPos(Frame::COG, estimate_mode_);

  const Eigen::Vector3d cog_pos_world(
      cog_pos_world_tf.x(),
      cog_pos_world_tf.y(),
      cog_pos_world_tf.z());

  const Eigen::Vector3d pivot_pos_world(
      pivot_world_tf.x(),
      pivot_world_tf.y(),
      pivot_world_tf.z());

  const bool finite_input =
      wrench_cog_world.allFinite() &&
      cog_pos_world.allFinite() &&
      pivot_pos_world.allFinite() &&
      R_world_constraint.allFinite() &&
      constraint_axis_world.allFinite();

  const double constraint_axis_norm = constraint_axis_world.norm();

  if(!finite_input || !std::isfinite(constraint_axis_norm) || constraint_axis_norm <= 1.0e-6)
  {
    {
      std::lock_guard<std::mutex> lock(perching_state_mutex_);

      enterZeroInputRecoveryUnsafe();
    }

    ROS_ERROR_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Rejected non-finite external wrench or "
        "perching geometry.");

    return;
  }

  constraint_axis_world /= constraint_axis_norm;

  const Eigen::Vector3d force_world = wrench_cog_world.head<3>();
  const Eigen::Vector3d torque_cog_world = wrench_cog_world.tail<3>();
  const Eigen::Vector3d torque_pivot_world =
      torque_cog_world +
      (cog_pos_world - pivot_pos_world).cross(force_world);

  Eigen::Matrix<double, 6, 1> wrench_pivot_world = wrench_cog_world;
  wrench_pivot_world.tail<3>() = torque_pivot_world;

  Eigen::Matrix<double, 6, 1> wrench_pivot_frame;
  wrench_pivot_frame.head<3>() =
      R_world_constraint.transpose() *
      wrench_pivot_world.head<3>();
  wrench_pivot_frame.tail<3>() =
      R_world_constraint.transpose() *
      wrench_pivot_world.tail<3>();

  if(!torque_pivot_world.allFinite() ||
     !wrench_pivot_world.allFinite() ||
     !wrench_pivot_frame.allFinite())
  {
    {
      std::lock_guard<std::mutex> lock(perching_state_mutex_);

      enterZeroInputRecoveryUnsafe();
    }

    ROS_ERROR_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Rejected non-finite pivot wrench.");

    return;
  }

  Eigen::Matrix<double, 6, 1> accepted_equilibrium_wrench = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> residual_wrench_pivot_world = Eigen::Matrix<double, 6, 1>::Zero();

  bool tare_completed_now = false;
  bool tare_accumulation_failed = false;
  int collected_samples = 0;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    const bool same_lock =
        has_locked_pose_ &&
        accepted_locked_pose_stamp_ == accepted_pose_stamp_snapshot &&
        accepted_locked_pivot_stamp_ == accepted_pivot_stamp_snapshot;

    if(!perching_enabled_for_constraint_ || !same_lock)
    {
      prepared_perching_admittance_wrench_world_.setZero();
      return;
    }

    if(wrench_sequence != last_published_pivot_wrench_sequence_)
    {
      /*
      * Publish exactly once for each received estimated_external_wrench sample.
       */
      publishPivotWrenchFrame(pivot_world_tf, R_world_constraint, measurement_stamp);

      geometry_msgs::WrenchStamped pivot_wrench_msg;

      pivot_wrench_msg.header.stamp = measurement_stamp;

      pivot_wrench_msg.header.frame_id = perching_pivot_wrench_frame_id_;

      pivot_wrench_msg.wrench.force.x = wrench_pivot_frame(0);
      pivot_wrench_msg.wrench.force.y = wrench_pivot_frame(1);
      pivot_wrench_msg.wrench.force.z = wrench_pivot_frame(2);
      pivot_wrench_msg.wrench.torque.x = wrench_pivot_frame(3);
      pivot_wrench_msg.wrench.torque.y = wrench_pivot_frame(4);
      pivot_wrench_msg.wrench.torque.z = wrench_pivot_frame(5);
      pivot_external_wrench_est_pub_.publish(pivot_wrench_msg);

      last_published_pivot_wrench_sequence_ = wrench_sequence;
    }

    arm_enabled = perching_admittance_enabled_;
    tare_ready = equilibrium_wrench_ready_;

    if(!arm_enabled)
    {
      if(!equilibrium_wrench_ready_)
      {
        equilibrium_wrench_sum_ += wrench_pivot_world;
        ++equilibrium_wrench_sample_count_;

        if(!equilibrium_wrench_sum_.allFinite())
        {
          resetEquilibriumWrenchUnsafe();
          resetContactGateUnsafe();
          tare_accumulation_failed = true;
        }
        else if(equilibrium_wrench_sample_count_ >=
           equilibrium_wrench_required_samples_)
        {
          equilibrium_wrench_pivot_world_ =
              equilibrium_wrench_sum_ /
              static_cast<double>(equilibrium_wrench_sample_count_);

          if(equilibrium_wrench_pivot_world_.allFinite())
          {
            equilibrium_wrench_ready_ = true;
            tare_completed_now = true;
          }
          else
          {
            resetEquilibriumWrenchUnsafe();
            resetContactGateUnsafe();
            tare_accumulation_failed = true;
          }
        }
      }

      collected_samples = equilibrium_wrench_sample_count_;
      accepted_equilibrium_wrench = equilibrium_wrench_pivot_world_;

      resetContactGateUnsafe();
    }
    else
    {
      collected_samples = equilibrium_wrench_sample_count_;
      accepted_equilibrium_wrench = equilibrium_wrench_pivot_world_;

      if(tare_ready)
      {
        residual_wrench_pivot_world =
            wrench_pivot_world -
            equilibrium_wrench_pivot_world_;
      }
    }
  }

  if(tare_accumulation_failed)
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Equilibrium-wrench accumulation became "
        "non-finite and was restarted.");

    return;
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

  if(!arm_enabled)
  {
    ROS_INFO_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Collecting equilibrium pivot wrench: %d / %d.",
        collected_samples,
        equilibrium_wrench_required_samples_);

    return;
  }

  if(!tare_ready)
  {
    {
      std::lock_guard<std::mutex> lock(perching_state_mutex_);

      perching_admittance_enabled_ = false;
      resetEquilibriumWrenchUnsafe();
      resetContactGateUnsafe();
      admittance_reset_requested_ = true;
    }

    ROS_ERROR_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Perching admittance was armed without a valid "
        "equilibrium wrench. It was disarmed and tare "
        "collection was restarted.");

    return;
  }

  const Eigen::Vector3d residual_torque_constraint =
      R_world_constraint.transpose() *
      residual_wrench_pivot_world.tail<3>();

  const double residual_pitch_torque =
      perching_pitch_torque_sign_ *
      residual_torque_constraint(1);

  if(!residual_wrench_pivot_world.allFinite() ||
     !residual_torque_constraint.allFinite() ||
     !std::isfinite(residual_pitch_torque))
  {
    {
      std::lock_guard<std::mutex> lock(perching_state_mutex_);

      enterZeroInputRecoveryUnsafe();
    }

    ROS_ERROR_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Rejected non-finite residual pivot torque.");

    return;
  }

  const ros::Time now = ros::Time::now();

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    const bool same_lock =
        has_locked_pose_ &&
        accepted_locked_pose_stamp_ == accepted_pose_stamp_snapshot &&
        accepted_locked_pivot_stamp_ == accepted_pivot_stamp_snapshot;

    if(!perching_enabled_for_constraint_ ||
       !perching_admittance_enabled_ ||
       !equilibrium_wrench_ready_ ||
       !same_lock)
    {
      prepared_perching_admittance_wrench_world_.setZero();
      return;
    }

    residual_pitch_torque_raw_ = residual_pitch_torque;

    double dt = 0.0;
    bool valid_dt = false;

    if(previous_contact_gate_time_.isZero())
    {
      previous_contact_gate_time_ = now;
    }
    else
    {
      dt = (now - previous_contact_gate_time_).toSec();
      previous_contact_gate_time_ = now;
      valid_dt = std::isfinite(dt) && dt > 0.0 && dt <= 0.1;
    }

    if(valid_dt)
    {
      residual_pitch_torque_filtered_ =
          contact_torque_filter_alpha_ *
          residual_pitch_torque_raw_ +
          (1.0 - contact_torque_filter_alpha_) *
          residual_pitch_torque_filtered_;

      updateContactGate(
          residual_pitch_torque_filtered_,
          dt);
    }
    else if(dt != 0.0)
    {
      /*
       * Invalid control timing means the measured torque must not
       * be applied. Preserve a valid existing correction through
       * zero-input recovery rather than resetting it abruptly.
       */
      enterZeroInputRecoveryUnsafe();

      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorPerchingAdmittanceController] "
          "Invalid contact-gate dt %.6f s. "
          "Contact forcing removed; recovery retained.",
          dt);
    }

    if(contact_active_)
    {
      prepared_perching_admittance_wrench_world_.setZero();
      prepared_perching_admittance_wrench_world_.tail<3>() =
          constraint_axis_world *
          residual_pitch_torque_filtered_;
    }
    else
    {
      prepared_perching_admittance_wrench_world_.setZero();
    }
  }
}

void GimbalrotorPerchingAdmittanceController::controlCore()
{
  const bool servo_neutral_mode = perching_servo_neutral_mode_;
  bool effective_enabled = false;
  bool reset_requested = false;
  bool perching_active = false;

  /*
   * Suppress the ROS pitch integral term only while
   * contact admittance is active or recovering.
   *
   * Perching mode by itself does not suppress the integral.
   */
  bool suppress_pitch_i_limit = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    perching_active = perching_enabled_for_constraint_;

    if(servo_neutral_mode)
    {
      const bool had_active_admittance_state =
          perching_admittance_enabled_ ||
          contact_active_ ||
          recovery_active_ ||
          effective_admittance_enabled_;

      /*
       * Takeoff and landing must use the exact navigator pitch/arc target.
       * A compliance offset would make target position and pitch inconsistent.
       */
      perching_admittance_enabled_ = false;
      resetContactGateUnsafe();

      if(had_active_admittance_state)
      {
        admittance_reset_requested_ = true;
      }
    }

    if(recovery_active_ && recoveryComplete())
    {
      recovery_active_ = false;
      admittance_reset_requested_ = true;

      ROS_WARN(
          "[GimbalrotorPerchingAdmittanceController] "
          "Pitch admittance recovery complete.");
    }
  }

  if(perching_active &&
     !servo_neutral_mode)
  {
    preparePerchingAdmittanceInput();
  }

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    suppress_pitch_i_limit =
        !servo_neutral_mode &&
        perching_enabled_for_constraint_ &&
        perching_admittance_enabled_ &&
        (contact_active_ || recovery_active_);

    if(perching_enabled_for_constraint_)
    {
      /*
       * Publishing perching/admittance_enable=true arms the
       * contact-gated controller.
       *
       * While armed, keep the core disabled until contact is
       * detected. During recovery, keep the core enabled with
       * zero input so damping and stiffness return the offset.
       *
       *   - no contact: PID only;
       *   - contact: admittance correction + PID;
       *   - recovery: zero-input admittance + PID.
       */
      effective_enabled =
          !servo_neutral_mode &&
          perching_admittance_enabled_ &&
          has_locked_pose_ &&
          equilibrium_wrench_ready_ &&
          (contact_active_ || recovery_active_);
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
    if(admittance_reset_requested_)
    {
      reset_requested = true;
      admittance_reset_requested_ = false;
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

      ROS_WARN(
          "[GimbalrotorPerchingAdmittanceController] "
          "Perching admittance enabled: pitch limit_i changed from %.6f to 0.0.",
          normal_pitch_i_limit_);
    }

    /*
     * Force the integral output limit to zero.
     */
    pitch_pid.setLimitI(0.0);

    /*
     * limit_i = 0 only forces the resulting I term to zero.
     * The stored integral error would otherwise continue
     * accumulating, so clear it before PID::update().
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

    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Perching admittance disabled: pitch limit_i restored to %.6f.",
        normal_pitch_i_limit_);
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

  if(admittance_enabled_)
  {
    const ros::Time now = ros::Time::now();
    const double pending_dt = (now - prev_admittance_time_).toSec();

    if(!std::isfinite(pending_dt) ||
       pending_dt <= 0.0 ||
       pending_dt > 0.1)
    {
      prev_admittance_time_ =
          now - ros::Duration(safeAdmittanceDt());

      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorPerchingAdmittanceController] "
          "Sanitized invalid admittance dt %.6f s.",
          pending_dt);
    }
  }

  /*
   * This eventually calls PID::update().
   *
   * During contact or recovery, pitch limit_i is already zero,
   * so the calculated pitch I term will be clamped to zero.
   */
  GimbalrotorAdmittanceController::controlCore();

  /*
   * PID::update() still integrates err_i even when limit_i
   * is zero. Clear the stored error after the update so it
   * cannot build up while contact admittance is active.
   */
  if(suppress_pitch_i_limit)
  {
    pitch_pid.setErrI(0.0);
  }

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    if(recovery_active_ && recoveryComplete())
    {
      recovery_active_ = false;
      admittance_reset_requested_ = true;

      ROS_WARN(
          "[GimbalrotorPerchingAdmittanceController] "
          "Pitch admittance recovery complete.");
    }
  }

  publishContactAdmittanceDiagnostics();
}

void GimbalrotorPerchingAdmittanceController::perchingEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  bool mode_changed = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    mode_changed = perching_enabled_for_constraint_ != msg->data;

    if(!mode_changed)
    {
      return;
    }

    perching_enabled_for_constraint_ = msg->data;

    /*
     * A mode change invalidates the previous lock-dependent equilibrium wrench and contact state.
     */
    perching_admittance_enabled_ = false;

    resetEquilibriumWrenchUnsafe();
    resetContactGateUnsafe();

    admittance_reset_requested_ = true;
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

void GimbalrotorPerchingAdmittanceController::constraintValidCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
  bool became_valid = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    if(perching_constraint_valid_ == msg->data)
    {
      return;
    }

    perching_constraint_valid_ = msg->data;
    became_valid = perching_constraint_valid_;

    if(!perching_constraint_valid_)
    {
      has_locked_pose_ = false;
      has_locked_pose_msg_ = false;
      has_locked_pivot_ = false;

      locked_pose_stamp_ = ros::Time(0);
      locked_pivot_stamp_ = ros::Time(0);
      accepted_locked_pose_stamp_ = ros::Time(0);
      accepted_locked_pivot_stamp_ = ros::Time(0);

      resetEquilibriumWrenchUnsafe();
      resetContactGateUnsafe();
      admittance_reset_requested_ = true;
    }
  }

  if(became_valid)
  {
    updateLockedConstraintFromLockedPoseAndPivot();
  }
}

void GimbalrotorPerchingAdmittanceController::perchingPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  perching_point_world_.setValue(
      msg->point.x,
      msg->point.y,
      msg->point.z);

  has_perching_point_ = true;
}

void GimbalrotorPerchingAdmittanceController::branchPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  branch_pos_world_.setValue(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);

  has_branch_pose_ = true;

  if(!has_perching_point_ && use_branch_pose_if_no_point_)
  {
    perching_point_world_ = branch_pos_world_;
  }
}

void GimbalrotorPerchingAdmittanceController::lockedPivotCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  const bool finite_pivot =
      std::isfinite(msg->point.x) &&
      std::isfinite(msg->point.y) &&
      std::isfinite(msg->point.z);

  if(!finite_pivot)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Rejected locked pivot containing non-finite values.");

    return;
  }

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    locked_pivot_world_.setValue(
        msg->point.x,
        msg->point.y,
        msg->point.z);

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

void GimbalrotorPerchingAdmittanceController::
lockedPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  const geometry_msgs::Point& p =
      msg->pose.position;

  const geometry_msgs::Quaternion& q =
      msg->pose.orientation;

  const bool finite_position =
      std::isfinite(p.x) &&
      std::isfinite(p.y) &&
      std::isfinite(p.z);

  const bool finite_quaternion =
      std::isfinite(q.x) &&
      std::isfinite(q.y) &&
      std::isfinite(q.z) &&
      std::isfinite(q.w);

  const double quaternion_norm =
      std::sqrt(
          q.x * q.x +
          q.y * q.y +
          q.z * q.z +
          q.w * q.w);

  if(!finite_position ||
     !finite_quaternion ||
     !std::isfinite(quaternion_norm) ||
     quaternion_norm < 1.0e-6)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Rejected locked pose with invalid position "
        "or quaternion.");

    return;
  }

  {
    std::lock_guard<std::mutex> lock(
        perching_state_mutex_);

    poseMsgToTfPosRpy(
        *msg,
        locked_robot_pos_world_,
        locked_robot_rpy_);

    if(msg->header.stamp.isZero())
    {
      locked_pose_stamp_ =
          ros::Time::now();
    }
    else
    {
      locked_pose_stamp_ =
          msg->header.stamp;
    }

    has_locked_pose_msg_ = true;
  }

  /*
   * This function locks perching_state_mutex_ internally.
   */
  updateLockedConstraintFromLockedPoseAndPivot();
}

void GimbalrotorPerchingAdmittanceController::updateLockedConstraintFromLockedPoseAndPivot()
{
  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  const bool previously_valid = has_locked_pose_;

  if(require_perching_lock_ && !perching_constraint_valid_)
  {
    has_locked_pose_ = false;

    if(previously_valid)
    {
      resetContactGateUnsafe();
      admittance_reset_requested_ = true;
    }

    return;
  }

  /*
   * A locked robot pose is always required.
   */
  if(!has_locked_pose_msg_)
  {
    has_locked_pose_ = false;

    if(previously_valid)
    {
      resetContactGateUnsafe();
      admittance_reset_requested_ = true;
    }

    return;
  }

  /*
   * If require_perching_lock is enabled, accept only
   * the pivot published by the navigator's lock logic.
   */
  if(!has_locked_pivot_)
  {
    if(require_perching_lock_)
    {
      has_locked_pose_ = false;

      if(previously_valid)
      {
        resetContactGateUnsafe();
        admittance_reset_requested_ = true;
      }

      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorPerchingAdmittanceController] "
          "Waiting for locked pivot from navigator.");

      return;
    }

    /*
     * Fallback behavior is allowed only when
     * require_perching_lock is false.
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
        resetContactGateUnsafe();
        admittance_reset_requested_ = true;
      }

      return;
    }
  }

  /*
   * When using navigator lock messages, ensure that
   * pose and pivot belong to the same lock operation.
   */
  if(require_perching_lock_)
  {
    if(locked_pose_stamp_.isZero() || locked_pivot_stamp_.isZero())
    {
      has_locked_pose_ = false;

      if(previously_valid)
      {
        resetContactGateUnsafe();
        admittance_reset_requested_ = true;
      }

      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorPerchingAdmittanceController] "
          "Locked pose or pivot has an invalid timestamp.");

      return;
    }

    const double stamp_difference = std::abs((locked_pose_stamp_ - locked_pivot_stamp_).toSec());

    if(stamp_difference > maximum_lock_stamp_difference_)
    {
      has_locked_pose_ = false;

      if(previously_valid)
      {
        resetContactGateUnsafe();
        admittance_reset_requested_ = true;
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
   * Ignore duplicate delivery of the already accepted
   * latched pose/pivot pair.
   */
  if(has_locked_pose_ &&
     locked_pose_stamp_ == accepted_locked_pose_stamp_ &&
     locked_pivot_stamp_ == accepted_locked_pivot_stamp_)
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

  if(!std::isfinite(locked_radius_) || locked_radius_ < min_valid_radius_)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
      resetContactGateUnsafe();
      admittance_reset_requested_ = true;
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
   * Minimum current implementation:
   * physical branch axis is world Y.
   */
  constraint_axis_world_ = Eigen::Vector3d::UnitY();

  Eigen::Vector3d radial_world(
      locked_radius_vec_world_.x(),
      locked_radius_vec_world_.y(),
      locked_radius_vec_world_.z());

  /*
   * Remove the component parallel to the branch.
   */
  radial_world -= constraint_axis_world_ * constraint_axis_world_.dot(radial_world);

  const double radial_norm = radial_world.norm();

  if(!std::isfinite(radial_norm) || radial_norm < 1.0e-6)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
      resetContactGateUnsafe();
      admittance_reset_requested_ = true;
    }

    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Cannot build constraint frame: "
        "invalid radial vector.");

    return;
  }

  radial_world.normalize();

  /*
   * Use radial x branch axis so the resulting frame
   * is right-handed:
   *
   * constraint X x constraint Y = constraint Z.
   */
  Eigen::Vector3d tangent_world = radial_world.cross(constraint_axis_world_);

  const double tangent_norm = tangent_world.norm();

  if(!std::isfinite(tangent_norm) || tangent_norm < 1.0e-6)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
      resetContactGateUnsafe();
      admittance_reset_requested_ = true;
    }

    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Cannot build constraint frame: "
        "invalid tangent vector.");

    return;
  }

  tangent_world.normalize();

  /*
   * Columns transform constraint-frame vectors
   * into world-frame vectors:
   *
   * constraint X = radial
   * constraint Y = branch axis
   * constraint Z = tangent
   */
  R_world_constraint_.col(0) = radial_world;
  R_world_constraint_.col(1) = constraint_axis_world_;
  R_world_constraint_.col(2) = tangent_world;

  const double determinant = R_world_constraint_.determinant();

  if(!std::isfinite(determinant) || std::abs(determinant - 1.0) > 1.0e-3)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
      resetContactGateUnsafe();
      admittance_reset_requested_ = true;
    }

    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Invalid constraint rotation determinant: "
        "%.6f",
        determinant);

    return;
  }

  has_locked_pose_ = true;
  accepted_locked_pose_stamp_ = locked_pose_stamp_;
  accepted_locked_pivot_stamp_ = locked_pivot_stamp_;

  /*
   * A new lock changes the pivot, radius, nominal pose and
   * possibly the constraint frame.
   *
   * Disarm contact-gated admittance so a fresh no-contact
   * equilibrium wrench can be collected for this lock.
   */
  perching_admittance_enabled_ = false;

  resetEquilibriumWrenchUnsafe();
  resetContactGateUnsafe();

  /*
   * A new pivot/frame must not inherit displacement,
   * velocity or filtering state from the old frame.
   */
  admittance_reset_requested_ = true;

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "Locked perching constraint accepted.");

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "Locked pivot: %.3f %.3f %.3f",
      locked_pivot_world_.x(),
      locked_pivot_world_.y(),
      locked_pivot_world_.z());

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "Locked position: %.3f %.3f %.3f",
      locked_robot_pos_world_.x(),
      locked_robot_pos_world_.y(),
      locked_robot_pos_world_.z());

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "Locked pitch: %.2f deg",
      locked_robot_rpy_.y() *
      180.0 / PI);

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "Pitch-plane radius: %.3f m",
      locked_radius_);

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "Constraint determinant: %.6f",
      determinant);
}

Eigen::Matrix3d GimbalrotorPerchingAdmittanceController::getComplianceToWorldRotation() const
{
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    if(perching_enabled_for_constraint_)
    {
      return prepared_R_world_constraint_;
    }
  }

  return GimbalrotorAdmittanceController::getComplianceToWorldRotation();
}

void GimbalrotorPerchingAdmittanceController::perchingAdmittanceEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  const bool config_valid = perchingPitchAdmittanceConfigValid();

  bool enable_accepted = false;
  bool became_armed = false;
  bool fresh_tare_requested = false;

  bool lock_valid = false;
  bool tare_ready = false;

  int collected_samples = 0;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    lock_valid = perching_enabled_for_constraint_ && has_locked_pose_;
    tare_ready = equilibrium_wrench_ready_;
    collected_samples = equilibrium_wrench_sample_count_;

    if(!msg->data)
    {
      perching_admittance_enabled_ = false;

      /*
       * Publishing false explicitly requests a fresh
       * equilibrium-wrench collection.
       */
      resetEquilibriumWrenchUnsafe();
      resetContactGateUnsafe();

      admittance_reset_requested_ = true;

      enable_accepted = true;
      fresh_tare_requested = true;
    }
    else if(!config_valid)
    {
      perching_admittance_enabled_ = false;

      resetContactGateUnsafe();

      admittance_reset_requested_ = true;
    }
    else if(!lock_valid)
    {
      /*
       * Never arm pivot admittance without a valid
       * perching lock.
       */
      perching_admittance_enabled_ = false;

      resetContactGateUnsafe();

      admittance_reset_requested_ = true;
    }
    else if(!tare_ready)
    {
      /*
       * Do not arm admittance before the no-contact
       * equilibrium wrench has been collected.
       */
      perching_admittance_enabled_ = false;

      resetContactGateUnsafe();

      admittance_reset_requested_ = true;
    }
    else
    {
      if(!perching_admittance_enabled_)
      {
        perching_admittance_enabled_ = true;
        admittance_reset_requested_ = true;
        became_armed = true;
      }

      enable_accepted = true;
    }
  }

  if(fresh_tare_requested)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Perching admittance disabled. "
        "Fresh equilibrium-wrench collection requested.");
  }
  else if(!config_valid)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Cannot enable perching admittance: invalid pitch "
        "admittance configuration. Check use_admittance, "
        "enable_pitch, pitch inertia/damping/stiffness, "
        "torque_ref_pitch, and pitch limits.");
  }
  else if(!lock_valid)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Cannot enable perching admittance: "
        "no valid perching lock.");
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
        collected_samples,
        equilibrium_wrench_required_samples_);
  }
  else if(enable_accepted && became_armed)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Perching admittance armed; waiting for residual "
        "pivot torque contact.");
  }
}

void GimbalrotorPerchingAdmittanceController::normalAdmittanceEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  bool state_changed = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    state_changed = normal_admittance_enabled_ != msg->data;

    if(state_changed)
    {
      normal_admittance_enabled_ = msg->data;

      if(!perching_enabled_for_constraint_)
      {
        admittance_reset_requested_ = true;
      }
    }
  }

  if(!state_changed)
  {
    return;
  }

  ROS_WARN("[GimbalrotorPerchingAdmittanceController] Normal admittance %s.", msg->data ? "enabled" : "disabled");
}

Eigen::Matrix<double, 6, 1> GimbalrotorPerchingAdmittanceController::getExternalWrenchWorld() const
{
  bool perching_active = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);
    perching_active = perching_enabled_for_constraint_;

    if(perching_active)
    {
      return prepared_perching_admittance_wrench_world_;
    }
  }

  return GimbalrotorAdmittanceController::getExternalWrenchWorld();
}

void GimbalrotorPerchingAdmittanceController::
applyAdmittanceOutputToNavigator(
    const tf::Vector3& original_target_pos,
    const tf::Vector3& original_target_rpy,
    const AdmittanceCoreOutput& output)
{
  bool perching_active = false;

  {
    std::lock_guard<std::mutex> lock(
        perching_state_mutex_);

    perching_active =
        perching_enabled_for_constraint_;
  }

  /*
   * Normal-flight path.
   */
  if(!perching_active)
  {
    GimbalrotorAdmittanceController::
        applyAdmittanceOutputToNavigator(
            original_target_pos,
            original_target_rpy,
            output);

    return;
  }

  tf::Vector3 modified_target_pos;
  tf::Vector3 modified_target_rpy;

  double nominal_pitch = 0.0;
  double admittance_pitch_offset = 0.0;
  double target_pitch = 0.0;

  {
    std::lock_guard<std::mutex> lock(
        perching_state_mutex_);

    /*
     * Recheck the state while holding the lock.
     * It may have changed after the first snapshot.
     */
    if(!perching_enabled_for_constraint_ ||
       !perching_admittance_enabled_ ||
       !has_locked_pose_ ||
       !(contact_active_ || recovery_active_) ||
       locked_radius_ < min_valid_radius_)
    {
      return;
    }

    if(!perchingPitchOutputFinite(output) ||
       !std::isfinite(original_target_rpy.y()))
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[GimbalrotorPerchingAdmittanceController] "
          "Rejected non-finite pitch admittance target injection.");

      return;
    }

  nominal_pitch = original_target_rpy.y();

  /*
  * Constraint coordinate Y is the branch axis.
  * Rotational admittance Y therefore produces
  * the perching pitch correction.
  */
  admittance_pitch_offset = output.angle_offset_compliance(1);

  /*
  * The final pitch command must obey the same
  * locked-pose pitch limit used by the perching arc.
  *
  * Without this clamp, the position can be generated
  * at the clamped angle while the attitude controller
  * receives an unclamped pitch target.
  */
  double target_pitch_delta =
      normalizeAngle(
          nominal_pitch +
          admittance_pitch_offset -
          locked_robot_rpy_.y());

  target_pitch_delta =
      clamp(
          target_pitch_delta,
          -max_pitch_delta_,
          max_pitch_delta_);

  target_pitch =
      normalizeAngle(
          locked_robot_rpy_.y() +
          target_pitch_delta);

    if(!std::isfinite(target_pitch))
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[GimbalrotorPerchingAdmittanceController] "
          "Rejected non-finite perching pitch target.");

      return;
    }

    /*
     * This function reads locked geometry.
     * The perching-state mutex is held here.
     */
    modified_target_pos =
        computePerchingArcPositionFromPitch(
            target_pitch,
            original_target_pos);

    if(!std::isfinite(modified_target_pos.x()) ||
       !std::isfinite(modified_target_pos.y()) ||
       !std::isfinite(modified_target_pos.z()))
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[GimbalrotorPerchingAdmittanceController] "
          "Rejected non-finite perching arc target.");

      return;
    }

    modified_target_rpy = original_target_rpy;
    modified_target_rpy.setY(target_pitch);
  }

  /*
   * Do not hold the state mutex while calling into navigator code.
   */
  navigator_->setTargetPos(modified_target_pos);
  navigator_->setTargetRPY(modified_target_rpy);
}

tf::Vector3
GimbalrotorPerchingAdmittanceController::computePerchingArcPositionFromPitch(
    double target_pitch,
    const tf::Vector3& original_target_pos) const
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
  double delta_pitch =
      normalizeAngle(target_pitch - locked_robot_rpy_.y());

  delta_pitch =
      clamp(delta_pitch, -max_pitch_delta_, max_pitch_delta_);

  const double signed_delta =
      arc_pitch_sign_ * delta_pitch;

  tf::Vector3 locked_radius_xz(
      locked_radius_vec_world_.x(),
      0.0,
      locked_radius_vec_world_.z());

  double length_xz =
      norm2D(
          locked_radius_xz.x(),
          locked_radius_xz.z());

  if(length_xz < 1.0e-6)
    {
      locked_radius_xz.setX(locked_x_side_ * locked_radius_);
      locked_radius_xz.setY(0.0);
      locked_radius_xz.setZ(0.0);
      length_xz = locked_radius_;
    }

  if(length_xz < 1.0e-6)
    {
      locked_radius_xz.setX(locked_x_side_);
      locked_radius_xz.setY(0.0);
      locked_radius_xz.setZ(0.0);
      length_xz = 1.0;
    }

  locked_radius_xz.setX(
      locked_radius_xz.x() / length_xz * locked_radius_);
  locked_radius_xz.setY(0.0);
  locked_radius_xz.setZ(
      locked_radius_xz.z() / length_xz * locked_radius_);

  tf::Matrix3x3 rot(
      tf::createQuaternionFromRPY(
          0.0,
          signed_delta,
          0.0));

  tf::Vector3 rotated_radius =
      rot * locked_radius_xz;

  double rotated_length_xz =
      norm2D(
          rotated_radius.x(),
          rotated_radius.z());

  if(rotated_length_xz < 1.0e-6)
    {
      rotated_radius.setX(locked_x_side_ * locked_radius_);
      rotated_radius.setY(0.0);
      rotated_radius.setZ(0.0);
    }
  else
    {
      rotated_radius.setX(
          rotated_radius.x() / rotated_length_xz * locked_radius_);
      rotated_radius.setY(0.0);
      rotated_radius.setZ(
          rotated_radius.z() / rotated_length_xz * locked_radius_);
    }

  tf::Vector3 target_pos =
      perching_point_world_ + rotated_radius;

  /*
   * Preserve Y generated by GimbalrotorPerchingNavigator.
   *
   * The navigator already handles Y deadband/compliance. This controller should
   * not duplicate that logic.
   */
  target_pos.setY(original_target_pos.y());

  return target_pos;
}

double GimbalrotorPerchingAdmittanceController::clamp(
    double value,
    double min_value,
    double max_value) const
{
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

double GimbalrotorPerchingAdmittanceController::normalizeAngle(
    double angle) const
{
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

double GimbalrotorPerchingAdmittanceController::norm2D(
    double x,
    double z) const
{
  return std::sqrt(x * x + z * z);
}

void GimbalrotorPerchingAdmittanceController::poseMsgToTfPosRpy(
    const geometry_msgs::PoseStamped& msg,
    tf::Vector3& pos,
    tf::Vector3& rpy) const
{
  pos.setValue(
      msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z);

  tf::Quaternion q;
  tf::quaternionMsgToTF(msg.pose.orientation, q);
  q.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  rpy.setValue(roll, pitch, yaw);
}

void GimbalrotorPerchingAdmittanceController::publishPivotWrenchFrame(
    const tf::Vector3& pivot_world,
    const Eigen::Matrix3d& R_world_constraint,
    const ros::Time& stamp)
{
  if(!R_world_constraint.allFinite())
    {
      ROS_WARN_THROTTLE(1.0,
                        "[GimbalrotorPerchingAdmittanceController] invalid pivot wrench frame rotation");
      return;
    }

  Eigen::Quaterniond q(R_world_constraint);
  q.normalize();
  if(!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
     !std::isfinite(q.y()) || !std::isfinite(q.z()))
    {
      ROS_WARN_THROTTLE(1.0,
                        "[GimbalrotorPerchingAdmittanceController] invalid pivot wrench frame quaternion");
      return;
    }

  tf::Transform transform;
  transform.setOrigin(pivot_world);
  transform.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
  pivot_wrench_tf_broadcaster_.sendTransform(
      tf::StampedTransform(transform, stamp, "world", perching_pivot_wrench_frame_id_));
}

} // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(
    aerial_robot_control::GimbalrotorPerchingAdmittanceController,
    aerial_robot_control::ControlBase)
