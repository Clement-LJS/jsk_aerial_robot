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
    normal_admittance_enabled_(false),
    perching_admittance_enabled_(false),
    effective_admittance_enabled_(false),
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
    admittance_reset_requested_(false)
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

  R_world_constraint_.setIdentity();

  constraint_axis_world_ = Eigen::Vector3d::UnitY();

  equilibrium_wrench_pivot_world_.setZero();
  equilibrium_wrench_sum_.setZero();
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

  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] initialized.");
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] perching_enable_topic: "
                  << perching_enable_topic_for_constraint_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] perching_point_topic: "
                  << perching_point_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] branch_pose_topic: "
                  << perching_branch_pose_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingAdmittanceController] locked_pose_topic: "
                  << perching_locked_pose_topic_);
}

void GimbalrotorPerchingAdmittanceController::reset()
{
  GimbalrotorAdmittanceController::reset();

  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  normal_admittance_enabled_ = false;
  perching_admittance_enabled_ = false;
  effective_admittance_enabled_ = false;

  perching_enabled_for_constraint_ = false;

  has_perching_point_ = false;
  has_branch_pose_ = false;
  has_locked_pose_msg_ = false;
  has_locked_pivot_ = false;
  has_locked_pose_ = false;

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

  constraint_axis_world_ = Eigen::Vector3d::UnitY();

  resetEquilibriumWrenchUnsafe();

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

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "equilibrium_wrench_required_samples: %d",
      equilibrium_wrench_required_samples_);


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

void GimbalrotorPerchingAdmittanceController::controlCore()
{
  bool effective_enabled = false;
  bool reset_requested = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    if(perching_enabled_for_constraint_)
    {
      effective_enabled = perching_admittance_enabled_;
    }
    else
    {
      effective_enabled = normal_admittance_enabled_;
    }

    /*
     * Reset when effective enable changes.
     */
    if(effective_enabled != effective_admittance_enabled_)
    {
      reset_requested = true;
    }

    effective_admittance_enabled_ = effective_enabled;

    /*
     * Reset when mode, lock, pivot or constraint frame has changed.
     */
    if(admittance_reset_requested_)
    {
      reset_requested = true;
      admittance_reset_requested_ = false;
    }
  }

  /*
   * This assignment now happens only in the main control thread.
   */
  admittance_enabled_ = effective_enabled;

  if(reset_requested)
  {
    admittance_core_.reset();
    admittance_output_ = AdmittanceCoreOutput();
    prev_admittance_time_ = ros::Time::now();

    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Admittance state reset.");
  }

  GimbalrotorAdmittanceController::controlCore();
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
      /*
       * A mode change invalidates the previous tare because
       * the wrench reference point and contact condition may
       * have changed.
       */
      resetEquilibriumWrenchUnsafe();

      admittance_reset_requested_ = true;
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

  if(locked_radius_ < min_valid_radius_)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
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

  if(radial_norm < 1.0e-6)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
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

  if(tangent_norm < 1.0e-6)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
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

  if(std::abs(determinant - 1.0) > 1.0e-3)
  {
    has_locked_pose_ = false;

    R_world_constraint_.setIdentity();

    if(previously_valid)
    {
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
  * A new lock means the pivot, radius, nominal pose or constraint frame may have changed.
  *
  * The previous equilibrium wrench must not be reused.
  */
  resetEquilibriumWrenchUnsafe();

  /*
  * A new pivot/frame must not inherit displacement, velocity or filtering state from the old frame.
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

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    lock_valid = perching_enabled_for_constraint_ && has_locked_pose_;
    tare_ready = equilibrium_wrench_ready_;
    collected_samples = equilibrium_wrench_sample_count_;

    if(!msg->data)
    {
      /*
       * Publishing false is also the command to start a
       * fresh equilibrium-wrench collection.
       */
      perching_admittance_enabled_ = false;

      resetEquilibriumWrenchUnsafe();

      admittance_reset_requested_ = true;
      enable_accepted = true;
    }
    else if(!lock_valid)
    {
      /*
       * Never enable pivot admittance without a valid
       * perching lock.
       */
      perching_admittance_enabled_ = false;
      admittance_reset_requested_ = true;
      enable_accepted = false;
    }
    else if(!tare_ready)
    {
      /*
       * Do not enable admittance before the no-contact
       * equilibrium wrench has been collected.
       */
      perching_admittance_enabled_ = false;
      admittance_reset_requested_ = true;
      enable_accepted = false;
    }
    else
    {
      if(!perching_admittance_enabled_)
      {
        perching_admittance_enabled_ = true;
        admittance_reset_requested_ = true;
      }

      enable_accepted = true;
    }
  }

  if(!msg->data)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Perching admittance disabled. "
        "Equilibrium-wrench collection restarted.");
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
  else if(enable_accepted)
  {
    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Perching admittance enabled using the collected "
        "equilibrium pivot wrench.");
  }
}

void GimbalrotorPerchingAdmittanceController::normalAdmittanceEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    if(normal_admittance_enabled_ != msg->data)
    {
      normal_admittance_enabled_ = msg->data;

      admittance_reset_requested_ = true;
    }
  }

  ROS_WARN(
      "[GimbalrotorPerchingAdmittanceController] "
      "normal_admittance_enabled: %d",
      static_cast<int>(msg->data));
}

Eigen::Matrix<double, 6, 1> GimbalrotorPerchingAdmittanceController::getExternalWrenchWorld() const
{
  /*
   * Base-controller result:
   *
   * force:
   *   world coordinates
   *
   * torque:
   *   world coordinates, moment about the COG
   */
  const Eigen::Matrix<double, 6, 1> wrench_cog_world = GimbalrotorAdmittanceController::getExternalWrenchWorld();

  /*
   * Do not build a tare from missing or stale wrench data.
   *
   * The base function returns zero for missing/stale data,
   * but zero could also be a physically valid measurement.
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
   * Normal-flight path:
   * preserve the normal COG-referenced wrench.
   */
  if(!perching_active)
  {
    return wrench_cog_world;
  }

  /*
   * Never pass a COG-referenced wrench into the perching
   * admittance calculation when the pivot lock is invalid.
   */
  if(!lock_valid)
  {
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  const tf::Vector3 cog_pos_world_tf =
      estimator_->getPos(
          Frame::COG,
          estimate_mode_);

  const Eigen::Vector3d cog_pos_world(
      cog_pos_world_tf.x(),
      cog_pos_world_tf.y(),
      cog_pos_world_tf.z());

  const Eigen::Vector3d pivot_pos_world(
      pivot_world_tf.x(),
      pivot_world_tf.y(),
      pivot_world_tf.z());

  const Eigen::Vector3d force_world =
      wrench_cog_world.head<3>();

  const Eigen::Vector3d torque_cog_world =
      wrench_cog_world.tail<3>();

  const Eigen::Vector3d pivot_to_cog_world =
      cog_pos_world -
      pivot_pos_world;

  /*
   * Shift the torque reference point:
   *
   * tau_pivot =
   *     tau_cog +
   *     (p_cog - p_pivot) x force
   */
  const Eigen::Vector3d torque_pivot_world =
      torque_cog_world +
      pivot_to_cog_world.cross(force_world);

  Eigen::Matrix<double, 6, 1>
      wrench_pivot_world =
          wrench_cog_world;

  wrench_pivot_world.tail<3>() =
      torque_pivot_world;

  Eigen::Matrix<double, 6, 1>
      residual_wrench_pivot_world =
          Eigen::Matrix<double, 6, 1>::Zero();

  Eigen::Matrix<double, 6, 1>
      accepted_equilibrium_wrench =
          Eigen::Matrix<double, 6, 1>::Zero();

  bool admittance_enabled_now = false;
  bool tare_ready_now = false;
  bool tare_completed_now = false;

  int collected_samples = 0;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    /*
     * Recheck the state because it may have changed while
     * the wrench and current COG position were calculated.
     */
    if(!perching_enabled_for_constraint_ || !has_locked_pose_)
    {
      return Eigen::Matrix<double, 6, 1>::Zero();
    }

    admittance_enabled_now = perching_admittance_enabled_;

    /*
     * Collect the no-contact equilibrium only while
     * perching admittance is disabled.
     *
     * During these samples:
     *   - do not touch the robot,
     *   - keep the nominal pitch fixed,
     *   - keep the saw off,
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
         * This is the wrench that enters the admittance
         * controller:
         *
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
    ROS_INFO_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Collecting equilibrium pivot wrench: %d / %d.",
        collected_samples,
        equilibrium_wrench_required_samples_);

    /*
     * Admittance is disabled, so do not pass the raw biased
     * pivot wrench to the core.
     */
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  if(!tare_ready_now)
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "[GimbalrotorPerchingAdmittanceController] "
        "Perching admittance has no valid equilibrium "
        "wrench. Returning zero wrench.");

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
       locked_radius_ < min_valid_radius_)
    {
      return;
    }

    nominal_pitch =
        original_target_rpy.y();

    /*
     * Constraint coordinate Y is the branch axis.
     * Rotational admittance Y therefore produces
     * the perching pitch correction.
     */
    admittance_pitch_offset =
        output.angle_offset_compliance(1);

    target_pitch =
        normalizeAngle(
            nominal_pitch +
            admittance_pitch_offset);

    /*
     * This function reads locked geometry.
     * The perching-state mutex is held here.
     */
    modified_target_pos =
        computePerchingArcPositionFromPitch(
            target_pitch,
            original_target_pos);

    /*
     * Translational constraint Y is motion along
     * the branch axis.
     */
    const Eigen::Vector3d branch_offset_world =
        constraint_axis_world_ *
        output.pos_offset_compliance(1);

    modified_target_pos +=
        tf::Vector3(
            branch_offset_world.x(),
            branch_offset_world.y(),
            branch_offset_world.z());

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

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  rpy.setValue(roll, pitch, yaw);
}

} // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(
    aerial_robot_control::GimbalrotorPerchingAdmittanceController,
    aerial_robot_control::ControlBase)