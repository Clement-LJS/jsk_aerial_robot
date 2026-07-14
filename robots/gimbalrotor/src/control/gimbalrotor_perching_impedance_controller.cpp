#include <gimbalrotor/control/gimbalrotor_perching_impedance_controller.h>

#include <pluginlib/class_list_macros.h>

#include <cmath>

namespace
{
const double PI = 3.14159265358979323846;
}

namespace aerial_robot_control
{

GimbalrotorPerchingImpedanceController::GimbalrotorPerchingImpedanceController()
  : GimbalrotorImpedanceController(),
    perching_enable_topic_for_constraint_("perching/enable"),
    perching_impedance_enable_topic_("perching/impedance_enable"),
    perching_point_topic_("perching/point"),
    perching_branch_pose_topic_("perching/branch_pose"),
    perching_locked_pose_topic_("perching/locked_pose"),
    perching_locked_pivot_topic_("perching/locked_pivot"),
    normal_impedance_enabled_(false),
    perching_impedance_enabled_(false),
    effective_impedance_enabled_(false),
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
}

void GimbalrotorPerchingImpedanceController::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate)
{
  /*
   * First initialize the normal gimbalrotor impedance controller.
   *
   * This preserves normal impedance behavior.
   * This subclass only changes target injection while perching is active.
   */
  GimbalrotorImpedanceController::initialize(
      nh,
      nhp,
      robot_model,
      estimator,
      navigator,
      ctrl_loop_rate);

  perchingRosParamInit();

  /*
   * The base impedance controller subscribed to impedance_enable_topic_.
   * For this mode-aware controller, replace that subscription with our own
   * normal/perching trigger separation.
   */
  impedance_enable_sub_.shutdown();

  normal_impedance_enable_sub_ =
      nh_.subscribe(
          impedance_enable_topic_,
          1,
          &GimbalrotorPerchingImpedanceController::normalImpedanceEnableCallback,
          this);

  perching_impedance_enable_sub_ =
      nh_.subscribe(
          perching_impedance_enable_topic_,
          1,
          &GimbalrotorPerchingImpedanceController::perchingImpedanceEnableCallback,
          this);

  perching_enable_sub_for_constraint_ =
      nh_.subscribe(
          perching_enable_topic_for_constraint_,
          1,
          &GimbalrotorPerchingImpedanceController::perchingEnableCallback,
          this);

  perching_point_sub_ =
      nh_.subscribe(
          perching_point_topic_,
          1,
          &GimbalrotorPerchingImpedanceController::perchingPointCallback,
          this);

  branch_pose_sub_ =
      nh_.subscribe(
          perching_branch_pose_topic_,
          1,
          &GimbalrotorPerchingImpedanceController::branchPoseCallback,
          this);

  /*
   * GimbalrotorPerchingNavigator publishes this as latched.
   * It contains:
   *   - locked robot position
   *   - locked robot RPY
   *
   * We use this to reconstruct the same pitch arc in the impedance controller.
   */
  locked_pose_sub_ =
      nh_.subscribe(
          perching_locked_pose_topic_,
          1,
          &GimbalrotorPerchingImpedanceController::lockedPoseCallback,
          this);

  locked_pivot_sub_ =
      nh_.subscribe(
          perching_locked_pivot_topic_,
          1,
          &GimbalrotorPerchingImpedanceController::lockedPivotCallback,
          this);

  ROS_WARN_STREAM("[GimbalrotorPerchingImpedanceController] initialized.");
  ROS_WARN_STREAM("[GimbalrotorPerchingImpedanceController] perching_enable_topic: "
                  << perching_enable_topic_for_constraint_);
  ROS_WARN_STREAM("[GimbalrotorPerchingImpedanceController] perching_point_topic: "
                  << perching_point_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingImpedanceController] branch_pose_topic: "
                  << perching_branch_pose_topic_);
  ROS_WARN_STREAM("[GimbalrotorPerchingImpedanceController] locked_pose_topic: "
                  << perching_locked_pose_topic_);
}

void GimbalrotorPerchingImpedanceController::
reset()
{
  GimbalrotorImpedanceController::
      reset();

  std::lock_guard<std::mutex> lock(
      perching_state_mutex_);

  normal_impedance_enabled_ = false;
  perching_impedance_enabled_ = false;
  effective_impedance_enabled_ = false;

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

  locked_pose_stamp_ =
      ros::Time(0);

  locked_pivot_stamp_ =
      ros::Time(0);

  accepted_locked_pose_stamp_ =
      ros::Time(0);

  accepted_locked_pivot_stamp_ =
      ros::Time(0);

  R_world_constraint_.setIdentity();

  constraint_axis_world_ =
      Eigen::Vector3d::UnitY();

  admittance_reset_requested_ = false;

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] "
      "Perching admittance state reset.");
}

void GimbalrotorPerchingImpedanceController::perchingRosParamInit()
{
  ros::NodeHandle imp_perch_nh(nh_, "controller/impedance/perching");

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
   * If they are not explicitly set under controller/impedance/perching,
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
      "perching_impedance_enable_topic",
      perching_impedance_enable_topic_,
      std::string("perching/impedance_enable"));

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
        "[GimbalrotorPerchingImpedanceController] "
        "Invalid maximum_lock_stamp_difference %.6f. "
        "Using 0.05 seconds.",
        maximum_lock_stamp_difference_);

    maximum_lock_stamp_difference_ = 0.05;
  }

  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching_min_valid_radius: %.4f",
           min_valid_radius_);
  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching_max_pitch_delta deg: %.2f",
           max_pitch_delta_ * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching_arc_pitch_sign: %.2f",
           arc_pitch_sign_);
}

void GimbalrotorPerchingImpedanceController::controlCore()
{
  bool effective_enabled = false;
  bool reset_requested = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    if(perching_enabled_for_constraint_)
    {
      effective_enabled = perching_impedance_enabled_;
    }
    else
    {
      effective_enabled = normal_impedance_enabled_;
    }

    /*
     * Reset when effective enable changes.
     */
    if(effective_enabled != effective_impedance_enabled_)
    {
      reset_requested = true;
    }

    effective_impedance_enabled_ = effective_enabled;

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
  impedance_enabled_ = effective_enabled;

  if(reset_requested)
  {
    impedance_core_.reset();
    impedance_output_ = ImpedanceCoreOutput();
    prev_impedance_time_ = ros::Time::now();

    ROS_WARN(
        "[GimbalrotorPerchingImpedanceController] "
        "Admittance state reset.");
  }

  GimbalrotorImpedanceController::controlCore();
}

void GimbalrotorPerchingImpedanceController::perchingEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  bool mode_changed = false;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    mode_changed = perching_enabled_for_constraint_ != msg->data;

    perching_enabled_for_constraint_ = msg->data;

    if(mode_changed)
    {
      admittance_reset_requested_ = true;
    }
  }

  if(msg->data)
  {
    ROS_WARN_STREAM(
        "[GimbalrotorPerchingImpedanceController] "
        "Perching navigation enabled. "
        "Waiting for a valid lock and enable topic: "
        << perching_impedance_enable_topic_);
  }
  else
  {
    ROS_WARN(
        "[GimbalrotorPerchingImpedanceController] "
        "Perching navigation disabled. "
        "Normal admittance trigger is active.");
  }
}

void GimbalrotorPerchingImpedanceController::perchingPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(perching_state_mutex_);

  perching_point_world_.setValue(
      msg->point.x,
      msg->point.y,
      msg->point.z);

  has_perching_point_ = true;
}

void GimbalrotorPerchingImpedanceController::branchPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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

void GimbalrotorPerchingImpedanceController::lockedPivotCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
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

void GimbalrotorPerchingImpedanceController::lockedPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
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

void GimbalrotorPerchingImpedanceController::updateLockedConstraintFromLockedPoseAndPivot()
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
          "[GimbalrotorPerchingImpedanceController] "
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
          "[GimbalrotorPerchingImpedanceController] "
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
          "[GimbalrotorPerchingImpedanceController] "
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
        "[GimbalrotorPerchingImpedanceController] "
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
        "[GimbalrotorPerchingImpedanceController] "
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
        "[GimbalrotorPerchingImpedanceController] "
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
        "[GimbalrotorPerchingImpedanceController] "
        "Invalid constraint rotation determinant: "
        "%.6f",
        determinant);

    return;
  }

  has_locked_pose_ = true;
  accepted_locked_pose_stamp_ = locked_pose_stamp_;
  accepted_locked_pivot_stamp_ = locked_pivot_stamp_;

  /*
   * A new pivot/frame must not inherit displacement,
   * velocity or filtering state from the old frame.
   */
  admittance_reset_requested_ = true;

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] "
      "Locked perching constraint accepted.");

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] "
      "Locked pivot: %.3f %.3f %.3f",
      locked_pivot_world_.x(),
      locked_pivot_world_.y(),
      locked_pivot_world_.z());

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] "
      "Locked position: %.3f %.3f %.3f",
      locked_robot_pos_world_.x(),
      locked_robot_pos_world_.y(),
      locked_robot_pos_world_.z());

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] "
      "Locked pitch: %.2f deg",
      locked_robot_rpy_.y() *
      180.0 / PI);

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] "
      "Pitch-plane radius: %.3f m",
      locked_radius_);

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] "
      "Constraint determinant: %.6f",
      determinant);
}

Eigen::Matrix3d GimbalrotorPerchingImpedanceController::getComplianceToWorldRotation() const
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

  return GimbalrotorImpedanceController::getComplianceToWorldRotation();
}

void GimbalrotorPerchingImpedanceController::perchingImpedanceEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    if(perching_impedance_enabled_ != msg->data)
    {
      perching_impedance_enabled_ = msg->data;

      admittance_reset_requested_ = true;
    }
  }

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] "
      "perching_admittance_enabled: %d",
      static_cast<int>(msg->data));
}

void GimbalrotorPerchingImpedanceController::normalImpedanceEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    if(normal_impedance_enabled_ != msg->data)
    {
      normal_impedance_enabled_ = msg->data;

      admittance_reset_requested_ = true;
    }
  }

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] "
      "normal_admittance_enabled: %d",
      static_cast<int>(msg->data));
}

Eigen::Matrix<double, 6, 1> GimbalrotorPerchingImpedanceController::getExternalWrenchWorld() const
{
  /*
   * Base result:
   * force: world coordinates
   * torque: world coordinates
   *
   * torque reference point: COG
   */
  const Eigen::Matrix<double, 6, 1> wrench_cog_world = GimbalrotorImpedanceController::getExternalWrenchWorld();

  bool perching_active = false;
  bool lock_valid = false;

  tf::Vector3 pivot_world_tf;

  {
    std::lock_guard<std::mutex> lock(perching_state_mutex_);

    perching_active = perching_enabled_for_constraint_;
    lock_valid = has_locked_pose_;
    pivot_world_tf = locked_pivot_world_;
  }

  if(!perching_active)
  {
    return wrench_cog_world;
  }

  /*
   * Do not integrate a COG-referenced wrench while
   * the controller believes it is in pivot mode.
   */
  if(!lock_valid)
  {
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  const tf::Vector3 cog_pos_world_tf = estimator_->getPos(Frame::COG, estimate_mode_);

  const Eigen::Vector3d cog_pos_world(
      cog_pos_world_tf.x(),
      cog_pos_world_tf.y(),
      cog_pos_world_tf.z());

  const Eigen::Vector3d pivot_pos_world(
      pivot_world_tf.x(),
      pivot_world_tf.y(),
      pivot_world_tf.z());

  const Eigen::Vector3d force_world = wrench_cog_world.head<3>();
  const Eigen::Vector3d torque_cog_world = wrench_cog_world.tail<3>();
  const Eigen::Vector3d pivot_to_cog_world = cog_pos_world - pivot_pos_world;

  /*
   * Shift torque from COG to perching pivot:
   * tau_pivot = tau_cog + (p_cog - p_pivot) x force.
   */
  const Eigen::Vector3d torque_pivot_world = torque_cog_world + pivot_to_cog_world.cross(force_world);

  Eigen::Matrix<double, 6, 1> wrench_pivot_world = wrench_cog_world;

  wrench_pivot_world.tail<3>() = torque_pivot_world;

  return wrench_pivot_world;
}

void GimbalrotorPerchingImpedanceController::
applyImpedanceOutputToNavigator(
    const tf::Vector3& original_target_pos,
    const tf::Vector3& original_target_rpy,
    const ImpedanceCoreOutput& output)
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
    GimbalrotorImpedanceController::
        applyImpedanceOutputToNavigator(
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
       !perching_impedance_enabled_ ||
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
      "[GimbalrotorPerchingImpedanceController] "
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
GimbalrotorPerchingImpedanceController::computePerchingArcPositionFromPitch(
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

double GimbalrotorPerchingImpedanceController::clamp(
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

double GimbalrotorPerchingImpedanceController::normalizeAngle(
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

double GimbalrotorPerchingImpedanceController::norm2D(
    double x,
    double z) const
{
  return std::sqrt(x * x + z * z);
}

void GimbalrotorPerchingImpedanceController::poseMsgToTfPosRpy(
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
    aerial_robot_control::GimbalrotorPerchingImpedanceController,
    aerial_robot_control::ControlBase)