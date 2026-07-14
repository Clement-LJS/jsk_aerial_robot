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
    locked_x_side_(1.0)
{
  perching_point_world_.setValue(0.0, 0.0, 0.0);
  branch_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_rpy_.setValue(0.0, 0.0, 0.0);
  locked_pivot_world_.setValue(0.0, 0.0, 0.0);
  locked_radius_vec_world_.setValue(0.0, 0.0, 0.0);
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

  R_world_constraint_.setIdentity();

  constraint_axis_world_ = Eigen::Vector3d::UnitY();

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

void GimbalrotorPerchingImpedanceController::reset()
{
  GimbalrotorImpedanceController::reset();

  normal_impedance_enabled_ = false;
  perching_impedance_enabled_ = false;
  effective_impedance_enabled_ = false;

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
  
  R_world_constraint_.setIdentity();

  constraint_axis_world_ = Eigen::Vector3d::UnitY();

  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching impedance state reset.");
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

  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching_min_valid_radius: %.4f",
           min_valid_radius_);
  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching_max_pitch_delta deg: %.2f",
           max_pitch_delta_ * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching_arc_pitch_sign: %.2f",
           arc_pitch_sign_);
}

void GimbalrotorPerchingImpedanceController::controlCore()
{
  updateEffectiveImpedanceEnable();

  GimbalrotorImpedanceController::controlCore();
}


Eigen::Matrix<double, 6, 1> GimbalrotorPerchingImpedanceController::getExternalWrenchWorld() const
{
  /*
   * Base function returns:
   * force: world coordinates
   * torque: world coordinates
   * torque reference point: COG
   */
  const Eigen::Matrix<double, 6, 1> wrench_cog_world = GimbalrotorImpedanceController::getExternalWrenchWorld();

  /*
   * During normal flight, preserve the normal COG-referenced wrench.
   */
  if(!perching_enabled_for_constraint_)
  {
    return wrench_cog_world;
  }

  /*
   * A pivot transformation is not valid before
   * the perching lock has been received.
   *
   * Return zero rather than integrating a wrench
   * with the wrong reference point.
   */
  if(!has_locked_pose_)
  {
    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  const tf::Vector3 cog_pos_world_tf = estimator_->getPos(Frame::COG, estimate_mode_);

  const Eigen::Vector3d cog_pos_world(
      cog_pos_world_tf.x(),
      cog_pos_world_tf.y(),
      cog_pos_world_tf.z());

  const Eigen::Vector3d pivot_pos_world(
      locked_pivot_world_.x(),
      locked_pivot_world_.y(),
      locked_pivot_world_.z());

  const Eigen::Vector3d force_world = wrench_cog_world.head<3>();

  const Eigen::Vector3d torque_cog_world = wrench_cog_world.tail<3>();

  /*
   * Vector from the perching pivot to the COG.
   */
  const Eigen::Vector3d pivot_to_cog_world = cog_pos_world - pivot_pos_world;

  /*
   * Change the torque reference point: tau_pivot = tau_cog + (p_cog - p_pivot) x force
   */
  const Eigen::Vector3d torque_pivot_world = torque_cog_world + pivot_to_cog_world.cross(force_world);

  Eigen::Matrix<double, 6, 1> wrench_pivot_world = wrench_cog_world;

  wrench_pivot_world.tail<3>() = torque_pivot_world;

  return wrench_pivot_world;
}

void GimbalrotorPerchingImpedanceController::perchingEnableCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
  perching_enabled_for_constraint_ = msg->data;
  updateEffectiveImpedanceEnable();

  if(!perching_enabled_for_constraint_)
  {
    ROS_WARN("[GimbalrotorPerchingImpedanceController] perching navigation DISABLED; normal impedance trigger is active if enabled.");
    return;
  }

  ROS_WARN("[GimbalrotorPerchingImpedanceController] perching navigation ENABLED; waiting for perching/impedance_enable for perching impedance.");
}

void GimbalrotorPerchingImpedanceController::perchingPointCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg)
{
  perching_point_world_.setValue(
      msg->point.x,
      msg->point.y,
      msg->point.z);

  has_perching_point_ = true;
}

void GimbalrotorPerchingImpedanceController::branchPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
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

void GimbalrotorPerchingImpedanceController::lockedPivotCallback(
    const geometry_msgs::PointStamped::ConstPtr& msg)
{
  locked_pivot_world_.setValue(
      msg->point.x,
      msg->point.y,
      msg->point.z);

  has_locked_pivot_ = true;

  updateLockedConstraintFromLockedPoseAndPivot();
}

void GimbalrotorPerchingImpedanceController::lockedPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  poseMsgToTfPosRpy(*msg, locked_robot_pos_world_, locked_robot_rpy_);

  has_locked_pose_msg_ = true;

  updateLockedConstraintFromLockedPoseAndPivot();
}

void GimbalrotorPerchingImpedanceController::updateLockedConstraintFromLockedPoseAndPivot()
{
  if(!has_locked_pose_msg_)
  {
    return;
  }

  if(!has_locked_pivot_)
  {
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
      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorPerchingImpedanceController] waiting for locked pivot from navigator.");
      return;
    }
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

    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingImpedanceController] locked constraint invalid: radius %.4f is too small.",
        locked_radius_);

    return;
  }

  has_locked_pose_ = true;

    /*
  * Minimum implementation:
  * the physical branch axis is assumed to be world Y.
  */
  constraint_axis_world_ = Eigen::Vector3d::UnitY();

  Eigen::Vector3d radial_world(
      locked_radius_vec_world_.x(),
      locked_radius_vec_world_.y(),
      locked_radius_vec_world_.z());

  /*
  * Remove the component along the branch.
  */
  radial_world -= constraint_axis_world_ * constraint_axis_world_.dot(radial_world);

  const double radial_norm = radial_world.norm();

  if(radial_norm < 1.0e-6)
  {
    has_locked_pose_ = false;

    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Cannot build constraint frame: invalid radial vector.");

    return;
  }

  radial_world.normalize();

  Eigen::Vector3d tangent_world = constraint_axis_world_.cross(radial_world);

  const double tangent_norm = tangent_world.norm();

  if(tangent_norm < 1.0e-6)
  {
    has_locked_pose_ = false;

    ROS_ERROR(
        "[GimbalrotorPerchingAdmittanceController] "
        "Cannot build constraint frame: invalid tangent.");

    return;
  }

  tangent_world.normalize();

  /*
  * Columns map constraint-frame vectors into world.
  *
  * constraint X = radial
  * constraint Y = branch axis
  * constraint Z = tangent
  */
  R_world_constraint_.col(0) = radial_world;
  R_world_constraint_.col(1) = constraint_axis_world_;
  R_world_constraint_.col(2) = tangent_world;
      
  ROS_WARN("[GimbalrotorPerchingImpedanceController] locked perching constraint received.");
  ROS_WARN("[GimbalrotorPerchingImpedanceController] locked pivot: %.3f %.3f %.3f", locked_pivot_world_.x(), locked_pivot_world_.y(), locked_pivot_world_.z());
  ROS_WARN("[GimbalrotorPerchingImpedanceController] locked pos: %.3f %.3f %.3f", locked_robot_pos_world_.x(), locked_robot_pos_world_.y(), locked_robot_pos_world_.z());
  ROS_WARN("[GimbalrotorPerchingImpedanceController] locked pitch deg: %.2f", locked_robot_rpy_.y() * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingImpedanceController] hand-center radius: %.3f", locked_radius_);
}

Eigen::Matrix3d GimbalrotorPerchingImpedanceController::getComplianceToWorldRotation() const
{
  if(perching_enabled_for_constraint_ && has_locked_pose_)
  {
    return R_world_constraint_;
  }

  return GimbalrotorImpedanceController::getComplianceToWorldRotation();
}

void GimbalrotorPerchingImpedanceController::normalImpedanceEnableCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
  normal_impedance_enabled_ = msg->data;
  updateEffectiveImpedanceEnable();

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] normal_impedance_enabled: %d",
      static_cast<int>(normal_impedance_enabled_));
}

void GimbalrotorPerchingImpedanceController::perchingImpedanceEnableCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
  perching_impedance_enabled_ = msg->data;
  updateEffectiveImpedanceEnable();

  ROS_WARN(
      "[GimbalrotorPerchingImpedanceController] perching_impedance_enabled: %d",
      static_cast<int>(perching_impedance_enabled_));
}

void GimbalrotorPerchingImpedanceController::updateEffectiveImpedanceEnable()
{
  const bool previous_effective = effective_impedance_enabled_;

  if(perching_enabled_for_constraint_)
  {
    effective_impedance_enabled_ = perching_impedance_enabled_;
  }
  else
  {
    effective_impedance_enabled_ = normal_impedance_enabled_;
  }

  impedance_enabled_ = effective_impedance_enabled_;

  if(previous_effective && !effective_impedance_enabled_)
  {
    impedance_core_.reset();
    impedance_output_ = ImpedanceCoreOutput();
  }
}

bool GimbalrotorPerchingImpedanceController::hasValidPerchingConstraint() const
{
  if(!perching_enabled_for_constraint_)
  {
    return false;
  }

  if(!perching_impedance_enabled_)
  {
    return false;
  }

  if(require_perching_lock_ && !has_locked_pose_)
  {
    return false;
  }

  if(locked_radius_ < min_valid_radius_)
  {
    return false;
  }

  return true;
}

void GimbalrotorPerchingImpedanceController::applyImpedanceOutputToNavigator(
    const tf::Vector3& original_target_pos,
    const tf::Vector3& original_target_rpy,
    const ImpedanceCoreOutput& output)
{
  /*
   * Case 1: Normal flight.
   * Use normal impedance only when /gimbalrotor/impedance_enable is true.
   */
  if(!perching_enabled_for_constraint_)
  {
    GimbalrotorImpedanceController::applyImpedanceOutputToNavigator(
        original_target_pos,
        original_target_rpy,
        output);

    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingImpedanceController] using NORMAL impedance.");

    return;
  }

  /*
   * Case 2: Perching navigation is enabled, but perching impedance is not enabled.
   * Do not fall back to normal impedance here. This is exactly your requested behavior: perching navigation + normal PID
   */
  if(!perching_impedance_enabled_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingImpedanceController] perching navigation active, "
        "perching impedance disabled -> PID only.");

    return;
  }

  /*
   * Case 3: Perching impedance was requested, but the perching lock/pivot is not valid.
   * Do not use normal impedance during perching.
   */
  if(!hasValidPerchingConstraint())
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingImpedanceController] perching impedance requested, "
        "but perching constraint is invalid -> PID only.");

    return;
  }

  const double nominal_pitch = original_target_rpy.y();
  const double admittance_pitch_offset = output.angle_offset_compliance(1);
  const double target_pitch = normalizeAngle(nominal_pitch + admittance_pitch_offset);

  tf::Vector3 modified_target_pos = computePerchingArcPositionFromPitch(target_pitch, original_target_pos);

 /*
  * Translational compliance along the branch axis.
  *
  * output.pos_offset_compliance(1): scalar displacement along constraint Y
  *
  * constraint_axis_world_: branch-axis unit vector in world coordinates
  */
  const Eigen::Vector3d branch_offset_world = constraint_axis_world_ * output.pos_offset_compliance(1);

 /*
  * The arc function already provides the constrained radial/tangential position.
  *
  * Add only the permitted branch-axis displacement.
  */
  modified_target_pos += tf::Vector3(
      branch_offset_world.x(),
      branch_offset_world.y(),
      branch_offset_world.z());

  tf::Vector3 modified_target_rpy = original_target_rpy;

  /*
   * Roll and yaw can still use normal small impedance offsets if enabled.
   * For your current YAML, roll/yaw are disabled, so these are normally zero.
   */
  modified_target_rpy.setX(
      modified_target_rpy.x() + output.rpy_offset_world(0));

  modified_target_rpy.setY(target_pitch);

  modified_target_rpy.setZ(
      modified_target_rpy.z() + output.rpy_offset_world(2));

  navigator_->setTargetPos(modified_target_pos);
  navigator_->setTargetRPY(modified_target_rpy);

  ROS_WARN_THROTTLE(
      0.5,
      "[GimbalrotorPerchingImpedanceController] PERCHING impedance | "
      "nominal_pitch %.2f deg | d_pitch %.2f deg | target_pitch %.2f deg | "
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