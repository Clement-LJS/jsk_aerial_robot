// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_perching_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

namespace
{
const int NAV_MODE_NONE = 0;
const int NAV_MODE_VEL = 1;
const int NAV_MODE_POS = 2;
const int NAV_MODE_POS_VEL = 3;

const double PI = 3.14159265358979323846;
}

GimbalrotorPerchingNavigator::GimbalrotorPerchingNavigator():
  GimbalrotorNavigator(),

  perching_enable_(false),
  perching_locked_(false),
  perching_lock_once_(true),

  require_branch_point_(true),
  command_pitch_as_delta_(false),
  constrain_position_command_(true),
  constrain_velocity_command_(true),
  use_pitch_command_for_arc_(true),
  hold_locked_pose_without_pitch_command_(true),

  accept_uav_nav_pitch_command_(false),

  active_perching_hold_enable_(true),

  min_valid_radius_(0.05),
  max_pitch_delta_(0.78539816339),  // 45 deg
  arc_pitch_sign_(1.0),
  command_pitch_sign_(1.0),
  y_compliance_deadband_(0.03),

  pivot_source_("manual"),

  perching_enable_topic_("perching/enable"),
  branch_pose_topic_("perching/branch_pose"),
  perching_point_topic_("perching/point"),
  locked_pivot_topic_("perching/locked_pivot"),
  relock_topic_("perching/relock"),
  reset_topic_("perching/reset"),
  manual_pitch_delta_topic_("perching/manual_pitch_delta"),

  has_branch_pose_(false),
  has_perching_point_(false),

  has_active_pitch_target_(false),
  active_target_pitch_(0.0),

  locked_radius_(0.0),
  locked_radius_pitch_arc_angle_(0.0),
  locked_pitch_to_radius_angle_offset_(0.0),
  locked_y_offset_(0.0),
  locked_x_side_(1.0)
{
  branch_pos_world_.setValue(0.0, 0.0, 0.0);
  perching_point_world_.setValue(0.0, 0.0, 0.0);

  hand_perching_center_offset_baselink_.setValue(0.0, 0.0, 0.0);
  
  locked_robot_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_rpy_.setValue(0.0, 0.0, 0.0);
  locked_pivot_world_.setValue(0.0, 0.0, 0.0);
  locked_radius_vec_world_.setValue(0.0, 0.0, 0.0);
}

void GimbalrotorPerchingNavigator::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    double loop_du)
{
  GimbalrotorNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  perching_enable_sub_ = nh_.subscribe(perching_enable_topic_, 1, &GimbalrotorPerchingNavigator::perchingEnableCallback, this);
  branch_pose_sub_ = nh_.subscribe(branch_pose_topic_, 1, &GimbalrotorPerchingNavigator::branchPoseCallback, this);
  perching_point_sub_ = nh_.subscribe(perching_point_topic_, 1, &GimbalrotorPerchingNavigator::perchingPointCallback, this);
  relock_sub_ = nh_.subscribe(relock_topic_, 1, &GimbalrotorPerchingNavigator::relockCallback, this);
  reset_sub_ = nh_.subscribe(reset_topic_, 1, &GimbalrotorPerchingNavigator::resetCallback, this);

  manual_pitch_delta_sub_ = nh_.subscribe(
      manual_pitch_delta_topic_,
      1,
      &GimbalrotorPerchingNavigator::manualPitchDeltaCallback,
      this);

  locked_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("perching/locked_pose", 1, true);
  locked_pivot_pub_ = nh_.advertise<geometry_msgs::PointStamped>(locked_pivot_topic_, 1, true);
  commanded_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("perching/commanded_pose", 1);

  ROS_WARN("[GimbalrotorPerchingNavigator] initialized");
  ROS_WARN("[GimbalrotorPerchingNavigator] enable topic: %s", perching_enable_topic_.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] branch pose topic: %s", branch_pose_topic_.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] perching point topic: %s", perching_point_topic_.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] manual pitch delta topic: %s", manual_pitch_delta_topic_.c_str());
}

void GimbalrotorPerchingNavigator::rosParamInit()
{
  GimbalrotorNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<bool>(navi_nh, "perching_enable", perching_enable_, false);
  getParam<bool>(navi_nh, "perching_lock_once", perching_lock_once_, true);
  getParam<bool>(navi_nh, "perching_require_branch_point", require_branch_point_, true);
  getParam<bool>(navi_nh, "perching_command_pitch_as_delta", command_pitch_as_delta_, false);
  getParam<bool>(navi_nh, "perching_constrain_position_command", constrain_position_command_, true);
  getParam<bool>(navi_nh, "perching_constrain_velocity_command", constrain_velocity_command_, true);
  getParam<bool>(navi_nh, "perching_use_pitch_command_for_arc", use_pitch_command_for_arc_, true);
  getParam<bool>(navi_nh, "perching_hold_locked_pose_without_pitch_command", hold_locked_pose_without_pitch_command_, true);

  /*
   * Important:
   *
   * Default false.
   *
   * If this is false, /gimbalrotor/uav/nav pitch_nav_mode == POS will not be
   * treated as a manual perching pitch command.
   *
   * This prevents /perching_cutting_mission from overwriting manual keyboard
   * pitch by repeatedly sending target_pitch = 0.0.
   */
  getParam<bool>(navi_nh, "perching_accept_uav_nav_pitch_command", accept_uav_nav_pitch_command_, false);

  getParam<bool>(navi_nh, "perching_active_hold_enable", active_perching_hold_enable_, true);

  getParam<double>(navi_nh, "perching_min_valid_radius", min_valid_radius_, 0.05);
  getParam<double>(navi_nh, "perching_max_pitch_delta", max_pitch_delta_, 0.78539816339);
  getParam<double>(navi_nh, "perching_arc_pitch_sign", arc_pitch_sign_, 1.0);
  getParam<double>(navi_nh, "perching_command_pitch_sign", command_pitch_sign_, 1.0);
  getParam<double>(navi_nh, "perching_y_compliance_deadband", y_compliance_deadband_, 0.03);

  getParam<std::string>(navi_nh, "perching_pivot_source", pivot_source_, std::string("hand_center"));

  double hand_center_x = hand_perching_center_offset_baselink_.x();
  double hand_center_y = hand_perching_center_offset_baselink_.y();
  double hand_center_z = hand_perching_center_offset_baselink_.z();

  getParam<double>(navi_nh, "hand_perching_center_offset_baselink_x", hand_center_x, hand_center_x);
  getParam<double>(navi_nh, "hand_perching_center_offset_baselink_y", hand_center_y, hand_center_y);
  getParam<double>(navi_nh, "hand_perching_center_offset_baselink_z", hand_center_z, hand_center_z);

  hand_perching_center_offset_baselink_.setValue(hand_center_x, hand_center_y, hand_center_z);

  getParam<std::string>(navi_nh, "perching_enable_topic", perching_enable_topic_, "perching/enable");
  getParam<std::string>(navi_nh, "perching_branch_pose_topic", branch_pose_topic_, "perching/branch_pose");
  getParam<std::string>(navi_nh, "perching_point_topic", perching_point_topic_, "perching/point");
  getParam<std::string>(navi_nh, "perching_relock_topic", relock_topic_, "perching/relock");
  getParam<std::string>(navi_nh, "perching_reset_topic", reset_topic_, "perching/reset");

  /*
   * Use this for keyboard pitch add-angle.
   *
   * Full resolved topic is usually: /gimbalrotor/perching/manual_pitch_delta
   */
  getParam<std::string>(navi_nh, "perching_manual_pitch_delta_topic", manual_pitch_delta_topic_, "perching/manual_pitch_delta");
  getParam<std::string>(navi_nh, "perching_locked_pivot_topic", locked_pivot_topic_, "perching/locked_pivot");
}

void GimbalrotorPerchingNavigator::update()
{
  if(getNaviState() != HOVER_STATE && perching_enable_)
  {
    perching_enable_ = false;
    resetPerchingLock();

    ROS_WARN(
        "[GimbalrotorPerchingNavigator] "
        "Perching navigation automatically deactivated because "
        "the flight state is no longer HOVER_STATE. "
        "Publish perching/enable=true again after returning to "
        "HOVER_STATE to create a fresh perching lock.");
  }

  GimbalrotorNavigator::update();

  if(getNaviState() != HOVER_STATE)
  {
    if(perching_enable_)
    {
      perching_enable_ = false;
      resetPerchingLock();

      ROS_WARN(
          "[GimbalrotorPerchingNavigator] "
          "Perching navigation automatically deactivated after "
          "leaving HOVER_STATE. "
          "A fresh perching enable command is required before "
          "perching control can be used again.");
    }

    return;
  }

  if(perching_enable_ && active_perching_hold_enable_)
  {
    applyActivePerchingTarget();
  }
}

void GimbalrotorPerchingNavigator::perchingEnableCallback(const std_msgs::BoolConstPtr& msg)
{
  if(!msg->data)
  {
    const bool was_enabled = perching_enable_;

    perching_enable_ = false;
    resetPerchingLock();

    if(was_enabled)
    {
      ROS_WARN("[GimbalrotorPerchingNavigator] perching DISABLED");
    }

    return;
  }

  if(getNaviState() != HOVER_STATE)
  {
    perching_enable_ = false;
    resetPerchingLock();

    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] "
        "perching enable rejected because navigation state is "
        "not HOVER_STATE. Enable perching again after entering "
        "HOVER_STATE.");

    return;
  }

  perching_enable_ = true;

  ROS_WARN("[GimbalrotorPerchingNavigator] perching ENABLED");

  if(!perching_locked_ || !perching_lock_once_)
  {
    if(!tryLockPerching("enable callback"))
    {
      perching_enable_ = false;
      resetPerchingLock();

      ROS_ERROR(
          "[GimbalrotorPerchingNavigator] "
          "perching enable failed because a valid lock could "
          "not be created.");

      return;
    }
  }

  active_target_pitch_ = locked_robot_rpy_.y();
  has_active_pitch_target_ = false;
}

void GimbalrotorPerchingNavigator::branchPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  branch_pos_world_.setValue(msg->pose.position.x,
                             msg->pose.position.y,
                             msg->pose.position.z);

  has_branch_pose_ = true;
}

void GimbalrotorPerchingNavigator::perchingPointCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
  perching_point_world_.setValue(msg->point.x,
                                 msg->point.y,
                                 msg->point.z);

  has_perching_point_ = true;
}

void GimbalrotorPerchingNavigator::relockCallback(const std_msgs::EmptyConstPtr& msg)
{
  (void)msg;

  if(!perching_enable_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] "
        "perching relock ignored because perching is disabled.");

    return;
  }

  if(getNaviState() != HOVER_STATE)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] "
        "perching relock ignored because navigation state is "
        "not HOVER_STATE.");

    return;
  }

  perching_locked_ = false;
  locked_radius_ = 0.0;
  locked_radius_vec_world_.setValue(
      0.0,
      0.0,
      0.0);

  if(!tryLockPerching("manual relock"))
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator] "
        "manual perching relock failed");
    return;
  }

  active_target_pitch_ = locked_robot_rpy_.y();
  has_active_pitch_target_ = false;
}

void GimbalrotorPerchingNavigator::resetCallback(const std_msgs::EmptyConstPtr& msg)
{
  (void)msg;
  resetPerchingLock();
}

void GimbalrotorPerchingNavigator::resetPerchingLock()
{
  perching_locked_ = false;
  locked_radius_ = 0.0;
  locked_radius_pitch_arc_angle_ = 0.0;
  locked_pitch_to_radius_angle_offset_ = 0.0;
  locked_y_offset_ = 0.0;
  locked_x_side_ = 1.0;

  has_active_pitch_target_ = false;
  active_target_pitch_ = 0.0;

  locked_robot_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_rpy_.setValue(0.0, 0.0, 0.0);
  locked_pivot_world_.setValue(0.0, 0.0, 0.0);
  locked_radius_vec_world_.setValue(0.0, 0.0, 0.0);

  ROS_WARN("[GimbalrotorPerchingNavigator] perching lock reset");
}

void GimbalrotorPerchingNavigator::manualPitchDeltaCallback(const std_msgs::Float64ConstPtr& msg)
{
  if(!perching_enable_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] manual pitch delta ignored because perching is disabled");
    return;
  }

  if(getNaviState() != HOVER_STATE)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] "
        "manual perching pitch command ignored because "
        "navigation state is not HOVER_STATE.");

    return;
  }

  if(!perching_locked_)
  {
    if(!tryLockPerching("manual pitch delta command"))
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorPerchingNavigator] manual pitch delta ignored because perching lock failed");
      return;
    }
  }

  double delta_pitch = command_pitch_sign_ * msg->data;
  delta_pitch = clamp(delta_pitch, -max_pitch_delta_, max_pitch_delta_);

  const double target_pitch = normalizeAngle(locked_robot_rpy_.y() + delta_pitch);
  const tf::Vector3 target_pos = computeArcPositionFromPitch(target_pitch);

  has_active_pitch_target_ = true;
  active_target_pitch_ = target_pitch;

  publishCommandedDebugPose(target_pos, target_pitch);

  // ROS_WARN_THROTTLE(
  //     0.5,
  //     "[GimbalrotorPerchingNavigator] manual pitch delta %.3f deg -> target pitch %.3f deg",
  //     delta_pitch * 180.0 / PI,
  //     target_pitch * 180.0 / PI);
}

bool GimbalrotorPerchingNavigator::tryLockPerching(const std::string& reason)
{
  if(getNaviState() != HOVER_STATE)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] "
        "cannot create perching lock outside HOVER_STATE.");

    return false;
  }

  if(perching_locked_ && perching_lock_once_)
  {
    return true;
  }

  /*
   * Pivot source logic:
   *
   * manual:
   *   Use base_link -> hand_center fixed robot geometry.
   *   Branch mocap is NOT required and is NOT used.
   *
   * branch:
   *   Use /perching/point if available.
   *   Otherwise use /perching/branch_pose.
   *   Branch/perching point info is required.
   *
   * Supported legacy aliases:
   *   hand_center     -> manual
   *   perching_point  -> branch
   */
  if(!isManualPivotMode() && !isBranchPivotMode())
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] cannot lock: invalid perching_pivot_source '%s'. "
        "Use 'manual' or 'branch'. Legacy aliases: 'hand_center', 'perching_point'.",
        pivot_source_.c_str());
    return false;
  }

  if(isBranchPivotMode() && !hasBranchPivotSource())
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] cannot lock: pivot_source='%s' requires "
        "/perching/point or /perching/branch_pose, but neither exists.",
        pivot_source_.c_str());
    return false;
  }

  locked_robot_pos_world_ = getCurrentRobotPos();
  locked_robot_rpy_ = getCurrentRobotRPY();

  locked_pivot_world_ = computeLockPivotWorld();
  perching_point_world_ = locked_pivot_world_;

  locked_radius_vec_world_ = locked_robot_pos_world_ - locked_pivot_world_;

  /*
   * Radius for the pitch arc.
   *
   * manual mode:
   *   locked_pivot_world_ is base_link + R_base_link * hand_center_offset.
   *
   * branch mode:
   *   locked_pivot_world_ is /perching/point or /perching/branch_pose.
   */
  locked_radius_ = norm2D(locked_radius_vec_world_.x(), locked_radius_vec_world_.z());
  locked_radius_pitch_arc_angle_ =
      computeRadiusPitchArcAngle(locked_radius_vec_world_);
  locked_pitch_to_radius_angle_offset_ =
      normalizeAngle(locked_robot_rpy_.y() - locked_radius_pitch_arc_angle_);

  if(locked_radius_ < min_valid_radius_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] cannot lock: invalid radius %.4f. "
        "pivot_source='%s'",
        locked_radius_,
        pivot_source_.c_str());
    return false;
  }

  locked_y_offset_ = locked_robot_pos_world_.y() - locked_pivot_world_.y();

  if(locked_robot_pos_world_.x() - locked_pivot_world_.x() >= 0.0)
  {
    locked_x_side_ = 1.0;
  }
  else
  {
    locked_x_side_ = -1.0;
  }

  active_target_pitch_ = locked_robot_rpy_.y();
  has_active_pitch_target_ = false;

  perching_locked_ = true;

  ROS_WARN("[GimbalrotorPerchingNavigator] perching locked by %s", reason.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] pivot source: %s", pivot_source_.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] locked pivot: x %.3f, y %.3f, z %.3f",
           locked_pivot_world_.x(),
           locked_pivot_world_.y(),
           locked_pivot_world_.z());
  ROS_WARN("[GimbalrotorPerchingNavigator] locked robot pos: x %.3f, y %.3f, z %.3f",
           locked_robot_pos_world_.x(),
           locked_robot_pos_world_.y(),
           locked_robot_pos_world_.z());
  ROS_WARN("[GimbalrotorPerchingNavigator] locked rpy deg: roll %.2f, pitch %.2f, yaw %.2f",
           locked_robot_rpy_.x() * 180.0 / PI,
           locked_robot_rpy_.y() * 180.0 / PI,
           locked_robot_rpy_.z() * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingNavigator] pitch-plane radius: %.3f m",
           locked_radius_);
  ROS_WARN("[GimbalrotorPerchingNavigator] locked radius pitch-arc angle deg: %.2f",
           locked_radius_pitch_arc_angle_ * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingNavigator] locked body-minus-arc pitch offset deg: %.2f",
           locked_pitch_to_radius_angle_offset_ * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingNavigator] locked pivot-relative Y offset: %.3f m",
           locked_y_offset_);
  ROS_WARN("[GimbalrotorPerchingNavigator] Y compliance deadband: %.3f m",
           y_compliance_deadband_);

  publishLockedDebugPose();
  publishLockedPivot();

  return true;
}

void GimbalrotorPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  aerial_robot_msgs::FlightNav nav_msg = *msg;

  if(perching_enable_ && getNaviState() == HOVER_STATE)
  {
    applyPerchingConstraint(nav_msg);
  }

  aerial_robot_msgs::FlightNavConstPtr nav_msg_ptr(new aerial_robot_msgs::FlightNav(nav_msg));

  GimbalrotorNavigator::naviCallback(nav_msg_ptr);
}

void GimbalrotorPerchingNavigator::applyActivePerchingTarget()
{
  if(!perching_enable_ || getNaviState() != HOVER_STATE)
  {
    return;
  }

  if(!perching_locked_)
  {
    if(!tryLockPerching("active update"))
    {
      return;
    }
  }

  aerial_robot_msgs::FlightNav nav_msg = buildActivePerchingNavCommand();
  aerial_robot_msgs::FlightNavConstPtr nav_msg_ptr(new aerial_robot_msgs::FlightNav(nav_msg));

  GimbalrotorNavigator::naviCallback(nav_msg_ptr);
}

aerial_robot_msgs::FlightNav GimbalrotorPerchingNavigator::buildActivePerchingNavCommand()
{
  aerial_robot_msgs::FlightNav nav_msg;

  const double target_pitch = computeActiveHoldPitch();
  const tf::Vector3 target_pos = has_active_pitch_target_
      ? computeArcPositionFromPitch(target_pitch)
      : computeActiveHoldPosition();

  nav_msg.pos_xy_nav_mode = NAV_MODE_POS;
  nav_msg.pos_z_nav_mode = NAV_MODE_POS;

  nav_msg.target_pos_x = target_pos.x();
  nav_msg.target_pos_y = target_pos.y();
  nav_msg.target_pos_z = target_pos.z();

  nav_msg.target_vel_x = 0.0;
  nav_msg.target_vel_y = 0.0;
  nav_msg.target_vel_z = 0.0;

  nav_msg.roll_nav_mode = NAV_MODE_NONE;

  nav_msg.pitch_nav_mode = NAV_MODE_POS;
  nav_msg.target_pitch = target_pitch;

  publishCommandedDebugPose(target_pos, target_pitch);

  return nav_msg;
}

tf::Vector3 GimbalrotorPerchingNavigator::getCurrentBaselinkPos() const
{
  return estimator_->getPos(Frame::BASELINK, estimate_mode_);
}

tf::Matrix3x3 GimbalrotorPerchingNavigator::getCurrentBaselinkRot() const
{
  return estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
}

tf::Vector3 GimbalrotorPerchingNavigator::computeHandPerchingCenterWorldFromBaselink() const
{
  const tf::Vector3 baselink_pos_world = getCurrentBaselinkPos();
  const tf::Matrix3x3 baselink_rot_world = getCurrentBaselinkRot();

  return baselink_pos_world + baselink_rot_world * hand_perching_center_offset_baselink_;
}

double GimbalrotorPerchingNavigator::computeRadiusPitchArcAngle(
    const tf::Vector3& radius_vec_world) const
{
  const double radius_xz =
      norm2D(radius_vec_world.x(), radius_vec_world.z());

  if(radius_xz < 1.0e-6)
  {
    return 0.0;
  }

  return std::atan2(-radius_vec_world.z(), radius_vec_world.x());
}

bool GimbalrotorPerchingNavigator::isManualPivotMode() const
{
  /*
   * manual:
   *   New clear name.
   *
   * hand_center:
   *   Legacy alias from previous implementation.
   */
  return pivot_source_ == "manual" ||
         pivot_source_ == "hand_center";
}

bool GimbalrotorPerchingNavigator::isBranchPivotMode() const
{
  /*
   * branch:
   *   New clear name.
   *
   * perching_point:
   *   Legacy alias from previous implementation.
   */
  return pivot_source_ == "branch" ||
         pivot_source_ == "perching_point";
}

bool GimbalrotorPerchingNavigator::hasBranchPivotSource() const
{
  return has_perching_point_ || has_branch_pose_;
}

tf::Vector3 GimbalrotorPerchingNavigator::computeLockPivotWorld() const
{
  if(isManualPivotMode())
  {
    /*
     * Manual mode:
     *
     * Use only robot geometry.
     * Branch mocap is ignored.
     */
    return computeHandPerchingCenterWorldFromBaselink();
  }

  if(isBranchPivotMode())
  {
    /*
     * Branch mode:
     *
     * Prefer corrected /perching/point if it exists.
     * Otherwise use raw /perching/branch_pose.
     */
    if(has_perching_point_)
    {
      return perching_point_world_;
    }

    return branch_pos_world_;
  }

  /*
   * Should never reach here because tryLockPerching() checks source validity.
   */
  return computeHandPerchingCenterWorldFromBaselink();
}

tf::Vector3 GimbalrotorPerchingNavigator::computeActiveHoldPosition() const
{
  tf::Vector3 locked_radius_xz(
      locked_radius_vec_world_.x(),
      0.0,
      locked_radius_vec_world_.z());

  double length_xz = norm2D(
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

  locked_radius_xz.setX(locked_radius_xz.x() / length_xz * locked_radius_);
  locked_radius_xz.setY(0.0);
  locked_radius_xz.setZ(locked_radius_xz.z() / length_xz * locked_radius_);

  tf::Vector3 target_pos = locked_pivot_world_ + locked_radius_xz;

  target_pos.setY(computeCompliantTargetY());

  return target_pos;
}

double GimbalrotorPerchingNavigator::computeActiveHoldPitch() const
{
  if(has_active_pitch_target_)
  {
    return active_target_pitch_;
  }

  return locked_robot_rpy_.y();
}

double GimbalrotorPerchingNavigator::computeCompliantTargetY() const
{
  const double branch_relative_y_ref = locked_pivot_world_.y() + locked_y_offset_;
  const double current_y = getCurrentRobotPos().y();
  const double dy = current_y - branch_relative_y_ref;

  if(std::fabs(dy) <= y_compliance_deadband_)
  {
    return current_y;
  }

  if(dy > 0.0)
  {
    return branch_relative_y_ref + y_compliance_deadband_;
  }

  return branch_relative_y_ref - y_compliance_deadband_;
}

void GimbalrotorPerchingNavigator::applyPerchingConstraint(aerial_robot_msgs::FlightNav& nav_msg)
{
  if(!perching_enable_ || getNaviState() != HOVER_STATE)
  {
    return;
  }

  if(!perching_locked_)
  {
    if(!tryLockPerching("first perching command"))
    {
      return;
    }
  }

  /*
   * Main cutting/perching mode:
   *
   * If pitch command is given:
   *   target_pitch -> target position on branch arc.
   *
   * Also store this pitch as the active pitch target, so the update loop keeps
   * tracking it even after /uav/nav stops.
   */
  if(use_pitch_command_for_arc_ && hasPitchCommand(nav_msg))
  {
    const double target_pitch = getCommandedPitch(nav_msg);
    const tf::Vector3 target_pos = computeArcPositionFromPitch(target_pitch);

    has_active_pitch_target_ = true;
    active_target_pitch_ = target_pitch;

    nav_msg.pos_xy_nav_mode = NAV_MODE_POS;
    nav_msg.pos_z_nav_mode = NAV_MODE_POS;

    nav_msg.target_pos_x = target_pos.x();
    nav_msg.target_pos_y = target_pos.y();
    nav_msg.target_pos_z = target_pos.z();

    nav_msg.target_vel_x = 0.0;
    nav_msg.target_vel_y = 0.0;
    nav_msg.target_vel_z = 0.0;

    nav_msg.pitch_nav_mode = NAV_MODE_POS;
    nav_msg.target_pitch = target_pitch;

    publishCommandedDebugPose(target_pos, target_pitch);

    return;
  }

  if(hold_locked_pose_without_pitch_command_ && !hasPositionCommand(nav_msg))
  {
    const double target_pitch = computeActiveHoldPitch();
    const tf::Vector3 target_pos = has_active_pitch_target_
        ? computeArcPositionFromPitch(target_pitch)
        : computeActiveHoldPosition();

    nav_msg.pos_xy_nav_mode = NAV_MODE_POS;
    nav_msg.pos_z_nav_mode = NAV_MODE_POS;

    nav_msg.target_pos_x = target_pos.x();
    nav_msg.target_pos_y = target_pos.y();
    nav_msg.target_pos_z = target_pos.z();

    nav_msg.target_vel_x = 0.0;
    nav_msg.target_vel_y = 0.0;
    nav_msg.target_vel_z = 0.0;

    nav_msg.pitch_nav_mode = NAV_MODE_POS;
    nav_msg.target_pitch = target_pitch;

    publishCommandedDebugPose(target_pos, target_pitch);

    return;
  }

  /*
   * Optional: if external code sends position commands during perching, project
   * them onto the branch arc.
   */
  if(constrain_position_command_ && hasPositionCommand(nav_msg))
  {
    tf::Vector3 desired_pos = getDesiredPosition(nav_msg);
    tf::Vector3 constrained_pos = projectPositionToPitchArc(desired_pos);

    nav_msg.pos_xy_nav_mode = NAV_MODE_POS;
    nav_msg.pos_z_nav_mode = NAV_MODE_POS;

    nav_msg.target_pos_x = constrained_pos.x();
    nav_msg.target_pos_y = constrained_pos.y();
    nav_msg.target_pos_z = constrained_pos.z();

    nav_msg.target_vel_x = 0.0;
    nav_msg.target_vel_y = 0.0;
    nav_msg.target_vel_z = 0.0;

    publishCommandedDebugPose(constrained_pos, getCurrentRobotRPY().y());
  }

  /*
   * Optional: if external code sends velocity command, remove radial velocity
   * that would push/pull away from the branch.
   */
  if(constrain_velocity_command_ && hasVelocityCommand(nav_msg))
  {
    tf::Vector3 desired_vel = getDesiredVelocity(nav_msg);
    tf::Vector3 tangent_vel = projectVelocityToPitchArcTangent(desired_vel);

    if(nav_msg.pos_xy_nav_mode == NAV_MODE_VEL || nav_msg.pos_xy_nav_mode == NAV_MODE_POS_VEL)
    {
      nav_msg.target_vel_x = tangent_vel.x();
      nav_msg.target_vel_y = 0.0;
    }

    if(nav_msg.pos_z_nav_mode == NAV_MODE_VEL)
    {
      nav_msg.target_vel_z = tangent_vel.z();
    }
  }
}

bool GimbalrotorPerchingNavigator::hasPitchCommand(const aerial_robot_msgs::FlightNav& nav_msg) const
{
  if(!accept_uav_nav_pitch_command_)
  {
    return false;
  }
  return nav_msg.pitch_nav_mode == NAV_MODE_POS;
}

bool GimbalrotorPerchingNavigator::hasPositionCommand(const aerial_robot_msgs::FlightNav& nav_msg) const
{
  return nav_msg.pos_xy_nav_mode == NAV_MODE_POS ||
         nav_msg.pos_xy_nav_mode == NAV_MODE_POS_VEL ||
         nav_msg.pos_z_nav_mode == NAV_MODE_POS;
}

bool GimbalrotorPerchingNavigator::hasVelocityCommand(const aerial_robot_msgs::FlightNav& nav_msg) const
{
  return nav_msg.pos_xy_nav_mode == NAV_MODE_VEL ||
         nav_msg.pos_xy_nav_mode == NAV_MODE_POS_VEL ||
         nav_msg.pos_z_nav_mode == NAV_MODE_VEL;
}

double GimbalrotorPerchingNavigator::getCommandedPitch(const aerial_robot_msgs::FlightNav& nav_msg) const
{
  double target_pitch = nav_msg.target_pitch;

  if(command_pitch_as_delta_)
  {
    target_pitch = locked_robot_rpy_.y() + command_pitch_sign_ * nav_msg.target_pitch;
  }

  double delta_pitch = normalizeAngle(target_pitch - locked_robot_rpy_.y());
  delta_pitch = clamp(delta_pitch, -max_pitch_delta_, max_pitch_delta_);

  return normalizeAngle(locked_robot_rpy_.y() + delta_pitch);
}

tf::Vector3 GimbalrotorPerchingNavigator::getCurrentRobotPos() const
{
  return estimator_->getPos(Frame::COG, estimate_mode_);
}

tf::Vector3 GimbalrotorPerchingNavigator::getCurrentRobotRPY() const
{
  return estimator_->getEuler(Frame::COG, estimate_mode_);
}

tf::Vector3 GimbalrotorPerchingNavigator::computeArcPositionFromPitch(double target_pitch) const
{
  /*
   * Hand-center pitch arc.
   *
   * Locked:
   *   pivot C = physical hand perching center
   *   robot position P0
   *   radius vector r0 = P0 - C
   *   pitch theta0
   *
   * Command:
   *   target pitch theta
   *
   * Compute:
   *   dtheta = theta - theta0
   *   r_des = RotY(dtheta) * r0_xz
   *   P_des = C + r_des
   *
   * Important:
   *   C is NOT the branch mocap origin.
  */
  double delta_pitch = normalizeAngle(target_pitch - locked_robot_rpy_.y());
  delta_pitch = clamp(delta_pitch, -max_pitch_delta_, max_pitch_delta_);

  const double signed_delta = arc_pitch_sign_ * delta_pitch;

  tf::Vector3 locked_radius_xz(locked_radius_vec_world_.x(), 0.0, locked_radius_vec_world_.z());

  double length_xz = norm2D(locked_radius_xz.x(), locked_radius_xz.z());

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

  locked_radius_xz.setX(locked_radius_xz.x() / length_xz * locked_radius_);
  locked_radius_xz.setY(0.0);
  locked_radius_xz.setZ(locked_radius_xz.z() / length_xz * locked_radius_);

  tf::Matrix3x3 rot(tf::createQuaternionFromRPY(0.0, signed_delta, 0.0));
  tf::Vector3 rotated_radius = rot * locked_radius_xz;

  double rotated_length_xz = norm2D(rotated_radius.x(), rotated_radius.z());

  if(rotated_length_xz < 1.0e-6)
  {
    rotated_radius.setX(locked_x_side_ * locked_radius_);
    rotated_radius.setY(0.0);
    rotated_radius.setZ(0.0);
  }
  else
  {
    rotated_radius.setX(rotated_radius.x() / rotated_length_xz * locked_radius_);
    rotated_radius.setY(0.0);
    rotated_radius.setZ(rotated_radius.z() / rotated_length_xz * locked_radius_);
  }

  tf::Vector3 target_pos = locked_pivot_world_ + rotated_radius;

  target_pos.setY(computeCompliantTargetY());

  return target_pos;
}

tf::Vector3 GimbalrotorPerchingNavigator::projectPositionToPitchArc(const tf::Vector3& desired_pos) const
{
  const double cx = locked_pivot_world_.x();
  const double cz = locked_pivot_world_.z();

  double dx = desired_pos.x() - cx;
  double dz = desired_pos.z() - cz;

  double length_xz = norm2D(dx, dz);

  if(length_xz < 1.0e-6)
  {
    dx = getCurrentRobotPos().x() - cx;
    dz = getCurrentRobotPos().z() - cz;
    length_xz = norm2D(dx, dz);
  }

  if(length_xz < 1.0e-6)
  {
    dx = locked_x_side_;
    dz = 0.0;
    length_xz = 1.0;
  }

  dx /= length_xz;
  dz /= length_xz;

  tf::Vector3 constrained_pos;
  constrained_pos.setX(cx + locked_radius_ * dx);
  constrained_pos.setY(computeCompliantTargetY());
  constrained_pos.setZ(cz + locked_radius_ * dz);

  return constrained_pos;
}

tf::Vector3 GimbalrotorPerchingNavigator::projectVelocityToPitchArcTangent(const tf::Vector3& desired_vel) const
{
  const double cx = locked_pivot_world_.x();
  const double cz = locked_pivot_world_.z();

  double rx = getCurrentRobotPos().x() - cx;
  double rz = getCurrentRobotPos().z() - cz;

  double length_xz = norm2D(rx, rz);

  if(length_xz < 1.0e-6)
  {
    return tf::Vector3(0.0, 0.0, 0.0);
  }

  rx /= length_xz;
  rz /= length_xz;

  tf::Vector3 tangent(-rz, 0.0, rx);

  const double tangent_speed = desired_vel.dot(tangent);

  return tangent * tangent_speed;
}

tf::Vector3 GimbalrotorPerchingNavigator::getDesiredPosition(const aerial_robot_msgs::FlightNav& nav_msg) const
{
  tf::Vector3 desired_pos = getCurrentRobotPos();

  if(nav_msg.pos_xy_nav_mode == NAV_MODE_POS || nav_msg.pos_xy_nav_mode == NAV_MODE_POS_VEL)
  {
    desired_pos.setX(nav_msg.target_pos_x);
    desired_pos.setY(nav_msg.target_pos_y);
  }

  if(nav_msg.pos_z_nav_mode == NAV_MODE_POS)
  {
    desired_pos.setZ(nav_msg.target_pos_z);
  }

  return desired_pos;
}

tf::Vector3 GimbalrotorPerchingNavigator::getDesiredVelocity(const aerial_robot_msgs::FlightNav& nav_msg) const
{
  tf::Vector3 desired_vel(0.0, 0.0, 0.0);

  if(nav_msg.pos_xy_nav_mode == NAV_MODE_VEL || nav_msg.pos_xy_nav_mode == NAV_MODE_POS_VEL)
  {
    desired_vel.setX(nav_msg.target_vel_x);
    desired_vel.setY(nav_msg.target_vel_y);
  }

  if(nav_msg.pos_z_nav_mode == NAV_MODE_VEL)
  {
    desired_vel.setZ(nav_msg.target_vel_z);
  }

  return desired_vel;
}

double GimbalrotorPerchingNavigator::clamp(double value, double min_value, double max_value) const
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

double GimbalrotorPerchingNavigator::normalizeAngle(double angle) const
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

double GimbalrotorPerchingNavigator::norm2D(double x, double z) const
{
  return std::sqrt(x * x + z * z);
}

double GimbalrotorPerchingNavigator::norm3D(const tf::Vector3& v) const
{
  return std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
}

void GimbalrotorPerchingNavigator::publishLockedDebugPose()
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";

  msg.pose.position.x = locked_robot_pos_world_.x();
  msg.pose.position.y = locked_robot_pos_world_.y();
  msg.pose.position.z = locked_robot_pos_world_.z();

  tf::Quaternion q;
  q.setRPY(locked_robot_rpy_.x(),
           locked_robot_rpy_.y(),
           locked_robot_rpy_.z());
  tf::quaternionTFToMsg(q, msg.pose.orientation);

  locked_pose_pub_.publish(msg);
}

void GimbalrotorPerchingNavigator::publishLockedPivot()
{
  geometry_msgs::PointStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";

  msg.point.x = locked_pivot_world_.x();
  msg.point.y = locked_pivot_world_.y();
  msg.point.z = locked_pivot_world_.z();

  locked_pivot_pub_.publish(msg);
}

void GimbalrotorPerchingNavigator::publishCommandedDebugPose(const tf::Vector3& pos, double pitch)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";

  msg.pose.position.x = pos.x();
  msg.pose.position.y = pos.y();
  msg.pose.position.z = pos.z();

  tf::Quaternion q;
  q.setRPY(locked_robot_rpy_.x(), pitch, locked_robot_rpy_.z());
  tf::quaternionTFToMsg(q, msg.pose.orientation);

  commanded_pose_pub_.publish(msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorPerchingNavigator,
                       aerial_robot_navigation::BaseNavigator);
