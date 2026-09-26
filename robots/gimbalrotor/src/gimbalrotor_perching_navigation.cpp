// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_perching_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

namespace
{
const int NAV_MODE_VEL = aerial_robot_msgs::FlightNav::VEL_MODE;
const int NAV_MODE_POS = aerial_robot_msgs::FlightNav::POS_MODE;
const int NAV_MODE_POS_VEL = aerial_robot_msgs::FlightNav::POS_VEL_MODE;

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

  active_pitch_delta_(0.0),
  locked_radius_(0.0)
{
  branch_pos_world_.setValue(0.0, 0.0, 0.0);
  perching_point_world_.setValue(0.0, 0.0, 0.0);

  hand_perching_center_offset_baselink_.setValue(0.0, 0.0, 0.0);
  
  locked_robot_pos_world_.setValue(0.0, 0.0, 0.0);
  reference_locked_rpy_.setValue(0.0, 0.0, 0.0);
  locked_pivot_world_.setValue(0.0, 0.0, 0.0);
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
  commanded_pitch_delta_pub_ = nh_.advertise<std_msgs::Float64>("perching/commanded_pitch_delta", 1);

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

  if(!std::isfinite(min_valid_radius_) || min_valid_radius_ <= 1.0e-6) min_valid_radius_ = 0.05;
  if(!std::isfinite(max_pitch_delta_) || max_pitch_delta_ <= 0.0 || max_pitch_delta_ > PI)
    max_pitch_delta_ = 0.5235987756;
  if(!std::isfinite(arc_pitch_sign_) || arc_pitch_sign_ == 0.0) arc_pitch_sign_ = 1.0;
  arc_pitch_sign_ = arc_pitch_sign_ >= 0.0 ? 1.0 : -1.0;
  if(!std::isfinite(command_pitch_sign_) || command_pitch_sign_ == 0.0) command_pitch_sign_ = 1.0;
  command_pitch_sign_ = command_pitch_sign_ >= 0.0 ? 1.0 : -1.0;
  if(!std::isfinite(y_compliance_deadband_) || y_compliance_deadband_ < 0.0) y_compliance_deadband_ = 0.03;

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

  active_pitch_delta_ = 0.0;
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

  if(!tryLockPerching("manual relock"))
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator] "
        "manual perching relock failed");
    return;
  }

  active_pitch_delta_ = 0.0;
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
  active_pitch_delta_ = 0.0;
  lock_stamp_ = ros::Time(0);
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

  perching_geometry::Pose pose;
  if(!geometry_.target(command_pitch_sign_ * msg->data, max_pitch_delta_, arc_pitch_sign_, pose))
    return;
  active_pitch_delta_ = pose.delta;
  applyAxialCompliance(pose);
  aerial_robot_msgs::FlightNav command;
  setPoseCommand(command, pose);
  aerial_robot_msgs::FlightNavConstPtr command_ptr(new aerial_robot_msgs::FlightNav(command));
  GimbalrotorNavigator::naviCallback(command_ptr);
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

  locked_pivot_world_ = computeLockPivotWorld();
  perching_point_world_ = locked_pivot_world_;

  const tf::Vector3 existing_target_rpy = getTargetRPY();
  const tf::Vector3 measured_rpy = getCurrentRobotRPY();
  // NORMAL preserves roll/yaw intent; measured tracking error is not a target.
  const tf::Vector3 reference_rpy(existing_target_rpy.x(), measured_rpy.y(), existing_target_rpy.z());
  const tf::Quaternion reference_orientation = tf::createQuaternionFromRPY(
      reference_rpy.x(), reference_rpy.y(), reference_rpy.z());
  if(!geometry_.initialize(locked_robot_pos_world_, locked_pivot_world_, reference_orientation, min_valid_radius_))
  {
    ROS_WARN("[GimbalrotorPerchingNavigator] rejected invalid perching geometry");
    return false;
  }
  reference_locked_rpy_ = reference_rpy;
  locked_radius_ = geometry_.radial().length();
  active_pitch_delta_ = 0.0;
  lock_stamp_ = ros::Time::now();

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
  ROS_WARN("[GimbalrotorPerchingNavigator] measured locked rpy deg: roll %.2f, pitch %.2f, yaw %.2f",
           measured_rpy.x() * 180.0 / PI,
           measured_rpy.y() * 180.0 / PI,
           measured_rpy.z() * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingNavigator] pre-lock target rpy deg: roll %.2f, pitch %.2f, yaw %.2f",
           existing_target_rpy.x() * 180.0 / PI,
           existing_target_rpy.y() * 180.0 / PI,
           existing_target_rpy.z() * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingNavigator] NORMAL reference rpy deg: roll %.2f, pitch %.2f, yaw %.2f",
           reference_locked_rpy_.x() * 180.0 / PI,
           reference_locked_rpy_.y() * 180.0 / PI,
           reference_locked_rpy_.z() * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingNavigator] reference local +Y axis in world: x %.6f, y %.6f, z %.6f",
           geometry_.axis.x(), geometry_.axis.y(), geometry_.axis.z());
  ROS_WARN("[GimbalrotorPerchingNavigator] pitch-plane radius: %.3f m",
           locked_radius_);
  publishLockedDebugPose();
  publishLockedPivot();
  perching_geometry::Pose pose;
  if(activePose(pose)) publishCommandedDebugPose(pose);

  return true;
}

void GimbalrotorPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  aerial_robot_msgs::FlightNav nav_msg = *msg;

  if(perching_enable_ && getNaviState() == HOVER_STATE)
  {
    if(!applyPerchingConstraint(nav_msg)) return;
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
  aerial_robot_msgs::FlightNav msg;
  perching_geometry::Pose pose;
  if(activePose(pose)) setPoseCommand(msg, pose);
  return msg;
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

bool GimbalrotorPerchingNavigator::applyPerchingConstraint(aerial_robot_msgs::FlightNav& msg)
{
  if(!perching_enable_ || getNaviState() != HOVER_STATE) return false;
  if(!perching_locked_ && !tryLockPerching("first perching command")) return false;

  const bool position_command = hasPositionCommand(msg);
  const bool velocity_command = hasVelocityCommand(msg);
  const tf::Vector3 desired_velocity = getDesiredVelocity(msg);
  perching_geometry::Pose pose;
  if(use_pitch_command_for_arc_ && hasPitchCommand(msg))
  {
    if(command_pitch_as_delta_)
    {
      if(!geometry_.target(command_pitch_sign_ * msg.target_pitch,
                           max_pitch_delta_, arc_pitch_sign_, pose)) return false;
    }
    else
    {
      // Legacy absolute-attitude input: project the complete requested attitude
      // onto the allowed local-Y rotation; never subtract world Euler pitch.
      const tf::Quaternion desired = tf::createQuaternionFromRPY(
          msg.roll_nav_mode == NAV_MODE_POS ? msg.target_roll : reference_locked_rpy_.x(),
          msg.target_pitch,
          msg.yaw_nav_mode == NAV_MODE_POS ? msg.target_yaw : reference_locked_rpy_.z());
      const tf::Matrix3x3 relative(geometry_.orientation.inverse() * desired);
      const double angle = std::atan2(relative[0][2] - relative[2][0],
                                     relative[0][0] + relative[2][2]);
      if(!geometry_.target(angle / arc_pitch_sign_, max_pitch_delta_, arc_pitch_sign_, pose)) return false;
    }
    active_pitch_delta_ = pose.delta;
    applyAxialCompliance(pose);
    setPoseCommand(msg, pose);
    return true;
  }

  if(constrain_position_command_ && position_command)
  {
    if(!geometry_.project(getDesiredPosition(msg), max_pitch_delta_, arc_pitch_sign_, pose) &&
       !geometry_.target(active_pitch_delta_, max_pitch_delta_, arc_pitch_sign_, pose)) return false;
    active_pitch_delta_ = pose.delta;
    applyAxialCompliance(pose);
    setPoseCommand(msg, pose);
  }
  else if(hold_locked_pose_without_pitch_command_ && !position_command && !velocity_command)
  {
    if(!activePose(pose)) return false;
    setPoseCommand(msg, pose);
    return true;
  }

  if(constrain_velocity_command_ && velocity_command)
  {
    const tf::Vector3 tangent = geometry_.tangentVelocity(getCurrentRobotPos(), desired_velocity);
    // Projection can couple every world component, including previously idle axes.
    if(position_command && !constrain_position_command_)
    {
      const tf::Vector3 position = getDesiredPosition(msg);
      msg.target_pos_x = position.x();
      msg.target_pos_y = position.y();
      msg.target_pos_z = position.z();
    }
    msg.pos_xy_nav_mode = position_command ? NAV_MODE_POS_VEL : NAV_MODE_VEL;
    msg.pos_z_nav_mode = position_command ? NAV_MODE_POS_VEL : NAV_MODE_VEL;
    msg.control_frame = aerial_robot_msgs::FlightNav::WORLD_FRAME;
    msg.target_vel_x = tangent.x();
    msg.target_vel_y = tangent.y();
    msg.target_vel_z = tangent.z();
  }
  return true;
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
         nav_msg.pos_z_nav_mode == NAV_MODE_POS ||
         nav_msg.pos_z_nav_mode == NAV_MODE_POS_VEL;
}

bool GimbalrotorPerchingNavigator::hasVelocityCommand(const aerial_robot_msgs::FlightNav& nav_msg) const
{
  return nav_msg.pos_xy_nav_mode == NAV_MODE_VEL ||
         nav_msg.pos_xy_nav_mode == NAV_MODE_POS_VEL ||
         nav_msg.pos_z_nav_mode == NAV_MODE_VEL ||
         nav_msg.pos_z_nav_mode == NAV_MODE_POS_VEL;
}

tf::Vector3 GimbalrotorPerchingNavigator::getCurrentRobotPos() const
{
  return estimator_->getPos(Frame::COG, estimate_mode_);
}

tf::Vector3 GimbalrotorPerchingNavigator::getCurrentRobotRPY() const
{
  return estimator_->getEuler(Frame::COG, estimate_mode_);
}

tf::Vector3 GimbalrotorPerchingNavigator::getDesiredPosition(const aerial_robot_msgs::FlightNav& nav_msg) const
{
  tf::Vector3 desired_pos = getCurrentRobotPos();

  if(nav_msg.pos_xy_nav_mode == NAV_MODE_POS || nav_msg.pos_xy_nav_mode == NAV_MODE_POS_VEL)
  {
    desired_pos.setX(nav_msg.target_pos_x);
    desired_pos.setY(nav_msg.target_pos_y);
  }

  if(nav_msg.pos_z_nav_mode == NAV_MODE_POS || nav_msg.pos_z_nav_mode == NAV_MODE_POS_VEL)
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

  if(nav_msg.pos_z_nav_mode == NAV_MODE_VEL || nav_msg.pos_z_nav_mode == NAV_MODE_POS_VEL)
  {
    desired_vel.setZ(nav_msg.target_vel_z);
  }

  // Match BaseNavigator's existing local-XY velocity convention before projecting.
  // Z remains a world coordinate; generic joystick/local-Z behavior is unchanged.
  if(nav_msg.control_frame == aerial_robot_msgs::FlightNav::LOCAL_FRAME &&
     nav_msg.pos_xy_nav_mode == NAV_MODE_VEL)
  {
    const double yaw = getCurrentRobotRPY().z();
    desired_vel = tf::Matrix3x3(tf::createQuaternionFromRPY(0, 0, yaw)) * desired_vel;
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

void GimbalrotorPerchingNavigator::publishLockedDebugPose()
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = lock_stamp_;
  msg.header.frame_id = "world";

  msg.pose.position.x = locked_robot_pos_world_.x();
  msg.pose.position.y = locked_robot_pos_world_.y();
  msg.pose.position.z = locked_robot_pos_world_.z();

  tf::quaternionTFToMsg(geometry_.orientation, msg.pose.orientation);

  locked_pose_pub_.publish(msg);
}

void GimbalrotorPerchingNavigator::publishLockedPivot()
{
  geometry_msgs::PointStamped msg;
  msg.header.stamp = lock_stamp_;
  msg.header.frame_id = "world";

  msg.point.x = locked_pivot_world_.x();
  msg.point.y = locked_pivot_world_.y();
  msg.point.z = locked_pivot_world_.z();

  locked_pivot_pub_.publish(msg);
}

void GimbalrotorPerchingNavigator::publishCommandedDebugPose(const perching_geometry::Pose& pose)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  tf::pointTFToMsg(pose.position, msg.pose.position);
  tf::quaternionTFToMsg(pose.orientation, msg.pose.orientation);
  commanded_pose_pub_.publish(msg);
  std_msgs::Float64 delta;
  delta.data = pose.delta;
  commanded_pitch_delta_pub_.publish(delta);
}

bool GimbalrotorPerchingNavigator::activePose(perching_geometry::Pose& pose) const
{
  if(!perching_locked_ || !geometry_.target(active_pitch_delta_, max_pitch_delta_, arc_pitch_sign_, pose))
    return false;
  applyAxialCompliance(pose);
  return true;
}

void GimbalrotorPerchingNavigator::applyAxialCompliance(perching_geometry::Pose& pose) const
{
  const double displacement = geometry_.axis.dot(getCurrentRobotPos() - geometry_.position);
  if(std::isfinite(displacement))
    pose.position += geometry_.axis * clamp(displacement, -y_compliance_deadband_, y_compliance_deadband_);
}

void GimbalrotorPerchingNavigator::setPoseCommand(
    aerial_robot_msgs::FlightNav& msg, const perching_geometry::Pose& pose)
{
  msg.control_frame = aerial_robot_msgs::FlightNav::WORLD_FRAME;
  msg.target = aerial_robot_msgs::FlightNav::COG;
  msg.pos_xy_nav_mode = NAV_MODE_POS;
  msg.pos_z_nav_mode = NAV_MODE_POS;
  msg.target_pos_x = pose.position.x();
  msg.target_pos_y = pose.position.y();
  msg.target_pos_z = pose.position.z();
  msg.target_vel_x = msg.target_vel_y = msg.target_vel_z = 0.0;
  msg.roll_nav_mode = msg.pitch_nav_mode = msg.yaw_nav_mode = NAV_MODE_POS;
  double roll, pitch, yaw;
  tf::Matrix3x3(pose.orientation).getRPY(roll, pitch, yaw);
  // NORMAL controls pitch while preserving the captured roll/yaw intent.
  // The full local-Y rotation remains in pose for geometry and diagnostics;
  // at nonzero reference roll its Euler roll/yaw need not stay constant.
  msg.target_roll = reference_locked_rpy_.x();
  msg.target_pitch = pitch;
  msg.target_yaw = reference_locked_rpy_.z();
  publishCommandedDebugPose(pose);
}

bool GimbalrotorPerchingNavigator::perchingAdmittanceTarget(
    const ros::Time& lock_stamp, double physical_offset,
    const tf::Vector3& nominal_position, perching_geometry::Pose& pose) const
{
  if(!perching_enable_ || !perching_locked_ || navi_state_ != HOVER_STATE ||
     lock_stamp != lock_stamp_ || !perching_geometry::finite(nominal_position)) return false;
  // Compliance Y is a physical axis angle. Convert to the logical coordinate
  // before the shared helper applies arc_pitch_sign exactly once.
  if(!geometry_.target(active_pitch_delta_ + physical_offset / arc_pitch_sign_,
                       max_pitch_delta_, arc_pitch_sign_, pose)) return false;
  pose.position += geometry_.axis * geometry_.axis.dot(nominal_position - geometry_.position);
  return true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorPerchingNavigator,
                       aerial_robot_navigation::BaseNavigator);
