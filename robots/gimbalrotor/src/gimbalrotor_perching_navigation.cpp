// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_perching_navigation.h>

#include <tf_conversions/tf_eigen.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <vector>

using aerial_robot_control::SpatialConstraintConfig;
using aerial_robot_control::SpatialConstraintTarget;
using aerial_robot_control::Vector6d;

namespace
{
constexpr int NAV_MODE_POS = 2;
constexpr double PI = 3.14159265358979323846;
}

namespace aerial_robot_navigation
{

GimbalrotorPerchingNavigator::GimbalrotorPerchingNavigator()
  : GimbalrotorNavigator()
{
  // Current application default: rotation about constraint-frame Y.
  allowed_dof_(4) = 1.0;
}

void GimbalrotorPerchingNavigator::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    double loop_du)
{
  GimbalrotorNavigator::initialize(
      nh,
      nhp,
      robot_model,
      estimator,
      loop_du);

  perching_enable_sub_ = nh_.subscribe(perching_enable_topic_, 1, &GimbalrotorPerchingNavigator::perchingEnableCallback, this);
  relock_sub_ = nh_.subscribe(relock_topic_, 1, &GimbalrotorPerchingNavigator::relockCallback, this);
  reset_sub_ = nh_.subscribe(reset_topic_, 1, &GimbalrotorPerchingNavigator::resetConstraintCallback, this);
  branch_pose_sub_ = nh_.subscribe(branch_pose_topic_, 1, &GimbalrotorPerchingNavigator::branchPoseCallback, this);
  perching_point_sub_ = nh_.subscribe(perching_point_topic_, 1, &GimbalrotorPerchingNavigator::perchingPointCallback, this);
  target_angle_sub_ = nh_.subscribe(target_angle_topic_, 1, &GimbalrotorPerchingNavigator::targetAngleDegCallback, this);
  add_angle_sub_ = nh_.subscribe(add_angle_topic_, 1, &GimbalrotorPerchingNavigator::addAngleDegCallback, this);

  // Keep the current mission/helper topic names as aliases.
  legacy_target_pitch_sub_ = nh_.subscribe(legacy_target_pitch_topic_, 1, &GimbalrotorPerchingNavigator::targetAngleDegCallback, this);
  legacy_add_pitch_sub_ = nh_.subscribe(legacy_add_pitch_topic_, 1, &GimbalrotorPerchingNavigator::addAngleDegCallback, this);
  legacy_manual_pitch_delta_sub_ = nh_.subscribe(legacy_manual_pitch_delta_topic_, 1, &GimbalrotorPerchingNavigator::addAngleRadCallback, this);
  locked_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("perching/locked_pose", 1, true);
  locked_pivot_pub_ = nh_.advertise<geometry_msgs::PointStamped>("perching/locked_pivot", 1, true);
  commanded_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("perching/commanded_pose", 1);

  ROS_WARN("[GimbalrotorPerchingNavigator] initialized");
}

void GimbalrotorPerchingNavigator::rosParamInit()
{
  GimbalrotorNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<bool>(navi_nh, "perching_enable", perching_enabled_, false);
  getParam<bool>(navi_nh, "perching_lock_once", lock_once_, true);
  getParam<bool>(navi_nh, "perching_active_hold_enable", active_hold_enabled_, true);
  getParam<bool>(navi_nh, "perching_accept_uav_nav_angle_command", accept_uav_nav_angle_command_, false);

  // Compatibility with the old pitch-specific parameter.
  if(!navi_nh.hasParam("perching_accept_uav_nav_angle_command"))
    getParam<bool>(navi_nh, "perching_accept_uav_nav_pitch_command", accept_uav_nav_angle_command_, false);

  getParam<bool>(navi_nh, "perching_command_angle_as_delta", command_angle_as_delta_, true);
  if(!navi_nh.hasParam("perching_command_angle_as_delta"))
    getParam<bool>(navi_nh, "perching_command_pitch_as_delta", command_angle_as_delta_, true);

  getParam<double>(navi_nh, "perching_min_valid_radius", minimum_valid_radius_, 0.05);

  getParam<double>(navi_nh, "perching_max_constraint_angle", maximum_coordinate_, 0.5235987756);
  if(!navi_nh.hasParam("perching_max_constraint_angle"))
    getParam<double>(navi_nh, "perching_max_pitch_delta", maximum_coordinate_, 0.5235987756);

  getParam<double>(navi_nh, "perching_constraint_coordinate_sign", coordinate_sign_, 1.0);
  if(!navi_nh.hasParam("perching_constraint_coordinate_sign"))
    getParam<double>(navi_nh, "perching_arc_pitch_sign", coordinate_sign_, 1.0);

  getParam<double>(navi_nh, "perching_command_sign", command_sign_, 1.0);
  if(!navi_nh.hasParam("perching_command_sign"))
    getParam<double>(navi_nh, "perching_command_pitch_sign", command_sign_, 1.0);

  getParam<std::string>(navi_nh, "perching_pivot_source", pivot_source_, std::string("manual"));

  loadVector6Param(navi_nh, "perching_constraint_allowed_dof", allowed_dof_);
  loadVector3Param(navi_nh, "perching_constraint_frame_rpy", constraint_frame_rpy_);


  double hand_x = hand_center_offset_baselink_.x();
  double hand_y = hand_center_offset_baselink_.y();
  double hand_z = hand_center_offset_baselink_.z();

  getParam<double>(navi_nh, "hand_perching_center_offset_baselink_x", hand_x, hand_x);
  getParam<double>(navi_nh, "hand_perching_center_offset_baselink_y", hand_y, hand_y);
  getParam<double>(navi_nh, "hand_perching_center_offset_baselink_z", hand_z, hand_z);

  hand_center_offset_baselink_ = Eigen::Vector3d(hand_x, hand_y, hand_z);

  getParam<std::string>(navi_nh, "perching_enable_topic", perching_enable_topic_, std::string("perching/enable"));
  getParam<std::string>(navi_nh, "perching_relock_topic", relock_topic_, std::string("perching/relock"));
  getParam<std::string>(navi_nh, "perching_reset_topic", reset_topic_, std::string("perching/reset"));
  getParam<std::string>(navi_nh, "perching_branch_pose_topic", branch_pose_topic_, std::string("perching/branch_pose"));
  getParam<std::string>(navi_nh, "perching_point_topic", perching_point_topic_, std::string("perching/point"));
  getParam<std::string>(navi_nh, "perching_target_angle_topic", target_angle_topic_, std::string("perching/target_angle_deg"));
  getParam<std::string>(navi_nh, "perching_add_angle_topic", add_angle_topic_, std::string("perching/add_angle_deg"));
  getParam<std::string>(navi_nh, "perching_target_pitch_topic", legacy_target_pitch_topic_, std::string("perching/target_pitch_deg"));
  getParam<std::string>(navi_nh, "perching_add_pitch_topic", legacy_add_pitch_topic_, std::string("perching/add_pitch_deg"));
  getParam<std::string>(navi_nh, "perching_manual_pitch_delta_topic", legacy_manual_pitch_delta_topic_, std::string("perching/manual_pitch_delta"));
}

void GimbalrotorPerchingNavigator::update()
{
  // Update the constrained target first, then let GimbalrotorNavigator process
  // the resulting attitude target in the same navigation cycle.
  if(perching_enabled_ && active_hold_enabled_)
  {
    if(!perching_locked_)
    {
      tryLockConstraint("navigator update");
    }

    applyActiveConstraintTarget();
  }

  GimbalrotorNavigator::update();
}

bool GimbalrotorPerchingNavigator::isPerchingEnabled() const
{
  return perching_enabled_;
}

bool GimbalrotorPerchingNavigator::isPerchingLocked() const
{
  return perching_locked_ && spatial_constraint_.isValid();
}

const aerial_robot_control::SpatialConstraint& GimbalrotorPerchingNavigator::getSpatialConstraint() const
{
  return spatial_constraint_;
}

double GimbalrotorPerchingNavigator::getActiveConstraintCoordinate() const
{
  return active_coordinate_;
}

SpatialConstraintTarget
GimbalrotorPerchingNavigator::calculateConstrainedTarget(
    double coordinate,
    double coordinate_velocity,
    double coordinate_acceleration) const
{
  /*
   * The SpatialConstraint now represents the base_link geometry.
   *
   * Therefore, this function returns a constrained BASE_LINK target.
   */
  return spatial_constraint_.calculateTarget(
      coordinate,
      coordinate_velocity,
      coordinate_acceleration);
}

SpatialConstraintTarget
GimbalrotorPerchingNavigator::convertBaselinkTargetToCogTarget(
    const SpatialConstraintTarget& baselink_target) const
{
  SpatialConstraintTarget cog_target = baselink_target;

  if(!baselink_target.valid)
  {
    return cog_target;
  }

  /*
   * Read the current relationship between base_link and COG automatically.
   *
   * The user does not configure this.
   */
  const Eigen::Vector3d current_baselink_position_world =
      getCurrentBaselinkPositionWorld();

  const Eigen::Matrix3d current_baselink_rotation_world =
      getCurrentBaselinkRotationWorld();

  const Eigen::Vector3d current_cog_position_world =
      getCurrentCogPositionWorld();

  if(!current_baselink_position_world.allFinite() ||
     !current_baselink_rotation_world.allFinite() ||
     !current_cog_position_world.allFinite())
  {
    cog_target.valid = false;
    return cog_target;
  }

  /*
   * Current vector:
   *
   *   base_link origin -> COG
   *
   * expressed in base_link coordinates.
   */
  const Eigen::Vector3d baselink_to_cog_baselink =
      current_baselink_rotation_world.transpose()
      * (current_cog_position_world
         - current_baselink_position_world);

  /*
   * Rotate that vector using the desired base_link orientation.
   */
  const Eigen::Vector3d baselink_to_cog_world =
      baselink_target.rotation_world
      * baselink_to_cog_baselink;

  /*
   * The base_link constrained pose has already been calculated.
   *
   * This only calculates the corresponding COG position required by the
   * existing position PID.
   */
  cog_target.position_world =
      baselink_target.position_world
      + baselink_to_cog_world;

  /*
   * Rigid-body velocity conversion:
   *
   *   v_COG = v_base + omega x r_base_to_COG
   */
  cog_target.linear_velocity_world =
      baselink_target.linear_velocity_world
      +
      baselink_target.angular_velocity_world.cross(
          baselink_to_cog_world);

  /*
   * Rigid-body acceleration conversion:
   *
   *   a_COG =
   *       a_base
   *       + alpha x r
   *       + omega x (omega x r)
   */
  cog_target.linear_acceleration_world =
      baselink_target.linear_acceleration_world
      +
      baselink_target.angular_acceleration_world.cross(
          baselink_to_cog_world)
      +
      baselink_target.angular_velocity_world.cross(
          baselink_target.angular_velocity_world.cross(
              baselink_to_cog_world));

  /*
   * Rotation, angular velocity and angular acceleration do not require an
   * origin translation, so the copied values remain valid.
   */
  cog_target.valid =
      cog_target.position_world.allFinite()
      &&
      cog_target.rotation_world.allFinite()
      &&
      cog_target.linear_velocity_world.allFinite()
      &&
      cog_target.angular_velocity_world.allFinite()
      &&
      cog_target.linear_acceleration_world.allFinite()
      &&
      cog_target.angular_acceleration_world.allFinite();

  return cog_target;
}

void GimbalrotorPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  GimbalrotorNavigator::naviCallback(msg);

  if(!perching_enabled_)
  {
    return;
  }

  if(!perching_locked_ && !tryLockConstraint("first perching navigation command"))
  {
    return;
  }

  if(accept_uav_nav_angle_command_ && hasConstraintAngleCommand(*msg))
  {
    setActiveCoordinateFromAngle(getConstraintAngleCommand(*msg));
  }

  // During perching, the constrained target is the final navigation target.
  applyActiveConstraintTarget();
}

void GimbalrotorPerchingNavigator::handleFinalTargetBaselinkRPYCommand(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  /*
   * Preserve normal gimbalrotor behavior outside perching mode.
   */
  if(!perching_enabled_)
  {
    GimbalrotorNavigator::handleFinalTargetBaselinkRPYCommand(msg);

    return;
  }

  if(!perching_locked_ && !tryLockConstraint("base_link RPY command"))
  {
    return;
  }

  const tf::Vector3 requested_baselink_rpy(msg->vector.x, msg->vector.y, msg->vector.z);

  const double requested_constraint_angle = getConstraintAngleFromBaselinkRpy(requested_baselink_rpy);

  /* Convert the requested absolute base_link pitch/roll/yaw into q relative to the locked base_link orientation. */
  setActiveCoordinateFromAbsoluteBaselinkAngle(requested_constraint_angle);

  /*
   * This generates together:
   *
   *   1. desired base_link orientation,
   *   2. desired base_link position on the pivot arc,
   *   3. corresponding COG target for the existing position PID.
   */
  applyActiveConstraintTarget();
}

void GimbalrotorPerchingNavigator::reset()
{
  GimbalrotorNavigator::reset();
  clearConstraint();
}

void GimbalrotorPerchingNavigator::perchingEnableCallback(const std_msgs::BoolConstPtr& msg)
{
  perching_enabled_ = msg->data;

  if(perching_enabled_ && !perching_locked_)
  {
    tryLockConstraint("enable command");
  }

  if(!perching_enabled_)
  {
    ROS_WARN("[GimbalrotorPerchingNavigator] perching disabled");
  }
}

void GimbalrotorPerchingNavigator::relockCallback(const std_msgs::EmptyConstPtr&)
{
  clearConstraint();
  tryLockConstraint("relock command");
}

void GimbalrotorPerchingNavigator::resetConstraintCallback(const std_msgs::EmptyConstPtr&)
{
  clearConstraint();
}

void GimbalrotorPerchingNavigator::branchPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  branch_position_world_ = Eigen::Vector3d(
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z);
  has_branch_pose_ = true;
}

void GimbalrotorPerchingNavigator::perchingPointCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
  perching_point_world_ = Eigen::Vector3d(
      msg->point.x,
      msg->point.y,
      msg->point.z);
  has_perching_point_ = true;
}

void GimbalrotorPerchingNavigator::targetAngleDegCallback(const std_msgs::Float64ConstPtr& msg)
{
  if(!perching_enabled_)
  {
    ROS_WARN_THROTTLE(1.0, "[GimbalrotorPerchingNavigator] target angle ignored: perching disabled");
    return;
  }

  if(!perching_locked_ && !tryLockConstraint("target-angle command"))
  {
    return;
  }

  setActiveCoordinateFromAngle(msg->data * PI / 180.0);
  applyActiveConstraintTarget();
}

void GimbalrotorPerchingNavigator::addAngleDegCallback(const std_msgs::Float64ConstPtr& msg)
{
  if(!perching_enabled_)
  {
    ROS_WARN_THROTTLE(1.0, "[GimbalrotorPerchingNavigator] angle increment ignored: perching disabled");
    return;
  }

  if(!perching_locked_ && !tryLockConstraint("add-angle command"))
  {
    return;
  }

  addActiveCoordinate(msg->data * PI / 180.0);
  applyActiveConstraintTarget();
}

void GimbalrotorPerchingNavigator::addAngleRadCallback(const std_msgs::Float64ConstPtr& msg)
{
  if(!perching_enabled_)
  {
    ROS_WARN_THROTTLE(1.0, "[GimbalrotorPerchingNavigator] radian angle increment ignored: perching disabled");
    return;
  }

  if(!perching_locked_ && !tryLockConstraint("radian add-angle command"))
  {
    return;
  }

  addActiveCoordinate(msg->data);
  applyActiveConstraintTarget();
}

bool GimbalrotorPerchingNavigator::tryLockConstraint(const std::string& reason)
{
  if(perching_locked_ && lock_once_)
  {
    return true;
  }

  if(!isManualPivotSource() && !isBranchPivotSource())
  {
    ROS_ERROR("[GimbalrotorPerchingNavigator] " "invalid pivot source: %s", pivot_source_.c_str());

    return false;
  }

  if(isBranchPivotSource() && !hasBranchPivot())
  {
    ROS_WARN_THROTTLE(1.0, "[GimbalrotorPerchingNavigator] " "branch pivot is not available");

    return false;
  }

  /*
   * Capture base_link exactly once.
   *
   * These values define the perching geometry.
   */
  const Eigen::Vector3d baselink_position_world = getCurrentBaselinkPositionWorld();

  const Eigen::Matrix3d baselink_rotation_world = getCurrentBaselinkRotationWorld();
  const tf::Vector3 current_cog_target_rpy = getTargetRPY();

  locked_cog_target_rpy_ = Eigen::Vector3d(current_cog_target_rpy.x(), current_cog_target_rpy.y(), current_cog_target_rpy.z());
  /*
   * Use the same captured base_link pose when calculating the manual pivot.
   *
   * This removes the timing discrepancy that previously appeared in RViz.
   */
  const Eigen::Vector3d pivot_world = computePivotWorld(baselink_position_world, baselink_rotation_world);

  if(!baselink_position_world.allFinite() || !baselink_rotation_world.allFinite() || !pivot_world.allFinite())
  {
    ROS_ERROR("[GimbalrotorPerchingNavigator] " "invalid base_link or pivot state");

    return false;
  }

  /*
   * Physical geometry:
   *
   *   radius vector = pivot -> base_link
   *
   * No COG is used here.
   */
  const Eigen::Vector3d locked_radius_world = baselink_position_world - pivot_world;

  const double radius = locked_radius_world.norm();

  if(!std::isfinite(radius) || radius < minimum_valid_radius_)
  {
    ROS_ERROR("[GimbalrotorPerchingNavigator] " "invalid base_link-to-pivot radius: %.4f", radius);

    return false;
  }

  const Eigen::Matrix3d rotation_world_constraint =
      Eigen::AngleAxisd(
          constraint_frame_rpy_.z(),
          Eigen::Vector3d::UnitZ())
          .toRotationMatrix()
      *
      Eigen::AngleAxisd(
          constraint_frame_rpy_.y(),
          Eigen::Vector3d::UnitY())
          .toRotationMatrix()
      *
      Eigen::AngleAxisd(
          constraint_frame_rpy_.x(),
          Eigen::Vector3d::UnitX())
          .toRotationMatrix();

  SpatialConstraintConfig config;

  config.active = true;
  config.allowed_dof = allowed_dof_;
  config.coordinate_sign = coordinate_sign_;

  config.pivot_world = pivot_world;

  config.rotation_world_constraint = rotation_world_constraint;

  /*
   * Important:
   *
   * SpatialConstraint now rotates BASE_LINK around the perching pivot.
   *
   * Previously these two fields contained COG pose.
   */
  config.locked_position_world = baselink_position_world;
  config.locked_rotation_world = baselink_rotation_world;

  config.minimum_coordinate = -std::abs(maximum_coordinate_);
  config.maximum_coordinate = std::abs(maximum_coordinate_);

  if(!spatial_constraint_.configure(config))
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator] "
        "constraint invalid. Current version requires "
        "exactly one rotational allowed DOF.");

    return false;
  }

  /*
   * The command-angle reference is also based on base_link orientation.
   */
  tf::Matrix3x3 baselink_rotation_tf;

  tf::matrixEigenToTF(baselink_rotation_world, baselink_rotation_tf);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  baselink_rotation_tf.getRPY(roll, pitch, yaw);

  locked_rpy_ =
      Eigen::Vector3d(roll, pitch, yaw);

  active_coordinate_ = 0.0;
  perching_locked_ = true;

  if(isManualPivotSource())
  {
    const double configured_radius = hand_center_offset_baselink_.norm();

    ROS_WARN(
        "[GimbalrotorPerchingNavigator] "
        "locked by %s; rotational DOF=%d; "
        "base_link-to-pivot radius=%.6f m; "
        "configured offset norm=%.6f m; "
        "difference=%+.9f m",
        reason.c_str(),
        spatial_constraint_.getRotationalDofIndex(),
        radius,
        configured_radius,
        radius - configured_radius);
  }
  else
  {
    ROS_WARN(
        "[GimbalrotorPerchingNavigator] "
        "locked by %s; rotational DOF=%d; "
        "base_link-to-pivot radius=%.6f m",
        reason.c_str(),
        spatial_constraint_.getRotationalDofIndex(),
        radius);
  }

  publishLockedState();

  return true;
}

void GimbalrotorPerchingNavigator::clearConstraint()
{
  spatial_constraint_.clear();
  perching_locked_ = false;
  active_coordinate_ = 0.0;
}

void GimbalrotorPerchingNavigator::commandBaselinkRotationTarget(const SpatialConstraintTarget& baselink_target)
{
  if(!baselink_target.valid)
  {
    return;
  }

  tf::Matrix3x3 target_baselink_rotation_tf;

  tf::matrixEigenToTF(baselink_target.rotation_world, target_baselink_rotation_tf);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  target_baselink_rotation_tf.getRPY(roll, pitch, yaw);

  /*
   * This controls the physical base_link orientation through the existing
   * gimbalrotor desire_coordinate path.
   */
  setFinalTargetBaselinkRPY(tf::Vector3(roll, pitch, yaw));
}

void GimbalrotorPerchingNavigator::applyActiveConstraintTarget()
{
  if(!isPerchingLocked())
  {
    return;
  }

  /*
   * Geometry is calculated entirely as:
   *
   *   base_link rotating around the fixed pivot.
   */
  const SpatialConstraintTarget baselink_target = spatial_constraint_.calculateTarget(active_coordinate_);

  if(!baselink_target.valid)
  {
    return;
  }

  /*
   * Command the physical base_link orientation.
   *
   * This was missing when using perching/target_angle_deg or
   * perching/add_angle_deg.
   */
  commandBaselinkRotationTarget(baselink_target);

  /*
   * The existing translational pose PID tracks COG, not base_link.
   *
   * Convert only after the base_link arc has been calculated.
   */
  const SpatialConstraintTarget cog_target = convertBaselinkTargetToCogTarget(baselink_target);

  if(!cog_target.valid)
  {
    return;
  }

  tf::Vector3 target_cog_position;
  tf::Vector3 target_cog_velocity;
  tf::Vector3 target_cog_acceleration;

  tf::vectorEigenToTF(cog_target.position_world, target_cog_position);
  tf::vectorEigenToTF(cog_target.linear_velocity_world, target_cog_velocity);
  tf::vectorEigenToTF(cog_target.linear_acceleration_world, target_cog_acceleration);

  setXyControlMode(POS_CONTROL_MODE);

  /*
   * Translational target:
   * COG target corresponding to the constrained base_link pose.
   */
  setTargetPos(target_cog_position);
  setTargetVel(target_cog_velocity);
  setTargetAcc(target_cog_acceleration);

  /*
   * Important:
   *
   * Do not put the physical base_link pitch into the COG attitude PID.
   *
   * The base_link orientation is handled through
   * setFinalTargetBaselinkRPY().
   *
   * The COG attitude target remains at the value captured during lock.
   */
  setTargetRPY(tf::Vector3(locked_cog_target_rpy_.x(), locked_cog_target_rpy_.y(), locked_cog_target_rpy_.z()));

  setTargetZeroOmega();
  setTargetZeroAngAcc();

  /*
   * RViz debug output represents the constrained base_link pose.
   */
  publishCommandedState(
      baselink_target);
}


bool GimbalrotorPerchingNavigator::hasConstraintAngleCommand(const aerial_robot_msgs::FlightNav& msg) const
{
  switch(spatial_constraint_.getRotationalDofIndex())
  {
    case 0:
      return msg.roll_nav_mode == NAV_MODE_POS;
    case 1:
      return msg.pitch_nav_mode == NAV_MODE_POS;
    case 2:
      return msg.yaw_nav_mode == NAV_MODE_POS;
    default:
      return false;
  }
}

double GimbalrotorPerchingNavigator::getConstraintAngleCommand(const aerial_robot_msgs::FlightNav& msg) const
{
  switch(spatial_constraint_.getRotationalDofIndex())
  {
    case 0:
      return msg.target_roll;
    case 1:
      return msg.target_pitch;
    case 2:
      return msg.target_yaw;
    default:
      return 0.0;
  }
}

void GimbalrotorPerchingNavigator::setActiveCoordinateFromAngle(double angle_rad)
{
  double coordinate = 0.0;

  if(command_angle_as_delta_)
  {
    coordinate = command_sign_ * angle_rad;
  }
  else
  {
    coordinate = command_sign_ * normalizeAngle(angle_rad - lockedEulerComponent());
  }

  active_coordinate_ = spatial_constraint_.clampCoordinate(coordinate);
}

void GimbalrotorPerchingNavigator::setActiveCoordinateFromAbsoluteBaselinkAngle(double absolute_angle_rad)
{
  const double coordinate = command_sign_ * normalizeAngle(absolute_angle_rad - lockedEulerComponent());

  active_coordinate_ = spatial_constraint_.clampCoordinate(coordinate);
}

double GimbalrotorPerchingNavigator::getConstraintAngleFromBaselinkRpy(const tf::Vector3& baselink_rpy) const
{
  switch(spatial_constraint_.getRotationalDofIndex())
  {
    case 0:
      return baselink_rpy.x();

    case 1:
      return baselink_rpy.y();

    case 2:
      return baselink_rpy.z();

    default:
      return 0.0;
  }
}

void GimbalrotorPerchingNavigator::addActiveCoordinate(double delta_rad)
{
  active_coordinate_ = spatial_constraint_.clampCoordinate(active_coordinate_ + command_sign_ * delta_rad);
}

Eigen::Vector3d
GimbalrotorPerchingNavigator::getCurrentCogPositionWorld() const
{
  Eigen::Vector3d position;
  tf::vectorTFToEigen(estimator_->getPos(Frame::COG, estimate_mode_), position);
  return position;
}

Eigen::Vector3d
GimbalrotorPerchingNavigator::getCurrentBaselinkPositionWorld() const
{
  Eigen::Vector3d position;
  tf::vectorTFToEigen(estimator_->getPos(Frame::BASELINK, estimate_mode_), position);
  return position;
}

Eigen::Matrix3d
GimbalrotorPerchingNavigator::getCurrentBaselinkRotationWorld() const
{
  Eigen::Matrix3d rotation;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::BASELINK, estimate_mode_), rotation);
  return rotation;
}

Eigen::Vector3d
GimbalrotorPerchingNavigator::computePivotWorld(
    const Eigen::Vector3d& baselink_position_world,
    const Eigen::Matrix3d& baselink_rotation_world) const
{
  if(isManualPivotSource())
  {
    /*
     * Exact physical geometry:
     *
     *   pivot_world =
     *       base_link_position_world
     *       + R_world_base_link
     *       * offset_base_link_to_pivot
     */
    return baselink_position_world
        +
        baselink_rotation_world
        * hand_center_offset_baselink_;
  }

  if(has_perching_point_)
  {
    return perching_point_world_;
  }

  return branch_position_world_;
}

bool GimbalrotorPerchingNavigator::isManualPivotSource() const
{
  return pivot_source_ == "manual" || pivot_source_ == "hand_center";
}

bool GimbalrotorPerchingNavigator::isBranchPivotSource() const
{
  return pivot_source_ == "branch" || pivot_source_ == "perching_point";
}

bool GimbalrotorPerchingNavigator::hasBranchPivot() const
{
  return has_perching_point_ || has_branch_pose_;
}

bool GimbalrotorPerchingNavigator::loadVector6Param(const ros::NodeHandle& nh, const std::string& name, Vector6d& value)
{
  std::vector<double> data;
  if(!nh.getParam(name, data))
  {
    return false;
  }

  if(data.size() != 6)
  {
    ROS_ERROR("[GimbalrotorPerchingNavigator] %s must contain 6 values", name.c_str());
    return false;
  }

  for(int i = 0; i < 6; ++i)
  {
    value(i) = data.at(i);
  }
  return true;
}

bool GimbalrotorPerchingNavigator::loadVector3Param(const ros::NodeHandle& nh, const std::string& name, Eigen::Vector3d& value)
{
  std::vector<double> data;
  if(!nh.getParam(name, data))
  {
    return false;
  }

  if(data.size() != 3)
  {
    ROS_ERROR("[GimbalrotorPerchingNavigator] %s must contain 3 values", name.c_str());
    return false;
  }

  value = Eigen::Vector3d(data.at(0), data.at(1), data.at(2));
  return true;
}

double GimbalrotorPerchingNavigator::lockedEulerComponent() const
{
  const int index = spatial_constraint_.getRotationalDofIndex();
  return index >= 0 ? locked_rpy_(index) : 0.0;
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

void GimbalrotorPerchingNavigator::publishLockedState()
{
  if(!isPerchingLocked())
  {
    return;
  }

  const auto& config = spatial_constraint_.getConfig();

  geometry_msgs::PointStamped pivot_msg;
  pivot_msg.header.stamp = ros::Time::now();
  pivot_msg.header.frame_id = "world";
  pivot_msg.point.x = config.pivot_world.x();
  pivot_msg.point.y = config.pivot_world.y();
  pivot_msg.point.z = config.pivot_world.z();
  locked_pivot_pub_.publish(pivot_msg);

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = pivot_msg.header;
  pose_msg.pose.position.x = config.locked_position_world.x();
  pose_msg.pose.position.y = config.locked_position_world.y();
  pose_msg.pose.position.z = config.locked_position_world.z();

  tf::Matrix3x3 rotation_tf;
  tf::matrixEigenToTF(config.locked_rotation_world, rotation_tf);
  tf::Quaternion quaternion;
  rotation_tf.getRotation(quaternion);
  tf::quaternionTFToMsg(quaternion, pose_msg.pose.orientation);
  locked_pose_pub_.publish(pose_msg);
}

void GimbalrotorPerchingNavigator::publishCommandedState(const SpatialConstraintTarget& target)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "world";
  pose_msg.pose.position.x = target.position_world.x();
  pose_msg.pose.position.y = target.position_world.y();
  pose_msg.pose.position.z = target.position_world.z();

  tf::Matrix3x3 rotation_tf;
  tf::matrixEigenToTF(target.rotation_world, rotation_tf);
  tf::Quaternion quaternion;
  rotation_tf.getRotation(quaternion);
  tf::quaternionTFToMsg(quaternion, pose_msg.pose.orientation);

  commanded_pose_pub_.publish(pose_msg);
}

}  // namespace aerial_robot_navigation

PLUGINLIB_EXPORT_CLASS(
    aerial_robot_navigation::GimbalrotorPerchingNavigator,
    aerial_robot_navigation::BaseNavigator)
