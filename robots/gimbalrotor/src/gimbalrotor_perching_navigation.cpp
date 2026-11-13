#include <gimbalrotor/gimbalrotor_perching_navigation.h>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <iomanip>
#include <aerial_robot_model/model/aerial_robot_model.h>
#include <kdl/chain.hpp>
#include <urdf/model.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <kdl/treefksolverpos_recursive.hpp>
#include <tf_conversions/tf_eigen.h>
#include <stdexcept>

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

  perching_mode_(perching_geometry::Mode::DISABLED),
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
  perching_slanted_enable_topic_("perching/slanted_enable"),
  branch_pose_topic_("perching/branch_pose"),
  perching_point_topic_("perching/point"),
  locked_pivot_topic_("perching/locked_pivot"),
  relock_topic_("perching/relock"),
  reset_topic_("perching/reset"),
  manual_pitch_delta_topic_("perching/manual_pitch_delta"),

  has_branch_pose_(false),
  has_perching_point_(false),

  active_pitch_delta_(0.0),
  locked_radius_(0.0),

  joint_command_topic_("joints_ctrl"),
  joint_state_topic_("joint_states"),
  secondary_joint_target_topic_(
      "perching/multilink/secondary_joint_target"),
  secondary_position_tolerance_(0.02),
  secondary_velocity_tolerance_(0.05),
  secondary_settle_duration_(0.30),
  joint_state_timeout_(0.15),
  joint_state_future_tolerance_(0.02),
  multilink_pitch_axis_alignment_threshold_(0.90),
  secondary_axis_alignment_threshold_(0.90),
  multilink_pitch_command_sign_(1.0),
  secondary_command_sign_(1.0),
  configured_pitch_limits_valid_(false),
  configured_secondary_limits_valid_(false),
  configured_pitch_lower_(0.0),
  configured_pitch_upper_(0.0),
  configured_secondary_lower_(0.0),
  configured_secondary_upper_(0.0),
  pitch_joint_index_(0),
  secondary_joint_index_(0),
  pitch_joint_lower_(0.0),
  pitch_joint_upper_(0.0),
  secondary_joint_lower_(0.0),
  secondary_joint_upper_(0.0),
  pitch_axis_local_(Eigen::Vector3d::Zero()),
  secondary_axis_local_(Eigen::Vector3d::Zero()),
  secondary_axis_type_(SECONDARY_AXIS_INVALID),
  multilink_model_resolution_attempted_(false),
  multilink_model_valid_(false),
  multilink_lock_valid_(false),
  locked_contact_world_(KDL::Frame::Identity()),
  locked_pitch_joint_(0.0),
  locked_secondary_joint_(0.0),
  pitch_joint_nominal_target_(0.0),
  secondary_joint_nominal_target_(0.0),
  pitch_joint_final_target_(0.0),
  secondary_joint_final_target_(0.0),
  joint_state_received_(false),
  joint_state_measurement_stamp_(0),
  joint_state_receive_stamp_(0),
  measured_pitch_joint_(0.0),
  measured_secondary_joint_(0.0),
  measured_pitch_velocity_(0.0),
  measured_secondary_velocity_(0.0),
  previous_measured_pitch_joint_(0.0),
  previous_measured_secondary_joint_(0.0),
  secondary_settled_(false),
  mechanism_target_generation_(0)
{
  resetMultilinkPitchRezeroState();

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
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  std::string configuration;
  nh.param<std::string>("navigation/perching_configuration", configuration, "baseline");
  if(configuration == "baseline")
    perching_configuration_ = PerchingConfiguration::BASELINE;
  else if(configuration == "pitch_only")
    perching_configuration_ = PerchingConfiguration::PITCH_ONLY;
  else if(configuration == "multilink")
    perching_configuration_ = PerchingConfiguration::MULTILINK;
  else if(configuration == "roll_yaw")
    perching_configuration_ = PerchingConfiguration::ROLL_YAW;
  else
  {
    ROS_FATAL_STREAM("Unknown navigation/perching_configuration: " << configuration
                     << "; expected baseline, pitch_only, multilink or roll_yaw");
    throw std::invalid_argument("Unknown navigation/perching_configuration: " + configuration);
  }

  GimbalrotorNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  // The base assigns robot_model_ only after its virtual rosParamInit() returns.
  // Initialize the selected mechanism before registering common contact callbacks.
  switch(perching_configuration_)
  {
    case PerchingConfiguration::BASELINE:
      break;
    case PerchingConfiguration::PITCH_ONLY:
      initializePitchOnly(nh, nhp);
      break;
    case PerchingConfiguration::MULTILINK:
      initializeMultilink(nh, nhp);
      break;
    case PerchingConfiguration::ROLL_YAW:
      initializeRollYaw(nh, nhp);
      break;
  }

  perching_enable_sub_ = nh_.subscribe(perching_enable_topic_, 1, &GimbalrotorPerchingNavigator::perchingEnableCallback, this);
  perching_slanted_enable_sub_ = nh_.subscribe(perching_slanted_enable_topic_, 1, &GimbalrotorPerchingNavigator::perchingSlantedEnableCallback, this);
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
  ROS_WARN("[GimbalrotorPerchingNavigator] NORMAL enable topic: %s", perching_enable_topic_.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] SLANTED enable topic: %s", perching_slanted_enable_topic_.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] branch pose topic: %s", branch_pose_topic_.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] perching point topic: %s", perching_point_topic_.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] manual pitch delta topic: %s", manual_pitch_delta_topic_.c_str());
}

void GimbalrotorPerchingNavigator::rosParamInit()
{
  GimbalrotorNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  bool perching_enable = false;
  getParam<bool>(navi_nh, "perching_enable", perching_enable, false);
  perching_mode_ = perching_enable ? perching_geometry::Mode::NORMAL : perching_geometry::Mode::DISABLED;
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
  getParam<std::string>(navi_nh, "perching_slanted_enable_topic", perching_slanted_enable_topic_, "perching/slanted_enable");
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

void GimbalrotorPerchingNavigator::updateCommonPerching()
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(getNaviState() != HOVER_STATE && perching_mode_ != perching_geometry::Mode::DISABLED)
  {
    perching_mode_ = perching_geometry::Mode::DISABLED;
    resetPerchingLock();

    ROS_WARN(
        "[GimbalrotorPerchingNavigator] "
        "Perching navigation automatically deactivated because "
        "the flight state is no longer HOVER_STATE. "
        "Publish the desired perching mode enable=true after returning to "
        "HOVER_STATE to create a fresh perching lock.");
  }

  GimbalrotorNavigator::update();

  if(getNaviState() != HOVER_STATE)
  {
    if(perching_mode_ != perching_geometry::Mode::DISABLED)
    {
      perching_mode_ = perching_geometry::Mode::DISABLED;
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

  if(perching_mode_ != perching_geometry::Mode::DISABLED && active_perching_hold_enable_)
  {
    applyActivePerchingTarget();
  }
}

void GimbalrotorPerchingNavigator::perchingEnableCallback(const std_msgs::BoolConstPtr& msg)
{
  selectPerchingMode(perching_geometry::Mode::NORMAL, msg->data);
}

void GimbalrotorPerchingNavigator::perchingSlantedEnableCallback(const std_msgs::BoolConstPtr& msg)
{
  selectPerchingMode(perching_geometry::Mode::SLANTED, msg->data);
}

void GimbalrotorPerchingNavigator::selectCommonPerchingMode(perching_geometry::Mode mode, bool enable)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(enable && !supportsPerchingMode(mode))
  {
    ROS_WARN_THROTTLE(1.0, "Multilink perching supports NORMAL mode only; slanted_enable was ignored.");
    return;
  }
  if(!enable && perching_mode_ != mode) return;

  resetPerchingLock();
  perching_mode_ = perching_geometry::Mode::DISABLED;
  if(!enable) return;

  if(getNaviState() != HOVER_STATE)
  {
    ROS_WARN("[GimbalrotorPerchingNavigator] perching enable rejected outside HOVER_STATE");
    return;
  }

  // Every explicit true selects a fresh lock, including reselecting this mode.
  perching_mode_ = mode;
  if(!tryLockPerching("mode selection"))
  {
    perching_mode_ = perching_geometry::Mode::DISABLED;
    resetPerchingLock();
    ROS_ERROR("[GimbalrotorPerchingNavigator] perching enable failed: invalid lock");
  }
}

void GimbalrotorPerchingNavigator::branchPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  branch_pos_world_.setValue(msg->pose.position.x,
                             msg->pose.position.y,
                             msg->pose.position.z);

  has_branch_pose_ = true;
}

void GimbalrotorPerchingNavigator::perchingPointCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  perching_point_world_.setValue(msg->point.x,
                                 msg->point.y,
                                 msg->point.z);

  has_perching_point_ = true;
}

void GimbalrotorPerchingNavigator::relockCommonCallback(const std_msgs::EmptyConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  (void)msg;

  if(perching_mode_ == perching_geometry::Mode::DISABLED)
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

  resetPerchingLock();

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
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  (void)msg;
  resetPerchingLock();
}

void GimbalrotorPerchingNavigator::resetCommonPerchingLock()
{
  perching_locked_ = false;
  locked_radius_ = 0.0;
  active_pitch_delta_ = 0.0;
  lock_stamp_ = ros::Time(0);
  geometry_ = perching_geometry::Lock();
  reference_locked_rpy_.setValue(0, 0, 0);
  locked_robot_pos_world_.setValue(0, 0, 0);
  locked_pivot_world_.setValue(0, 0, 0);
  ROS_WARN("[GimbalrotorPerchingNavigator] perching lock reset");
}

void GimbalrotorPerchingNavigator::manualPitchDeltaCallback(const std_msgs::Float64ConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(perching_mode_ == perching_geometry::Mode::DISABLED)
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

  if(!std::isfinite(msg->data)) return;
  applyManualPitchDelta(msg->data);
}

void GimbalrotorPerchingNavigator::applyBaselinePitchDelta(double delta)
{
  perching_geometry::Pose pose;
  if(!geometry_.target(command_pitch_sign_ * delta, max_pitch_delta_, arc_pitch_sign_, pose))
    return;
  active_pitch_delta_ = pose.delta;
  applyAxialCompliance(pose);
  aerial_robot_msgs::FlightNav command;
  setPoseCommand(command, pose);
  aerial_robot_msgs::FlightNavConstPtr command_ptr(new aerial_robot_msgs::FlightNav(command));
  GimbalrotorNavigator::naviCallback(command_ptr);
}

bool GimbalrotorPerchingNavigator::tryLockBaseline(const std::string& reason)
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
  tf::Vector3 measured_rpy = getCurrentRobotRPY();
  // NORMAL preserves roll/yaw intent; measured tracking error is not a target.
  tf::Vector3 reference_rpy(existing_target_rpy.x(), measured_rpy.y(), existing_target_rpy.z());
  tf::Quaternion reference_orientation = tf::createQuaternionFromRPY(
      reference_rpy.x(), reference_rpy.y(), reference_rpy.z());
  if(perching_mode_ == perching_geometry::Mode::SLANTED)
  {
    estimator_->getOrientation(Frame::COG, estimate_mode_).getRotation(reference_orientation);
    // Lock::initialize validates and normalizes the measured quaternion.
  }
  if(!geometry_.initialize(locked_robot_pos_world_, locked_pivot_world_, reference_orientation, min_valid_radius_))
  {
    ROS_WARN("[GimbalrotorPerchingNavigator] rejected invalid perching geometry");
    return false;
  }
  if(perching_mode_ == perching_geometry::Mode::SLANTED)
  {
    double roll, pitch, yaw;
    tf::Matrix3x3(geometry_.orientation).getRPY(roll, pitch, yaw);
    reference_rpy.setValue(roll, pitch, yaw);
    measured_rpy = reference_rpy;  // Log the same measured snapshot used by the lock.
  }
  reference_locked_rpy_ = reference_rpy;
  locked_radius_ = geometry_.radial().length();
  active_pitch_delta_ = 0.0;
  commitPerchingLockIdentity();

  ROS_WARN("[GimbalrotorPerchingNavigator] perching mode: %s, locked by %s",
           perching_mode_ == perching_geometry::Mode::NORMAL ? "NORMAL" : "SLANTED", reason.c_str());
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
  ROS_WARN("[GimbalrotorPerchingNavigator] reference rpy deg: roll %.2f, pitch %.2f, yaw %.2f",
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

void GimbalrotorPerchingNavigator::commitPerchingLockIdentity()
{
  lock_stamp_ = ros::Time::now();
  if(lock_stamp_ <= last_lock_stamp_) lock_stamp_ = last_lock_stamp_ + ros::Duration(0, 1);
  last_lock_stamp_ = lock_stamp_;

  perching_locked_ = true;
}

void GimbalrotorPerchingNavigator::commitFixedContactLock(
    const tf::Vector3& cog_position, const tf::Quaternion& cog_orientation,
    const tf::Vector3& contact_position)
{
  // A fixed-contact lock shares the session identity, never the circular-arc geometry.
  commitPerchingLockIdentity();
  locked_pivot_world_ = contact_position;
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = lock_stamp_;
  pose.header.frame_id = "world";
  tf::pointTFToMsg(cog_position, pose.pose.position);
  tf::quaternionTFToMsg(cog_orientation, pose.pose.orientation);
  locked_pose_pub_.publish(pose);
  publishLockedPivot();
}

void GimbalrotorPerchingNavigator::baselineNaviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  aerial_robot_msgs::FlightNav nav_msg = *msg;

  if(perching_mode_ != perching_geometry::Mode::DISABLED && getNaviState() == HOVER_STATE)
  {
    if(!applyPerchingConstraint(nav_msg)) return;
  }

  aerial_robot_msgs::FlightNavConstPtr nav_msg_ptr(new aerial_robot_msgs::FlightNav(nav_msg));

  GimbalrotorNavigator::naviCallback(nav_msg_ptr);
}

void GimbalrotorPerchingNavigator::applyBaselineTarget()
{
  if(perching_mode_ == perching_geometry::Mode::DISABLED || getNaviState() != HOVER_STATE)
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

bool GimbalrotorPerchingNavigator::applyBaselineConstraint(aerial_robot_msgs::FlightNav& msg)
{
  if(perching_mode_ == perching_geometry::Mode::DISABLED || getNaviState() != HOVER_STATE) return false;
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
      if(!geometry_.targetOrientation(desired, max_pitch_delta_, arc_pitch_sign_, pose)) return false;
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
  tf::Quaternion commanded_orientation = pose.orientation;
  if(perching_mode_ == perching_geometry::Mode::NORMAL)
  {
    double roll, pitch, yaw;
    tf::Matrix3x3(pose.orientation).getRPY(roll, pitch, yaw);
    commanded_orientation = tf::createQuaternionFromRPY(
        reference_locked_rpy_.x(), pitch, reference_locked_rpy_.z());
  }
  tf::quaternionTFToMsg(commanded_orientation, msg.pose.orientation);
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
  // SLANTED commands the complete revolute orientation; its Euler yaw can change.
  msg.target_roll = perching_mode_ == perching_geometry::Mode::NORMAL ? reference_locked_rpy_.x() : roll;
  msg.target_pitch = pitch;
  msg.target_yaw = perching_mode_ == perching_geometry::Mode::NORMAL ? reference_locked_rpy_.z() : yaw;
  publishCommandedDebugPose(pose);
}

perching_geometry::Session GimbalrotorPerchingNavigator::perchingSession() const
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  return {perching_mode_, lock_stamp_};
}

bool GimbalrotorPerchingNavigator::perchingAdmittanceTarget(
    const ros::Time& lock_stamp, double physical_offset,
    const tf::Vector3& nominal_position, perching_geometry::Pose& pose) const
{
  if(perching_configuration_ == PerchingConfiguration::PITCH_ONLY)
    return false;
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(perching_mode_ == perching_geometry::Mode::DISABLED || !perching_locked_ || navi_state_ != HOVER_STATE ||
     lock_stamp != lock_stamp_ || !perching_geometry::finite(nominal_position)) return false;
  // Compliance Y is a physical axis angle. Convert to the logical coordinate
  // before the shared helper applies arc_pitch_sign exactly once.
  if(!geometry_.target(active_pitch_delta_ + physical_offset / arc_pitch_sign_,
                       max_pitch_delta_, arc_pitch_sign_, pose)) return false;
  pose.position += geometry_.axis * geometry_.axis.dot(nominal_position - geometry_.position);
  return true;
}

// Configuration dispatch

GimbalrotorPerchingNavigator::~GimbalrotorPerchingNavigator()
{
  // Join the planning worker while all navigator state is still alive.
  prepare_worker_.reset();
}

bool GimbalrotorPerchingNavigator::supportsPerchingMode(perching_geometry::Mode mode) const
{
  return mode == perching_geometry::Mode::NORMAL ||
         (perching_configuration_ == PerchingConfiguration::BASELINE && mode == perching_geometry::Mode::SLANTED);
}

bool GimbalrotorPerchingNavigator::ownsNavigationTargets() const
{
  return perching_configuration_ == PerchingConfiguration::ROLL_YAW && ownsRollYawTargets();
}

std::unique_lock<std::recursive_mutex> GimbalrotorPerchingNavigator::lockNavigationTargets()
{
  if(perching_configuration_ == PerchingConfiguration::ROLL_YAW)
    return std::unique_lock<std::recursive_mutex>(perchingStateMutex());
  return {};
}

void GimbalrotorPerchingNavigator::navigationTargetConflict(const char* source)
{
  if(perching_configuration_ == PerchingConfiguration::ROLL_YAW)
    rollYawTargetConflict(source);
}

void GimbalrotorPerchingNavigator::selectPerchingMode(perching_geometry::Mode mode, bool enable)
{
  if(perching_configuration_ == PerchingConfiguration::PITCH_ONLY)
    selectPitchOnlyMode(mode, enable);
  else
    selectCommonPerchingMode(mode, enable);
}

void GimbalrotorPerchingNavigator::relockCallback(const std_msgs::EmptyConstPtr& msg)
{
  if(perching_configuration_ == PerchingConfiguration::PITCH_ONLY)
    relockPitchOnlyCallback(msg);
  else
    relockCommonCallback(msg);
}

void GimbalrotorPerchingNavigator::update()
{
  switch(perching_configuration_)
  {
    case PerchingConfiguration::BASELINE:
      return updateCommonPerching();
    case PerchingConfiguration::PITCH_ONLY:
      return updatePitchOnly();
    case PerchingConfiguration::MULTILINK:
      return updateMultilink();
    case PerchingConfiguration::ROLL_YAW:
      return updateRollYaw();
  }
}

bool GimbalrotorPerchingNavigator::tryLockPerching(const std::string& reason)
{
  switch(perching_configuration_)
  {
    case PerchingConfiguration::BASELINE:
      return tryLockBaseline(reason);
    case PerchingConfiguration::PITCH_ONLY:
      return tryLockPitchOnly(reason);
    case PerchingConfiguration::MULTILINK:
      return tryLockMultilink(reason);
    case PerchingConfiguration::ROLL_YAW:
      return tryLockRollYaw(reason);
  }
  return false;
}

void GimbalrotorPerchingNavigator::resetPerchingLock()
{
  switch(perching_configuration_)
  {
    case PerchingConfiguration::BASELINE:
      return resetCommonPerchingLock();
    case PerchingConfiguration::PITCH_ONLY:
      return resetPitchOnlyLock();
    case PerchingConfiguration::MULTILINK:
      return resetMultilinkLock();
    case PerchingConfiguration::ROLL_YAW:
      return resetRollYawLock();
  }
}

void GimbalrotorPerchingNavigator::applyActivePerchingTarget()
{
  switch(perching_configuration_)
  {
    case PerchingConfiguration::BASELINE:
      return applyBaselineTarget();
    case PerchingConfiguration::PITCH_ONLY:
      return applyPitchOnlyTarget();
    case PerchingConfiguration::MULTILINK:
      return applyMultilinkTarget();
    case PerchingConfiguration::ROLL_YAW:
      return applyRollYawTarget();
  }
}

void GimbalrotorPerchingNavigator::applyManualPitchDelta(double delta)
{
  switch(perching_configuration_)
  {
    case PerchingConfiguration::BASELINE:
      return applyBaselinePitchDelta(delta);
    case PerchingConfiguration::PITCH_ONLY:
      return applyPitchOnlyDelta(delta);
    case PerchingConfiguration::MULTILINK:
      return applyMultilinkPitchDelta(delta);
    case PerchingConfiguration::ROLL_YAW:
      return applyRollYawPitchDelta(delta);
  }
}

void GimbalrotorPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  switch(perching_configuration_)
  {
    case PerchingConfiguration::BASELINE:
      return baselineNaviCallback(msg);
    case PerchingConfiguration::PITCH_ONLY:
      return pitchOnlyNaviCallback(msg);
    case PerchingConfiguration::MULTILINK:
      return baselineNaviCallback(msg);
    case PerchingConfiguration::ROLL_YAW:
      return rollYawNaviCallback(msg);
  }
}

bool GimbalrotorPerchingNavigator::applyPerchingConstraint(aerial_robot_msgs::FlightNav& msg)
{
  switch(perching_configuration_)
  {
    case PerchingConfiguration::BASELINE:
      return applyBaselineConstraint(msg);
    case PerchingConfiguration::PITCH_ONLY:
      return !core_.ownsPose();
    case PerchingConfiguration::MULTILINK:
      return applyMultilinkConstraint(msg);
    case PerchingConfiguration::ROLL_YAW:
      return false;
  }
  return false;
}


// Pitch-only workflow

void GimbalrotorPerchingNavigator::initializePitchOnly(ros::NodeHandle nh, ros::NodeHandle nhp)
{
  ros::NodeHandle pn(nh, "navigation/pitch_only_perching");
  pn.param<std::string>("pitch_joint_name", joint_name_, "joint_pitch_only");
  pn.param<std::string>("contact_link_name", contact_link_, "pitch_joint_only_hand_link");
  pn.param<std::string>("world_frame", world_frame_, "world");
  pn.param<std::string>("topic_prefix", pitch_only_prefix_, "perching/pitch_only");
  pn.param("pitch_command_sign", pitch_only_pitch_command_sign_, 1.0);
  pitch_only_pitch_command_sign_ = std::isfinite(pitch_only_pitch_command_sign_) && pitch_only_pitch_command_sign_ < 0 ? -1 : 1;
  pn.param("pitch_axis_alignment_threshold", pitch_only_pitch_axis_alignment_threshold_, .90);
  pn.getParam("pitch_lower_limit", configured_lower_);
  pn.getParam("pitch_upper_limit", configured_upper_);
  pn.getParam("pitch_rate_limit", configured_rate_);
  auto& c = core_.config;
  pn.param("body_pitch_lower_limit", c.body_pitch_min, c.body_pitch_min);
  pn.param("body_pitch_upper_limit", c.body_pitch_max, c.body_pitch_max);
  pn.param("joint_state_timeout", c.feedback_timeout, c.feedback_timeout);
  pn.param("joint_state_future_tolerance", c.future_tolerance, c.future_tolerance);
  pn.param("pitch_zero_tolerance", c.q_zero_tolerance, c.q_zero_tolerance);
  pn.param("pitch_position_tolerance", c.q_hold_error, c.q_hold_error);
  pn.param("pitch_velocity_tolerance", c.joint_settle_rate, c.joint_settle_rate);
  pn.param("pitch_rezero_rate_tolerance", c.rezero_rate_tolerance, c.rezero_rate_tolerance);
  pn.param("pitch_rezero_stable_duration", c.settle_duration, c.settle_duration);
  pn.param("pitch_rezero_target_pitch", c.rezero_pitch, c.rezero_pitch);
  pn.param("pitch_rezero_command_sign", c.rezero_sign, c.rezero_sign);
  c.rezero_sign = std::isfinite(c.rezero_sign) && c.rezero_sign < 0 ? -1 : 1;
  pn.param("pitch_rezero_kp", c.rezero_kp, c.rezero_kp);
  pn.param("pitch_rezero_rate_limit", c.rezero_rate, c.rezero_rate);
  pn.param("pitch_rezero_max_delta", c.rezero_range, c.rezero_range);
  pn.param("pitch_rezero_pitch_tolerance", c.rezero_pitch_tolerance, c.rezero_pitch_tolerance);
  pn.param("pitch_rezero_timeout", c.rezero_timeout, c.rezero_timeout);
  pn.param("passive_lower_limit", c.passive_min, c.passive_min);
  pn.param("passive_upper_limit", c.passive_max, c.passive_max);
  pn.param("passive_rate_limit", c.passive_rate, c.passive_rate);
  auto vector = [&pn](const std::string& key, KDL::Vector& v)
  {
    if(!pn.hasParam(key))
      return;
    std::vector<double> a;
    if(pn.getParam(key, a) && a.size() == 3)
      v = KDL::Vector(a[0], a[1], a[2]);
    else
      v = KDL::Vector(pitch_only::unknown, pitch_only::unknown, pitch_only::unknown);
  };
  vector("pivot_offset", c.pivot_offset);
  vector("hinge_axis", c.hinge_axis);
  if(pitch_only::finite(c.hinge_axis) && c.hinge_axis.Norm() > 1e-6)
    c.hinge_axis = c.hinge_axis / c.hinge_axis.Norm();
  pitch_only_model_resolution_attempted_ = robot_model_ && robot_model_->initialized();
  if(pitch_only_model_resolution_attempted_)
    model_valid_ = resolveModel();
  core_.model = [this](double q, KDL::Frame& bc, KDL::Vector& g) { return modelAt(q, bc, g); };
  std::string command_topic, joint_topic;
  pn.param<std::string>("joint_command_topic", command_topic, "joints_ctrl");
  pn.param<std::string>("joint_state_topic", joint_topic, "joint_states");
  pitch_only_joints_pub_ = nh.advertise<sensor_msgs::JointState>(command_topic, 1);
  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pitch_only_prefix_ + "/target_body_pose", 1);
  pivot_pub_ = nh.advertise<geometry_msgs::PointStamped>(pitch_only_prefix_ + "/locked_pivot", 1);
  pitch_only_state_pub_ = nh.advertise<std_msgs::String>(pitch_only_prefix_ + "/state", 1);
  for(const auto* key : {"model_valid", "contact_valid", "rezero_active", "rezero_ready", "rezero_failed",
                         "servo_frozen", "cutting_enabled", "feedback_valid"})
    bool_pubs_[key] = nh.advertise<std_msgs::Bool>(pitch_only_prefix_ + "/" + key, 1);
  for(const auto* key : {"servo_measured", "servo_target_nominal", "servo_hold_error", "body_pitch", "body_pitch_rate",
                         "passive_alpha_cmd", "passive_alpha_measured", "cut_alpha_min", "cut_alpha_max", "pivot_error",
                         "lock_generation"})
    number_pubs_[key] = nh.advertise<std_msgs::Float64>(pitch_only_prefix_ + "/" + key, 1);
  subscribers_.push_back(nh.subscribe(joint_topic, 1, &GimbalrotorPerchingNavigator::pitchOnlyJointCallback, this));
  for(const auto* key :
      {"confirm_detached_zero", "pitch_rezero", "freeze", "unfreeze", "prepare_unperch", "confirm_released", "recover"})
  {
    const std::string action(key);
    subscribers_.push_back(nh.subscribe<std_msgs::Empty>(
        pitch_only_prefix_ + "/" + action, 1, [this, action](const std_msgs::EmptyConstPtr&) { requestPitchOnly(action); }));
  }
  subscribers_.push_back(nh.subscribe<std_msgs::Float64>(pitch_only_prefix_ + "/servo_command", 1,
                                                         [this](const std_msgs::Float64ConstPtr& m)
                                                         {
                                                           std::lock_guard<std::recursive_mutex> l(
                                                               perchingStateMutex());
                                                           core_.servo(pitch_only_pitch_command_sign_ * m->data, sample());
                                                         }));
  if(pitch_only_model_resolution_attempted_ && !model_valid_)
    ROS_ERROR("[PitchOnly] Invalid mechanism/parameters; check joint limits, pivot axis and positive rates/timeouts.");
}

bool GimbalrotorPerchingNavigator::resolveModel()
{
  if(!robot_model_ || !robot_model_->initialized() || contact_link_.empty())
    return false;
  const auto j = robot_model_->getUrdfModel().getJoint(joint_name_);
  if(!j || (j->type != urdf::Joint::REVOLUTE && j->type != urdf::Joint::CONTINUOUS) || j->mimic)
    return false;
  KDL::Vector a(j->axis.x, j->axis.y, j->axis.z);
  if(!pitch_only::finite(a) || a.Norm() < 1e-6 || !std::isfinite(pitch_only_pitch_axis_alignment_threshold_) ||
     pitch_only_pitch_axis_alignment_threshold_ <= 0 || pitch_only_pitch_axis_alignment_threshold_ > 1)
    return false;
  a = a / a.Norm();
  if(std::abs(a.y()) < pitch_only_pitch_axis_alignment_threshold_)
    return false;
  const auto& indices = robot_model_->getJointIndexMap();
  const auto it = indices.find(joint_name_);
  if(it == indices.end() || it->second >= robot_model_->getJointPositions().rows())
    return false;
  joint_index_ = it->second;
  KDL::Chain chain;
  if(!robot_model_->getTree().getChain(robot_model_->getBaselinkName(), contact_link_, chain) ||
     chain.getNrOfJoints() != 1)
    return false;
  for(unsigned i = 0; i < chain.getNrOfSegments(); ++i)
  {
    const auto& k = chain.getSegment(i).getJoint();
    if(k.getType() != KDL::Joint::None && k.getName() != joint_name_)
      return false;
  }
  auto& c = core_.config;
  c.q_min = configured_lower_;
  c.q_max = configured_upper_;
  if(j->type == urdf::Joint::REVOLUTE && j->limits)
  {
    c.q_min = std::isnan(configured_lower_) ? j->limits->lower : std::max(j->limits->lower, configured_lower_);
    c.q_max = std::isnan(configured_upper_) ? j->limits->upper : std::min(j->limits->upper, configured_upper_);
  }
  // Preserve the URDF velocity cap; an optional YAML value can reduce it.
  c.q_rate = j->limits && std::isfinite(j->limits->velocity) && j->limits->velocity > 0 ? j->limits->velocity
                                                                                        : configured_rate_;
  if(!std::isnan(configured_rate_))
    c.q_rate = std::min(c.q_rate, configured_rate_);
  if(!c.valid())
    return false;
  ROS_INFO("[PitchOnly] pitch limits [%.4f, %.4f] rad, rate %.4f rad/s, command sign %.0f", c.q_min, c.q_max, c.q_rate,
           pitch_only_pitch_command_sign_);
  return true;
}

bool GimbalrotorPerchingNavigator::modelAt(double q, KDL::Frame& bc, KDL::Vector& cog) const
{
  if(!model_valid_ || !std::isfinite(q))
    return false;
  auto joints = robot_model_->getJointPositions();
  if(joint_index_ >= joints.rows())
    return false;
  joints(joint_index_) = q;
  const auto frames = robot_model_->fullForwardKinematics(joints);
  const auto b = frames.find(robot_model_->getBaselinkName()), c = frames.find(contact_link_);
  if(b == frames.end() || c == frames.end() || !pitch_only::finite(b->second) || !pitch_only::finite(c->second))
    return false;
  bc = b->second.Inverse() * c->second;
  KDL::RigidBodyInertia total = KDL::RigidBodyInertia::Zero();
  for(const auto& item : robot_model_->getInertiaMap())
  {
    auto f = frames.find(item.first);
    if(f == frames.end() || !pitch_only::finite(f->second))
      return false;
    perching_geometry::accumulateLinkInertia(total, f->second, item.first, item.second,
                                             robot_model_->getExtraModuleMap());
  }
  if(!std::isfinite(total.getMass()) || total.getMass() <= 1e-9)
    return false;
  cog = b->second.Inverse() * total.getCOG();
  return pitch_only::finite(bc) && pitch_only::finite(cog);
}

pitch_only::Sample GimbalrotorPerchingNavigator::sample() const
{
  pitch_only::Sample s = feedback_;
  s.now = ros::Time::now().toSec();
  s.hover = navi_state_ == HOVER_STATE;
  const double receive_age = s.now - joint_receive_stamp_;
  s.model_valid =
      model_valid_ && joint_receive_stamp_ > 0 && receive_age >= 0 && receive_age <= core_.config.feedback_timeout;
  if(!estimator_)
  {
    s.model_valid = false;
    return s;
  }
  // Same estimator source and frame convention as multilink; no extra odometry subscription.
  const auto p = estimator_->getPos(Frame::BASELINK, estimate_mode_);
  const auto rotation = estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
  tf::Quaternion q;
  rotation.getRotation(q);
  if(!std::isfinite(q.length2()) || q.length2() < 1e-12)
  {
    s.model_valid = false;
    return s;
  }
  q.normalize();
  s.body = KDL::Frame(KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w()), KDL::Vector(p.x(), p.y(), p.z()));
  const auto omega = estimator_->getAngularVel(Frame::BASELINK, estimate_mode_);
  s.angular_speed = omega.length();
  s.pitch_rate = pitch_only::pitchRate(s.body, KDL::Vector(omega.x(), omega.y(), omega.z()));
  return s;
}
void GimbalrotorPerchingNavigator::pitchOnlyJointCallback(const sensor_msgs::JointStateConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const auto it = std::find(m->name.begin(), m->name.end(), joint_name_);
  if(it == m->name.end() || std::count(m->name.begin(), m->name.end(), joint_name_) != 1)
    return;
  const auto index = std::distance(m->name.begin(), it);
  if(static_cast<std::size_t>(index) >= m->position.size() || !std::isfinite(m->position[index]))
    return;
  const double now = ros::Time::now().toSec();
  const double stamp = m->header.stamp.isZero() ? now : m->header.stamp.toSec();
  if(!core_.fresh(now, stamp) || stamp <= feedback_.joint_stamp)
    return;
  const double q = m->position[index];
  double v = pitch_only::unknown;
  if(static_cast<std::size_t>(index) < m->velocity.size() && std::isfinite(m->velocity[index]))
    v = m->velocity[index];
  else if(feedback_.joint_stamp > 0 && stamp - feedback_.joint_stamp > 1e-6 && stamp - feedback_.joint_stamp <= .5)
    v = (q - feedback_.q) / (stamp - feedback_.joint_stamp);
  feedback_.q = q;
  feedback_.qdot = v;
  feedback_.joint_stamp = stamp;
  joint_receive_stamp_ = now;
}

void GimbalrotorPerchingNavigator::requestPitchOnly(const std::string& action)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const auto s = sample();
  const auto generation = core_.generation;
  if(action == "confirm_detached_zero")
    core_.armZero(s);
  else if(action == "pitch_rezero")
    core_.rezero(s);
  else if(action == "freeze")
    core_.freeze(s);
  else if(action == "unfreeze")
    core_.unfreeze(s);
  else if(action == "prepare_unperch")
    core_.prepare(s);
  else if(action == "confirm_released")
    core_.released(s);
  else if(action == "recover")
    core_.recover(s);
  if(core_.generation != generation && core_.ownsPose())
    commitLock();
  publishPitchOnlyDiagnostics();
}
void GimbalrotorPerchingNavigator::selectPitchOnlyMode(perching_geometry::Mode mode, bool enable)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  // An optional pre-contact zero hold can exist while perching is disabled.
  if(mode == perching_geometry::Mode::NORMAL && !enable)
    core_.zero_hold_armed = false;
  selectCommonPerchingMode(mode, enable);
}
void GimbalrotorPerchingNavigator::resetPitchOnlyLock()
{
  core_.reset();
  resetCommonPerchingLock();
}
bool GimbalrotorPerchingNavigator::tryLockPitchOnly(const std::string&)
{
  if(!core_.enable(sample()))
    return false;
  commitLock();
  return true;
}
void GimbalrotorPerchingNavigator::relockPitchOnlyCallback(const std_msgs::EmptyConstPtr&)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(core_.relock(sample()))
    commitLock();
  publishPitchOnlyDiagnostics();
}
void GimbalrotorPerchingNavigator::applyPitchOnlyDelta(double delta)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  core_.cut(delta, sample());
}
void GimbalrotorPerchingNavigator::pitchOnlyNaviCallback(const aerial_robot_msgs::FlightNavConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(!core_.ownsPose())
    GimbalrotorNavigator::naviCallback(m);
}
void GimbalrotorPerchingNavigator::commitLock()
{
  const auto p = core_.target * core_.cog;
  double x, y, z, w;
  core_.target.M.GetQuaternion(x, y, z, w);
  const tf::Quaternion body(x, y, z, w);
  const auto cog_orientation = body * getCommandedBaselinkRotationRelativeToCog().inverse();
  commitFixedContactLock(tf::Vector3(p.x(), p.y(), p.z()), cog_orientation,
                         tf::Vector3(core_.pivot.x(), core_.pivot.y(), core_.pivot.z()));
}

bool GimbalrotorPerchingNavigator::commandPose()
{
  if(!pitch_only::finite(core_.target) || !pitch_only::finite(core_.cog))
    return false;
  double x, y, z, w;
  core_.target.M.GetQuaternion(x, y, z, w);
  setBaselinkRotationTargetRelativeToCog(tf::Quaternion(0, 0, 0, 1));
  const auto relative = getCommandedBaselinkRotationRelativeToCog();
  if(!std::isfinite(relative.length2()) || relative.length2() < 1e-12)
    return false;
  auto orientation = tf::Quaternion(x, y, z, w) * relative.inverse();
  orientation.normalize();
  double r, p, yaw;
  tf::Matrix3x3(orientation).getRPY(r, p, yaw);
  const auto position = core_.target * core_.cog;
  setXyControlMode(POS_CONTROL_MODE);
  setTargetPos(tf::Vector3(position.x(), position.y(), position.z()));
  setTargetVel(0, 0, 0);
  setTargetAcc(0, 0, 0);
  setTargetRPY(tf::Vector3(r, p, yaw));
  setTargetOmega(0, 0, 0);
  setTargetAngAcc(0, 0, 0);
  return true;
}
void GimbalrotorPerchingNavigator::applyPitchOnlyTarget()
{
  if(core_.ownsPose() && !commandPose())
    core_.fault("controller target invalid");
  if(!model_valid_ || (!core_.ownsPose() && !core_.zero_hold_armed))
    return;
  sensor_msgs::JointState cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.name = {joint_name_};
  cmd.position = {core_.q_target};
  pitch_only_joints_pub_.publish(cmd);  // Joint positions are model radians; Servo.yaml performs motor conversion.
}
void GimbalrotorPerchingNavigator::updatePitchOnly()
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(!pitch_only_model_resolution_attempted_ && robot_model_ && robot_model_->initialized())
  {
    pitch_only_model_resolution_attempted_ = true;
    model_valid_ = resolveModel();
    if(!model_valid_)
      ROS_ERROR("[PitchOnly] Unable to resolve pitch mechanism/parameters.");
  }
  GimbalrotorNavigator::update();
  // Match multilink: leaving HOVER disables perching instead of retaining authority.
  if(navi_state_ != HOVER_STATE)
  {
    if(perchingMode() != perching_geometry::Mode::DISABLED)
      selectCommonPerchingMode(perching_geometry::Mode::NORMAL, false);
    core_.zero_hold_armed = false;
    last_update_ = ros::Time();
    publishPitchOnlyDiagnostics();
    return;
  }
  if(perchingMode() == perching_geometry::Mode::NORMAL && !core_.ownsPose())
    tryLockPitchOnly("active update");
  const auto now = ros::Time::now();
  const double dt = last_update_.isZero() ? 0.025 : (now - last_update_).toSec();
  last_update_ = now;
  core_.tick(sample(), dt);
  applyPitchOnlyTarget();
  publishPitchOnlyDiagnostics();
}
void GimbalrotorPerchingNavigator::publishPitchOnlyDiagnostics()
{
  const auto s = sample();
  auto boolean = [this](const std::string& key, bool v)
  {
    std_msgs::Bool m;
    m.data = v;
    bool_pubs_.at(key).publish(m);
  };
  auto number = [this](const std::string& key, double v)
  {
    std_msgs::Float64 m;
    m.data = v;
    number_pubs_.at(key).publish(m);
  };
  const bool valid = core_.feedback(s);
  boolean("model_valid", model_valid_);
  boolean("feedback_valid", valid);
  boolean("contact_valid", core_.attached() && valid);
  boolean("rezero_active", core_.stage == pitch_only::Stage::PITCH_REZERO_ACTIVE);
  boolean("rezero_ready", core_.rezeroReady(s));
  boolean("rezero_failed", core_.stage == pitch_only::Stage::FAULT_HOLD && !core_.rezero_completed);
  boolean("servo_frozen", core_.stage == pitch_only::Stage::PASSIVE_CUTTING);
  boolean("cutting_enabled", core_.stage == pitch_only::Stage::PASSIVE_CUTTING && valid);
  number("servo_measured", s.q);
  number("servo_target_nominal", core_.q_target);
  number("servo_hold_error", s.q - core_.q_target);
  number("body_pitch", pitch_only::pitch(s.body));
  number("body_pitch_rate", s.pitch_rate);
  number("passive_alpha_cmd", core_.alpha);
  number("passive_alpha_measured", core_.ownsPose() ? core_.measuredAlpha(s) : pitch_only::unknown);
  number("cut_alpha_min", core_.alpha_min);
  number("cut_alpha_max", core_.alpha_max);
  number("lock_generation", core_.generation);
  KDL::Frame bc;
  KDL::Vector g;
  const double error = core_.ownsPose() && modelAt(s.q, bc, g)
                           ? (s.body * bc * core_.config.pivot_offset - core_.pivot).Norm()
                           : pitch_only::unknown;
  number("pivot_error", error);
  const char* feedback_error = core_.feedbackError(s);
  // Preserve the 15-field snapshot layout for RViz; old saw-feedback slot is reserved.
  std::ostringstream out;
  out << std::setprecision(17) << name(core_.stage) << "|" << core_.generation << "|" << s.now << "|" << s.q << "|"
      << core_.q_target << "|" << pitch_only::pitch(s.body) << "|" << core_.alpha << "|" << core_.measuredAlpha(s) << "|"
      << core_.alpha_min << "|" << core_.alpha_max << "|" << valid << "|0|" << error << "|" << core_.zero_hold_armed
      << "|" << (feedback_error ? feedback_error : core_.status);
  std_msgs::String state;
  state.data = out.str();
  pitch_only_state_pub_.publish(state);
  if(core_.ownsPose())
  {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = core_.target.p.x();
    pose.pose.position.y = core_.target.p.y();
    pose.pose.position.z = core_.target.p.z();
    core_.target.M.GetQuaternion(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z,
                                 pose.pose.orientation.w);
    pose_pub_.publish(pose);
    geometry_msgs::PointStamped pivot;
    pivot.header = pose.header;
    pivot.point.x = core_.pivot.x();
    pivot.point.y = core_.pivot.y();
    pivot.point.z = core_.pivot.z();
    pivot_pub_.publish(pivot);
  }
}

// Multilink workflow






namespace aerial_robot_navigation
{


void GimbalrotorPerchingNavigator::initializeMultilink(ros::NodeHandle nh, ros::NodeHandle nhp)
{

  multilinkRosParamInit();

  multilink_model_valid_ = false;
  const bool robot_model_ready =
      robot_model_ && robot_model_->initialized();

  // Model topology is fixed; defer resolution only until the first model update.
  multilink_model_resolution_attempted_ = robot_model_ready;
  if(robot_model_ready)
  {
    multilink_model_valid_ = resolveMechanismModel();
  }
  else
  {
    ROS_WARN("[GimbalrotorPerchingNavigator/multilink] waiting for the robot model to initialize.");
  }

  joint_state_sub_ = nh_.subscribe(
      joint_state_topic_,
      1,
      &GimbalrotorPerchingNavigator::jointStateCallback,
      this);
  secondary_joint_target_sub_ = nh_.subscribe(
      secondary_joint_target_topic_,
      1,
      &GimbalrotorPerchingNavigator::
          secondaryJointTargetCallback,
      this);

  if(pitch_rezero_config_valid_)
    pitch_rezero_sub_ = nh_.subscribe(
        pitch_rezero_topic_, 1,
        &GimbalrotorPerchingNavigator::multilinkPitchRezeroCallback, this);
  pitch_rezero_active_pub_ = nh_.advertise<std_msgs::Bool>(
      "perching/multilink/pitch_rezero_active", 1);
  pitch_rezero_ready_pub_ = nh_.advertise<std_msgs::Bool>(
      "perching/multilink/pitch_rezero_ready", 1);
  pitch_rezero_failed_pub_ = nh_.advertise<std_msgs::Bool>(
      "perching/multilink/pitch_rezero_failed", 1);
  pitch_joint_hold_error_pub_ = nh_.advertise<std_msgs::Float64>(
      "perching/multilink/pitch_joint_hold_error", 1);
  secondary_joint_hold_error_pub_ = nh_.advertise<std_msgs::Float64>(
      "perching/multilink/secondary_joint_hold_error", 1);
  passive_pitch_delta_pub_ = nh_.advertise<std_msgs::Float64>(
      "perching/multilink/passive_pitch_delta", 1);
  body_pitch_pub_ = nh_.advertise<std_msgs::Float64>(
      "perching/multilink/body_pitch", 1);
  body_pitch_rate_pub_ = nh_.advertise<std_msgs::Float64>(
      "perching/multilink/body_pitch_rate", 1);

  joint_control_pub_ =
      nh_.advertise<sensor_msgs::JointState>(joint_command_topic_, 1);
  model_valid_pub_ =
      nh_.advertise<std_msgs::Bool>("perching/multilink/model_valid", 1, true);
  secondary_settled_pub_ =
      nh_.advertise<std_msgs::Bool>(
          "perching/multilink/secondary_settled", 1);
  pitch_measured_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/multilink/pitch_joint_measured", 1);
  pitch_nominal_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/multilink/pitch_joint_nominal", 1);
  pitch_final_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/multilink/pitch_joint_final", 1);
  secondary_measured_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/multilink/secondary_joint_measured", 1);
  // Keep diagnostics off the command topic (especially with a negative command sign).
  secondary_target_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/multilink/secondary_joint_nominal", 1);
  target_body_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(
          "perching/multilink/target_body_pose", 1);

  std_msgs::Bool valid_msg;
  valid_msg.data = multilink_model_valid_;
  model_valid_pub_.publish(valid_msg);

  if(multilink_model_valid_)
  {
    ROS_WARN(
        "[GimbalrotorPerchingNavigator/multilink] model ready: "
        "pitch '%s', secondary '%s' (%s), contact '%s'.",
        pitch_joint_name_.c_str(),
        secondary_joint_name_.c_str(),
        secondary_axis_type_ == SECONDARY_AXIS_ROLL ? "roll" : "yaw",
        contact_link_name_.c_str());
  }
  else if(robot_model_ready)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] multilink model is "
        "not valid. Locking and mechanism commands are disabled.");
  }
}

void GimbalrotorPerchingNavigator::multilinkRosParamInit()
{
  ros::NodeHandle multilink_nh(nh_, "navigation/multilink_perching");

  multilink_nh.param("pitch_joint_name", pitch_joint_name_, std::string(""));
  multilink_nh.param(
      "secondary_joint_name", secondary_joint_name_, std::string(""));
  multilink_nh.param("contact_link_name", contact_link_name_, std::string(""));
  multilink_nh.param(
      "joint_command_topic", joint_command_topic_, std::string("joints_ctrl"));
  multilink_nh.param(
      "joint_state_topic", joint_state_topic_, std::string("joint_states"));
  multilink_nh.param(
      "secondary_joint_target_topic",
      secondary_joint_target_topic_,
      std::string("perching/multilink/secondary_joint_target"));

  multilink_nh.param(
      "secondary_position_tolerance",
      secondary_position_tolerance_,
      0.02);
  multilink_nh.param(
      "secondary_velocity_tolerance",
      secondary_velocity_tolerance_,
      0.05);
  multilink_nh.param(
      "secondary_settle_duration", secondary_settle_duration_, 0.30);
  multilink_nh.param("joint_state_timeout", joint_state_timeout_, 0.15);
  multilink_nh.param(
      "joint_state_future_tolerance",
      joint_state_future_tolerance_,
      0.02);
  multilink_nh.param(
      "pitch_axis_alignment_threshold",
      multilink_pitch_axis_alignment_threshold_,
      0.90);
  multilink_nh.param(
      "secondary_axis_alignment_threshold",
      secondary_axis_alignment_threshold_,
      0.90);
  multilink_nh.param("pitch_command_sign", multilink_pitch_command_sign_, 1.0);
  multilink_nh.param("secondary_command_sign", secondary_command_sign_, 1.0);

  multilink_nh.param("pitch_rezero_topic", pitch_rezero_topic_,
                    std::string("perching/multilink/pitch_rezero"));
  multilink_nh.param("pitch_rezero_target_pitch", pitch_rezero_target_pitch_, 0.0);
  multilink_nh.param("pitch_rezero_kp", pitch_rezero_kp_, 1.0);
  multilink_nh.param("pitch_rezero_rate_limit", pitch_rezero_rate_limit_, 0.1745329);
  multilink_nh.param("pitch_rezero_max_delta", pitch_rezero_max_delta_, 0.5235988);
  multilink_nh.param("pitch_rezero_command_sign", pitch_rezero_command_sign_, 1.0);
  multilink_nh.param("pitch_rezero_pitch_tolerance", pitch_rezero_pitch_tolerance_, 0.0349066);
  multilink_nh.param("pitch_rezero_rate_tolerance", pitch_rezero_rate_tolerance_, 0.05);
  multilink_nh.param("pitch_rezero_stable_duration", pitch_rezero_stable_duration_, 0.3);
  multilink_nh.param("pitch_rezero_timeout", pitch_rezero_timeout_, 5.0);
  multilink_nh.param("pitch_rezero_pitch_joint_hold_tolerance",
                    pitch_rezero_pitch_joint_hold_tolerance_, 0.02);
  multilink_nh.param("pitch_rezero_secondary_joint_hold_tolerance",
                    pitch_rezero_secondary_joint_hold_tolerance_, 0.02);
  for(unsigned int i = 0; i < 3; ++i)
  {
    const std::string suffix(1, "xyz"[i]);
    multilink_nh.param("pitch_rezero_pivot_offset_" + suffix,
                      pitch_rezero_pivot_offset_(i), 0.0);
    multilink_nh.param("pitch_rezero_axis_" + suffix,
                      pitch_rezero_axis_(i), i == 1 ? 1.0 : 0.0);
  }
  const auto positive = [](double v) { return std::isfinite(v) && v > 0.0; };
  std::string topic_error;
  pitch_rezero_config_valid_ =
      !pitch_rezero_topic_.empty() &&
      ros::names::validate(pitch_rezero_topic_, topic_error) &&
      std::isfinite(pitch_rezero_target_pitch_) &&
      std::abs(pitch_rezero_target_pitch_) < M_PI / 2.0 &&
      positive(pitch_rezero_kp_) && positive(pitch_rezero_rate_limit_) &&
      positive(pitch_rezero_max_delta_) && pitch_rezero_max_delta_ <= M_PI &&
      std::isfinite(pitch_rezero_command_sign_) && pitch_rezero_command_sign_ != 0.0 &&
      positive(pitch_rezero_pitch_tolerance_) && pitch_rezero_pitch_tolerance_ < M_PI / 2.0 &&
      positive(pitch_rezero_rate_tolerance_) &&
      positive(pitch_rezero_stable_duration_) && positive(pitch_rezero_timeout_) &&
      pitch_rezero_timeout_ >= pitch_rezero_stable_duration_ &&
      positive(pitch_rezero_pitch_joint_hold_tolerance_) &&
      pitch_rezero_pitch_joint_hold_tolerance_ <= 0.5 &&
      positive(pitch_rezero_secondary_joint_hold_tolerance_) &&
      pitch_rezero_secondary_joint_hold_tolerance_ <= 0.5 &&
      frameFinite(KDL::Frame(pitch_rezero_pivot_offset_)) &&
      frameFinite(KDL::Frame(pitch_rezero_axis_)) &&
      positive(pitch_rezero_axis_.Norm()) && pitch_rezero_axis_.Norm() > 1.0e-6;
  if(pitch_rezero_config_valid_)
  {
    pitch_rezero_axis_ = pitch_rezero_axis_ / pitch_rezero_axis_.Norm();
    pitch_rezero_command_sign_ = pitch_rezero_command_sign_ < 0.0 ? -1.0 : 1.0;
  }
  else
    ROS_ERROR("[Multilink] Invalid pitch_rezero configuration; rezero is disabled.");

  configured_pitch_limits_valid_ =
      multilink_nh.getParam("pitch_lower_limit", configured_pitch_lower_) &&
      multilink_nh.getParam("pitch_upper_limit", configured_pitch_upper_) &&
      std::isfinite(configured_pitch_lower_) &&
      std::isfinite(configured_pitch_upper_) &&
      configured_pitch_lower_ < configured_pitch_upper_;
  configured_secondary_limits_valid_ =
      multilink_nh.getParam(
          "secondary_lower_limit", configured_secondary_lower_) &&
      multilink_nh.getParam(
          "secondary_upper_limit", configured_secondary_upper_) &&
      std::isfinite(configured_secondary_lower_) &&
      std::isfinite(configured_secondary_upper_) &&
      configured_secondary_lower_ < configured_secondary_upper_;

  if(!std::isfinite(secondary_position_tolerance_) || secondary_position_tolerance_ <= 0.0)
    secondary_position_tolerance_ = 0.02;
  if(!std::isfinite(secondary_velocity_tolerance_) || secondary_velocity_tolerance_ <= 0.0)
    secondary_velocity_tolerance_ = 0.05;
  if(!std::isfinite(secondary_settle_duration_) || secondary_settle_duration_ < 0.0)
    secondary_settle_duration_ = 0.30;
  if(!std::isfinite(joint_state_timeout_) || joint_state_timeout_ <= 0.0)
    joint_state_timeout_ = 0.15;
  if(!std::isfinite(joint_state_future_tolerance_) || joint_state_future_tolerance_ < 0.0)
    joint_state_future_tolerance_ = 0.02;

  joint_state_future_tolerance_ = std::min(joint_state_future_tolerance_, joint_state_timeout_);

  if(!std::isfinite(multilink_pitch_axis_alignment_threshold_) || multilink_pitch_axis_alignment_threshold_ <= 0.0 || multilink_pitch_axis_alignment_threshold_ > 1.0)
    multilink_pitch_axis_alignment_threshold_ = 0.90;
  if(!std::isfinite(secondary_axis_alignment_threshold_) || secondary_axis_alignment_threshold_ <= 0.0 || secondary_axis_alignment_threshold_ > 1.0)
    secondary_axis_alignment_threshold_ = 0.90;

  multilink_pitch_command_sign_ =
      std::isfinite(multilink_pitch_command_sign_) && multilink_pitch_command_sign_ < 0.0 ?
          -1.0 : 1.0;
  secondary_command_sign_ =
      std::isfinite(secondary_command_sign_) && secondary_command_sign_ < 0.0 ?
          -1.0 : 1.0;
}

bool GimbalrotorPerchingNavigator::resolveJointLimits(
    const std::string& joint_name,
    bool configured_limits_valid,
    double configured_lower,
    double configured_upper,
    double& lower,
    double& upper) const
{
  const urdf::JointConstSharedPtr joint = robot_model_->getUrdfModel().getJoint(joint_name);

  if(joint && joint->type == urdf::Joint::REVOLUTE && joint->limits &&
     std::isfinite(joint->limits->lower) &&
     std::isfinite(joint->limits->upper) &&
     joint->limits->lower < joint->limits->upper)
  {
    lower = joint->limits->lower;
    upper = joint->limits->upper;
    return true;
  }

  if(configured_limits_valid)
  {
    lower = configured_lower;
    upper = configured_upper;
    return true;
  }

  ROS_ERROR(
      "[GimbalrotorPerchingNavigator/multilink] joint '%s' has no "
      "finite position limits and no valid configured fallback limits.",
      joint_name.c_str());
  return false;
}

bool GimbalrotorPerchingNavigator::resolveMechanismModel()
{
  if(!robot_model_ || !robot_model_->initialized())
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] robot model is unavailable.");
    return false;
  }

  if(pitch_joint_name_.empty() || secondary_joint_name_.empty() || contact_link_name_.empty())
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] pitch_joint_name, "
        "secondary_joint_name, and contact_link_name must all be configured.");
    return false;
  }

  if(pitch_joint_name_ == secondary_joint_name_)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] pitch and secondary "
        "joint names must be different.");
    return false;
  }

  const urdf::JointConstSharedPtr pitch_joint = robot_model_->getUrdfModel().getJoint(pitch_joint_name_);
  const urdf::JointConstSharedPtr secondary_joint = robot_model_->getUrdfModel().getJoint(secondary_joint_name_);

  if(!pitch_joint || !secondary_joint)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] configured mechanism "
        "joint is missing from URDF.");
    return false;
  }

  const auto revolute_or_continuous = [](const urdf::JointConstSharedPtr& joint)
  {
    return joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS;
  };

  if(!revolute_or_continuous(pitch_joint) ||
     !revolute_or_continuous(secondary_joint))
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] both configured joints "
        "must be revolute or continuous.");
    return false;
  }

  pitch_axis_local_ = Eigen::Vector3d(pitch_joint->axis.x, pitch_joint->axis.y, pitch_joint->axis.z);
  secondary_axis_local_ = Eigen::Vector3d(
      secondary_joint->axis.x,
      secondary_joint->axis.y,
      secondary_joint->axis.z);

  if(!pitch_axis_local_.allFinite() || pitch_axis_local_.norm() <= 1.0e-6 ||
     !secondary_axis_local_.allFinite() ||
     secondary_axis_local_.norm() <= 1.0e-6)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] configured joint axis "
        "is zero or non-finite.");
    return false;
  }

  pitch_axis_local_.normalize();
  secondary_axis_local_.normalize();

  if(std::abs(pitch_axis_local_.y()) < multilink_pitch_axis_alignment_threshold_)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] pitch joint axis is not "
        "sufficiently local-Y aligned: [%.3f %.3f %.3f].",
        pitch_axis_local_.x(),
        pitch_axis_local_.y(),
        pitch_axis_local_.z());
    return false;
  }

  const double secondary_x = std::abs(secondary_axis_local_.x());
  const double secondary_z = std::abs(secondary_axis_local_.z());
  if(secondary_x >= secondary_axis_alignment_threshold_ && secondary_x > secondary_z)
    secondary_axis_type_ = SECONDARY_AXIS_ROLL;
  else if(secondary_z >= secondary_axis_alignment_threshold_ && secondary_z > secondary_x)
    secondary_axis_type_ = SECONDARY_AXIS_YAW;
  else
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] secondary joint axis is "
        "neither unambiguously local-X nor local-Z aligned: "
        "[%.3f %.3f %.3f].",
        secondary_axis_local_.x(),
        secondary_axis_local_.y(),
        secondary_axis_local_.z());
    return false;
  }

  if(!robot_model_->getUrdfModel().getLink(contact_link_name_) || robot_model_->getTree().getSegment(contact_link_name_) == robot_model_->getTree().getSegments().end())
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] contact link '%s' "
        "does not exist in the model.",
        contact_link_name_.c_str());
    return false;
  }

  const auto& joint_index_map = robot_model_->getJointIndexMap();
  const auto pitch_index_it = joint_index_map.find(pitch_joint_name_);
  const auto secondary_index_it = joint_index_map.find(secondary_joint_name_);
  if(pitch_index_it == joint_index_map.end() || secondary_index_it == joint_index_map.end())
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] configured joint is not "
        "a movable KDL model joint.");
    return false;
  }

  pitch_joint_index_ = pitch_index_it->second;
  secondary_joint_index_ = secondary_index_it->second;
  if(pitch_joint_index_ >= robot_model_->getJointPositions().rows() ||
     secondary_joint_index_ >= robot_model_->getJointPositions().rows())
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] configured joint index "
        "is outside the model joint array.");
    return false;
  }

  KDL::Chain chain;
  if(!robot_model_->getTree().getChain(robot_model_->getBaselinkName(), contact_link_name_, chain))
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] no KDL chain exists from "
        "baselink '%s' to contact link '%s'.",
        robot_model_->getBaselinkName().c_str(),
        contact_link_name_.c_str());
    return false;
  }

  int pitch_order = -1;
  int secondary_order = -1;
  int movable_order = 0;
  for(unsigned int i = 0; i < chain.getNrOfSegments(); ++i)
  {
    const KDL::Joint& joint = chain.getSegment(i).getJoint();
    if(joint.getType() == KDL::Joint::None)
      continue;

    if(joint.getName() == pitch_joint_name_)
      pitch_order = movable_order;
    else if(joint.getName() == secondary_joint_name_)
      secondary_order = movable_order;
    else
    {
      ROS_ERROR(
          "[GimbalrotorPerchingNavigator/multilink] unsupported movable "
          "joint '%s' lies in the baselink-to-contact chain.",
          joint.getName().c_str());
      return false;
    }
    ++movable_order;
  }

  if(pitch_order < 0 || secondary_order < 0 || pitch_order >= secondary_order || movable_order != 2)
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] chain must contain exactly "
        "pitch then secondary as its two movable joints.");
    return false;
  }

  if(!resolveJointLimits(
         pitch_joint_name_,
         configured_pitch_limits_valid_,
         configured_pitch_lower_,
         configured_pitch_upper_,
         pitch_joint_lower_,
         pitch_joint_upper_) ||
     !resolveJointLimits(
         secondary_joint_name_,
         configured_secondary_limits_valid_,
         configured_secondary_lower_,
         configured_secondary_upper_,
         secondary_joint_lower_,
         secondary_joint_upper_))
    return false;

  return true;
}

void GimbalrotorPerchingNavigator::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> state_lock(perchingStateMutex());
  if(!multilink_model_valid_)
    return;

  int pitch_msg_index = -1;
  int secondary_msg_index = -1;

  for(std::size_t i = 0; i < msg->name.size(); ++i)
  {
    if(msg->name.at(i) == pitch_joint_name_)
      pitch_msg_index = static_cast<int>(i);

    if(msg->name.at(i) == secondary_joint_name_)
      secondary_msg_index = static_cast<int>(i);
  }

  if(pitch_msg_index < 0 ||
     secondary_msg_index < 0 ||
     static_cast<std::size_t>(pitch_msg_index) >= msg->position.size() ||
     static_cast<std::size_t>(secondary_msg_index) >= msg->position.size())
  {
    return;
  }

  const double pitch_position = msg->position.at(pitch_msg_index);
  const double secondary_position = msg->position.at(secondary_msg_index);

  if(!std::isfinite(pitch_position) || !std::isfinite(secondary_position))
  {
    return;
  }

  const ros::Time receive_stamp = ros::Time::now();

  const ros::Time measurement_stamp =
      msg->header.stamp.isZero()
          ? receive_stamp
          : msg->header.stamp;

  const double measurement_age_at_receive = (receive_stamp - measurement_stamp).toSec();

  if(!std::isfinite(measurement_age_at_receive) ||
     measurement_age_at_receive < -joint_state_future_tolerance_ ||
     measurement_age_at_receive > joint_state_timeout_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator/multilink] "
        "received stale or invalid joint state measurement "
        "(measurement age %.3f s).",
        measurement_age_at_receive);

    return;
  }

  if(joint_state_received_)
  {
    const double measurement_dt = (measurement_stamp - joint_state_measurement_stamp_).toSec();

    if(!std::isfinite(measurement_dt) || measurement_dt <= 0.0)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorPerchingNavigator/multilink] "
          "ignored out-of-order/duplicate joint state timestamp.");

      return;
    }
  }

  double pitch_velocity = std::numeric_limits<double>::infinity();
  double secondary_velocity = std::numeric_limits<double>::infinity();

  if(static_cast<std::size_t>(pitch_msg_index) < msg->velocity.size() &&
     static_cast<std::size_t>(secondary_msg_index) < msg->velocity.size() &&
     std::isfinite(msg->velocity.at(pitch_msg_index)) &&
     std::isfinite(msg->velocity.at(secondary_msg_index)))
  {
    pitch_velocity = msg->velocity.at(pitch_msg_index);
    secondary_velocity = msg->velocity.at(secondary_msg_index);
  }

  else if(joint_state_received_)
  {
    const double dt = (measurement_stamp - joint_state_measurement_stamp_).toSec();

    if(std::isfinite(dt) && dt > 1.0e-6 && dt <= 0.5)
    {
      pitch_velocity = (pitch_position - previous_measured_pitch_joint_) / dt;
      secondary_velocity = (secondary_position - previous_measured_secondary_joint_) / dt;
    }
  }

  previous_measured_pitch_joint_ = pitch_position;
  previous_measured_secondary_joint_ = secondary_position;
  measured_pitch_joint_ = pitch_position;
  measured_secondary_joint_ = secondary_position;
  measured_pitch_velocity_ = pitch_velocity;
  measured_secondary_velocity_ = secondary_velocity;

  joint_state_measurement_stamp_ = measurement_stamp;
  joint_state_receive_stamp_ = receive_stamp;

  joint_state_received_ = true;
}

bool GimbalrotorPerchingNavigator::readCurrentMechanismState(
    double& pitch_position,
    double& secondary_position,
    double& pitch_velocity,
    double& secondary_velocity) const
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());

  if(!multilink_model_valid_ || !joint_state_received_)
    return false;

  const ros::Time now = ros::Time::now();

  const double receive_age = (now - joint_state_receive_stamp_).toSec();
  const double measurement_age = (now - joint_state_measurement_stamp_).toSec();

  if(!std::isfinite(receive_age) ||
    receive_age < 0.0 ||
    receive_age > joint_state_timeout_ ||
    !std::isfinite(measurement_age) ||
    measurement_age < -joint_state_future_tolerance_ ||
    measurement_age > joint_state_timeout_)
  {
    return false;
  }

  pitch_position = measured_pitch_joint_;
  secondary_position = measured_secondary_joint_;
  pitch_velocity = measured_pitch_velocity_;
  secondary_velocity = measured_secondary_velocity_;

  return std::isfinite(pitch_position) &&
         std::isfinite(secondary_position) &&
         std::isfinite(pitch_velocity) &&
         std::isfinite(secondary_velocity) &&
         pitch_position >= pitch_joint_lower_ - 1.0e-6 &&
         pitch_position <= pitch_joint_upper_ + 1.0e-6 &&
         secondary_position >= secondary_joint_lower_ - 1.0e-6 &&
         secondary_position <= secondary_joint_upper_ + 1.0e-6;
}

bool GimbalrotorPerchingNavigator::computeBaselinkToContactTransform(
    double pitch_position,
    double secondary_position,
    KDL::Frame& T_B_C) const
{
  if(!multilink_model_valid_ || !std::isfinite(pitch_position) ||
     !std::isfinite(secondary_position))
    return false;

  KDL::JntArray joints = robot_model_->getJointPositions();
  if(pitch_joint_index_ >= joints.rows() ||
     secondary_joint_index_ >= joints.rows())
    return false;

  joints(pitch_joint_index_) = pitch_position;
  joints(secondary_joint_index_) = secondary_position;

  const KDL::Frame T_R_B = robot_model_->forwardKinematics<KDL::Frame>(
      robot_model_->getBaselinkName(), joints);
  const KDL::Frame T_R_C = robot_model_->forwardKinematics<KDL::Frame>(
      contact_link_name_, joints);
  T_B_C = T_R_B.Inverse() * T_R_C;
  return frameFinite(T_R_B) && frameFinite(T_R_C) && frameFinite(T_B_C);
}

bool GimbalrotorPerchingNavigator::computeBaselinkToCogVector(
    double pitch_position,
    double secondary_position,
    KDL::Vector& p_B_G) const
{
  if(!multilink_model_valid_ || !std::isfinite(pitch_position) ||
     !std::isfinite(secondary_position))
    return false;

  KDL::JntArray joints = robot_model_->getJointPositions();
  if(pitch_joint_index_ >= joints.rows() || secondary_joint_index_ >= joints.rows())
    return false;
  joints(pitch_joint_index_) = pitch_position;
  joints(secondary_joint_index_) = secondary_position;

  const std::map<std::string, KDL::Frame> frames =
      robot_model_->fullForwardKinematics(joints);
  KDL::RigidBodyInertia total_inertia = KDL::RigidBodyInertia::Zero();
  for(const auto& inertia : robot_model_->getInertiaMap())
  {
    const auto frame_it = frames.find(inertia.first);
    if(frame_it == frames.end() || !frameFinite(frame_it->second))
      return false;

    perching_geometry::accumulateLinkInertia(
        total_inertia, frame_it->second, inertia.first, inertia.second, robot_model_->getExtraModuleMap());
  }

  if(!std::isfinite(total_inertia.getMass()) ||
     total_inertia.getMass() <= 1.0e-9)
    return false;

  const KDL::Frame T_R_B = robot_model_->forwardKinematics<KDL::Frame>(
      robot_model_->getBaselinkName(), joints);
  p_B_G = T_R_B.Inverse() * total_inertia.getCOG();

  return frameFinite(T_R_B) &&
         std::isfinite(p_B_G.x()) &&
         std::isfinite(p_B_G.y()) &&
         std::isfinite(p_B_G.z());
}

bool GimbalrotorPerchingNavigator::tryLockMultilink(
    const std::string& reason)
{
  if(perchingMode() != perching_geometry::Mode::NORMAL ||
     getNaviState() != HOVER_STATE || !multilink_model_valid_)
    return false;

  double pitch_position = 0.0;
  double secondary_position = 0.0;
  double pitch_velocity = 0.0;
  double secondary_velocity = 0.0;
  if(!readCurrentMechanismState(
         pitch_position,
         secondary_position,
         pitch_velocity,
         secondary_velocity))
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator/multilink] cannot lock: named "
        "mechanism joint state is missing, invalid, or stale.");
    return false;
  }

  KDL::Frame T_B_C;
  if(!computeBaselinkToContactTransform(
         pitch_position, secondary_position, T_B_C))
  {
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator/multilink] cannot lock: contact FK "
        "failed.");
    return false;
  }

  const tf::Vector3 baselink_pos =
      estimator_->getPos(Frame::BASELINK, estimate_mode_);
  tf::Quaternion baselink_quaternion;
  estimator_->getOrientation(Frame::BASELINK, estimate_mode_)
      .getRotation(baselink_quaternion);

  if(!std::isfinite(baselink_pos.x()) ||
     !std::isfinite(baselink_pos.y()) ||
     !std::isfinite(baselink_pos.z()) ||
     !std::isfinite(baselink_quaternion.x()) ||
     !std::isfinite(baselink_quaternion.y()) ||
     !std::isfinite(baselink_quaternion.z()) ||
     !std::isfinite(baselink_quaternion.w()) ||
     !std::isfinite(baselink_quaternion.length2()) ||
     baselink_quaternion.length2() < 1.0e-12)
    return false;

  baselink_quaternion.normalize();
  const KDL::Frame T_W_B(
      KDL::Rotation::Quaternion(
          baselink_quaternion.x(),
          baselink_quaternion.y(),
          baselink_quaternion.z(),
          baselink_quaternion.w()),
      KDL::Vector(baselink_pos.x(), baselink_pos.y(), baselink_pos.z()));
  const KDL::Frame T_W_C = T_W_B * T_B_C;
  if(!frameFinite(T_W_C))
    return false;

  const tf::Vector3 cog_pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Quaternion cog_orientation;
  estimator_->getOrientation(Frame::COG, estimate_mode_).getRotation(cog_orientation);
  if(!perching_geometry::finite(cog_pos) ||
     !std::isfinite(cog_orientation.length2()) || cog_orientation.length2() < 1.0e-12)
    return false;
  cog_orientation.normalize();
  const tf::Vector3 contact_pos(T_W_C.p.x(), T_W_C.p.y(), T_W_C.p.z());

  {
    std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
    locked_contact_world_ = T_W_C;
    locked_pitch_joint_ = pitch_position;
    locked_secondary_joint_ = secondary_position;
    pitch_joint_nominal_target_ = pitch_position;
    secondary_joint_nominal_target_ = secondary_position;
    pitch_joint_final_target_ = pitch_position;
    secondary_joint_final_target_ = secondary_position;
    multilink_lock_valid_ = true;
    secondary_settled_ = false;
    secondary_settle_start_ = ros::Time(0);
    ++mechanism_target_generation_;
  }

  commitFixedContactLock(cog_pos, cog_orientation, contact_pos);

  ROS_WARN(
      "[GimbalrotorPerchingNavigator/multilink] multilink contact locked "
      "by %s at q=[%.4f, %.4f].",
      reason.c_str(),
      pitch_position,
      secondary_position);
  publishDiagnostics();
  return true;
}

void GimbalrotorPerchingNavigator::resetMultilinkLock()
{
  resetCommonPerchingLock();

  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  resetMultilinkPitchRezeroState();
  multilink_lock_valid_ = false;
  locked_contact_world_ = KDL::Frame::Identity();
  locked_pitch_joint_ = 0.0;
  locked_secondary_joint_ = 0.0;
  pitch_joint_nominal_target_ = 0.0;
  secondary_joint_nominal_target_ = 0.0;
  pitch_joint_final_target_ = 0.0;
  secondary_joint_final_target_ = 0.0;
  secondary_settled_ = false;
  secondary_settle_start_ = ros::Time(0);
  ++mechanism_target_generation_;
  publishDiagnostics();
}

bool GimbalrotorPerchingNavigator::buildFinalJointTarget(
    double& pitch_final,
    double& secondary_final) const
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  if(!multilink_model_valid_ || !multilink_lock_valid_)
    return false;

  pitch_final = clampMultilink(
      pitch_joint_nominal_target_,
      pitch_joint_lower_,
      pitch_joint_upper_);
  secondary_final = clampMultilink(
      secondary_joint_nominal_target_,
      secondary_joint_lower_,
      secondary_joint_upper_);

  return std::isfinite(pitch_final) && std::isfinite(secondary_final);
}

void GimbalrotorPerchingNavigator::applyMultilinkTarget()
{
  std::lock_guard<std::recursive_mutex> state_lock(perchingStateMutex());
  if(multilink_pitch_rezero_state_ != MultilinkPitchRezeroState::IDLE)
  {
    applyMultilinkPitchRezeroTarget();
    return;
  }
  if(perchingMode() != perching_geometry::Mode::NORMAL || getNaviState() != HOVER_STATE ||
     !multilink_model_valid_)
    return;

  if(!multilinkLockValid() && !tryLockMultilink("active multilink update"))
    return;

  double measured_pitch = 0.0;
  double measured_secondary = 0.0;
  double pitch_velocity = 0.0;
  double secondary_velocity = 0.0;
  if(!readCurrentMechanismState(
         measured_pitch,
         measured_secondary,
         pitch_velocity,
         secondary_velocity))
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator/multilink] holding the last target: "
        "mechanism joint state is stale or invalid.");
    return;
  }

  double pitch_final = 0.0;
  double secondary_final = 0.0;
  if(!buildFinalJointTarget(pitch_final, secondary_final))
    return;

  KDL::Frame T_B_C_final;
  KDL::Vector p_B_G_final;
  if(!computeBaselinkToContactTransform(
         pitch_final, secondary_final, T_B_C_final) ||
     !computeBaselinkToCogVector(
         pitch_final, secondary_final, p_B_G_final))
  {
    ROS_ERROR_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator/multilink] rejected invalid target FK.");
    return;
  }

  KDL::Frame locked_contact;
  {
    std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
    if(!multilink_lock_valid_)
      return;
    locked_contact = locked_contact_world_;
  }

  const KDL::Frame T_W_B_des = locked_contact * T_B_C_final.Inverse();
  if(!commandMultilinkBaselinkPose(T_W_B_des, p_B_G_final))
    return;
  pitch_joint_final_target_ = pitch_final;
  secondary_joint_final_target_ = secondary_final;
  publishMultilinkJointTarget(pitch_final, secondary_final);
}

bool GimbalrotorPerchingNavigator::commandMultilinkBaselinkPose(
    const KDL::Frame& T_W_B_des, const KDL::Vector& p_B_G)
{
  const KDL::Vector p_W_G_des = T_W_B_des * p_B_G;
  if(!frameFinite(T_W_B_des) ||
     !std::isfinite(p_W_G_des.x()) ||
     !std::isfinite(p_W_G_des.y()) ||
     !std::isfinite(p_W_G_des.z()))
    return false;

  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  T_W_B_des.M.GetQuaternion(qx, qy, qz, qw);
  tf::Quaternion R_W_B(qx, qy, qz, qw);

  /*
   * Multilink perching owns aerial-body orientation.  The normal
   * gimbalrotor controller tracks CoG attitude, while the navigator's
   * baselink rotation is baselink relative to CoG.  Drive that relative
   * target toward identity and compensate its current commanded value so
   * the resulting baselink target remains exactly T_W_B_des.
   */
  setBaselinkRotationTargetRelativeToCog(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  const tf::Quaternion R_C_B =
      getCommandedBaselinkRotationRelativeToCog();
  if(!std::isfinite(R_C_B.length2()) || R_C_B.length2() < 1.0e-12)
    return false;
  tf::Quaternion R_W_G = R_W_B * R_C_B.inverse();
  R_W_G.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf::Matrix3x3(R_W_G).getRPY(roll, pitch, yaw);
  if(!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))
    return false;

  setXyControlMode(POS_CONTROL_MODE);
  setTargetPos(tf::Vector3(p_W_G_des.x(), p_W_G_des.y(), p_W_G_des.z()));
  setTargetVel(0.0, 0.0, 0.0);
  setTargetAcc(0.0, 0.0, 0.0);
  setTargetRPY(tf::Vector3(roll, pitch, yaw));
  setTargetOmega(0.0, 0.0, 0.0);
  setTargetAngAcc(0.0, 0.0, 0.0);

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "world";
  pose_msg.pose.position.x = T_W_B_des.p.x();
  pose_msg.pose.position.y = T_W_B_des.p.y();
  pose_msg.pose.position.z = T_W_B_des.p.z();
  pose_msg.pose.orientation.x = qx;
  pose_msg.pose.orientation.y = qy;
  pose_msg.pose.orientation.z = qz;
  pose_msg.pose.orientation.w = qw;
  target_body_pose_pub_.publish(pose_msg);
  return true;
}

bool GimbalrotorPerchingNavigator::applyMultilinkConstraint(
    aerial_robot_msgs::FlightNav& nav_msg)
{
  /*
   * While the multilink lock is active, external body commands cannot be
   * applied independently of the fixed contact.  The active update owns the
   * complete body target through fixed-contact FK.
   */
  nav_msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  nav_msg.pos_z_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  nav_msg.roll_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  nav_msg.pitch_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  nav_msg.yaw_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  return true;
}

void GimbalrotorPerchingNavigator::applyMultilinkPitchDelta(double delta)
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  if(multilink_pitch_rezero_state_ != MultilinkPitchRezeroState::IDLE)
  {
    ROS_WARN_THROTTLE(1.0, "[Multilink] Active joints frozen during pitch rezero/HOLD; relock after ready.");
    return;
  }

  if(!multilink_model_valid_ ||
     !multilink_lock_valid_ ||
     !std::isfinite(delta))
    return;

  const double target = clampMultilink(locked_pitch_joint_ + multilink_pitch_command_sign_ * delta,
                              pitch_joint_lower_, pitch_joint_upper_);
  if(std::abs(target - pitch_joint_nominal_target_) <= 1.0e-9)
    return;
  pitch_joint_nominal_target_ = target;
  ++mechanism_target_generation_;
}

void GimbalrotorPerchingNavigator::secondaryJointTargetCallback(
    const std_msgs::Float64ConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> state_lock(perchingStateMutex());
  if(multilink_pitch_rezero_state_ != MultilinkPitchRezeroState::IDLE)
  {
    ROS_WARN_THROTTLE(1.0, "[Multilink] Active joints frozen during pitch rezero/HOLD; relock after ready.");
    return;
  }
  if(perchingMode() != perching_geometry::Mode::NORMAL ||
     getNaviState() != HOVER_STATE || !std::isfinite(msg->data))
    return;

  if(!multilink_model_valid_ || !multilink_lock_valid_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator/multilink] secondary target ignored "
        "without a valid multilink perching lock.");
    return;
  }

  const double target = clampMultilink(
      secondary_command_sign_ * msg->data,
      secondary_joint_lower_,
      secondary_joint_upper_);
  if(std::abs(target - secondary_joint_nominal_target_) <= 1.0e-9)
    return;

  secondary_joint_nominal_target_ = target;
  secondary_settled_ = false;
  secondary_settle_start_ = ros::Time(0);
  ++mechanism_target_generation_;
}

void GimbalrotorPerchingNavigator::updateSecondarySettledState()
{
  double pitch_position = 0.0;
  double secondary_position = 0.0;
  double pitch_velocity = 0.0;
  double secondary_velocity = 0.0;
  const bool state_valid = readCurrentMechanismState(
      pitch_position,
      secondary_position,
      pitch_velocity,
      secondary_velocity);
  const ros::Time now = ros::Time::now();

  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  if(!state_valid || !multilink_lock_valid_ ||
     getNaviState() != HOVER_STATE)
  {
    secondary_settled_ = false;
    secondary_settle_start_ = ros::Time(0);
    return;
  }

  const bool within_tolerance =
      std::abs(secondary_position - secondary_joint_nominal_target_) <=
          secondary_position_tolerance_ &&
      std::abs(secondary_velocity) <= secondary_velocity_tolerance_;

  if(!within_tolerance)
  {
    secondary_settled_ = false;
    secondary_settle_start_ = ros::Time(0);
    return;
  }

  if(secondary_settle_start_.isZero())
    secondary_settle_start_ = now;

  secondary_settled_ =
      (now - secondary_settle_start_).toSec() >= secondary_settle_duration_;
}

void GimbalrotorPerchingNavigator::updateMultilink()
{
  std::lock_guard<std::recursive_mutex> state_lock(perchingStateMutex());
  if(!multilink_model_resolution_attempted_ &&
     robot_model_ &&
     robot_model_->initialized())
  {
    multilink_model_resolution_attempted_ = true;
    multilink_model_valid_ = resolveMechanismModel();

    std_msgs::Bool valid_msg;
    valid_msg.data = multilink_model_valid_;
    model_valid_pub_.publish(valid_msg);

    if(multilink_model_valid_)
    {
      ROS_WARN(
          "[GimbalrotorPerchingNavigator/multilink] "
          "multilink model initialized after receiving robot state.");
    }
    else
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[GimbalrotorPerchingNavigator/multilink] "
          "robot model is initialized, but the multilink mechanism "
          "configuration is invalid.");
    }
  }

  if(multilink_pitch_rezero_state_ != MultilinkPitchRezeroState::IDLE &&
     (!multilinkLockValid() || perchingMode() != perching_geometry::Mode::NORMAL ||
      getNaviState() != HOVER_STATE))
    resetMultilinkPitchRezeroState();
  updateMultilinkBodyPitchFeedback();
  rezero_target_applied_this_update_ = false;
  updateCommonPerching();
  // Rezero owns a continuous hold even if the common optional fixed-contact
  // hold is disabled. Avoid integrating twice when the common update called the selected target method.
  if(multilink_pitch_rezero_state_ != MultilinkPitchRezeroState::IDLE && !rezero_target_applied_this_update_)
    applyMultilinkPitchRezeroTarget();
  updateSecondarySettledState();
  publishDiagnostics();
}

void GimbalrotorPerchingNavigator::publishMultilinkJointTarget(
    double pitch_final,
    double secondary_final)
{
  if(!multilink_model_valid_ || !multilinkLockValid() ||
     !std::isfinite(pitch_final) || !std::isfinite(secondary_final))
    return;

  sensor_msgs::JointState joint_cmd;
  joint_cmd.header.stamp = ros::Time::now();
  joint_cmd.name.push_back(pitch_joint_name_);
  joint_cmd.name.push_back(secondary_joint_name_);
  joint_cmd.position.push_back(pitch_final);
  joint_cmd.position.push_back(secondary_final);
  joint_control_pub_.publish(joint_cmd);
}

bool GimbalrotorPerchingNavigator::multilinkModelValid() const
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  return multilink_model_valid_;
}

bool GimbalrotorPerchingNavigator::multilinkLockValid() const
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  return multilink_model_valid_ && multilink_lock_valid_;
}

bool GimbalrotorPerchingNavigator::secondaryJointSettled() const
{
  double pitch_position = 0.0;
  double secondary_position = 0.0;
  double pitch_velocity = 0.0;
  double secondary_velocity = 0.0;
  if(!readCurrentMechanismState(
         pitch_position,
         secondary_position,
         pitch_velocity,
         secondary_velocity))
    return false;

  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  return multilink_model_valid_ && multilink_lock_valid_ &&
         secondary_settled_;
}

std::uint64_t
GimbalrotorPerchingNavigator::mechanismTargetGeneration() const
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  return mechanism_target_generation_;
}

void GimbalrotorPerchingNavigator::publishDiagnostics() const
{
  std::lock_guard<std::recursive_mutex> state_lock(perchingStateMutex());
  std_msgs::Bool rezero_bool;
  rezero_bool.data = multilink_pitch_rezero_state_ == MultilinkPitchRezeroState::ACTIVE;
  pitch_rezero_active_pub_.publish(rezero_bool);
  rezero_bool.data = multilink_pitch_rezero_ready_;
  pitch_rezero_ready_pub_.publish(rezero_bool);
  rezero_bool.data = multilink_pitch_rezero_failed_;
  pitch_rezero_failed_pub_.publish(rezero_bool);
  std_msgs::Float64 rezero_value;
  rezero_value.data = rezero_pitch_joint_hold_error_;
  pitch_joint_hold_error_pub_.publish(rezero_value);
  rezero_value.data = rezero_secondary_joint_hold_error_;
  secondary_joint_hold_error_pub_.publish(rezero_value);
  rezero_value.data = rezero_alpha_cmd_;
  passive_pitch_delta_pub_.publish(rezero_value);
  rezero_value.data = currentBaselinkPitch();
  body_pitch_pub_.publish(rezero_value);
  rezero_value.data = rezero_pitch_rate_valid_ ? rezero_measured_pitch_rate_ :
      std::numeric_limits<double>::quiet_NaN();
  body_pitch_rate_pub_.publish(rezero_value);

  bool settled = false;
  double pitch_measured = 0.0;
  double pitch_nominal = 0.0;
  double pitch_final = 0.0;
  double secondary_measured = 0.0;
  double secondary_target = 0.0;

  {
    std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
    settled = secondary_settled_;
    pitch_measured = measured_pitch_joint_;
    pitch_nominal = pitch_joint_nominal_target_;
    pitch_final = pitch_joint_final_target_;
    secondary_measured = measured_secondary_joint_;
    secondary_target = secondary_joint_nominal_target_;
  }

  // Do not make stale mechanism samples look fresh to diagnostic consumers.
  double pitch_velocity, secondary_velocity;
  if(!readCurrentMechanismState(pitch_measured, secondary_measured,
                                pitch_velocity, secondary_velocity))
    pitch_measured = secondary_measured = std::numeric_limits<double>::quiet_NaN();

  std_msgs::Bool bool_msg;
  std_msgs::Float64 value_msg;
  bool_msg.data = settled;
  secondary_settled_pub_.publish(bool_msg);
  value_msg.data = pitch_measured;
  pitch_measured_pub_.publish(value_msg);
  value_msg.data = pitch_nominal;
  pitch_nominal_pub_.publish(value_msg);
  value_msg.data = pitch_final;
  pitch_final_pub_.publish(value_msg);
  value_msg.data = secondary_measured;
  secondary_measured_pub_.publish(value_msg);
  value_msg.data = secondary_target;
  secondary_target_pub_.publish(value_msg);
}

void GimbalrotorPerchingNavigator::resetMultilinkPitchRezeroState()
{
  multilink_pitch_rezero_state_ = MultilinkPitchRezeroState::IDLE;
  multilink_pitch_rezero_ready_ = false;
  multilink_pitch_rezero_failed_ = false;
  rezero_frozen_pitch_joint_ = 0.0;
  rezero_frozen_secondary_joint_ = 0.0;
  rezero_pitch_joint_hold_error_ = 0.0;
  rezero_secondary_joint_hold_error_ = 0.0;
  rezero_alpha_cmd_ = 0.0;
  rezero_start_stamp_ = ros::Time(0);
  rezero_stable_start_stamp_ = ros::Time(0);
  rezero_previous_body_pitch_stamp_ = ros::Time(0);
  rezero_previous_body_pitch_ = 0.0;
  rezero_measured_pitch_rate_ = 0.0;
  multilink_rezero_feedback_dt_ = 0.0;
  rezero_pitch_rate_valid_ = false;
  rezero_pivot_world_ = KDL::Vector::Zero();
  rezero_axis_world_ = KDL::Vector::Zero();
  rezero_p_B_G_ = KDL::Vector::Zero();
  rezero_reference_body_world_ = KDL::Frame::Identity();
  rezero_last_body_target_ = KDL::Frame::Identity();
}

bool GimbalrotorPerchingNavigator::readBaselinkPose(KDL::Frame& pose) const
{
  const tf::Vector3 p = estimator_->getPos(Frame::BASELINK, estimate_mode_);
  const tf::Matrix3x3 orientation = estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
  for(unsigned int i = 0; i < 3; ++i)
    for(unsigned int j = 0; j < 3; ++j)
      if(!std::isfinite(orientation[i][j])) return false;
  tf::Quaternion q;
  orientation.getRotation(q);
  if(!perching_geometry::finite(p) || !std::isfinite(q.length2()) || q.length2() < 1.0e-12)
    return false;
  q.normalize();
  pose = KDL::Frame(KDL::Rotation::Quaternion(q.x(), q.y(), q.z(), q.w()),
                    KDL::Vector(p.x(), p.y(), p.z()));
  return frameFinite(pose);
}

double GimbalrotorPerchingNavigator::currentBaselinkPitch() const
{
  KDL::Frame pose;
  if(!readBaselinkPose(pose)) return std::numeric_limits<double>::quiet_NaN();
  double roll, pitch, yaw;
  pose.M.GetRPY(roll, pitch, yaw);
  return pitch;
}

double GimbalrotorPerchingNavigator::wrapAngle(double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

void GimbalrotorPerchingNavigator::multilinkPitchRezeroCallback(
    const std_msgs::EmptyConstPtr& msg)
{
  (void)msg;
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  double pitch, secondary, pitch_velocity, secondary_velocity;
  if(perchingMode() != perching_geometry::Mode::NORMAL || getNaviState() != HOVER_STATE ||
     !multilinkLockValid() || !pitch_rezero_config_valid_ ||
     multilink_pitch_rezero_state_ != MultilinkPitchRezeroState::IDLE ||
     !readCurrentMechanismState(pitch, secondary, pitch_velocity, secondary_velocity))
  {
    ROS_WARN_THROTTLE(1.0, "[Multilink] Rezero rejected: require NORMAL/HOVER, valid model/config/lock, fresh joints and IDLE.");
    return;
  }

  KDL::Frame T_W_B, T_B_C;
  KDL::Vector p_B_G;
  const ros::Time now = ros::Time::now();
  if(now.isZero() || !readBaselinkPose(T_W_B) ||
     !computeBaselinkToContactTransform(pitch, secondary, T_B_C) ||
     !computeBaselinkToCogVector(pitch, secondary, p_B_G))
  {
    ROS_WARN_THROTTLE(1.0, "[Multilink] Rezero rejected: invalid time, BASELINK pose or mechanism FK/CoG.");
    return;
  }
  const KDL::Frame T_W_C = T_W_B * T_B_C;
  const KDL::Vector pivot = T_W_C * pitch_rezero_pivot_offset_;
  KDL::Vector axis = T_W_C.M * pitch_rezero_axis_;
  if(!frameFinite(T_W_C) || !frameFinite(KDL::Frame(pivot)) ||
     !frameFinite(KDL::Frame(axis)) || !std::isfinite(axis.Norm()) || axis.Norm() < 1.0e-6)
  {
    ROS_WARN_THROTTLE(1.0, "[Multilink] Rezero rejected: invalid passive pivot/axis.");
    return;
  }
  axis = axis / axis.Norm();
  // Establish a valid initial hold before accepting the trigger.
  if(!commandMultilinkBaselinkPose(T_W_B, p_B_G))
  {
    ROS_WARN_THROTTLE(1.0, "[Multilink] Rezero rejected: cannot command initial body pose.");
    return;
  }
  resetMultilinkPitchRezeroState();
  rezero_frozen_pitch_joint_ = pitch;
  rezero_frozen_secondary_joint_ = secondary;
  rezero_reference_body_world_ = T_W_B;
  rezero_last_body_target_ = T_W_B;
  rezero_pivot_world_ = pivot;
  rezero_axis_world_ = axis;
  rezero_p_B_G_ = p_B_G;
  rezero_start_stamp_ = now;
  double roll, yaw;
  T_W_B.M.GetRPY(roll, rezero_previous_body_pitch_, yaw);
  rezero_previous_body_pitch_stamp_ = now;
  pitch_joint_nominal_target_ = pitch_joint_final_target_ = pitch;
  secondary_joint_nominal_target_ = secondary_joint_final_target_ = secondary;
  secondary_settled_ = false;
  secondary_settle_start_ = ros::Time(0);
  ++mechanism_target_generation_;
  multilink_pitch_rezero_state_ = MultilinkPitchRezeroState::ACTIVE;
  publishMultilinkJointTarget(pitch, secondary);
  publishDiagnostics();
  ROS_WARN("[Multilink] Pitch rezero started about external passive pivot; both active joints frozen.");
}

void GimbalrotorPerchingNavigator::updateMultilinkBodyPitchFeedback()
{
  const ros::Time now = ros::Time::now();
  const double pitch = currentBaselinkPitch();
  rezero_pitch_rate_valid_ = false;
  multilink_rezero_feedback_dt_ = 0.0;
  if(!std::isfinite(pitch) || now.isZero())
  {
    rezero_previous_body_pitch_stamp_ = ros::Time(0);
    return;
  }
  if(!rezero_previous_body_pitch_stamp_.isZero())
  {
    multilink_rezero_feedback_dt_ = (now - rezero_previous_body_pitch_stamp_).toSec();
    // Long gaps cannot count as continuously observed stabilization.
    if(std::isfinite(multilink_rezero_feedback_dt_) && multilink_rezero_feedback_dt_ > 0.0 &&
       multilink_rezero_feedback_dt_ <= 0.1)
    {
      rezero_measured_pitch_rate_ =
          wrapAngle(pitch - rezero_previous_body_pitch_) / multilink_rezero_feedback_dt_;
      rezero_pitch_rate_valid_ = std::isfinite(rezero_measured_pitch_rate_);
    }
  }
  rezero_previous_body_pitch_ = pitch;
  rezero_previous_body_pitch_stamp_ = now;
}

void GimbalrotorPerchingNavigator::failMultilinkPitchRezero(const char* reason)
{
  multilink_pitch_rezero_state_ = MultilinkPitchRezeroState::HOLD;
  multilink_pitch_rezero_ready_ = false;
  multilink_pitch_rezero_failed_ = true;
  rezero_stable_start_stamp_ = ros::Time(0);
  ROS_WARN("[Multilink] Pitch rezero failed (%s); holding last valid rezero pose and frozen joints. Disable perching and retry safely.", reason);
}

bool GimbalrotorPerchingNavigator::verifyMultilinkPitchRezeroFrozenJoints()
{
  if(multilink_pitch_rezero_state_ == MultilinkPitchRezeroState::IDLE) return true;

  double q_pitch, q_secondary, v_pitch, v_secondary;
  if(!readCurrentMechanismState(q_pitch, q_secondary, v_pitch, v_secondary))
  {
    rezero_pitch_joint_hold_error_ = std::numeric_limits<double>::quiet_NaN();
    rezero_secondary_joint_hold_error_ = std::numeric_limits<double>::quiet_NaN();
    if(!multilink_pitch_rezero_failed_)
      failMultilinkPitchRezero("active-joint state unavailable while joints must remain frozen");
    return false;
  }

  rezero_pitch_joint_hold_error_ = wrapAngle(q_pitch - rezero_frozen_pitch_joint_);
  rezero_secondary_joint_hold_error_ = wrapAngle(q_secondary - rezero_frozen_secondary_joint_);
  if(std::abs(rezero_pitch_joint_hold_error_) > pitch_rezero_pitch_joint_hold_tolerance_)
  {
    if(!multilink_pitch_rezero_failed_)
    {
      ROS_WARN("[Multilink] Pitch rezero active pitch joint hold violation: error=%.4f rad, tolerance=%.4f rad.",
               rezero_pitch_joint_hold_error_, pitch_rezero_pitch_joint_hold_tolerance_);
      failMultilinkPitchRezero("active pitch joint moved outside rezero hold tolerance");
    }
    return false;
  }
  if(std::abs(rezero_secondary_joint_hold_error_) > pitch_rezero_secondary_joint_hold_tolerance_)
  {
    if(!multilink_pitch_rezero_failed_)
    {
      ROS_WARN("[Multilink] Pitch rezero active secondary joint hold violation: error=%.4f rad, tolerance=%.4f rad.",
               rezero_secondary_joint_hold_error_, pitch_rezero_secondary_joint_hold_tolerance_);
      failMultilinkPitchRezero("active secondary joint moved outside rezero hold tolerance");
    }
    return false;
  }
  return true;
}

void GimbalrotorPerchingNavigator::updateMultilinkPitchRezeroCommand()
{
  if(multilink_pitch_rezero_state_ != MultilinkPitchRezeroState::ACTIVE) return;
  const ros::Time now = ros::Time::now();
  const double elapsed = (now - rezero_start_stamp_).toSec();
  const double pitch = currentBaselinkPitch();
  double q_pitch, q_secondary, v_pitch, v_secondary;
  if(!std::isfinite(elapsed) || elapsed < 0.0 || multilink_rezero_feedback_dt_ < 0.0 ||
     multilink_rezero_feedback_dt_ > 0.5)
    return failMultilinkPitchRezero("invalid or discontinuous time");
  if(elapsed > pitch_rezero_timeout_)
    return failMultilinkPitchRezero("timeout before stabilization");
  if(!std::isfinite(pitch) ||
     !readCurrentMechanismState(q_pitch, q_secondary, v_pitch, v_secondary))
    return failMultilinkPitchRezero("invalid body feedback or stale mechanism state");

  const double error = wrapAngle(pitch_rezero_target_pitch_ - pitch);
  if(rezero_pitch_rate_valid_ && std::abs(error) <= pitch_rezero_pitch_tolerance_ &&
     std::abs(rezero_measured_pitch_rate_) <= pitch_rezero_rate_tolerance_)
  {
    if(rezero_stable_start_stamp_.isZero()) rezero_stable_start_stamp_ = now;
    if((now - rezero_stable_start_stamp_).toSec() >= pitch_rezero_stable_duration_)
    {
      multilink_pitch_rezero_state_ = MultilinkPitchRezeroState::HOLD;
      multilink_pitch_rezero_ready_ = true;
      multilink_pitch_rezero_failed_ = false;
      ROS_WARN("[Multilink] Pitch rezero ready; body pitch is stable. Publish /perching/relock to commit this pose as the new fixed-contact lock.");
      return;
    }
  }
  else
    rezero_stable_start_stamp_ = ros::Time(0);

  if(!std::isfinite(multilink_rezero_feedback_dt_) || multilink_rezero_feedback_dt_ <= 0.0) return;
  const double dt = std::min(multilink_rezero_feedback_dt_, 0.1);
  const double rate = pitch_rezero_command_sign_ *
      clampMultilink(pitch_rezero_kp_ * error, -pitch_rezero_rate_limit_, pitch_rezero_rate_limit_);
  const double alpha = clampMultilink(rezero_alpha_cmd_ + rate * dt,
                             -pitch_rezero_max_delta_, pitch_rezero_max_delta_);
  const KDL::Rotation rotation = KDL::Rotation::Rot(rezero_axis_world_, alpha);
  const KDL::Frame target(
      rotation * rezero_reference_body_world_.M,
      rezero_pivot_world_ + rotation * (rezero_reference_body_world_.p - rezero_pivot_world_));
  if(!std::isfinite(alpha) || !frameFinite(target) ||
     !commandMultilinkBaselinkPose(target, rezero_p_B_G_))
    return failMultilinkPitchRezero("invalid revolute body target");
  // Commit only a successfully issued target; failures retain the previous pose.
  rezero_alpha_cmd_ = alpha;
  rezero_last_body_target_ = target;
}

void GimbalrotorPerchingNavigator::applyMultilinkPitchRezeroTarget()
{
  rezero_target_applied_this_update_ = true;
  // Monitor ACTIVE and both HOLD outcomes before advancing the passive pivot.
  const bool frozen_joints_valid = verifyMultilinkPitchRezeroFrozenJoints();
  if(multilink_pitch_rezero_state_ == MultilinkPitchRezeroState::ACTIVE && frozen_joints_valid)
    updateMultilinkPitchRezeroCommand();
  pitch_joint_nominal_target_ = pitch_joint_final_target_ = rezero_frozen_pitch_joint_;
  secondary_joint_nominal_target_ = secondary_joint_final_target_ = rezero_frozen_secondary_joint_;
  publishMultilinkJointTarget(rezero_frozen_pitch_joint_, rezero_frozen_secondary_joint_);
  if(!commandMultilinkBaselinkPose(rezero_last_body_target_, rezero_p_B_G_) &&
     !multilink_pitch_rezero_failed_)
    failMultilinkPitchRezero("cannot reissue last valid body pose");
}

bool GimbalrotorPerchingNavigator::frameFinite(
    const KDL::Frame& frame)
{
  if(!std::isfinite(frame.p.x()) || !std::isfinite(frame.p.y()) ||
     !std::isfinite(frame.p.z()))
    return false;

  for(unsigned int row = 0; row < 3; ++row)
    for(unsigned int column = 0; column < 3; ++column)
      if(!std::isfinite(frame.M(row, column)))
        return false;
  return true;
}

double GimbalrotorPerchingNavigator::clampMultilink(
    double value,
    double lower,
    double upper)
{
  return std::max(lower, std::min(value, upper));
}

}  // namespace aerial_robot_navigation

// Roll-yaw workflow

namespace aerial_robot_navigation
{

namespace
{
struct CallbackTimer
{
  double& elapsed;
  double started;
  ~CallbackTimer()
  {
    elapsed = roll_yaw::steadySeconds() - started;
  }
};
tf::Vector3 tfVector(const Eigen::Vector3d& v)
{
  return {v.x(), v.y(), v.z()};
}
tf::Quaternion tfQuaternion(const Eigen::Matrix3d& r)
{
  Eigen::Quaterniond q(r);
  return {q.x(), q.y(), q.z(), q.w()};
}
geometry_msgs::Pose poseMsg(const Eigen::Isometry3d& p)
{
  geometry_msgs::Pose m;
  tf::pointTFToMsg(tfVector(p.translation()), m.position);
  tf::quaternionTFToMsg(tfQuaternion(p.linear()), m.orientation);
  return m;
}
std::string frameName(std::string s)
{
  if(!s.empty() && s.front() == '/')
    s.erase(0, 1);
  return s;
}
bool readVector(ros::NodeHandle& nh, const std::string& key, Eigen::Vector3d& value, bool required = false)
{
  if(!nh.hasParam(key))
    return !required;
  std::vector<double> v;
  if(!nh.getParam(key, v) || v.size() != 3)
    return false;
  value = Eigen::Vector3d(v[0], v[1], v[2]);
  return value.allFinite();
}
}  // namespace
void GimbalrotorPerchingNavigator::initializeRollYaw(ros::NodeHandle nh, ros::NodeHandle nhp)
{
  prepare_worker_ = std::make_unique<roll_yaw::Worker<roll_yaw::Plan>>();
  rollYawRosParamInit(nh, nhp);
  ros::NodeHandle pn(nh, "navigation/roll_yaw_perching");
  std::string joint_topic, command_topic, body_topic;
  pn.param<std::string>("joint_state_topic", joint_topic, "joint_states");
  pn.param<std::string>("joint_command_topic", command_topic, "joints_ctrl");
  pn.param<std::string>("body_state_topic", body_topic, "uav/baselink/odom");
  roll_yaw_joints_pub_ = nh.advertise<sensor_msgs::JointState>(command_topic, 1);
  roll_yaw_state_pub_ = nh.advertise<std_msgs::String>(roll_yaw_prefix_ + "/state", 1, true);
  session_pub_ = nh.advertise<std_msgs::UInt64>(roll_yaw_prefix_ + "/session", 1, true);
  details_pub_ = nh.advertise<std_msgs::String>(roll_yaw_prefix_ + "/diagnostics", 1);
  status_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>(roll_yaw_prefix_ + "/diagnostics_status", 1);
  task_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(roll_yaw_prefix_ + "/task_pitch", 1);
  compliance_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(roll_yaw_prefix_ + "/compliance", 1);
  ack_pub_ = nh.advertise<std_msgs::String>(roll_yaw_prefix_ + "/command_ack", 1);
  subscriptions_.push_back(
      nh.subscribe("perching/admittance_enable", 1, &GimbalrotorPerchingNavigator::admittanceCallback, this));
  subscriptions_.push_back(
      nh.subscribe(interaction_topic_, 1, &GimbalrotorPerchingNavigator::externalWrenchCallback, this));
  subscriptions_.push_back(
      nh.subscribe(roll_yaw_prefix_ + "/admittance_tare", 1, &GimbalrotorPerchingNavigator::tareCallback, this));
  nominal_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(roll_yaw_prefix_ + "/nominal_angles", 1);
  position_lower_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(roll_yaw_prefix_ + "/position_lower", 1);
  position_upper_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(roll_yaw_prefix_ + "/position_upper", 1);
  angles_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(roll_yaw_prefix_ + "/target_angles", 1);
  rezero_active_pub_ = nh.advertise<std_msgs::Bool>(roll_yaw_prefix_ + "/pitch_rezero_active", 1, true);
  rezero_ready_pub_ = nh.advertise<std_msgs::Bool>(roll_yaw_prefix_ + "/pitch_rezero_ready", 1, true);
  rezero_failed_pub_ = nh.advertise<std_msgs::Bool>(roll_yaw_prefix_ + "/pitch_rezero_failed", 1, true);
  pitch_pub_ = nh.advertise<std_msgs::Float64>("perching/commanded_pitch_delta", 1);
  reference_pub_ = nh.advertise<nav_msgs::Odometry>(roll_yaw_prefix_ + "/body_reference", 1);
  measured_pub_ = nh.advertise<nav_msgs::Odometry>(roll_yaw_prefix_ + "/body_measured", 1);
  cog_pub_ = nh.advertise<nav_msgs::Odometry>(roll_yaw_prefix_ + "/cog_reference", 1);
  subscriptions_.push_back(nh.subscribe(joint_topic, 1, &GimbalrotorPerchingNavigator::rollYawJointCallback, this));
  subscriptions_.push_back(nh.subscribe(body_topic, 1, &GimbalrotorPerchingNavigator::bodyCallback, this));
  for(const auto* key : {"prepare", "start", "pause", "resume", "stop", "cancel"})
  {
    std::string action(key);
    subscriptions_.push_back(nh.subscribe<std_msgs::UInt64>(roll_yaw_prefix_ + "/" + action, 1,
                                                            [this, action](const std_msgs::UInt64ConstPtr& m)
                                                            { taskCommandCallback(m, action); }));
  }
  for(int axis = 0; axis < 3; ++axis)
  {
    const std::string name = axis == 0 ? "alpha_target" : axis == 1 ? "roll_target" : "yaw_target";
    subscriptions_.push_back(nh.subscribe<std_msgs::Float64>(roll_yaw_prefix_ + "/" + name, 1,
                                                             [this, axis](const std_msgs::Float64ConstPtr& msg)
                                                             { mechanismTargetCallback(msg, axis); }));
  }
  for(int axis = 0; axis < 2; ++axis)
  {
    subscriptions_.push_back(nh.subscribe<std_msgs::Float64>(
        roll_yaw_prefix_ + (axis == 0 ? "/body_yaw_target" : "/body_roll_target"), 1,
        [this, axis](const std_msgs::Float64ConstPtr& msg) { bodyAlignmentCallback(msg, axis); }));
  }
  subscriptions_.push_back(
      nh.subscribe(roll_yaw_prefix_ + "/body_pitch_delta", 1, &GimbalrotorPerchingNavigator::bodyPitchCallback, this));
  subscriptions_.push_back(
      nh.subscribe(roll_yaw_prefix_ + "/target", 1, &GimbalrotorPerchingNavigator::manualCallback, this));
  subscriptions_.push_back(
      nh.subscribe(roll_yaw_prefix_ + "/pitch_rezero", 1, &GimbalrotorPerchingNavigator::rollYawPitchRezeroCallback, this));
  if(!planning_configuration_valid_)
    ROS_WARN("[RollYaw] Invalid offline configuration; manual navigation remains usable.");
  if(!configuration_valid_)
    ROS_ERROR("[RollYaw] Lock disabled: require valid hinge geometry, limits and ordinary fully actuated controller.");
  publishRollYawDiagnostics();
}
void GimbalrotorPerchingNavigator::rollYawRosParamInit(ros::NodeHandle nh, ros::NodeHandle nhp)
{
  ros::NodeHandle pn(nh, "navigation/roll_yaw_perching");
  pn.param<std::string>("roll_joint", roll_, "joint_roll");
  pn.param<std::string>("yaw_joint", yaw_, "joint_yaw");
  pn.param<std::string>("contact_link", contact_, "roll_hand_link");
  pn.param<std::string>("debug_prefix", roll_yaw_prefix_, "perching/roll_yaw");
  pn.param<std::string>("world_frame", world_, "world");
  std::string tf_prefix;
  nhp.param<std::string>("tf_prefix", tf_prefix, "");
  pn.param<std::string>("body_frame", body_frame_, tf::resolve(tf_prefix, robot_model_->getBaselinkName()));
  Eigen::Vector3d xyz = Eigen::Vector3d::Zero(), rpy = Eigen::Vector3d::Zero();
  configuration_valid_ = readVector(pn, "pivot_offset", xyz, true) && readVector(pn, "pivot_rpy", rpy, true) &&
                         readVector(pn, "hinge_axis", axis_, true) && std::abs(axis_.norm() - 1) < 1e-7;
  cp_.translation() = xyz;
  cp_.linear() = roll_yaw::expSO3(Eigen::Vector3d::UnitZ() * rpy.z()) * roll_yaw::expSO3(Eigen::Vector3d::UnitY() * rpy.y()) *
                 roll_yaw::expSO3(Eigen::Vector3d::UnitX() * rpy.x());
  auto& p = planning_config_;
  auto& c = feedback_config_;
  pn.param("body_pitch_jog_max_delta", body_pitch_jog_max_delta_, body_pitch_jog_max_delta_);
  pn.param("prepare_deadline", prepare_deadline_, prepare_deadline_);
  pn.param("max_execution_step", c.max_execution_step, c.max_execution_step);
  pn.param("soft_tracking_fraction", c.soft_tracking_fraction, c.soft_tracking_fraction);
  pn.param("resume_fraction", c.resume_fraction, c.resume_fraction);
  pn.param("preparation_tolerance", c.preparation_tolerance, c.preparation_tolerance);
  pn.param("preparation_quiet_time", c.preparation_quiet_time, c.preparation_quiet_time);
  pn.param("reference_lead", c.reference_lead, c.reference_lead);
  pn.param("progress_filter_time", c.progress_filter_time, c.progress_filter_time);
  pn.param("goal_tolerance", c.goal_tolerance, c.goal_tolerance);
  pn.param("progress_timeout", c.progress_timeout, c.progress_timeout);
  ros::NodeHandle an(pn, "admittance");
  auto& a = admittance_.config;
  an.param("inertia", a.inertia, a.inertia);
  an.param("damping", a.damping, a.damping);
  an.param("stiffness", a.stiffness, a.stiffness);
  an.param("torque_limit", a.torque_limit, a.torque_limit);
  an.param("angle_limit", a.angle_limit, a.angle_limit);
  an.param("rate_limit", a.rate_limit, a.rate_limit);
  an.param("acceleration_limit", a.acceleration_limit, a.acceleration_limit);
  an.param("reference_torque", a.reference_torque, a.reference_torque);
  an.param("filter_time", a.filter_time, a.filter_time);
  an.param("on_threshold", a.on_threshold, a.on_threshold);
  an.param("off_threshold", a.off_threshold, a.off_threshold);
  an.param("on_dwell", a.on_dwell, a.on_dwell);
  an.param("off_dwell", a.off_dwell, a.off_dwell);
  an.param("baseline_samples", a.baseline_samples, a.baseline_samples);
  an.param<std::string>("external_wrench_topic", interaction_topic_, "estimated_external_wrench");
  an.param<std::string>("wrench_frame", interaction_frame_, "world");
  an.param<std::string>("wrench_point", interaction_point_, "cog");
  an.param("wrench_sign", interaction_sign_, interaction_topic_ == "estimated_external_wrench" ? 1.0 : 0.0);
  an.param("wrench_scale", interaction_scale_, interaction_topic_ == "estimated_external_wrench" ? 1.0 : 0.0);
  interaction_configured_ =
      a.valid() && (interaction_sign_ == 1 || interaction_sign_ == -1) && std::isfinite(interaction_scale_) &&
      interaction_scale_ > 0 && (interaction_frame_ == "world" || interaction_frame_ == "body") &&
      (interaction_point_ == "cog" || interaction_point_ == "body" || interaction_point_ == "world_origin");
  const bool vectors =
      readVector(pn, "position_lower", p.limits.lower) && readVector(pn, "position_upper", p.limits.upper);
  // Shared mechanism and trajectory bounds.
  const bool rate_vectors =
      readVector(pn, "velocity_caps", p.limits.velocity) && readVector(pn, "acceleration_caps", p.limits.acceleration);
  pn.param("planning_dt", p.dt, p.dt);
  pn.param("body_max_tilt", p.max_tilt, p.max_tilt);
  pn.param("orientation_tolerance", p.orientation_tolerance, p.orientation_tolerance);
  pn.param("rate_tolerance", p.rate_tolerance, p.rate_tolerance);
  pn.param("alignment_rate", p.alignment_rate, p.alignment_rate);
  pn.param("alignment_acceleration", p.alignment_acceleration, p.alignment_acceleration);
  pn.param("max_phase_duration", p.max_duration, p.max_duration);
  pn.param("scaling_attempts", p.scaling_attempts, p.scaling_attempts);
  pn.param("cog_speed_limit", p.cog_speed_limit, p.cog_speed_limit);
  pn.param("cog_acceleration_limit", p.cog_acceleration_limit, p.cog_acceleration_limit);
  pn.param("min_singular_value", p.min_singular_value, p.min_singular_value);
  pn.param("feedback_timeout", c.timeout, c.timeout);
  pn.param("future_tolerance", c.future_tolerance, c.future_tolerance);
  pn.param("pivot_tolerance", c.pivot_tolerance, c.pivot_tolerance);
  pn.param("hinge_residual_tolerance", c.hinge_tolerance, c.hinge_tolerance);
  pn.param("tracking_tolerance", c.tracking_tolerance, c.tracking_tolerance);
  pn.param("body_tracking_tolerance", c.orientation_tolerance, c.orientation_tolerance);
  pn.param("settle_rate", c.settle_rate, c.settle_rate);
  pn.param("settle_time", c.settle_time, c.settle_time);
  pn.param("settle_timeout", c.settle_timeout, c.settle_timeout);
  pn.param("settle_use_window", c.settle_use_window, c.settle_use_window);
  pn.param("settle_body_rate", c.settle_body_rate, c.settle_body_rate);
  pn.param("settle_joint_rate", c.settle_joint_rate, c.settle_joint_rate);
  pn.param("settle_mean_rate", c.settle_mean_rate, c.settle_mean_rate);
  c.require_settled_on_enable = false;  // Same measured-pose capture policy as multilink.
  pn.param("feedback_sync_tolerance", synchronization_tolerance_, synchronization_tolerance_);
  c.synchronization_tolerance = synchronization_tolerance_;
  pn.param("cog_model_tolerance", cog_model_tolerance_, cog_model_tolerance_);
  const bool jog_bound_valid = std::isfinite(body_pitch_jog_max_delta_) && body_pitch_jog_max_delta_ > 0 &&
                               body_pitch_jog_max_delta_ <= M_PI / 180 + 1e-12;
  if(!jog_bound_valid)
    ROS_ERROR("[RollYaw] invalid body_pitch_jog_max_delta: local pitch disabled");
  planning_configuration_valid_ = rate_vectors && p.valid() && c.governorValid(p) && c.settlingValid() &&
                                  std::isfinite(prepare_deadline_) && prepare_deadline_ > 0 && prepare_deadline_ <= 300;
  configuration_valid_ = configuration_valid_ && roll_yaw::poseValid(cp_) && vectors && rate_vectors && p.limits.valid() &&
                         p.mechanismValid() && c.feedbackConfigValid() && std::isfinite(synchronization_tolerance_) &&
                         synchronization_tolerance_ > 0 && std::isfinite(cog_model_tolerance_) &&
                         cog_model_tolerance_ > 0;
  pn.param("pitch_rezero_target_pitch", rezero_pitch_, rezero_pitch_);
  pn.param("pitch_rezero_kp", rezero_kp_, rezero_kp_);
  pn.param("pitch_rezero_command_sign", rezero_sign_, rezero_sign_);
  pn.param("pitch_rezero_rate_limit", rezero_rate_, rezero_rate_);
  pn.param("pitch_rezero_max_delta", rezero_travel_, rezero_travel_);
  pn.param("pitch_rezero_pitch_tolerance", rezero_tolerance_, rezero_tolerance_);
  pn.param("pitch_rezero_rate_tolerance", rezero_rate_tolerance_, rezero_rate_tolerance_);
  pn.param("pitch_rezero_stable_duration", rezero_stable_duration_, rezero_stable_duration_);
  pn.param("pitch_rezero_timeout", rezero_timeout_, rezero_timeout_);
  pn.param("pitch_rezero_roll_joint_hold_tolerance", rezero_roll_tolerance_, rezero_roll_tolerance_);
  pn.param("pitch_rezero_yaw_joint_hold_tolerance", rezero_yaw_tolerance_, rezero_yaw_tolerance_);
  rezero_configuration_valid_ = std::isfinite(rezero_pitch_) && std::abs(rezero_pitch_) < M_PI / 2;
  for(double value : {rezero_kp_, rezero_rate_, rezero_travel_, rezero_tolerance_, rezero_rate_tolerance_,
                      rezero_stable_duration_, rezero_timeout_, rezero_roll_tolerance_, rezero_yaw_tolerance_})
    rezero_configuration_valid_ = rezero_configuration_valid_ && std::isfinite(value) && value > 0;
  rezero_configuration_valid_ = rezero_configuration_valid_ && rezero_travel_ <= M_PI &&
                                rezero_timeout_ >= rezero_stable_duration_ && rezero_roll_tolerance_ <= .5 &&
                                rezero_yaw_tolerance_ <= .5 && std::isfinite(rezero_sign_) && rezero_sign_ != 0;
  if(rezero_configuration_valid_)
    rezero_sign_ = rezero_sign_ < 0 ? -1 : 1;
  bool underactuated = false;
  nh.param("controller/underactuate", underactuated, false);
  std::string controller;
  nh.getParam("aerial_robot_control_name", controller);
  if(underactuated || controller != "aerial_robot_control/gimbalrotor_controller")
    configuration_valid_ = false;
}

bool GimbalrotorPerchingNavigator::snapshotModel(roll_yaw::Kinematics& out, std::string& error)
{
  if(!configuration_valid_ || !robot_model_ || !robot_model_->initialized())
  {
    error = "model/configuration unavailable or invalid";
    return false;
  }
  const auto indices = robot_model_->getJointIndexMap();
  if(indices.count(roll_) != 1 || indices.count(yaw_) != 1)
  {
    error = "missing roll/yaw model joints";
    return false;
  }
  const unsigned ri = indices.at(roll_), yi = indices.at(yaw_);
  auto joints = robot_model_->getJointPositions();
  if(ri >= joints.rows() || yi >= joints.rows() || !joints.data.allFinite())
  {
    error = "invalid model joint positions/indices";
    return false;
  }
  for(int i = 1; i < 3; ++i)
  {
    const auto joint = robot_model_->getUrdfModel().getJoint(i == 1 ? roll_ : yaw_);
    if(!joint || joint->type != urdf::Joint::REVOLUTE || joint->mimic || !joint->limits ||
       !std::isfinite(joint->limits->lower) || !std::isfinite(joint->limits->upper) ||
       !std::isfinite(joint->limits->velocity) || joint->limits->velocity <= 0)
    {
      error = "expected bounded URDF roll/yaw revolutes";
      return false;
    }
    Eigen::Vector3d axis(joint->axis.x, joint->axis.y, joint->axis.z);
    if(!axis.allFinite() || axis.norm() < 1e-9 ||
       std::abs(axis.normalized().dot(i == 1 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitZ())) < .99)
    {
      error = "unexpected URDF roll/yaw axes";
      return false;
    }
    planning_config_.limits.lower[i] = std::max(planning_config_.limits.lower[i], joint->limits->lower);
    planning_config_.limits.upper[i] = std::min(planning_config_.limits.upper[i], joint->limits->upper);
    planning_config_.limits.velocity[i] = std::min(planning_config_.limits.velocity[i], joint->limits->velocity);
  }
  if(!planning_config_.mechanismValid())
  {
    error = "empty URDF/configured limit intersection";
    return false;
  }
  const auto tree = robot_model_->getTree();
  const auto inertia = robot_model_->getInertiaMap();
  const auto extra = robot_model_->getExtraModuleMap();
  const auto base = robot_model_->getBaselinkName();
  const auto relative = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  KDL::Chain chain;
  if(!tree.getChain(base, contact_, chain))
  {
    error = "missing baselink-to-contact chain";
    return false;
  }
  planning_config_.saw_point_valid = false;
  KDL::Chain saw_chain;
  if(tree.getChain(base, "saw", saw_chain) && saw_chain.getNrOfJoints() == 0)
  {
    KDL::TreeFkSolverPos_recursive fk(tree);
    KDL::Frame b, saw;
    if(fk.JntToCart(joints, b, base) >= 0 && fk.JntToCart(joints, saw, "saw") >= 0)
    {
      planning_config_.saw_point = roll_yaw::fromKdl(b.Inverse() * saw).translation();
      planning_config_.saw_point_valid = planning_config_.saw_point.allFinite();
    }
  }
  // Snapshot all model data before launching the planning worker. No mutation of
  // the live robot model and no ROS access from the worker.
  auto cog = roll_yaw::articulatedCogModel(tree, joints, inertia, extra, base, relative, ri, yi);
  return out.configure(chain, roll_, yaw_, cp_, axis_, cog, error);
}
roll_yaw::Feedback GimbalrotorPerchingNavigator::feedback() const
{
  roll_yaw::Feedback s = measured_;
  s.now = ros::Time::now().toSec();
  s.hover = navi_state_ == HOVER_STATE && !force_landing_flag_;
  s.invalid_reason.clear();
  if(!joint_valid_)
    s.invalid_reason = joint_failure_;
  else if(!body_valid_)
    s.invalid_reason = body_failure_;
  else if(std::abs(s.joint_stamp - s.body_stamp) > synchronization_tolerance_)
  {
    std::ostringstream why;
    why << "joint/body feedback skew=" << std::abs(s.joint_stamp - s.body_stamp) << " s exceeds "
        << synchronization_tolerance_ << " s";
    s.invalid_reason = why.str();
  }
  if(estimator_)
  {
    for(auto axis : {State::X_BASE, State::Y_BASE, State::Z_BASE})
      if(!estimator_->getStateStatus(axis, estimate_mode_) && s.invalid_reason.empty())
        s.invalid_reason = "baselink position estimate unavailable";
  }
  else
    s.invalid_reason = "estimator unavailable";
  s.valid = s.invalid_reason.empty();
  return s;
}
void GimbalrotorPerchingNavigator::rollYawJointCallback(const sensor_msgs::JointStateConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const double source = m->header.stamp.toSec(), receipt = ros::Time::now().toSec();
  // Other publishers may send gimbal-only messages on this shared topic.
  if(std::find(m->name.begin(), m->name.end(), roll_) == m->name.end() &&
     std::find(m->name.begin(), m->name.end(), yaw_) == m->name.end())
    return;
  Eigen::Vector3d q = measured_.eta, rate = Eigen::Vector3d::Zero();
  const double now = receipt, stamp = source;
  const double previous = std::max(measured_.joint_stamp, joint_bootstrap_stamp_);
  if(now < previous - feedback_config_.future_tolerance)
  {
    measured_.joint_stamp = 0;
    joint_valid_ = false;
    joint_bootstrap_stamp_ = 0;
    if(lock_valid_)
      faultHold("joint feedback clock moved backwards; recapture");
  }
  if(stamp > 0 && stamp <= std::max(measured_.joint_stamp, joint_bootstrap_stamp_))
  {
    ++joint_ignored_stamps_;
    joint_failure_ = "duplicate/out-of-order joint stamp";
    return;
  }
  ++joint_invalid_messages_;
  last_joint_message_stamp_ = stamp;
  joint_failure_ = "invalid/stale/future joint timestamp";
  if(stamp <= 0 || now - stamp > feedback_config_.timeout || now - stamp < -feedback_config_.future_tolerance)
    return;
  joint_failure_ = "missing/duplicate joint name or invalid joint arrays";
  if(m->position.size() != m->name.size() || (!m->velocity.empty() && m->velocity.size() != m->name.size()))
    return;
  for(int i = 1; i < 3; ++i)
  {
    const auto& name = i == 1 ? roll_ : yaw_;
    if(std::count(m->name.begin(), m->name.end(), name) != 1)
      return;
    const auto index = std::distance(m->name.begin(), std::find(m->name.begin(), m->name.end(), name));
    if(!std::isfinite(m->position[index]))
      return;
    q[i] = m->position[index];
    if(!m->velocity.empty())
    {
      if(!std::isfinite(m->velocity[index]))
        return;
      rate[i] = m->velocity[index];
    }
  }
  if(m->velocity.empty())
  {
    const double dt = stamp - joint_bootstrap_stamp_;
    if(joint_bootstrap_stamp_ > 0 && dt > 1e-4 && dt <= feedback_config_.timeout)
      rate.tail<2>() = (q - joint_bootstrap_).tail<2>() / dt;
    else
    {
      // Bootstrap is not a complete control sample and never refreshes one.
      joint_bootstrap_ = q;
      joint_bootstrap_stamp_ = stamp;
      joint_failure_ = "joint rate bootstrap: need two advancing fresh samples";
      return;
    }
    if(!rate.allFinite())
    {
      joint_failure_ = "nonfinite derived joint rates";
      return;
    }
  }
  joint_bootstrap_ = q;
  joint_bootstrap_stamp_ = stamp;
  joint_receipt_ = now;
  measured_.eta = q;
  measured_.joint_rate = rate;
  measured_.joint_stamp = stamp;
  joint_valid_ = true;
  --joint_invalid_messages_;
  joint_failure_.clear();
}
void GimbalrotorPerchingNavigator::bodyCallback(const nav_msgs::OdometryConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const double stamp = m->header.stamp.toSec(), now = ros::Time::now().toSec();
  // The estimator publishes on a timer using the latest IMU measurement stamp;
  // consecutive odometry publications can legitimately share that stamp.
  // Keep the accepted sample, never replace its stamp with receipt time.
  if(now < measured_.body_stamp - feedback_config_.future_tolerance)
  {
    measured_.body_stamp = 0;
    body_valid_ = false;
    if(lock_valid_)
      faultHold("body feedback clock moved backwards; recapture");
  }
  if(stamp > 0 && stamp <= measured_.body_stamp)
  {
    ++body_ignored_stamps_;
    body_failure_ = "duplicate/out-of-order body stamp";
    return;
  }
  ++body_invalid_messages_;
  last_body_message_stamp_ = stamp;
  body_failure_ = "invalid/stale/future body timestamp";
  if(stamp <= 0 || now - stamp > feedback_config_.timeout || now - stamp < -feedback_config_.future_tolerance)
    return;
  body_failure_ = "body odometry world/child frame mismatch";
  if(frameName(m->header.frame_id) != frameName(world_) || frameName(m->child_frame_id) != frameName(body_frame_))
    return;
  body_failure_ = "invalid body quaternion";
  const auto& p = m->pose.pose;
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if(!q.coeffs().allFinite() || std::abs(q.norm() - 1) > 1e-3)
    return;
  Eigen::Isometry3d body = Eigen::Isometry3d::Identity();
  body.linear() = q.normalized().toRotationMatrix();
  body.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  const auto& w = m->twist.twist.angular;
  const Eigen::Vector3d omega(w.x, w.y, w.z);
  body_failure_ = "invalid body position/angular rate";
  if(!roll_yaw::poseValid(body) || !omega.allFinite())
    return;
  measured_.body = body;
  measured_.body_omega = omega;
  body_receipt_ = now;
  measured_.body_stamp = stamp;
  body_valid_ = true;
  --body_invalid_messages_;
  body_failure_.clear();
}
bool GimbalrotorPerchingNavigator::ownsRollYawTargets() const
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  return perchingSession().mode != perching_geometry::Mode::DISABLED;
}
void GimbalrotorPerchingNavigator::rollYawTargetConflict(const char* source)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  reason_ = std::string("unsupported free-flight target while perched: ") + source;
  ROS_WARN_STREAM_THROTTLE(1.0, "[RollYaw] " << reason_);
}
void GimbalrotorPerchingNavigator::rollYawNaviCallback(const aerial_robot_msgs::FlightNavConstPtr& m)
{
  const double started = roll_yaw::steadySeconds();
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  CallbackTimer timing{manual_wall_, started};
  if(!ownsRollYawTargets())
  {
    GimbalrotorNavigator::naviCallback(m);
    return;
  }
  if(m->pos_xy_nav_mode == aerial_robot_msgs::FlightNav::NO_NAVIGATION &&
     m->pos_z_nav_mode == aerial_robot_msgs::FlightNav::NO_NAVIGATION &&
     m->roll_nav_mode == aerial_robot_msgs::FlightNav::NO_NAVIGATION &&
     m->yaw_nav_mode == aerial_robot_msgs::FlightNav::NO_NAVIGATION &&
     m->pitch_nav_mode == aerial_robot_msgs::FlightNav::POS_MODE)
    applyRollYawPitchDelta(m->target_pitch);
  else
    rollYawTargetConflict(
        "uav/nav accepts lock-relative alpha only; use body_yaw_target/body_roll_target for body alignment");
}
void GimbalrotorPerchingNavigator::applyRollYawPitchDelta(double alpha)
{
  const double started = roll_yaw::steadySeconds();
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  CallbackTimer timing{manual_wall_, started};
  Eigen::Vector3d target = nominal_;
  target[0] = alpha;
  setNominalTarget(target, 0);
}
bool GimbalrotorPerchingNavigator::tryLockRollYaw(const std::string& cause)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const auto f = feedback();
  std::string error = roll_yaw::feedbackRejection(f, feedback_config_, planning_config_);
  roll_yaw::Kinematics model;
  roll_yaw::Reference held;
  Eigen::Vector3d q = f.eta;
  q[0] = 0;
  if(perchingSession().mode != perching_geometry::Mode::NORMAL || !error.empty() || !snapshotModel(model, error) ||
     !model.lock(f.body, q) || !roll_yaw::mechanismAllowed(model, q, planning_config_, error) ||
     !roll_yaw::mechanismReference(model, q, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), held))
  {
    reason_ = "contact capture failed: " + (error.empty() ? "invalid HOVER/model/FK snapshot" : error);
    if(!previous_failure_.empty())
      reason_ += "; previous failure: " + previous_failure_;
    ROS_WARN_STREAM("[RollYaw] " << reason_);
    publishRollYawDiagnostics();
    return false;
  }
  finishPitchJog("cancelled", "contact reset/relock");
  jog_ = PitchJog();
  invalidatePlanner();
  resetRollYawPitchRezeroState();
  stop_pending_ = false;
  preparation_window_.reset();
  preparation_stats_ = roll_yaw::SettlingStatistics();
  preparation_ready_ = false;
  preparation_clock_ = 0;
  manual_path_ = roll_yaw::Path();
  model_ = std::move(model);
  nominal_ = q;
  final_target_ = held;
  body_alignment_valid_ = false;
  lock_valid_ = true;
  faulted_ = false;
  feedback_waiting_ = false;
  previous_failure_.clear();
  neutral_ = held.body.linear();
  trajectory_mode_ = false;
  traj_generator_ptr_.reset();
  gps_waypoint_ = vel_based_waypoint_ = force_att_control_flag_ = false;
  commitFixedContactLock(tfVector(held.cog.translation()), tfQuaternion(held.cog.linear()),
                         tfVector(model_.pivotLock().translation()));
  roll_yaw_lock_stamp_ = perchingSession().lock_stamp;
  resetCompliance();
  planning_model_ = std::make_shared<roll_yaw::Kinematics>(model_);
  reason_ = "measured contact captured by " + cause + "; independent nominal targets initialized";
  publishRollYawDiagnostics();
  return true;
}
void GimbalrotorPerchingNavigator::resetRollYawLock()
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(faulted_ || roll_yaw_pitch_rezero_failed_)
  {
    if(previous_failure_.empty())
      previous_failure_ = reason_;
    ROS_WARN_STREAM("[RollYaw] Resetting reference after: " << previous_failure_);
  }
  resetCommonPerchingLock();
  finishPitchJog("cancelled", "contact reset/relock");
  jog_ = PitchJog();
  invalidatePlanner();
  resetRollYawPitchRezeroState();
  lock_valid_ = false;
  faulted_ = false;
  feedback_waiting_ = false;
  roll_yaw_lock_stamp_ = ros::Time();
  resetCompliance();
  manual_path_ = roll_yaw::Path();
  stop_pending_ = false;
  preparation_window_.reset();
  preparation_stats_ = roll_yaw::SettlingStatistics();
  preparation_ready_ = false;
  preparation_clock_ = 0;
  body_alignment_valid_ = false;
  nominal_.setZero();  // Never publish these reset values; capture must succeed first.
  manual_receipt_stamp_ = ros::Time();
  // Base enable performs another reset on a rejected capture. Keep its cause.
  if(reason_.find("contact capture failed:") != 0)
  {
    if(previous_failure_.empty())
      reason_ = "perching reference reset";
    else
      reason_ = "perching reference reset; previous failure: " + previous_failure_;
  }
}
void GimbalrotorPerchingNavigator::invalidatePlanner()
{
  if(prepare_pending_)
    ++prepare_cancellations_;
  ++producer_generation_;
  prepare_worker_->cancel();
  prepare_pending_ = false;
  connection_pending_ = false;
  planner_.reset();  // Late workers hold only immutable model snapshots.
}
void GimbalrotorPerchingNavigator::stopMotion(const std::string& why)
{
  body_alignment_valid_ = false;
  if(feedback_waiting_ || faulted_)
  {
    completeStop(why);
    return;
  }
  if(prepare_pending_)
  {
    ++prepare_cancellations_;
    ++producer_generation_;
    prepare_worker_->cancel();
    prepare_pending_ = false;
  }
  if(planner_ && (planner_->stage == roll_yaw::Stage::EXECUTING || planner_->stage == roll_yaw::Stage::PRE_ALIGN))
  {
    planner_->command("cancel", roll_yaw_lock_stamp_.toNSec());
    stop_pending_ = true;
    reason_ = why + "; STOPPING, retry takeover after stop complete";
    return;
  }
  if(!manual_path_.knots.empty())
  {
    stop_pending_ = true;
    reason_ = why + "; STOPPING along applied manual path";
    return;
  }
  completeStop(why);
}
void GimbalrotorPerchingNavigator::completeStop(const std::string& why)
{
  // Preserve the actual applied positions and contact identity, never recapture
  // from stale observations. All producers and compliance end before pending clears.
  finishPitchJog("cancelled", why);
  invalidatePlanner();
  manual_path_ = roll_yaw::Path();
  manual_phase_ = 0;
  manual_speed_ = 1;
  nominal_ = final_target_.eta;
  body_alignment_valid_ = false;
  roll_yaw::scaleReference(final_target_, 0, 0);
  resetCompliance();
  admittance_reason_ = "disabled: stopped; prepare and tare again";
  stop_pending_ = false;
  reason_ = why + "; stop complete, holding contact";
}
void GimbalrotorPerchingNavigator::faultHold(const std::string& why)
{
  if(!faulted_)
  {
    finishPitchJog("failed", why);
    manual_path_ = roll_yaw::Path();
    nominal_ = final_target_.eta;
    connection_pending_ = false;
    resetCompliance();
    if(prepare_pending_)
      ++prepare_cancellations_;
    ++producer_generation_;
    prepare_worker_->cancel();
    prepare_pending_ = false;
    body_alignment_valid_ = false;
    stop_pending_ = false;
    if(planner_)
    {
      planner_->fault(why);
      planner_->plan = roll_yaw::Plan();
    }
    roll_yaw::Reference held;
    if(lock_valid_ &&
       roll_yaw::mechanismReference(model_, final_target_.eta, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), held))
      final_target_ = held;
    admittance_.enabled = false;
    previous_failure_ = reason_ = why;
  }
  preparation_window_.reset();
  preparation_stats_ = roll_yaw::SettlingStatistics();
  preparation_ready_ = false;
  preparation_clock_ = 0;
  faulted_ = true;
  if(roll_yaw_pitch_rezero_state_ != RollYawPitchRezeroState::IDLE && !roll_yaw_pitch_rezero_failed_)
    failRollYawPitchRezero(why);
}
std::string GimbalrotorPerchingNavigator::contactRejection(const roll_yaw::Feedback& f) const
{
  auto why = roll_yaw::feedbackRejection(f, feedback_config_, planning_config_);
  if(!why.empty())
    return why;
  roll_yaw::Estimate contact;
  if(!model_.estimate(f.body, f.eta, contact))
    return "invalid passive hinge estimate";
  Eigen::Vector3d q = f.eta;
  q[0] = contact.alpha;
  if((q.array() < planning_config_.limits.lower.array()).any() ||
     (q.array() > planning_config_.limits.upper.array()).any() ||
     f.body.linear()(2, 2) < std::cos(planning_config_.max_tilt))
    return "measured mechanism/body outside operating limits";
  if(contact.pivot_error > feedback_config_.pivot_tolerance ||
     contact.hinge_residual > feedback_config_.hinge_tolerance)
  {
    std::ostringstream why;
    why << "pivot=" << contact.pivot_error << "/" << feedback_config_.pivot_tolerance
        << " m; hinge=" << contact.hinge_residual << "/" << feedback_config_.hinge_tolerance << " rad";
    return why.str();
  }
  if(!roll_yaw::mechanismAllowed(model_, q, planning_config_, why))
    return "measured operating limits: " + why;
  return "";
}
void GimbalrotorPerchingNavigator::mechanismTargetCallback(const std_msgs::Float64ConstPtr& msg, int axis)
{
  const double started = roll_yaw::steadySeconds();
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  CallbackTimer timing{manual_wall_, started};
  Eigen::Vector3d target = nominal_;
  target[axis] = msg->data;  // Untouched destinations survive axis changes.
  setNominalTarget(target, axis);
}
void GimbalrotorPerchingNavigator::bodyAlignmentCallback(const std_msgs::Float64ConstPtr& msg, int axis)
{
  std::uint64_t session;
  {
    std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
    session = roll_yaw_lock_stamp_.toNSec();
  }
  requestRollYaw("body_align", session, axis, msg->data);
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  std_msgs::String ack;
  ack.data = std::string("action=") + (axis == 0 ? "body_yaw_target" : "body_roll_target") +
             " session=" + std::to_string(roll_yaw_lock_stamp_.toNSec()) +
             " accepted=" + (last_command_accepted_ ? "true" : "false") + " reason=" + reason_;
  ack_pub_.publish(ack);
  last_diagnostics_ = 0;
  publishRollYawDiagnostics();
}
void GimbalrotorPerchingNavigator::bodyPitchCallback(const std_msgs::Float64ConstPtr& msg)
{
  const double started = roll_yaw::steadySeconds();
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  CallbackTimer timing{manual_wall_, started};
  last_command_accepted_ = admitPitchJog(msg->data);
  jog_admission_wall_ = roll_yaw::steadySeconds() - started;
  std_msgs::String ack;
  ack.data = "action=body_pitch_delta session=" + std::to_string(roll_yaw_lock_stamp_.toNSec()) +
             " accepted=" + (last_command_accepted_ ? "true" : "false") +
             " phase=" + (last_command_accepted_ ? (msg->data == 0 ? "noop" : "moving") : "rejected") +
             " route=manual_endpoint_ik delta_rad=" + std::to_string(msg->data) +
             " admission_wall_s=" + std::to_string(jog_admission_wall_) + " reason=" + reason_;
  ack_pub_.publish(ack);
  last_diagnostics_ = 0;
  publishRollYawDiagnostics();
}
bool GimbalrotorPerchingNavigator::admitPitchJog(double delta)
{
  auto reject = [&](const std::string& why)
  {
    reason_ = "body-local pitch rejected: " + why;
    return false;
  };
  if(!std::isfinite(delta))
    return reject("increment must be finite radians");
  if(!std::isfinite(body_pitch_jog_max_delta_) || body_pitch_jog_max_delta_ <= 0 ||
     body_pitch_jog_max_delta_ > M_PI / 180 + 1e-12)
    return reject("invalid body_pitch_jog_max_delta (positive, at most one degree)");
  if(std::abs(delta) > body_pitch_jog_max_delta_)
    return reject("increment exceeds body_pitch_jog_max_delta; use explicit alignment/full request");
  if(delta == 0)
  {
    reason_ = "body-local pitch zero increment: no-op";
    return true;
  }
  const auto f = feedback();
  if(!configuration_valid_ || !planning_config_.valid() || !feedback_config_.valid() || !lock_valid_ || faulted_ ||
     !f.hover)
    return reject("require valid enabled HOVER contact/configuration");
  if(roll_yaw_pitch_rezero_state_ != RollYawPitchRezeroState::IDLE)
    return reject("pitch rezero owns reference");
  if(stop_pending_ || jog_.active || prepare_pending_ || connection_pending_ || !manual_path_.knots.empty() ||
     (planner_ && (planner_->stage == roll_yaw::Stage::PLANNING || planner_->stage == roll_yaw::Stage::PRE_ALIGN ||
                   planner_->stage == roll_yaw::Stage::EXECUTING || planner_->stage == roll_yaw::Stage::PAUSED || planner_->stopping())))
    return reject("busy; finish or cancel current motion, then resend (not queued)");
  std::string error = contactRejection(f);
  if(!error.empty())
    return reject(error);
  const roll_yaw::Reference from = final_target_;
  if(!roll_yaw::referenceAllowed(from, planning_config_, error))
    return reject("held reference invalid: " + error);
  if(from.velocity.norm() > 1e-6 || from.acceleration.norm() > 1e-6 || from.body_omega.norm() > 1e-6 ||
     from.body_acceleration.norm() > 1e-6 || from.cog_velocity.norm() > 1e-6 || from.cog_acceleration.norm() > 1e-6 ||
     from.cog_omega.norm() > 1e-6 || from.cog_angular_acceleration.norm() > 1e-6)
    return reject("applied reference is not held");
  roll_yaw::Estimate estimate;
  if(!model_.estimate(f.body, f.eta, estimate))
    return reject("passive estimate unavailable");
  Eigen::Vector3d measured = f.eta;
  measured[0] = estimate.alpha;
  Eigen::Vector3d goal;
  const Eigen::Matrix3d endpoint = from.body.linear() * roll_yaw::expSO3(Eigen::Vector3d::UnitY() * delta);
  if(!roll_yaw::inverseOrientation(model_, endpoint, from.eta, planning_config_, goal, error))
    return reject(error);
  // A rest-to-rest quintic is monotone in every coordinate: actual variation is
  // abs(goal-start), not its deliberately looser Bernstein velocity envelope.
  const Eigen::Vector3d variation = (goal - from.eta).cwiseAbs();
  if(((measured - from.eta).cwiseAbs() + variation).maxCoeff() >
         feedback_config_.tracking_tolerance * feedback_config_.soft_tracking_fraction ||
     roll_yaw::logSO3(f.body.linear().transpose() * from.body.linear()).norm() + variation.sum() >
         feedback_config_.orientation_tolerance * feedback_config_.soft_tracking_fraction)
    return reject("whole jog would exhaust soft tracking allowance; wait for tracking");
  roll_yaw::Path path;
  if(!roll_yaw::mechanismPath(model_, from, goal, planning_config_, path, error))
    return reject(error);
  Eigen::Isometry3d previous = from.body;
  double pitch = 0;
  for(int i = 0; i <= 8; ++i)
  {
    const Eigen::Vector3d q = from.eta + (goal - from.eta) * (i / 8.0);
    Eigen::Isometry3d body;
    double next_pitch;
    if(!model_.evaluateBody(q, body) ||
       !roll_yaw::localPitchSample(model_, q, body, from.body, previous, delta, pitch, planning_config_, next_pitch, error))
      return reject(error.empty() ? "local pitch FK unavailable" : error);
    previous = body;
    pitch = next_pitch;
  }
  if((delta > 0 ? 1 : -1) *
         ((from.body * planning_config_.saw_point).z() - (previous * planning_config_.saw_point).z()) <=
     1e-6)
    return reject("no meaningful requested-direction saw-origin displacement");
  // All rejecting work precedes ownership transfer.
  invalidatePlanner();
  resetCompliance();
  body_alignment_valid_ = false;
  jog_ = PitchJog();
  jog_.active = true;
  jog_.session = roll_yaw_lock_stamp_.toNSec();
  jog_.start = from.body;
  jog_.endpoint = endpoint;
  jog_.delta = delta;
  jog_.phase = "moving";
  nominal_ = goal;
  manual_path_ = std::move(path);
  manual_phase_ = 0;
  manual_speed_ = 1;
  manual_time_ = f.now;
  final_target_.t = 0;
  reason_ = "body-local pitch moving via bounded manual endpoint IK/interpolation";
  return true;
}
void GimbalrotorPerchingNavigator::finishPitchJog(const std::string& phase, const std::string& why)
{
  if(!jog_.active)
    return;
  jog_.active = false;
  jog_.phase = phase;
  const auto f = feedback();
  double measured_pitch = std::numeric_limits<double>::quiet_NaN(), cross;
  double endpoint_error = std::numeric_limits<double>::quiet_NaN();
  if(contactRejection(f).empty())
  {
    roll_yaw::taskPitch(jog_.start.linear(), f.body.linear(), jog_.progress, measured_pitch, cross);
    endpoint_error = roll_yaw::logSO3(f.body.linear().transpose() * jog_.endpoint).norm();
  }
  std_msgs::String ack;
  ack.data = "action=body_pitch_delta session=" + std::to_string(jog_.session) + " accepted=true phase=" + phase +
             " applied_pitch_rad=" + std::to_string(jog_.progress) +
             " measured_pitch_rad=" + std::to_string(measured_pitch) +
             " measured_endpoint_error_rad=" + std::to_string(endpoint_error) + " reason=" + why;
  ack_pub_.publish(ack);
  last_diagnostics_ = 0;
}
void GimbalrotorPerchingNavigator::manualCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  const double started = roll_yaw::steadySeconds();
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  CallbackTimer timing{manual_wall_, started};
  if(!lock_valid_ || msg->header.frame_id != "perching_lock_" + std::to_string(roll_yaw_lock_stamp_.toNSec()))
  {
    reason_ = "manual rejected: stale/missing base lock identity";
    publishRollYawDiagnostics();
    return;
  }
  // Compatibility interface for deliberate full nominal targets. Independent
  // axis topics below are preferred; publisher timestamps are informational.
  setNominalTarget(Eigen::Vector3d(msg->vector.y, msg->vector.x, msg->vector.z));
}
bool GimbalrotorPerchingNavigator::setNominalTarget(const Eigen::Vector3d& target, int changed_axis)
{
  // Correction details on the existing status channel let the keyboard reconcile
  // only the matching request/axis, without waiting or introducing a handshake.
  auto correction = [&]()
  {
    if(changed_axis < 0 || changed_axis > 2 || !lock_valid_)
      return;
    const char* axis = changed_axis == 0 ? "alpha" : changed_axis == 1 ? "roll" : "yaw";
    std::ostringstream detail;
    detail << std::setprecision(17) << " [axis=" << axis << " lock=" << roll_yaw_lock_stamp_.toNSec()
           << " requested=" << target[changed_axis] << " accepted=" << nominal_[changed_axis] << "]";
    reason_ += detail.str();
  };
  auto reject = [&](const std::string& why)
  {
    reason_ = "manual rejected: " + why;
    correction();
    last_diagnostics_ = 0;
    publishRollYawDiagnostics();
    return false;
  };
  if(!lock_valid_ || faulted_ || !feedback().hover)
    return reject("require valid enabled HOVER contact");
  if(roll_yaw_pitch_rezero_state_ != RollYawPitchRezeroState::IDLE)
    return reject("pitch rezero ACTIVE/HOLD; wait ready then final relock");
  if(!target.allFinite())
    return reject("nonfinite mechanism target");
  const auto why = contactRejection(feedback());
  if(!why.empty())
    return reject(why);
  const Eigen::Vector3d bounded =
      target.cwiseMax(planning_config_.limits.lower).cwiseMin(planning_config_.limits.upper);
  std::string error;
  if(stop_pending_)
    return reject("stopping, retry; command not queued");
  if(planner_ && (planner_->stage == roll_yaw::Stage::EXECUTING || planner_->stage == roll_yaw::Stage::PRE_ALIGN))
  {
    stopMotion("manual takeover requested");
    return reject("stopping, retry; command not queued");
  }
  if(jog_.active)
  {
    stopMotion("mechanism-key takeover of local pitch");
    return reject("stopping, retry; command not queued");
  }
  roll_yaw::Path transition;
  if(!roll_yaw::mechanismPath(model_, final_target_, bounded, planning_config_, transition, error))
    return reject(error);
  // Multilink semantics: accept a destination, independent of its current applied
  // or measured value. No lookahead, endpoint-tracking gate or per-key executor.
  // Cancel the producer at its coherent applied reference before transferring
  // ownership; bounded contains the untouched independently accepted axes.
  if(planner_ || prepare_pending_)
    stopMotion("manual takeover");
  invalidatePlanner();
  resetCompliance();
  body_alignment_valid_ = false;
  nominal_ = bounded;
  manual_receipt_stamp_ = ros::Time::now();
  manual_path_ = std::move(transition);
  manual_time_ = ros::Time::now().toSec();
  manual_phase_ = 0;
  manual_speed_ = 1;
  final_target_.t = 0;
  reason_ = (bounded - target).norm() > 1e-10 ? "manual nominal target accepted at configured limit"
                                              : "manual nominal target accepted";
  if((bounded - target).norm() > 1e-10)
    correction();
  publishRollYawDiagnostics();
  return true;
}
bool GimbalrotorPerchingNavigator::buildFinalMechanismTarget(roll_yaw::Reference& next)
{
  if(!planner_ && !manual_path_.knots.empty())
  {
    const double now = ros::Time::now().toSec(), dt = now - manual_time_;
    manual_time_ = now;
    if(dt == 0)
    {
      next = final_target_;
      return true;
    }
    if(dt < 0 || dt > feedback_config_.max_execution_step)
    {
      faultHold("manual reference clock gap");
      return false;
    }
    const double a =
        .5 *
        std::min((planning_config_.limits.acceleration.array() / planning_config_.limits.velocity.array()).minCoeff(),
                 planning_config_.cog_acceleration_limit / planning_config_.cog_speed_limit);
    const double v = stop_pending_ ? std::max(0.0, manual_speed_ - a * dt) : manual_speed_;
    const double phase = std::min(manual_path_.duration(), manual_phase_ + .5 * (manual_speed_ + v) * dt);
    if(!roll_yaw::samplePath(model_, manual_path_, phase, next))
    {
      faultHold("invalid manual interpolation");
      return false;
    }
    // Reserve half the CoG budget for normal bounded scalar braking, just as
    // the polynomial coefficients reserve half the joint rate/acceleration.
    if(next.cog_velocity.norm() > planning_config_.cog_speed_limit * .5 ||
       next.cog_acceleration.norm() > planning_config_.cog_acceleration_limit * .5)
    {
      faultHold("manual candidate exceeds CoG braking reserve");
      return false;
    }
    roll_yaw::scaleReference(next, v, (v - manual_speed_) / dt);
    std::string error;
    if(!roll_yaw::referenceAllowed(next, planning_config_, error))
    {
      faultHold(error);
      return false;
    }
    if(jog_.active)
    {
      const auto f = feedback();
      roll_yaw::Estimate e;
      Eigen::Vector3d q = f.eta;
      if(!model_.estimate(f.body, q, e))
      {
        faultHold("local pitch measured estimate lost");
        return false;
      }
      q[0] = e.alpha;
      if((q - next.eta).cwiseAbs().maxCoeff() > feedback_config_.tracking_tolerance ||
         roll_yaw::logSO3(f.body.linear().transpose() * next.body.linear()).norm() > feedback_config_.orientation_tolerance)
      {
        faultHold("local pitch candidate exceeds live hard tracking allowance");
        return false;
      }
    }
    if(jog_.active)
    {
      double pitch;
      if(jog_.session != roll_yaw_lock_stamp_.toNSec() ||
         !roll_yaw::localPitchSample(model_, next.eta, next.body, jog_.start, final_target_.body, jog_.delta, jog_.progress,
                           planning_config_, pitch, error))
      {
        faultHold("local pitch candidate rejected: " + error);
        return false;
      }
      jog_.progress = pitch;
    }
    manual_phase_ = phase;
    manual_speed_ = v;
    if(manual_phase_ >= manual_path_.duration() || (stop_pending_ && v == 0))
    {
      manual_path_ = roll_yaw::Path();
      nominal_ = next.eta;
      roll_yaw::scaleReference(next, 0, 0);
      if(stop_pending_)
      {
        final_target_ = next;
        completeStop("manual braking finished");
      }
      else
      {
        reason_ = jog_.active ? "body-local pitch reference_complete; measured accuracy reported separately"
                              : "motion complete; holding contact";
        finishPitchJog("reference_complete", reason_);
      }
    }
    return true;
  }
  if(planner_)
  {
    next = planner_->target;
    return true;
  }
  const Eigen::Vector3d bounded =
      nominal_.cwiseMax(planning_config_.limits.lower).cwiseMin(planning_config_.limits.upper);
  std::string error;
  if(!roll_yaw::mechanismAllowed(model_, bounded, planning_config_, error))
  {
    reason_ = "target FK rejected: " + error;
    return false;
  }
  Eigen::Vector3d rate = Eigen::Vector3d::Zero(), acc = Eigen::Vector3d::Zero();
  return roll_yaw::mechanismReference(model_, bounded, rate, acc, next);
}
void GimbalrotorPerchingNavigator::taskCommandCallback(const std_msgs::UInt64ConstPtr& m,
                                                              const std::string& action)
{
  requestRollYaw(action, m->data);
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  std_msgs::String ack;
  ack.data = "action=" + action + " session=" + std::to_string(m->data) +
             " accepted=" + (last_command_accepted_ ? "true" : "false") + " reason=" + reason_;
  ack_pub_.publish(ack);
}

roll_yaw::Plan GimbalrotorPerchingNavigator::prepareTrajectory(
    const roll_yaw::Kinematics& model_snapshot, Eigen::Vector3d initial, const Eigen::Matrix3d& neutral, const roll_yaw::Request& request,
    const roll_yaw::Reference& measured_start, const roll_yaw::Reference& connection_start, roll_yaw::PlanningConfig& config, std::uint64_t session,
    const std::shared_ptr<PlanningStatus>& status, double deadline, double consistency,
    const std::function<bool()>& cancel)
{
  const double started = roll_yaw::steadySeconds();
  config.cancelled = [cancel, started, deadline] { return cancel() || roll_yaw::steadySeconds() - started > deadline; };
  config.report = [status, last = started, was_validation = false](int i, int n, bool validation) mutable
  {
    const double now = roll_yaw::steadySeconds();
    auto& elapsed = was_validation ? status->validation_seconds : status->solve_seconds;
    elapsed.store(elapsed.load() + now - last);
    last = now;
    was_validation = validation;
    status->interval = i;
    status->total = n;
    status->validating = validation;
  };
  const auto model = model_snapshot;
  // Physical consistency is distinct from the numerical IK tolerance.
  Eigen::Vector3d reconciled;
  std::string error;
  roll_yaw::Plan plan;
  if(!roll_yaw::inverseOrientation(model, measured_start.body.linear(), initial, config, reconciled, error) ||
     (reconciled - initial).cwiseAbs().maxCoeff() > consistency ||
     (reconciled - connection_start.eta).cwiseAbs().maxCoeff() > consistency)
    plan.reason = "measured orientation/joints cannot be reconciled within contact tolerance: " + error;
  else
  {
    initial = reconciled;
    plan = roll_yaw::prepare(model, request.align_only ? connection_start.eta : initial, neutral, request, config);
  }
  plan.session = session;
  plan.goal = request;  // Preserve purpose even when start reconciliation failed.
  plan.submission = measured_start;
  plan.applied_start = connection_start;
  plan.submission_valid = true;
  if(plan.ok)
  {
    roll_yaw::Path connection;
    std::string why;
    if(!request.align_only)
    {
      if(!roll_yaw::validatedMechanismPath(model, connection_start, initial, config, connection, why))
      {
        plan.ok = false;
        plan.reason = "start connection: " + why;
      }
      else
        plan.align = std::move(connection);
    }
  }
  plan.wall_seconds = roll_yaw::steadySeconds() - started;
  if(cancel() || plan.wall_seconds > deadline)
  {
    plan.ok = false;
    plan.reason = "offline cancelled/deadline exceeded";
  }
  return plan;
}

void GimbalrotorPerchingNavigator::requestRollYaw(const std::string& action, std::uint64_t session, int axis,
                                                  double value)
{
  roll_yaw::Request requested;
  bool request_valid = true;
  // Preserve live per-request settings without holding navigation across XMLRPC.
  if(action == "prepare")
  {
    ros::NodeHandle pn(nh_, "navigation/roll_yaw_perching/request");
    request_valid = pn.getParam("pitch_increment", requested.pitch_end) && pn.getParam("pitch_rate", requested.rate) &&
                    pn.getParam("pitch_acceleration", requested.acceleration);
  }
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  last_command_accepted_ = false;
  if(!lock_valid_ || session != roll_yaw_lock_stamp_.toNSec())
  {
    reason_ = "stale/missing base lock identity";
    return;
  }
  if(action == "cancel" || action == "stop")
  {
    last_command_accepted_ = true;
    if(roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::ACTIVE)
      failRollYawPitchRezero("operator stopped rezero");
    else
      stopMotion("stopped at applied contact reference");
    last_diagnostics_ = 0;
    publishRollYawDiagnostics();
    return;
  }
  if(stop_pending_)
  {
    reason_ = "stopping, retry; command not queued";
    return;
  }
  if(!planning_configuration_valid_)
  {
    reason_ = "invalid offline planning configuration";
    return;
  }
  if(faulted_ || roll_yaw_pitch_rezero_state_ != RollYawPitchRezeroState::IDLE)
  {
    reason_ = "orientation request rejected during fault/setup";
    return;
  }
  const auto rejection = contactRejection(feedback());
  if(!rejection.empty())
  {
    reason_ = rejection;
    return;
  }
  if(action == "prepare" || action == "body_align")
  {
    const bool alignment = action == "body_align";
    if(prepare_pending_)
    {
      reason_ = "worker busy; retry after status changes (request not queued)";
      return;
    }
    if(planner_ && (planner_->stage == roll_yaw::Stage::EXECUTING || planner_->stage == roll_yaw::Stage::PRE_ALIGN))
    {
      reason_ = "hold or complete current motion before preparing a cut";
      return;
    }
    if(!request_valid)
    {
      reason_ = "require pitch_increment, pitch_rate and pitch_acceleration";
      return;
    }
    roll_yaw::Request request = requested;
    if(alignment)
    {
      request = alignment_request_;
      if(!body_alignment_valid_)
      {
        const Eigen::Matrix3d relative = neutral_.transpose() * feedback().body.linear();
        request = roll_yaw::Request();
        request.roll = std::asin(std::clamp(relative(2, 1), -1.0, 1.0));
        request.yaw = std::atan2(-relative(0, 1), relative(1, 1));
        request.pitch_start = std::atan2(-relative(2, 0), relative(2, 2));
      }
      if(axis == 0)
        request.yaw = value;
      else
        request.roll = value;
      request.align_only = true;
    }
    if(!request.valid())
    {
      reason_ = "invalid orientation request (radians, positive rate/acceleration)";
      return;
    }
    const auto f = feedback();
    roll_yaw::Estimate estimate;
    if(!model_.estimate(f.body, f.eta, estimate))
    {
      reason_ = "invalid measured start";
      return;
    }
    Eigen::Vector3d initial = f.eta;
    initial[0] = estimate.alpha;
    roll_yaw::Reference measured_start;
    measured_start.eta = initial;
    measured_start.body = f.body;
    roll_yaw::Reference held_nominal;
    if(!roll_yaw::mechanismReference(model_, initial, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), held_nominal) ||
       roll_yaw::logSO3(held_nominal.body.linear().transpose() * f.body.linear()).norm() > feedback_config_.preparation_tolerance)
    {
      reason_ = "measured start inconsistent with locked hinge FK";
      return;
    }
    const auto readiness = preparationRejection(f, final_target_);
    if(!readiness.empty())
    {
      reason_ = "prepare waiting: " + readiness;
      return;
    }
    // Noise-tolerant rate and reference-connection gates, not a zero-rate demand.
    if((initial - final_target_.eta).cwiseAbs().maxCoeff() > feedback_config_.preparation_tolerance ||
       (!alignment && ((nominal_ - initial).cwiseAbs().maxCoeff() > feedback_config_.preparation_tolerance ||
                       f.body_omega.norm() > feedback_config_.settle_body_rate ||
                       f.joint_rate.tail<2>().norm() > feedback_config_.settle_joint_rate)))
    {
      reason_ = "wait for current alignment to track/settle, or hold before preparing";
      return;
    }
    if(!manual_path_.knots.empty() || final_target_.velocity.norm() > 1e-6)
    {
      stopMotion("prepare requested during manual motion");
      reason_ = "stopping, retry prepare; not queued";
      return;
    }
    // Preserve applied reference; the worker adds a bounded connection to the
    // reconciled fresh measured start. No callback-time reference jump.
    const roll_yaw::Reference connection_start = final_target_;
    resetCompliance();
    invalidatePlanner();

    planner_.reset(new roll_yaw::Executor());
    planner_->config = feedback_config_;
    planner_->planning = planning_config_;
    if(!planner_->attach(model_, connection_start, session, f.now) || !planner_->command("prepare", session))
    {
      reason_ = planner_->reason;
      invalidatePlanner();
      return;
    }
    // Cut R0 is the measured controlled-body orientation now, never lock neutral.
    const Eigen::Matrix3d neutral = alignment ? neutral_ : Eigen::Matrix3d(f.body.linear());
    planner_->plan.goal = request;
    planner_->plan.neutral = neutral;
    planner_->plan.session = session;
    planner_->planned_pitch = request.pitch_start;
    planner_->task.reset(request.pitch_start);
    // Model, measured/applied starts and generation are captured under the state lock.
    const auto model_snapshot = planning_model_;
    const auto config = planning_config_;
    const double consistency = feedback_config_.preparation_tolerance;
    pending_generation_ = producer_generation_;
    prepare_started_ = roll_yaw::steadySeconds();
    prepare_result_ = "pending";
    ++plan_id_;
    planning_status_ = std::make_shared<PlanningStatus>();
    const auto status = planning_status_;
    const double deadline = prepare_deadline_;
    prepare_pending_ = prepare_worker_->submit(
        [model_snapshot, initial, neutral, request, measured_start, connection_start, config = roll_yaw::PlanningConfig(config),
         session, status, deadline, consistency](const std::function<bool()>& cancel) mutable
        {
          return prepareTrajectory(*model_snapshot, initial, neutral, request, measured_start, connection_start, config,
                                   session, status, deadline, consistency, cancel);
        });
    if(!prepare_pending_)
    {
      planner_->hold("planner worker busy/cancelling; retry prepare");
    }
    else
    {
      planner_->reason = "JOB_ACCEPTED: preparing; motion not yet accepted";
    }
    last_command_accepted_ = prepare_pending_;
  }
  else if(!planner_)
  {
    reason_ = "no prepared path; prepare first";
    return;
  }
  else
  {
    if((action == "start" && planner_->stage != roll_yaw::Stage::READY) ||
       (action == "resume" && planner_->stage != roll_yaw::Stage::PAUSED))
    {
      reason_ = "start requires measured READY; resume requires PAUSED; wait/retry (not queued)";
      return;
    }
    if(action == "start")
    {
      if(!planner_->plan.ok || planner_->plan.cut.knots.empty())
      {
        reason_ = "prepare first";
        return;
      }
      if((final_target_.eta - planner_->plan.cut.knots.front().eta).norm() > 1e-9)
      {
        stopMotion("applied prepared start changed; prepare again");
        return;
      }
    }
    if(action == "start" || action == "resume")
    {
      const auto current = feedback();
      const auto ready_error = preparationRejection(current, final_target_);
      if(!ready_error.empty())
      {
        reason_ = "motion waiting: " + ready_error;
        return;
      }
      if(admittance_.enabled && (!interaction_valid_ || !admittance_.baseline_ready ||
                                 current.now - interaction_stamp_ > feedback_config_.timeout ||
                                 current.now - interaction_stamp_ < -feedback_config_.future_tolerance ||
                                 std::abs(interaction_stamp_ - current.body_stamp) > synchronization_tolerance_ ||
                                 roll_yaw::steadySeconds() - interaction_receipt_ > feedback_config_.timeout))
      {
        reason_ = "motion denied: enabled admittance input/baseline unavailable; restore source or explicitly disable";
        return;
      }
    }
    last_command_accepted_ = planner_->command(action, session);
    if(action == "start" && last_command_accepted_)
      body_alignment_valid_ = false;
  }
  reason_ = planner_->reason;
  publishRollYawDiagnostics();
}
std::string GimbalrotorPerchingNavigator::preparationRejection(const roll_yaw::Feedback& f, const roll_yaw::Reference& held) const
{
  auto why = contactRejection(f);
  if(!why.empty())
    return why;
  roll_yaw::Estimate e;
  if(!model_.estimate(f.body, f.eta, e))
    return "passive estimate unavailable";
  Eigen::Vector3d q = f.eta;
  q[0] = e.alpha;
  if((q - held.eta).cwiseAbs().maxCoeff() > feedback_config_.preparation_tolerance ||
     roll_yaw::logSO3(f.body.linear().transpose() * held.body.linear()).norm() > feedback_config_.preparation_tolerance)
    return "current measured-to-applied preparation tracking allowance";
  const auto& c = feedback_config_;
  const auto& w = preparation_stats_;
  if(f.body_omega.norm() >= c.settle_body_rate || f.joint_rate.tail<2>().norm() >= c.settle_joint_rate)
    return "quiet window current rate exceeds bound";
  if(!w.complete)
    return "quiet window coverage: " + w.status;
  if(w.body_rms >= c.settle_body_rate || w.joint_rms >= c.settle_joint_rate)
    return "quiet window RMS rate exceeds bound";
  if(w.body_mean >= c.settle_mean_rate || w.joint_mean >= c.settle_mean_rate)
    return "quiet window mean rate exceeds bound";
  if(!preparation_ready_)
    return "quiet window current rate/coverage unavailable";
  return "";
}
void GimbalrotorPerchingNavigator::advancePreparedConnection()
{
  if(!connection_pending_ || !planner_)
    return;
  const auto f = feedback();
  if(!planner_->plan.ok || planner_->stage != roll_yaw::Stage::CONTACT_LOCKED || pending_generation_ != producer_generation_ ||
     !planner_->submissionMatches(planner_->plan, f, final_target_))
  {
    connection_pending_ = false;
    planner_->plan = roll_yaw::Plan();
    planner_->hold("prepared connection identity changed; prepare again");
    resetCompliance();
    return;
  }
  if(roll_yaw::steadySeconds() - connection_wait_started_ > feedback_config_.settle_timeout)
  {
    connection_pending_ = false;
    planner_->plan = roll_yaw::Plan();
    planner_->hold("prepared connection live-readiness timeout; prepare again");
    resetCompliance();
    return;
  }
  const auto why = preparationRejection(f, planner_->plan.applied_start);
  if(!why.empty())
  {
    planner_->reason = "prepared, waiting for live readiness: " + why;
    return;
  }
  connection_pending_ = false;
  if(planner_->command("align", roll_yaw_lock_stamp_.toNSec()))
  {
    if(planner_->plan.goal.align_only)
    {
      alignment_request_ = planner_->plan.goal;
      body_alignment_valid_ = true;
    }
    planner_->reason = "PLAN_READY: geometric connection moving; wait measured READY";
  }
  else
  {
    planner_->plan = roll_yaw::Plan();
    planner_->hold("prepared connection denied; prepare again");
    resetCompliance();
  }
}
void GimbalrotorPerchingNavigator::resetCompliance()
{
  admittance_.reset();
  external_wrench_ = geometry_msgs::WrenchStamped();
  tare_requested_ = false;
  interaction_valid_ = false;
  interaction_stamp_ = 0;
  interaction_receipt_ = 0;
  compliance_time_ = 0;
  admittance_reason_ = "disabled: prepare to measured READY, then no-cut tare with a fresh external wrench";
}
void GimbalrotorPerchingNavigator::admittanceCallback(const std_msgs::BoolConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(!msg->data)
  {
    admittance_.enabled = false;
    admittance_reason_ = "disabled; bounded correction release along prepared path";
  }
  else if(!interaction_configured_ || !admittance_.config.valid())
  {
    admittance_reason_ =
        "unavailable: invalid admittance dynamics or wrench frame/point/sign (+/-1)/scale (>0) configuration";
  }
  else if(!planner_ || !planner_->plan.ok || planner_->plan.goal.align_only || planner_->stage != roll_yaw::Stage::READY ||
          !preparationRejection(feedback(), final_target_).empty())
  {
    admittance_reason_ =
        "unavailable: require a prepared cut at measured READY with current quiet/tracking/contact feedback";
  }
  else if(!admittance_.baseline_ready)
  {
    admittance_reason_ =
        "unavailable: no completed baseline; tare at quiet no-cut READY with a fresh synchronized wrench";
  }
  else if(!interaction_valid_ || interaction_stamp_ <= 0 ||
          ros::Time::now().toSec() - interaction_stamp_ > feedback_config_.timeout ||
          roll_yaw::steadySeconds() - interaction_receipt_ > feedback_config_.timeout ||
          std::abs(interaction_stamp_ - feedback().body_stamp) > synchronization_tolerance_)
  {
    admittance_reason_ = "unavailable: external wrench invalid, stale or unsynchronized with body feedback";
  }
  else
  {
    admittance_.enabled = true;
    admittance_reason_ = "enabled/ready: approximate path interaction; assumed no-cut tare";
  }
  ROS_INFO_STREAM("[RollYaw] " << admittance_reason_);
  publishRollYawDiagnostics();
}
void GimbalrotorPerchingNavigator::tareCallback(const std_msgs::EmptyConstPtr&)
{
  std::uint64_t session, generation;
  {
    std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
    session = roll_yaw_lock_stamp_.toNSec();
    generation = producer_generation_;
  }
  ros::NodeHandle an(nh_, "navigation/roll_yaw_perching/admittance");
  double sign = 0, scale = 0;
  std::string frame, point;
  an.param("wrench_sign", sign, interaction_topic_ == "estimated_external_wrench" ? 1.0 : 0.0);
  an.param("wrench_scale", scale, interaction_topic_ == "estimated_external_wrench" ? 1.0 : 0.0);
  an.param<std::string>("wrench_frame", frame, "world");
  an.param<std::string>("wrench_point", point, "cog");
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(session != roll_yaw_lock_stamp_.toNSec() || generation != producer_generation_ || admittance_.enabled || !planner_ ||
     planner_->stage != roll_yaw::Stage::READY || std::abs(admittance_.offset) > 1e-8 ||
     !preparationRejection(feedback(), final_target_).empty())
  {
    admittance_reason_ =
        "tare rejected: disable, wait for zero correction and measured READY cut; retry after state change";
    return;
  }
  admittance_.reset();
  tare_requested_ = false;
  interaction_valid_ = false;
  interaction_sign_ = sign;
  interaction_scale_ = scale;
  interaction_frame_ = frame;
  interaction_point_ = point;
  interaction_configured_ =
      admittance_.config.valid() && (interaction_sign_ == 1 || interaction_sign_ == -1) &&
      std::isfinite(interaction_scale_) && interaction_scale_ > 0 &&
      (interaction_frame_ == "world" || interaction_frame_ == "body") &&
      (interaction_point_ == "cog" || interaction_point_ == "body" || interaction_point_ == "world_origin");
  if(!interaction_configured_)
  {
    admittance_reason_ =
        "tare rejected: invalid admittance dynamics or wrench frame/point/sign (+/-1)/scale (>0) configuration";
    return;
  }
  if(!planner_ || !planner_->plan.ok || planner_->plan.goal.align_only || planner_->stage != roll_yaw::Stage::READY ||
     admittance_.enabled || std::abs(admittance_.offset) > 1e-8)
  {
    admittance_reason_ = "tare rejected: require READY cut, disabled and zero correction; remove cutting contact";
    return;
  }
  admittance_.reset();
  tare_requested_ = true;
  interaction_stamp_ = external_wrench_.header.stamp.toSec();
  interaction_receipt_ = 0;
  admittance_reason_ = "taring: operator asserts no cutting contact; collecting fresh quiet samples";
}
void GimbalrotorPerchingNavigator::externalWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const double t = msg->header.stamp.toSec(), now = ros::Time::now().toSec();
  const auto& force = msg->wrench.force;
  const auto& moment = msg->wrench.torque;
  const std::string expected = interaction_frame_ == "world" ? world_ : body_frame_;
  if(!std::isfinite(t) || t <= 0 || now - t > feedback_config_.timeout ||
     now - t < -feedback_config_.future_tolerance || frameName(msg->header.frame_id) != frameName(expected) ||
     !Eigen::Vector3d(force.x, force.y, force.z).allFinite() ||
     !Eigen::Vector3d(moment.x, moment.y, moment.z).allFinite())
  {
    interaction_receipt_ = 0;
    interaction_valid_ = false;
    admittance_reason_ = "unavailable: external wrench frame/stamp/values invalid";
    return;
  }
  if(t <= external_wrench_.header.stamp.toSec())
    return;
  external_wrench_ = *msg;
  interaction_receipt_ = roll_yaw::steadySeconds();
}
bool GimbalrotorPerchingNavigator::observeInteractionSample(const roll_yaw::Feedback& f, double stamp, bool quiet)
{
  roll_yaw::Estimate e;
  Eigen::Vector3d q = f.eta;
  Eigen::Isometry3d body, cog;
  bool fresh = model_.estimate(f.body, q, e);
  q[0] = e.alpha;
  if(fresh)
    fresh = model_.evaluate(q, body, cog);
  if(fresh)
  {
    const double direction = planner_->plan.goal.pitch_end >= planner_->plan.goal.pitch_start ? 1 : -1;
    const auto& w = external_wrench_.wrench;
    Eigen::Vector3d force(w.force.x, w.force.y, w.force.z), moment(w.torque.x, w.torque.y, w.torque.z);
    if(interaction_frame_ == "body")
    {
      force = f.body.linear() * force;
      moment = f.body.linear() * moment;
    }
    Eigen::Vector3d source = Eigen::Vector3d::Zero();
    if(interaction_point_ == "body")
      source = f.body.translation();
    if(interaction_point_ == "cog")
      source = f.body * (body.inverse() * cog).translation();
    fresh = roll_yaw::pathInteractionTorque(model_, q, direction, planning_config_.min_singular_value, force, moment, source,
                                  interaction_torque_);
    interaction_torque_ *= interaction_sign_ * interaction_scale_;
    fresh = fresh && std::isfinite(interaction_torque_);
    if(fresh)
    {
      admittance_.observe(interaction_torque_, stamp, tare_requested_ && quiet && planner_->stage == roll_yaw::Stage::READY);
      interaction_stamp_ = stamp;
      if(tare_requested_ && admittance_.baseline_ready)
      {
        tare_requested_ = false;
        admittance_reason_ = "disabled/ready: no-cut baseline complete; enable explicitly";
      }
    }
  }
  return fresh;
}

void GimbalrotorPerchingNavigator::updateInteraction(const roll_yaw::Feedback& f)
{
  const double dt = compliance_time_ > 0 ? f.now - compliance_time_ : 0;
  compliance_time_ = f.now;
  if(!planner_ || !planner_->plan.ok || planner_->plan.goal.align_only)
    return;
  const double stamp = external_wrench_.header.stamp.toSec();
  bool scalar_failed = false;
  bool fresh = interaction_configured_ && interaction_receipt_ > 0 &&
               roll_yaw::steadySeconds() - interaction_receipt_ <= feedback_config_.timeout && contactRejection(f).empty() &&
               stamp > 0 && f.now - stamp >= -feedback_config_.future_tolerance &&
               f.now - stamp <= feedback_config_.timeout &&
               std::abs(stamp - f.body_stamp) <= synchronization_tolerance_;
  const bool quiet = f.body_omega.norm() < feedback_config_.settle_body_rate &&
                     f.joint_rate.tail<2>().norm() < feedback_config_.settle_joint_rate &&
                     (!tare_requested_ || preparationRejection(f, final_target_).empty());
  if(tare_requested_ && !quiet)
  {
    admittance_.reset();
    admittance_reason_ = "tare waiting for quiet no-cut posture";
  }
  if(fresh && stamp == interaction_stamp_)
    fresh = interaction_valid_;
  if(fresh && stamp > interaction_stamp_)
  {
    fresh = observeInteractionSample(f, stamp, quiet);
  }
  if(tare_requested_ && !fresh)
  {
    admittance_.reset();
    admittance_reason_ = "tare waiting for a continuous fresh synchronized no-cut source";
  }
  if(admittance_.enabled && !fresh)
    admittance_reason_ =
        "stale/unavailable interaction: bounded path stop, correction braking/held; original goal retained";
  else if(admittance_.enabled)
    admittance_reason_ = "enabled/ready: approximate external work, resistance reduces penetration";
  // Observation/filtering continues at a hold; integration never advances an
  // invisible correction while PAUSED, aligning, READY, or braking to a pause.
  if(planner_->stage == roll_yaw::Stage::EXECUTING && !planner_->stopping() &&
     (admittance_.enabled || std::abs(admittance_.offset) > 1e-10 || std::abs(admittance_.rate) > 1e-10))
  {
    const auto& plan = planner_->plan;
    double angle = 0, cross = 0;
    const Eigen::Matrix3d pre = roll_yaw::cuttingOrientation(plan.neutral, plan.goal, 0, 0, 0).rotation;
    const bool mapped =
        roll_yaw::taskPitch(pre, plan.cut.desired(planner_->nominal_progress).rotation, plan.goal.pitch_start, angle, cross);
    const double available =
        std::max(0.0, (plan.goal.pitch_end >= plan.goal.pitch_start ? 1 : -1) * (angle - plan.goal.pitch_start));
    if(!mapped || dt <= 0 || dt > feedback_config_.max_execution_step ||
       !admittance_.step(dt, fresh, true, std::min(available, planner_->relief_available)))
    {
      fresh = false;
      scalar_failed = true;
      admittance_reason_ = "scalar state unavailable; brake to PAUSED";
    }
  }
  interaction_valid_ = fresh;
  planner_->relief = std::min(0.0, admittance_.offset);
  planner_->interaction_hold = (admittance_.enabled && !fresh) || scalar_failed;
  planner_->resisting_load = admittance_.enabled && fresh && admittance_.contact;
  planner_->relief_enabled = admittance_.enabled;
  planner_->relief_angle_limit = admittance_.config.angle_limit;
  planner_->relief_rate_limit = admittance_.config.rate_limit;
  planner_->relief_acceleration_limit = admittance_.config.acceleration_limit;
}
void GimbalrotorPerchingNavigator::resetRollYawPitchRezeroState()
{
  roll_yaw_pitch_rezero_state_ = RollYawPitchRezeroState::IDLE;
  roll_yaw_pitch_rezero_ready_ = roll_yaw_pitch_rezero_failed_ = false;
  rezero_frozen_.setZero();
  rezero_started_ = rezero_stable_since_ = rezero_alpha_ = 0;
  rezero_previous_pitch_ = rezero_previous_time_ = rezero_measured_rate_ = roll_yaw_rezero_feedback_dt_ = 0;
  rezero_rate_valid_ = false;
}
void GimbalrotorPerchingNavigator::rollYawPitchRezeroCallback(const std_msgs::EmptyConstPtr&)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const auto f = feedback();
  if(!lock_valid_ || faulted_ || !f.hover || !rezero_configuration_valid_ ||
     roll_yaw_pitch_rezero_state_ != RollYawPitchRezeroState::IDLE)
  {
    reason_ = "pitch rezero rejected: require valid enabled HOVER contact, configuration and IDLE";
    publishRollYawDiagnostics();
    return;
  }
  const auto why = contactRejection(f);
  if(!why.empty())
  {
    reason_ = "pitch rezero rejected: " + why;
    publishRollYawDiagnostics();
    return;
  }
  if(stop_pending_ || !manual_path_.knots.empty() ||
     (planner_ && (planner_->stage == roll_yaw::Stage::EXECUTING || planner_->stage == roll_yaw::Stage::PRE_ALIGN)))
  {
    stopMotion("pitch rezero takeover");
    reason_ = "pitch rezero waiting for stop; retry (not queued)";
    publishRollYawDiagnostics();
    return;
  }
  roll_yaw::Kinematics model = model_;
  Eigen::Vector3d frozen = f.eta;
  frozen[0] = 0;
  roll_yaw::Reference held;
  if(!model.lock(f.body, frozen) ||
     !roll_yaw::mechanismReference(model, frozen, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), held))
  {
    reason_ = "pitch rezero rejected: invalid measured body/joint capture";
    publishRollYawDiagnostics();
    return;
  }
  finishPitchJog("cancelled", "contact reset/relock");
  jog_ = PitchJog();
  invalidatePlanner();
  resetRollYawPitchRezeroState();
  manual_path_ = roll_yaw::Path();
  stop_pending_ = false;
  body_alignment_valid_ = false;
  resetCompliance();
  model_ = std::move(model);
  final_target_ = held;
  nominal_ = rezero_frozen_ = frozen;
  rezero_started_ = rezero_previous_time_ = f.body_stamp;
  double roll, yaw;
  tf::Matrix3x3(tfQuaternion(f.body.linear())).getRPY(roll, rezero_previous_pitch_, yaw);
  roll_yaw_pitch_rezero_state_ = RollYawPitchRezeroState::ACTIVE;
  reason_ = "pitch rezero ACTIVE: signed measured-feedback control, active roll/yaw frozen";
  publishRollYawDiagnostics();
}
void GimbalrotorPerchingNavigator::updateRollYawBodyPitchFeedback()
{
  rezero_rate_valid_ = false;
  roll_yaw_rezero_feedback_dt_ = 0;
  const auto f = feedback();
  if(!roll_yaw::feedbackRejection(f, feedback_config_, planning_config_).empty())
  {
    rezero_previous_time_ = 0;
    return;
  }
  double roll, pitch, yaw;
  tf::Matrix3x3(tfQuaternion(f.body.linear())).getRPY(roll, pitch, yaw);
  if(rezero_previous_time_ > 0)
  {
    roll_yaw_rezero_feedback_dt_ = f.body_stamp - rezero_previous_time_;
    if(roll_yaw_rezero_feedback_dt_ > 0 && roll_yaw_rezero_feedback_dt_ <= .1)
    {
      rezero_measured_rate_ = std::remainder(pitch - rezero_previous_pitch_, 2 * M_PI) / roll_yaw_rezero_feedback_dt_;
      rezero_rate_valid_ = std::isfinite(rezero_measured_rate_);
    }
  }
  rezero_previous_pitch_ = pitch;
  rezero_previous_time_ = f.body_stamp;
}
void GimbalrotorPerchingNavigator::failRollYawPitchRezero(const std::string& why)
{
  invalidatePlanner();
  roll_yaw_pitch_rezero_state_ = RollYawPitchRezeroState::HOLD;
  roll_yaw_pitch_rezero_ready_ = false;
  roll_yaw_pitch_rezero_failed_ = true;
  rezero_stable_since_ = 0;
  reason_ = "pitch rezero failed: " + why + "; holding last valid pose and frozen joints";
  previous_failure_ = reason_;
  ROS_WARN_STREAM("[RollYaw] " << reason_);
}
bool GimbalrotorPerchingNavigator::verifyRollYawPitchRezeroFrozenJoints()
{
  const auto f = feedback();
  const auto why = roll_yaw::feedbackRejection(f, feedback_config_, planning_config_);
  if(!why.empty())
  {
    if(!roll_yaw_pitch_rezero_failed_)
      failRollYawPitchRezero(why);
    return false;
  }
  if(std::abs(std::remainder(f.eta[1] - rezero_frozen_[1], 2 * M_PI)) > rezero_roll_tolerance_ ||
     std::abs(std::remainder(f.eta[2] - rezero_frozen_[2], 2 * M_PI)) > rezero_yaw_tolerance_)
  {
    if(!roll_yaw_pitch_rezero_failed_)
      failRollYawPitchRezero("active roll/yaw moved outside frozen-joint tolerance");
    return false;
  }
  return true;
}
void GimbalrotorPerchingNavigator::updateRollYawPitchRezeroCommand()
{
  const auto f = feedback();
  const double elapsed = f.now - rezero_started_;
  if(!std::isfinite(elapsed) || elapsed < 0 || roll_yaw_rezero_feedback_dt_ < 0 || roll_yaw_rezero_feedback_dt_ > .5)
    return failRollYawPitchRezero("invalid or discontinuous time");
  if(elapsed > rezero_timeout_)
    return failRollYawPitchRezero("timeout before measured stabilization");
  double roll, pitch, yaw;
  tf::Matrix3x3(tfQuaternion(f.body.linear())).getRPY(roll, pitch, yaw);
  const double error = std::remainder(rezero_pitch_ - pitch, 2 * M_PI);
  if(rezero_rate_valid_ && std::abs(error) <= rezero_tolerance_ &&
     std::abs(rezero_measured_rate_) <= rezero_rate_tolerance_)
  {
    if(rezero_stable_since_ == 0)
      rezero_stable_since_ = f.body_stamp;
    if(f.body_stamp - rezero_stable_since_ >= rezero_stable_duration_)
    {
      roll_yaw_pitch_rezero_state_ = RollYawPitchRezeroState::HOLD;
      roll_yaw_pitch_rezero_ready_ = true;
      roll_yaw_pitch_rezero_failed_ = false;
      reason_ = "pitch rezero ready-HOLD: measured pitch/rate stable; request final relock";
      return;
    }
  }
  else
    rezero_stable_since_ = 0;
  if(!std::isfinite(roll_yaw_rezero_feedback_dt_) || roll_yaw_rezero_feedback_dt_ <= 0)
    return;
  // Exact multilink signed error/rate law. Alpha rotates about R_WP * axis_,
  // including the URDF's approximately pi mounting yaw; never world Euler Y.
  const double rate = rezero_sign_ * std::clamp(rezero_kp_ * error, -rezero_rate_, rezero_rate_);
  const double alpha =
      std::clamp(rezero_alpha_ + rate * std::min(roll_yaw_rezero_feedback_dt_, .1), -rezero_travel_, rezero_travel_);
  Eigen::Vector3d q = rezero_frozen_;
  q[0] = alpha;
  roll_yaw::Reference next;
  std::string why;
  if(!roll_yaw::mechanismAllowed(model_, q, planning_config_, why) ||
     !roll_yaw::mechanismReference(model_, q, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), next))
    return failRollYawPitchRezero("invalid/limited passive hinge target: " + why);
  // Reject a sign/geometry combination that drives measured pitch away from the
  // goal. This is a local forward response check, not an endpoint IK/controller.
  Eigen::Vector3d lo = rezero_frozen_, hi = rezero_frozen_;
  roll_yaw::Estimate estimate;
  if(!model_.estimate(f.body, f.eta, estimate))
    return failRollYawPitchRezero("invalid hinge estimate");
  lo[0] = estimate.alpha - 1e-5;
  hi[0] = estimate.alpha + 1e-5;
  Eigen::Isometry3d b0, g0, b1, g1;
  if(!model_.evaluate(lo, b0, g0) || !model_.evaluate(hi, b1, g1))
    return failRollYawPitchRezero("invalid hinge response FK");
  const double slope =
      (std::asin(std::clamp(-b1.linear()(2, 0), -1.0, 1.0)) - std::asin(std::clamp(-b0.linear()(2, 0), -1.0, 1.0))) /
      2e-5;
  if(std::abs(error) > rezero_tolerance_ && rezero_sign_ * slope <= 1e-6)
    return failRollYawPitchRezero("configured rezero sign does not reduce pitch error about this physical hinge");
  final_target_ = next;
  rezero_alpha_ = alpha;
}
void GimbalrotorPerchingNavigator::applyRollYawPitchRezeroTarget()
{
  const bool frozen_valid = verifyRollYawPitchRezeroFrozenJoints();
  if(roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::ACTIVE && frozen_valid)
    updateRollYawPitchRezeroCommand();
  nominal_ = final_target_.eta;
  nominal_[1] = rezero_frozen_[1];
  nominal_[2] = rezero_frozen_[2];
}
void GimbalrotorPerchingNavigator::commandRollYawBaselinkPose(const roll_yaw::Reference& r)
{
  const Eigen::Matrix3d relative = r.cog.linear().transpose() * r.body.linear();
  setSynchronizedBaselinkRotationRelativeToCog(tfQuaternion(relative));
  double roll, pitch, yaw;
  tf::Matrix3x3(tfQuaternion(r.cog.linear())).getRPY(roll, pitch, yaw);
  setXyControlMode(POS_CONTROL_MODE);
  setTargetPos(tfVector(r.cog.translation()));
  setTargetVel(tfVector(r.cog_velocity));
  setTargetAcc(tfVector(r.cog_acceleration));
  setTargetRPY(tf::Vector3(roll, pitch, yaw));
  setTargetOmega(tfVector(r.cog_omega));
  setTargetAngAcc(tfVector(r.cog_angular_acceleration));
}
void GimbalrotorPerchingNavigator::publishRollYawJointTarget()
{
  sensor_msgs::JointState cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.name = {roll_, yaw_};
  cmd.position = {final_target_.eta[1], final_target_.eta[2]};
  roll_yaw_joints_pub_.publish(cmd);  // Passive alpha is never an actuator.
}
void GimbalrotorPerchingNavigator::applyRollYawTarget()
{
  target_applied_this_update_ = true;
  if(!lock_valid_ || !feedback().hover)
    return;
  const auto f = feedback();
  const bool clock_valid = preparation_clock_ == 0 || f.now >= preparation_clock_;
  preparation_clock_ = f.now;
  if(!clock_valid || !std::isfinite(f.now))
    faultHold("feedback clock moved backwards; recapture contact");
  const auto observation = roll_yaw::feedbackStatus(f, feedback_config_, planning_config_);
  if(!faulted_ && observation == roll_yaw::FeedbackStatus::WAITING)
  {
    if(!feedback_waiting_)
    {
      if(jog_.active)
      {
        finishPitchJog("cancelled", "feedback lost; fresh jog required after recovery");
        nominal_ = final_target_.eta;
        resetCompliance();
      }
      if(connection_pending_)
      {
        invalidatePlanner();
        resetCompliance();
      }
      if(prepare_pending_)
      {
        ++prepare_cancellations_;
        ++producer_generation_;
        prepare_worker_->cancel();
        prepare_pending_ = false;
        if(planner_)
        {
          planner_->plan = roll_yaw::Plan();
          planner_->hold("coherent preparation start lost; prepare again");
        }
      }
      // Emergency observation hold: no claim of acceleration-continuous stopping.
      scaleReference(final_target_, 0, 0);
      manual_path_ = roll_yaw::Path();
      manual_phase_ = 0;
      manual_speed_ = 1;
    }
    feedback_waiting_ = true;
    manual_time_ = f.now;
    compliance_time_ = f.now;
    preparation_window_.reset();
    preparation_stats_ = roll_yaw::SettlingStatistics();
    preparation_ready_ = false;
    rezero_previous_time_ = rezero_stable_since_ = 0;
    rezero_rate_valid_ = false;
    if(roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::HOLD && roll_yaw_pitch_rezero_ready_ && !roll_yaw_pitch_rezero_failed_)
      roll_yaw_pitch_rezero_state_ = RollYawPitchRezeroState::ACTIVE;  // Re-establish measured settling, with the original timeout.
    if(roll_yaw_pitch_rezero_state_ != RollYawPitchRezeroState::IDLE)
      roll_yaw_pitch_rezero_ready_ = false;
    if(roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::ACTIVE && f.now - rezero_started_ > rezero_timeout_)
      failRollYawPitchRezero("timeout while waiting for coherent feedback");
    reason_ = "waiting for coherent feedback: " + roll_yaw::feedbackRejection(f, feedback_config_, planning_config_);
    if(planner_)
    {
      planner_->applied = final_target_;
      planner_->applied_valid = true;
      planner_->waitForFeedback(f.now, reason_);
      admittance_.offset = admittance_.enabled || std::abs(admittance_.offset) > 1e-10
                               ? std::clamp(planner_->applied_relief, -admittance_.config.angle_limit, 0.0)
                               : 0;
      admittance_.rate = admittance_.acceleration = 0;
      planner_->relief = admittance_.offset;
    }
    if(stop_pending_)
      completeStop("observation hold completed cancellation");
    commandRollYawBaselinkPose(final_target_);
    publishRollYawJointTarget();
    return;
  }
  const auto why = contactRejection(f);
  if(why.empty() && !faulted_)
  {
    // A faster source can change while min(body,joint) repeats. Its excursion
    // must invalidate quiet history, not disappear before the next joint sample.
    if(f.body_omega.norm() >= feedback_config_.settle_body_rate ||
       f.joint_rate.tail<2>().norm() >= feedback_config_.settle_joint_rate)
      preparation_window_.reset();
    preparation_stats_ = preparation_window_.observe(
        std::min(f.body_stamp, f.joint_stamp), {{f.body_omega.x(), f.body_omega.y(), f.body_omega.z()}},
        {{f.joint_rate[1], f.joint_rate[2]}}, feedback_config_.preparation_quiet_time, feedback_config_.timeout);
    preparation_ready_ = preparation_stats_.qualifies(
        feedback_config_.settle_body_rate, feedback_config_.settle_joint_rate, feedback_config_.settle_mean_rate);
  }
  else
  {
    preparation_window_.reset();
    preparation_stats_ = roll_yaw::SettlingStatistics();
    preparation_ready_ = false;
  }
  if(!why.empty())
    faultHold(why);
  // Preserve the articulated-model drift guard outside the two active joints.
  if(robot_model_ && robot_model_->initialized())
  {
    const auto frames = robot_model_->getSegmentsTf();
    const auto b = frames.find(robot_model_->getBaselinkName());
    Eigen::Isometry3d body, cog;
    if(b != frames.end() && model_.evaluate(f.eta, body, cog))
    {
      const Eigen::Vector3d actual = roll_yaw::fromKdl(b->second.Inverse() * robot_model_->getCog<KDL::Frame>()).translation();
      if((actual - (body.inverse() * cog).translation()).norm() > cog_model_tolerance_)
        faultHold("articulated CoG changed beyond snapshot tolerance; recapture required");
    }
  }
  Eigen::Vector3d current_eta = f.eta;
  roll_yaw::Estimate current_contact;
  if(!faulted_ && model_.estimate(f.body, f.eta, current_contact))
    current_eta[0] = current_contact.alpha;
  if(!faulted_ && roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::IDLE &&
     ((current_eta - final_target_.eta).cwiseAbs().maxCoeff() > feedback_config_.tracking_tolerance ||
      roll_yaw::logSO3(f.body.linear().transpose() * final_target_.body.linear()).norm() >
          feedback_config_.orientation_tolerance))
    faultHold("body/active-joint reference tracking tolerance exceeded");
  if(feedback_waiting_ && !faulted_)
  {
    feedback_waiting_ = false;
    manual_time_ = f.now;
    manual_phase_ = 0;
    manual_speed_ = 1;
    rezero_previous_time_ = 0;
    rezero_rate_valid_ = false;
    roll_yaw_rezero_feedback_dt_ = 0;
    if(!planner_ && roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::IDLE)
    {
      std::string error;
      if(!roll_yaw::mechanismPath(model_, final_target_, nominal_, planning_config_, manual_path_, error))
        faultHold("manual recovery reference: " + error);
      else
        reason_ = "coherent feedback restored; resuming bounded manual target";
    }
  }
  if(!faulted_)
  {
    if(roll_yaw_pitch_rezero_state_ != RollYawPitchRezeroState::IDLE)
      applyRollYawPitchRezeroTarget();
    else
    {
      if(connection_pending_)
        advancePreparedConnection();
      if(planner_ && !faulted_)
      {
        const auto before_compliance = admittance_;
        updateInteraction(f);
        planner_->applied = final_target_;
        planner_->applied_valid = true;
        planner_->tick(f);
        if(!planner_->reference_updated)
        {
          // Keep fresh observation/filter history, but commit correction dynamics
          // only with an accepted reference step (clock/bootstrap returns included).
          admittance_.offset = before_compliance.offset;
          admittance_.rate = before_compliance.rate;
          admittance_.acceleration = before_compliance.acceleration;
          planner_->relief = admittance_.offset;
        }
        if(planner_->stage == roll_yaw::Stage::PAUSED)
        {
          // Reconcile against the actual held on-path reference. No hidden spring
          // displacement or velocity is accumulated across a long pause.
          admittance_.offset = admittance_.enabled || std::abs(admittance_.offset) > 1e-10
                                   ? std::clamp(planner_->applied_relief, -admittance_.config.angle_limit, 0.0)
                                   : 0;
          admittance_.rate = admittance_.acceleration = 0;
          planner_->relief = admittance_.offset;
        }
        if(planner_->stage == roll_yaw::Stage::FAULT_HOLD)
          faultHold("trajectory fault: " + planner_->first_fault);
        else
        {
          nominal_ = planner_->target.eta;
          reason_ = planner_->reason;
        }
      }
      roll_yaw::Reference next;
      if(!faulted_ && buildFinalMechanismTarget(next))
      {
        final_target_ = next;
        if(stop_pending_ && planner_ && planner_->stage == roll_yaw::Stage::COMPLETED_HOLD)
        {
          completeStop("trajectory braking finished");
        }
        if(planner_ && planner_->plan.ok)
        {
          double residual;
          roll_yaw::taskPitch(roll_yaw::cuttingOrientation(planner_->plan.neutral, planner_->plan.goal, 0, 0, 0).rotation,
                    final_target_.body.linear(), control_pitch_, control_pitch_, residual);
        }
      }
    }
  }
  if(!roll_yaw::poseValid(final_target_.body) || !roll_yaw::poseValid(final_target_.cog))
  {
    faultHold("invalid controller reference");
    return;
  }
  commandRollYawBaselinkPose(final_target_);
  publishRollYawJointTarget();
}
void GimbalrotorPerchingNavigator::updateRollYaw()
{
  const double update_started = roll_yaw::steadySeconds();
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(force_landing_flag_ && ownsRollYawTargets())
    selectPerchingMode(perching_geometry::Mode::NORMAL, false);
  if(lock_valid_ && perchingSession().lock_stamp != roll_yaw_lock_stamp_)
    faultHold("base lock identity replaced");
  updateRollYawBodyPitchFeedback();
  target_applied_this_update_ = false;
  updateCommonPerching();
  // Roll-yaw owns coherent body/joint output throughout an enabled lock.
  if(lock_valid_ && !target_applied_this_update_)
    applyRollYawTarget();
  if(auto plan = prepare_worker_->take())
  {
    const bool expected = prepare_pending_;
    prepare_pending_ = false;
    prepare_wall_ = plan->wall_seconds;
    if(expected)
      prepare_result_ = plan->reason;
    if(expected && planner_ && pending_generation_ == producer_generation_ && lock_valid_ && !faulted_)
    {
      if(plan->session != roll_yaw_lock_stamp_.toNSec())
        faultHold("invalid prepare worker session");
      else if(!planner_->submissionMatches(*plan, feedback(), final_target_))
      {
        planner_->hold("applied start changed during prepare; prepare again");
        planner_->plan = roll_yaw::Plan();
        resetCompliance();
      }
      else if(planner_->accept(std::move(*plan)))
      {
        connection_pending_ = true;
        connection_wait_started_ = roll_yaw::steadySeconds();
        advancePreparedConnection();
      }
      reason_ = planner_->reason;
      prepare_result_ = reason_;
      std_msgs::String result_ack;
      result_ack.data = "action=prepare_result session=" + std::to_string(roll_yaw_lock_stamp_.toNSec()) +
                        " accepted=" + (planner_->plan.ok ? "true" : "false") + " phase=" +
                        (connection_pending_                   ? "waiting"
                         : planner_->stage == roll_yaw::Stage::PRE_ALIGN ? "moving"
                                                               : "failed") +
                        " plan_ready=" + (planner_->plan.ok ? "true" : "false") +
                        " motion_active=" + (planner_->stage == roll_yaw::Stage::PRE_ALIGN ? "true" : "false") +
                        " reason=" + reason_;
      ack_pub_.publish(result_ack);
    }
  }
  if(lock_valid_ && !faulted_ && roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::IDLE)
  {
    if(!planner_)
      updateInteraction(feedback());
  }
  publishRollYawDiagnostics();
  navigation_wall_ = roll_yaw::steadySeconds() - update_started;
}
void GimbalrotorPerchingNavigator::publishRollYawDiagnostics()
{
  const auto f = feedback();
  const double now = roll_yaw::steadySeconds();
  const std::string transition = !ownsRollYawTargets()                          ? "FREE_FLIGHT"
                                 : faulted_                                        ? "FAULT_HOLD"
                                 : roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::ACTIVE ? "PITCH_REZERO_ACTIVE"
                                 : roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::HOLD   ? "PITCH_REZERO_HOLD"
                                 : planner_                                        ? roll_yaw::stageName(planner_->stage)
                                                                                   : "CONTACT_LOCKED";
  const bool changed = transition != last_status_;
  if(changed)
  {
    if(faulted_)
      ROS_ERROR_STREAM("[RollYaw] " << transition << ": " << previous_failure_);
    else
      ROS_INFO_STREAM("[RollYaw] " << transition << ": " << reason_);
    last_status_ = transition;
  }
  if(!changed && now - last_diagnostics_ < .2)
    return;
  last_diagnostics_ = now;
  if(lock_valid_)
    ROS_INFO_STREAM_THROTTLE(2.0, "[RollYaw] " << transition << ": " << reason_ << "; " << admittance_reason_);
  std_msgs::UInt64 session;
  session.data = lock_valid_ ? roll_yaw_lock_stamp_.toNSec() : 0;
  session_pub_.publish(session);
  std_msgs::Bool flag;
  flag.data = roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::ACTIVE;
  rezero_active_pub_.publish(flag);
  flag.data = roll_yaw_pitch_rezero_ready_;
  rezero_ready_pub_.publish(flag);
  flag.data = roll_yaw_pitch_rezero_failed_;
  rezero_failed_pub_.publish(flag);
  const char* state = !ownsRollYawTargets()                          ? "FREE_FLIGHT"
                      : !lock_valid_                                    ? "LOCK_FAILED"
                      : roll_yaw_pitch_rezero_failed_                            ? "PITCH_REZERO_FAILED"
                      : faulted_                                        ? "FAULT_HOLD"
                      : roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::ACTIVE ? "PITCH_REZERO_ACTIVE"
                      : roll_yaw_pitch_rezero_state_ == RollYawPitchRezeroState::HOLD   ? "PITCH_REZERO_HOLD"
                      : planner_                                        ? roll_yaw::stageName(planner_->stage)
                                                                        : "CONTACT_LOCKED";
  std_msgs::String status;
  status.data = std::string(state) + ": " + reason_;
  if((faulted_ || roll_yaw_pitch_rezero_failed_) && !previous_failure_.empty() && previous_failure_ != reason_)
    status.data += "; hold cause: " + previous_failure_;
  status.data += " jog_active=" + std::to_string(jog_.active ? 1 : 0) + " jog_phase=" + jog_.phase;
  roll_yaw_state_pub_.publish(status);
  if(lock_valid_)
  {
    geometry_msgs::Vector3Stamped angles;
    angles.header.stamp = ros::Time::now();
    angles.header.frame_id = "perching_lock_" + std::to_string(roll_yaw_lock_stamp_.toNSec());
    // Effective configured/URDF intersection from snapshotModel(). Wire order
    // matches nominal_angles [roll, alpha, yaw], not internal [alpha, roll, yaw].
    const auto& limits = planning_config_.limits;
    angles.vector.x = limits.lower[1];
    angles.vector.y = limits.lower[0];
    angles.vector.z = limits.lower[2];
    position_lower_pub_.publish(angles);
    angles.vector.x = limits.upper[1];
    angles.vector.y = limits.upper[0];
    angles.vector.z = limits.upper[2];
    position_upper_pub_.publish(angles);
    angles.vector.x = nominal_[1];
    angles.vector.y = nominal_[0];
    angles.vector.z = nominal_[2];
    nominal_pub_.publish(angles);
    angles.vector.x = final_target_.eta[1];
    angles.vector.y = final_target_.eta[0];
    angles.vector.z = final_target_.eta[2];
    angles_pub_.publish(angles);
    std_msgs::Float64 pitch;
    pitch.data = nominal_[0];
    pitch_pub_.publish(pitch);
    nav_msgs::Odometry body;
    body.header.stamp = ros::Time::now();
    body.header.frame_id = world_;
    body.child_frame_id = body_frame_;
    body.pose.pose = poseMsg(final_target_.body);
    tf::vector3TFToMsg(tfVector(final_target_.body_omega), body.twist.twist.angular);
    reference_pub_.publish(body);
    body.header.stamp.fromSec(f.body_stamp);
    body.pose.pose = poseMsg(f.body);
    tf::vector3TFToMsg(tfVector(f.body_omega), body.twist.twist.angular);
    measured_pub_.publish(body);
    body.header.stamp = ros::Time::now();
    body.child_frame_id = "desired_cog";
    body.pose.pose = poseMsg(final_target_.cog);
    tf::vector3TFToMsg(tfVector(final_target_.cog_omega), body.twist.twist.angular);
    tf::vector3TFToMsg(tfVector(final_target_.cog.linear().transpose() * final_target_.cog_velocity),
                       body.twist.twist.linear);
    cog_pub_.publish(body);
    if(roll_yaw::feedbackStatus(f, feedback_config_, planning_config_) == roll_yaw::FeedbackStatus::READY)
      model_.estimate(f.body, f.eta, measured_contact_);
  }
  std::ostringstream out;
  out << std::setprecision(12) << "eta=[passive alpha, active roll, active yaw]; units=rad,m,s; state=" << state
      << " lock=" << session.data << " reason=[" << reason_ << "]"
      << " nominal=" << nominal_.transpose() << " applied=" << final_target_.eta.transpose()
      << " measured_active=" << f.eta.tail<2>().transpose() << " measured_alpha=" << measured_contact_.alpha
      << " joint_age=" << f.now - f.joint_stamp << " body_age=" << f.now - f.body_stamp << " joint_failure=["
      << joint_failure_ << "] body_failure=[" << body_failure_ << "]"
      << " pivot_error=" << measured_contact_.pivot_error << " hinge_residual=" << measured_contact_.hinge_residual
      << " rezero_alpha=" << rezero_alpha_ << " rezero_pitch=" << rezero_previous_pitch_
      << " rezero_pitch_rate=" << rezero_measured_rate_ << " rezero_rate_valid=" << rezero_rate_valid_
      << " rezero_sign=" << rezero_sign_ << " manual_receipt=" << manual_receipt_stamp_.toSec()
      << " ignored_body_stamps=" << body_ignored_stamps_ << " ignored_joint_stamps=" << joint_ignored_stamps_;
  out << " pivot_limit=" << feedback_config_.pivot_tolerance << " hinge_limit=" << feedback_config_.hinge_tolerance
      << " first_fault=[" << previous_failure_ << "] admittance=[" << admittance_reason_ << "]";
  if(planner_ && !planner_->plan.goal.align_only)
    out << " phase_s=" << planner_->progress << " planned_pitch=" << planner_->planned_pitch
        << " measured_pitch=" << planner_->task.raw << " measured_progress_rad=" << planner_->task.current
        << " original_goal=" << planner_->plan.goal.pitch_end << " goal_error=" << planner_->goal_error
        << " control_pitch=" << control_pitch_ << " goal_reached=" << planner_->goal_reached;
  std_msgs::String detail;
  detail.data = out.str();
  details_pub_.publish(detail);
  diagnostic_msgs::DiagnosticArray array;
  array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus diagnostic;
  diagnostic.name = "gimbalrotor/roll_yaw_perching";
  diagnostic.hardware_id = "perching_lock_" + std::to_string(session.data);
  diagnostic.level = faulted_ ? diagnostic_msgs::DiagnosticStatus::ERROR
                     : (feedback_waiting_ || prepare_pending_ || (planner_ && planner_->progress_lag))
                         ? diagnostic_msgs::DiagnosticStatus::WARN
                         : diagnostic_msgs::DiagnosticStatus::OK;
  diagnostic.message = std::string(state) + ": " + reason_;
  auto field = [&](const std::string& name, const auto& value)
  {
    diagnostic_msgs::KeyValue item;
    item.key = name;
    std::ostringstream text;
    text << std::setprecision(17) << value;
    item.value = text.str();
    diagnostic.values.push_back(std::move(item));
  };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  field("lock_id", session.data);
  field("session_id", session.data);
  field("plan_id", plan_id_);
  field("producer_generation", producer_generation_);
  field("first_fault", previous_failure_);
  field("feedback_waiting", feedback_waiting_);
  field("stopping", stop_pending_);
  field("manual_callback_wall_s", manual_wall_);
  const bool jog_valid = lock_valid_ && jog_.session == roll_yaw_lock_stamp_.toNSec() && jog_.session != 0;
  double jog_measured = nan, jog_cross = nan, jog_error = nan;
  if(jog_valid && contactRejection(f).empty())
  {
    if(!roll_yaw::taskPitch(jog_.start.linear(), f.body.linear(), jog_.progress, jog_measured, jog_cross))
      jog_measured = nan;
    else
      jog_error = roll_yaw::logSO3(f.body.linear().transpose() * jog_.endpoint).norm();
  }
  field("body_pitch_jog", jog_.active);
  field("body_pitch_jog_phase", jog_.phase);
  field("body_pitch_jog_route", "manual_endpoint_ik");
  field("body_pitch_jog_admission_wall_s", jog_admission_wall_);
  field("body_pitch_jog_delta_rad", jog_valid ? jog_.delta : nan);
  field("body_pitch_jog_progress_rad", jog_valid ? jog_.progress : nan);
  field("body_pitch_jog_measured_pitch_rad", jog_measured);
  field("body_pitch_jog_measured_endpoint_error_rad", jog_error);
  field("body_pitch_jog_max_delta_rad", body_pitch_jog_max_delta_);
  field("prepared_connection_pending", connection_pending_);
  double alignment_error = nan;
  if(planner_ && planner_->plan.ok && planner_->plan.goal.align_only && contactRejection(f).empty())
    alignment_error =
        roll_yaw::logSO3(f.body.linear().transpose() * planner_->plan.align.desired(planner_->plan.align.duration()).rotation)
            .norm();
  field("alignment_endpoint_error_rad", alignment_error);
  field("geometry_prepared", planner_ && planner_->plan.ok);
  field("cut_duration_s", planner_ && planner_->plan.ok ? planner_->plan.cut.duration() : nan);
  field("time_scale", planner_ && planner_->plan.ok ? std::pow(2.0, planner_->plan.scale_attempt) : nan);
  field("saw_proxy_valid", planning_config_.saw_point_valid);
  field("saw_proxy_drop_m", planner_ && planner_->plan.ok ? planner_->plan.proxy_drop : nan);
  field("body_alignment_pending", prepare_pending_ && planner_ && planner_->plan.goal.align_only);
  field("body_alignment_pending_yaw_rad",
        prepare_pending_ && planner_ && planner_->plan.goal.align_only ? planner_->plan.goal.yaw : nan);
  field("body_alignment_pending_roll_rad",
        prepare_pending_ && planner_ && planner_->plan.goal.align_only ? planner_->plan.goal.roll : nan);
  field("body_alignment_yaw_rad", body_alignment_valid_ ? alignment_request_.yaw : nan);
  field("body_alignment_roll_rad", body_alignment_valid_ ? alignment_request_.roll : nan);
  field("prepare_count", plan_id_);
  field("prepare_cancellations", prepare_cancellations_);
  field("prepare_result", prepare_result_);
  field("prepare_generation", pending_generation_);
  field("preparation_ready", preparation_ready_);
  field("preparation_window_s", preparation_stats_.observed);
  field("prepare_pending", prepare_pending_);
  field("prepare_elapsed_s", prepare_pending_ ? now - prepare_started_ : prepare_wall_);
  field("prepare_solve_wall_s", planning_status_->solve_seconds.load());
  field("prepare_validation_wall_s", planning_status_->validation_seconds.load());
  field("prepare_deadline_s", prepare_deadline_);
  field("prepare_interval", planning_status_->interval.load());
  field("prepare_intervals", planning_status_->total.load());
  field("prepare_validating", planning_status_->validating.load());
  field("body_stamp_s", f.body_stamp);
  field("joint_stamp_s", f.joint_stamp);
  field("body_age_s", f.now - f.body_stamp);
  field("joint_age_s", f.now - f.joint_stamp);
  field("body_receipt_age_s", f.now - body_receipt_);
  field("joint_receipt_age_s", f.now - joint_receipt_);
  field("feedback_skew_s", std::abs(f.body_stamp - f.joint_stamp));
  field("feedback_skew_limit_s", synchronization_tolerance_);
  field("body_invalid_messages", body_invalid_messages_);
  field("joint_invalid_messages", joint_invalid_messages_);
  field("last_body_message_stamp_s", last_body_message_stamp_);
  field("last_joint_message_stamp_s", last_joint_message_stamp_);
  field("feedback_timeout_s", feedback_config_.timeout);
  field("body_ignored_stamps", body_ignored_stamps_);
  field("joint_ignored_stamps", joint_ignored_stamps_);
  field("body_failure", body_failure_);
  field("joint_failure", joint_failure_);
  field("pivot_error_m", lock_valid_ ? measured_contact_.pivot_error : nan);
  field("pivot_limit_m", feedback_config_.pivot_tolerance);
  field("hinge_residual_rad", lock_valid_ ? measured_contact_.hinge_residual : nan);
  field("hinge_limit_rad", feedback_config_.hinge_tolerance);
  field("effective_body_tracking_error_rad",
        lock_valid_ ? roll_yaw::logSO3(f.body.linear().transpose() * final_target_.body.linear()).norm() : nan);
  field("effective_body_tracking_limit_rad", feedback_config_.orientation_tolerance);
  field("effective_joint_tracking_error_rad",
        lock_valid_ ? (f.eta.tail<2>() - final_target_.eta.tail<2>()).cwiseAbs().maxCoeff() : nan);
  field("effective_joint_tracking_limit_rad", feedback_config_.tracking_tolerance);
  const bool goal_valid = planner_ && planner_->plan.goal.valid() && !planner_->plan.goal.align_only;
  const bool measured_task_valid = goal_valid && planner_->task.valid && !faulted_ &&
                                   f.now - planner_->task.stamp <= feedback_config_.timeout &&
                                   contactRejection(f).empty();
  const bool fresh_torque = interaction_valid_ && interaction_stamp_ > 0 && !faulted_ &&
                            f.now - interaction_stamp_ <= feedback_config_.timeout &&
                            now - interaction_receipt_ <= feedback_config_.timeout;
  field("goal_valid", goal_valid);
  field("original_goal_rad", goal_valid ? planner_->plan.goal.pitch_end : nan);
  field("planned_pitch_rad", goal_valid ? planner_->planned_pitch : nan);
  field("nominal_phase_s", planner_ ? planner_->nominal_progress : nan);
  field("effective_phase_s", planner_ ? planner_->progress : nan);
  field("phase_speed", planner_ ? planner_->phaseSpeed() : nan);
  field("resisting_load", planner_ && planner_->resisting_load);
  field("relief_limit_saturated", planner_ && planner_->relief_limited);
  field("measured_task_valid", measured_task_valid);
  field("measured_pitch_rad", measured_task_valid ? planner_->task.raw : nan);
  field("filtered_pitch_rad", measured_task_valid ? planner_->task.filtered : nan);
  field("physical_progress_rad", measured_task_valid ? planner_->task.current : nan);
  field("maximum_progress_rad", measured_task_valid ? planner_->task.distance : nan);
  field("cross_track_rad", measured_task_valid ? planner_->task.cross_track : nan);
  field("goal_error_rad", measured_task_valid ? planner_->goal_error : nan);
  field("goal_orientation_error_rad", measured_task_valid ? planner_->goal_orientation_error : nan);
  field("control_pitch_rad", goal_valid ? control_pitch_ : nan);
  field("path_end_commanded", planner_ && planner_->path_end_commanded);
  field("goal_reached", planner_ && planner_->goal_reached);
  field("goal_pending", goal_valid && !planner_->goal_reached);
  field("progress_lag", planner_ && planner_->progress_lag);
  field("reference_lead_rad", feedback_config_.reference_lead);
  field("goal_tolerance_rad", feedback_config_.goal_tolerance);
  field("current_body_rate_rad_s", body_valid_ ? f.body_omega.norm() : nan);
  field("current_joint_rate_rad_s", joint_valid_ ? f.joint_rate.tail<2>().norm() : nan);
  field("settling_valid", planner_ && planner_->settling.complete);
  field("settling_elapsed_s", planner_ ? planner_->settling_elapsed : nan);
  field("settling_window_s", planner_ ? planner_->settling.observed : nan);
  field("settling_body_rms_rad_s", planner_ && planner_->settling.complete ? planner_->settling.body_rms : nan);
  field("settling_joint_rms_rad_s", planner_ && planner_->settling.complete ? planner_->settling.joint_rms : nan);
  field("settling_body_mean_rad_s", planner_ && planner_->settling.complete ? planner_->settling.body_mean : nan);
  field("settling_joint_mean_rad_s", planner_ && planner_->settling.complete ? planner_->settling.joint_mean : nan);
  field("settling_body_limit_rad_s", feedback_config_.settle_body_rate);
  field("settling_joint_limit_rad_s", feedback_config_.settle_joint_rate);
  field("settling_mean_limit_rad_s", feedback_config_.settle_mean_rate);
  field("settling_timeout_s", feedback_config_.settle_timeout);
  field("admittance_enabled", admittance_.enabled);
  field("admittance_status", admittance_reason_);
  field("contact", admittance_.contact);
  field("baseline_ready", admittance_.baseline_ready);
  field("baseline_samples", admittance_.samples);
  field("bias_Nm", admittance_.baseline_ready ? admittance_.bias : nan);
  field("task_torque_valid", fresh_torque);
  field("task_torque_Nm", fresh_torque ? interaction_torque_ : nan);
  field("torque_stamp_s", interaction_stamp_);
  field("interaction_source", interaction_topic_);
  field("interaction_frame", interaction_frame_);
  field("interaction_point", interaction_point_);
  field("interaction_sign", interaction_sign_);
  field("interaction_scale", interaction_scale_);
  field("torque_age_s", interaction_stamp_ > 0 ? f.now - interaction_stamp_ : nan);
  field("reference_torque_Nm", admittance_.config.reference_torque);
  field("filtered_contact_torque_Nm", admittance_.baseline_ready ? admittance_.filtered : nan);
  double effective_offset = nan;
  if(goal_valid && !planner_->plan.goal.align_only && planner_->plan.ok)
  {
    effective_offset = planner_->applied_relief;
  }
  field("effective_offset_rad", effective_offset);
  field("offset_rad", admittance_.offset);
  field("offset_rate_rad_s", admittance_.rate);
  field("offset_acceleration_rad_s2", admittance_.acceleration);
  field("navigation_wall_s", navigation_wall_);
  array.status.push_back(std::move(diagnostic));
  status_pub_.publish(array);
  geometry_msgs::Vector3Stamped plot;
  plot.header = array.header;
  plot.header.frame_id = "perching_lock_" + std::to_string(session.data);
  plot.vector.x = goal_valid ? planner_->planned_pitch : nan;
  plot.vector.y = measured_task_valid ? planner_->task.raw : nan;
  plot.vector.z = goal_valid ? control_pitch_ : nan;
  task_pub_.publish(plot);
  plot.vector.x = admittance_.offset;
  plot.vector.y = admittance_.rate;
  plot.vector.z = fresh_torque ? interaction_torque_ : nan;
  compliance_pub_.publish(plot);
}
}  // namespace aerial_robot_navigation
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorPerchingNavigator,
                       aerial_robot_navigation::BaseNavigator);
