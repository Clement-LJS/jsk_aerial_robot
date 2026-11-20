#include <gimbalrotor/gimbalrotor_perching_navigation.h>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <iomanip>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <type_traits>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <kdl/treefksolverpos_recursive.hpp>
#include <tf_conversions/tf_eigen.h>
#include <stdexcept>
#include <tuple>
#include <Eigen/SVD>

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
  use_pitch_command_for_arc_(true),

  accept_uav_nav_pitch_command_(false),

  active_perching_hold_enable_(true),

  min_valid_radius_(0.05),
  max_pitch_delta_(0.78539816339),  // 45 deg
  arc_pitch_sign_(1.0),
  command_pitch_sign_(1.0),

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
  locked_radius_(0.0)
{
  branch_pos_world_.setValue(0.0, 0.0, 0.0);
  perching_point_world_.setValue(0.0, 0.0, 0.0);

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
  GimbalrotorNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);
  model_ = boost::dynamic_pointer_cast<Model>(robot_model);
  readProfile();
  initializeMechanism();

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
  getParam<bool>(navi_nh, "perching_use_pitch_command_for_arc", use_pitch_command_for_arc_, true);

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

  if(!std::isfinite(min_valid_radius_) || min_valid_radius_ <= 1.0e-6) min_valid_radius_ = 0.05;
  if(!std::isfinite(max_pitch_delta_) || max_pitch_delta_ <= 0.0 || max_pitch_delta_ > PI)
    max_pitch_delta_ = 0.5235987756;
  if(!std::isfinite(arc_pitch_sign_) || arc_pitch_sign_ == 0.0) arc_pitch_sign_ = 1.0;
  arc_pitch_sign_ = arc_pitch_sign_ >= 0.0 ? 1.0 : -1.0;
  if(!std::isfinite(command_pitch_sign_) || command_pitch_sign_ == 0.0) command_pitch_sign_ = 1.0;
  command_pitch_sign_ = command_pitch_sign_ >= 0.0 ? 1.0 : -1.0;
  if(!std::isfinite(y_compliance_deadband_) || y_compliance_deadband_ < 0.0) y_compliance_deadband_ = 0.03;

  getParam<std::string>(navi_nh, "perching_pivot_source", pivot_source_, std::string("hand_center"));

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

void GimbalrotorPerchingNavigator::update()
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  GimbalrotorNavigator::update();
  if(navi_state_ != HOVER_STATE)
  {
    servo_.zero_hold_armed = false;
    if(perching_mode_ != perching_geometry::Mode::DISABLED) resetPerchingLock();
    perching_mode_ = perching_geometry::Mode::DISABLED;
    return;
  }
  if(model_ && !model_valid_ && robot_model_->initialized()) model_valid_ = resolveMechanismModel();
  const double now = ros::Time::now().toSec();
  const double elapsed = last_update_ > 0 ? now - last_update_ : 0;
  last_update_ = now;
  if(elapsed < 0) { body_history_.clear(); joint_history_.clear(); applied_stamp_ = 0; }
  cycle_dt_ = elapsed > 0 && elapsed <= max_gap_ ? std::min(elapsed, max_step_) : 0;
  if(perching_mode_ == perching_geometry::Mode::DISABLED || !perching_locked_) return;
  if(articulated())
  {
    proposal_valid_ = false;
    if(observation(measured_)) model_->estimate(measured_.body, measured_.q, pivot_error_, hinge_error_);
    if(motionFeedbackValid() && cycle_dt_ > 0) advanceNominal(cycle_dt_);
  }
  else
  {
    Sample sample;
    if(observation(sample)) model_->estimate(sample.body, sample.q, pivot_error_, hinge_error_);
    if(velocity_active_ && now >= velocity_expires_) velocity_active_ = false;
    if(velocity_active_ && cycle_dt_ > 0)
    {
      perching_geometry::Pose pose;
      if(model_->hinge_arc.target(active_pitch_delta_ + velocity_command_ * cycle_dt_ / arc_pitch_sign_,
                                  max_pitch_delta_, arc_pitch_sign_, pose)) active_pitch_delta_ = pose.delta;
      if(std::abs(active_pitch_delta_) >= max_pitch_delta_) velocity_active_ = false;
    }
    if(active_perching_hold_enable_) applyActivePerchingTarget();
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

void GimbalrotorPerchingNavigator::selectPerchingMode(perching_geometry::Mode mode, bool enable)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(enable && !supportsPerchingMode(mode))
  {
    ROS_WARN_THROTTLE(1.0, "Articulated perching supports NORMAL mode only; slanted_enable was ignored.");
    return;
  }
  if(!enable && perching_mode_ != mode) return;

  resetPerchingLock();
  perching_mode_ = perching_geometry::Mode::DISABLED;
  if(!enable) return;

  if(navi_state_ != HOVER_STATE)
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

void GimbalrotorPerchingNavigator::relockCallback(const std_msgs::EmptyConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  (void)msg;
  if(positioning_ && perching_locked_)
  {
    workflowCommand("relock");
    return;
  }

  if(perching_mode_ == perching_geometry::Mode::DISABLED)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] "
        "perching relock ignored because perching is disabled.");

    return;
  }

  if(navi_state_ != HOVER_STATE)
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

void GimbalrotorPerchingNavigator::resetPerchingLock()
{
  resetMotion();
  perching_locked_ = false;
  locked_radius_ = 0.0;
  active_pitch_delta_ = 0.0;
  lock_stamp_ = ros::Time(0);
  if(model_) model_->unlock();
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

  if(navi_state_ != HOVER_STATE)
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

void GimbalrotorPerchingNavigator::applyManualPitchDelta(double delta)
{
  if(articulated())
  {
    if(positioning_) servo_.cut(delta, servoSample());
    else
    {
      const auto it = coordinate_map_.find(manual_coordinate_);
      if(it != coordinate_map_.end()) coordinateCommand(it->second, locked_q_[it->second] + command_pitch_sign_ * delta);
    }
    return;
  }
  perching_geometry::Pose pose;
  if(!model_->hinge_arc.target(command_pitch_sign_ * delta, max_pitch_delta_, arc_pitch_sign_, pose))
    return;
  active_pitch_delta_ = pose.delta;
  velocity_active_ = false;
  aerial_robot_msgs::FlightNav command;
  setPoseCommand(command, pose);
  aerial_robot_msgs::FlightNavConstPtr command_ptr(new aerial_robot_msgs::FlightNav(command));
  GimbalrotorNavigator::naviCallback(command_ptr);
}

bool GimbalrotorPerchingNavigator::tryLockRigid(const std::string& reason)
{
  if(navi_state_ != HOVER_STATE)
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
   *   Use FK from base_link to the URDF perching_pivot marker.
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

  if(!model_) return false;
  model_valid_ = resolveMechanismModel();
  if(!model_valid_) return false;
  locked_robot_pos_world_ = getCurrentRobotPos();

  locked_pivot_world_ = computeLockPivotWorld();
  perching_point_world_ = locked_pivot_world_;

  Pose body = Pose::Identity();
  Eigen::Matrix3d body_rotation;
  Eigen::Vector3d body_position;
  tf::matrixTFToEigen(getCurrentBaselinkRot(), body_rotation);
  tf::vectorTFToEigen(getCurrentBaselinkPos(), body_position);
  body.linear() = body_rotation;
  body.translation() = body_position;
  Eigen::Vector3d pivot;
  tf::vectorTFToEigen(locked_pivot_world_, pivot);
  if(!model_->lock(body, Vector::Zero(1), &pivot)) return false;
  Model::State anchored;
  if(!model_->evaluate(Vector::Zero(1), anchored)) return false;
  const auto& position = anchored.cog.translation();
  locked_robot_pos_world_.setValue(position.x(), position.y(), position.z());
  const Eigen::Quaterniond orientation(anchored.cog.linear());
  const tf::Quaternion reference_orientation(orientation.x(), orientation.y(), orientation.z(), orientation.w());
  const Eigen::Vector3d axis = model_->pivot().linear().col(1);
  if(!model_->hinge_arc.initialize(locked_robot_pos_world_, locked_pivot_world_, reference_orientation,
                                  min_valid_radius_, tf::Vector3(axis.x(), axis.y(), axis.z())))
  {
    ROS_WARN("[GimbalrotorPerchingNavigator] rejected invalid perching geometry");
    return false;
  }
  double roll, pitch, yaw;
  tf::Matrix3x3(reference_orientation).getRPY(roll, pitch, yaw);
  reference_locked_rpy_.setValue(roll, pitch, yaw);
  const tf::Vector3 measured_rpy = getCurrentRobotRPY();
  locked_radius_ = model_->hinge_arc.radial().length();
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
  ROS_WARN("[GimbalrotorPerchingNavigator] reference rpy deg: roll %.2f, pitch %.2f, yaw %.2f",
           reference_locked_rpy_.x() * 180.0 / PI,
           reference_locked_rpy_.y() * 180.0 / PI,
           reference_locked_rpy_.z() * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingNavigator] marker local +Y axis in world: x %.6f, y %.6f, z %.6f",
           model_->hinge_arc.axis.x(), model_->hinge_arc.axis.y(), model_->hinge_arc.axis.z());
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
  pose.header.frame_id = world_;
  tf::pointTFToMsg(cog_position, pose.pose.position);
  tf::quaternionTFToMsg(cog_orientation, pose.pose.orientation);
  locked_pose_pub_.publish(pose);
  publishLockedPivot();
}

void GimbalrotorPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(articulated() && perching_mode_ != perching_geometry::Mode::DISABLED)
  {
    if(accept_uav_nav_pitch_command_ && hasPitchCommand(*msg)) applyManualPitchDelta(msg->target_pitch);
    return;
  }
  aerial_robot_msgs::FlightNav nav_msg = *msg;

  if(perching_mode_ != perching_geometry::Mode::DISABLED && navi_state_ == HOVER_STATE)
  {
    if(!applyPerchingConstraint(nav_msg)) return;
  }

  aerial_robot_msgs::FlightNavConstPtr nav_msg_ptr(new aerial_robot_msgs::FlightNav(nav_msg));

  GimbalrotorNavigator::naviCallback(nav_msg_ptr);
}

void GimbalrotorPerchingNavigator::applyActivePerchingTarget()
{
  if(perching_mode_ == perching_geometry::Mode::DISABLED || navi_state_ != HOVER_STATE)
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

  const auto marker = model_->bodyPivot();
  const auto& p = marker.translation();
  return baselink_pos_world + baselink_rot_world * tf::Vector3(p.x(), p.y(), p.z());
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
  if(perching_mode_ == perching_geometry::Mode::DISABLED || navi_state_ != HOVER_STATE) return false;
  if(!perching_locked_ && !tryLockPerching("first perching command")) return false;

  const bool position_command = hasPositionCommand(msg);
  const bool velocity_command = hasVelocityCommand(msg);
  const tf::Vector3 desired_velocity = getDesiredVelocity(msg);
  perching_geometry::Pose pose;
  if(use_pitch_command_for_arc_ && hasPitchCommand(msg))
  {
    if(command_pitch_as_delta_)
    {
      if(!model_->hinge_arc.target(command_pitch_sign_ * msg.target_pitch,
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
      if(!model_->hinge_arc.targetOrientation(desired, max_pitch_delta_, arc_pitch_sign_, pose)) return false;
    }
    active_pitch_delta_ = pose.delta;
    velocity_active_ = false;
    setPoseCommand(msg, pose);
    return true;
  }

  if(position_command)
  {
    if(!model_->hinge_arc.project(getDesiredPosition(msg), max_pitch_delta_, arc_pitch_sign_, pose) &&
       !model_->hinge_arc.target(active_pitch_delta_, max_pitch_delta_, arc_pitch_sign_, pose)) return false;
    active_pitch_delta_ = pose.delta;
    velocity_active_ = false;
    setPoseCommand(msg, pose);
  }
  else if(!position_command && !velocity_command)
  {
    if(!activePose(pose)) return false;
    setPoseCommand(msg, pose);
    return true;
  }

  if(velocity_command)
  {
    if(!model_->hinge_arc.target(active_pitch_delta_, max_pitch_delta_, arc_pitch_sign_, pose)) return false;
    const tf::Vector3 direction = model_->hinge_arc.axis.cross(pose.position - model_->hinge_arc.pivot);
    const tf::Vector3 tangent = model_->hinge_arc.tangentVelocity(pose.position, desired_velocity);
    velocity_command_ = std::clamp(direction.dot(tangent) / direction.length2(), -rate_limit_, rate_limit_);
    velocity_active_ = std::abs(velocity_command_) > 1e-9;
    velocity_expires_ = ros::Time::now().toSec() + velocity_duration_;
    setPoseCommand(msg, pose);
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
  msg.header.frame_id = world_;

  msg.pose.position.x = locked_robot_pos_world_.x();
  msg.pose.position.y = locked_robot_pos_world_.y();
  msg.pose.position.z = locked_robot_pos_world_.z();

  tf::quaternionTFToMsg(model_->hinge_arc.orientation, msg.pose.orientation);

  locked_pose_pub_.publish(msg);
}

void GimbalrotorPerchingNavigator::publishLockedPivot()
{
  geometry_msgs::PointStamped msg;
  msg.header.stamp = lock_stamp_;
  msg.header.frame_id = world_;

  msg.point.x = locked_pivot_world_.x();
  msg.point.y = locked_pivot_world_.y();
  msg.point.z = locked_pivot_world_.z();

  locked_pivot_pub_.publish(msg);
}

void GimbalrotorPerchingNavigator::publishCommandedDebugPose(const perching_geometry::Pose& pose)
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = world_;
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
  if(!perching_locked_ || !model_->hinge_arc.target(active_pitch_delta_, max_pitch_delta_, arc_pitch_sign_, pose))
    return false;
  return true;
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
  // The physical revolute arc can change every Euler component.
  msg.target_roll = roll;
  msg.target_pitch = pitch;
  msg.target_yaw = yaw;
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
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(articulated() || perching_mode_ == perching_geometry::Mode::DISABLED || !perching_locked_ || navi_state_ != HOVER_STATE ||
     lock_stamp != lock_stamp_ || !perching_geometry::finite(nominal_position)) return false;
  // Compliance Y is a physical axis angle. Convert to the logical coordinate
  // before the shared helper applies arc_pitch_sign exactly once.
  if(!model_->hinge_arc.target(active_pitch_delta_ + physical_offset / arc_pitch_sign_,
                       max_pitch_delta_, arc_pitch_sign_, pose)) return false;
  return true;
}

namespace
{
tf::Vector3 tfVector(const Eigen::Vector3d& v) { return {v.x(), v.y(), v.z()}; }
tf::Quaternion tfQuaternion(const Eigen::Matrix3d& r)
{
  const Eigen::Quaterniond q(r); return {q.x(), q.y(), q.z(), q.w()};
}
geometry_msgs::Pose poseMessage(const Eigen::Isometry3d& p)
{
  geometry_msgs::Pose result;
  tf::pointTFToMsg(tfVector(p.translation()), result.position);
  tf::quaternionTFToMsg(tfQuaternion(p.linear()), result.orientation);
  return result;
}
double steadyTime()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

void GimbalrotorPerchingNavigator::readProfile()
{
  ros::NodeHandle pn(nh_, "navigation/perching");
  pn.getParam("active_joints", active_joints_);
  coordinate_map_["alpha"] = 0;
  for(std::size_t i = 0; i < active_joints_.size(); ++i) coordinate_map_[active_joints_[i]] = i + 1;
  pn.param<std::string>("debug_prefix", prefix_, "perching");
  pn.param<std::string>("diagnostic_name", status_name_, "perching");
  pn.param<std::string>("world_frame", world_, "world");
  pn.param<std::string>("joint_state_topic", joint_topic_, "joint_states");
  pn.param<std::string>("manual_coordinate", manual_coordinate_, "alpha");
  pn.param("pitch_command_sign", command_pitch_sign_, command_pitch_sign_);
  pn.param<std::string>("positioning_joint", positioning_joint_, "");
  positioning_ = !positioning_joint_.empty();
  pn.param("tracking_tolerance", tracking_tolerance_, tracking_tolerance_);
  pn.param("body_tracking_tolerance", body_tracking_tolerance_, body_tracking_tolerance_);
  pn.param("feedback_timeout", timeout_, timeout_);
  pn.param("future_tolerance", future_tolerance_, future_tolerance_);
  pn.param("feedback_sync_tolerance", sync_tolerance_, sync_tolerance_);
  pn.param("max_execution_step", max_step_, max_step_);
  pn.param("max_execution_gap", max_gap_, max_gap_);
  pn.param("body_max_tilt", max_tilt_, max_tilt_);
  pn.param("body_pitch_lower_limit", min_body_pitch_, min_body_pitch_);
  pn.param("body_pitch_upper_limit", max_body_pitch_, max_body_pitch_);
  pn.param("cog_speed_limit", cog_speed_limit_, cog_speed_limit_);
  pn.param("cog_acceleration_limit", cog_acceleration_limit_, cog_acceleration_limit_);
  pn.param("pitch_rate", rate_limit_, rate_limit_);
  pn.param("pitch_acceleration", acceleration_limit_, acceleration_limit_);
  pn.param("body_pitch_velocity_command_duration", velocity_duration_, velocity_duration_);
  pn.param("body_pitch_jog_max_delta", jog_limit_, jog_limit_);
  pn.param("pitch_rezero_target_pitch", rezero_target_, rezero_target_);
  pn.param("pitch_rezero_command_sign", rezero_sign_, rezero_sign_);
  pn.param("pitch_rezero_rate_limit", rezero_rate_, rezero_rate_);
  pn.param("pitch_rezero_max_delta", rezero_range_, rezero_range_);
  pn.param("pitch_rezero_pitch_tolerance", rezero_tolerance_, rezero_tolerance_);
  pn.param("pitch_rezero_timeout", rezero_timeout_, rezero_timeout_);
  pn.param("pitch_rezero_kp", rezero_kp_, rezero_kp_);
  pn.param("pitch_rezero_rate_tolerance", rezero_rate_tolerance_, rezero_rate_tolerance_);
  pn.param("pitch_rezero_stable_duration", rezero_stable_duration_, rezero_stable_duration_);
  pn.getParam("wire_angles", wire_angles_);
  XmlRpc::XmlRpcValue mapping;
  if(pn.getParam("wire_coordinates", mapping) && mapping.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    for(auto it = mapping.begin(); it != mapping.end(); ++it)
      wire_coordinates_[std::stoi(it->first)] = static_cast<std::string>(it->second);
  const double positive[] = {timeout_, sync_tolerance_, max_step_, max_gap_, max_tilt_, cog_speed_limit_,
      cog_acceleration_limit_, rate_limit_, acceleration_limit_, velocity_duration_, rezero_rate_, rezero_timeout_,
      rezero_kp_, rezero_rate_tolerance_, rezero_stable_duration_};
  for(double value : positive)
    if(!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid navigation/perching limits or timing");
  if(max_step_ > .1 || max_gap_ < max_step_ || !std::isfinite(future_tolerance_) || future_tolerance_ < 0)
    throw std::invalid_argument("invalid navigation/perching clock bounds");
}

void GimbalrotorPerchingNavigator::initializeMechanism()
{
  if(model_ && robot_model_->initialized()) model_valid_ = resolveMechanismModel();
  status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(prefix_ + "/diagnostics_status", 1);
  state_pub_ = nh_.advertise<std_msgs::String>(prefix_ + "/state", 1, true);
  if(!articulated()) return;
  ros::NodeHandle pn(nh_, "navigation/perching");
  std::string body_topic, source, effort_topic;
  pn.param<std::string>("body_state_topic", body_topic, "uav/baselink/odom");
  pn.param<std::string>("dynamics/joint_effort/source", source, "unavailable");
  pn.param<std::string>("dynamics/joint_effort/topic", effort_topic, joint_topic_);
  pn.param<std::string>("dynamics/joint_effort/producer", effort_producer_, nh_.resolveName("servo_bridge"));
  effort_source_valid_ = source == "servo_bridge_joint_states" && effort_topic == joint_topic_ &&
                        effort_producer_ == nh_.resolveName("servo_bridge");
  if(source == "gazebo_ode_joint_states")
  {
    std::string contract, units, sign, physics, description;
    pn.getParam("dynamics/joint_effort_contract", contract);
    pn.getParam("dynamics/joint_effort/units", units); pn.getParam("dynamics/joint_effort/sign", sign);
    nh_.getParam("robot_description", description); nh_.param<std::string>("simulation/physics_engine", physics, "ode");
    effort_source_valid_ = ros::Time::isSimTime() && effort_topic == joint_topic_ && physics == "ode" &&
        contract == "joint_side_actuation_nm" && units == "Nm" && sign == "urdf" &&
        description.find("gazebo_ros_control/AerialRobotHWSim") != std::string::npos;
  }
  reference_pub_ = nh_.advertise<nav_msgs::Odometry>(prefix_ + "/body_reference", 1);
  measured_pub_ = nh_.advertise<nav_msgs::Odometry>(prefix_ + "/body_measured", 1);
  cog_pub_ = nh_.advertise<nav_msgs::Odometry>(prefix_ + "/cog_reference", 1);
  body_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(prefix_ + "/target_body_pose", 1);
  session_pub_ = nh_.advertise<std_msgs::UInt64>(prefix_ + "/session", 1, true);
  epoch_pub_ = nh_.advertise<std_msgs::UInt64>(prefix_ + "/motion_epoch", 1, true);
  ack_pub_ = nh_.advertise<std_msgs::String>(prefix_ + "/command_ack", 10);
  nominal_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(prefix_ + "/nominal_angles", 1);
  angles_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(prefix_ + "/target_angles", 1);
  lower_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(prefix_ + "/position_lower", 1, true);
  upper_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(prefix_ + "/position_upper", 1, true);
  subscriptions_.push_back(nh_.subscribe(joint_topic_, 10, &GimbalrotorPerchingNavigator::jointCallback, this));
  subscriptions_.push_back(nh_.subscribe(body_topic, 10, &GimbalrotorPerchingNavigator::bodyCallback, this));
  subscriptions_.push_back(nh_.subscribe(prefix_ + "/motion", 10, &GimbalrotorPerchingNavigator::motionCallback, this));
  // Legacy names are data at the ROS boundary; algorithms only see coordinates.
  XmlRpc::XmlRpcValue topics;
  if(pn.getParam("coordinate_topics", topics) && topics.getType() == XmlRpc::XmlRpcValue::TypeStruct)
    for(auto it = topics.begin(); it != topics.end(); ++it)
    {
      const auto coordinate = coordinate_map_.find(static_cast<std::string>(it->second));
      if(coordinate == coordinate_map_.end()) throw std::invalid_argument("unknown coordinate in coordinate_topics");
      const int index = coordinate->second;
      subscriptions_.push_back(nh_.subscribe<std_msgs::Float64>(it->first, 1, [this, index](const std_msgs::Float64ConstPtr& m)
      { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_); if(motion_.source.empty()) coordinateCommand(index, m->data); }));
    }
  for(const std::string action : {"confirm_detached_zero", "pitch_rezero", "freeze", "unfreeze", "prepare_unperch", "confirm_released", "recover"})
    subscriptions_.push_back(nh_.subscribe<std_msgs::Empty>(prefix_ + "/" + action, 1,
      [this, action](const std_msgs::EmptyConstPtr&) { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_); workflowCommand(action); }));
  subscriptions_.push_back(nh_.subscribe<std_msgs::Float64>(prefix_ + "/servo_command", 1,
    [this](const std_msgs::Float64ConstPtr& m)
    { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_); if(positioning_) servo_.servo(m->data, servoSample()); }));
  subscriptions_.push_back(nh_.subscribe<std_msgs::Float64>(prefix_ + "/body_pitch_delta", 1,
    [this](const std_msgs::Float64ConstPtr& m)
    { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
      if(motion_.source.empty() && nominalHeld() && std::abs(m->data) <= jog_limit_)
        bodyPitchCommand(m->data, rate_limit_, body_frame_id_ + 1); }));
  subscriptions_.push_back(nh_.subscribe<std_msgs::Float64>(prefix_ + "/body_pitch_velocity", 1,
    [this](const std_msgs::Float64ConstPtr& m)
    { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
      if(!motion_.source.empty() || !std::isfinite(m->data)) return;
      if(m->data == 0) stopMotion();
      else if(perching_locked_) { velocity_active_ = true; body_target_valid_ = false;
        velocity_command_ = std::clamp(m->data, -rate_limit_, rate_limit_); velocity_expires_ = ros::Time::now().toSec() + velocity_duration_; } }));
  for(const std::string action : {"cancel", "stop", "pause", "resume", "prepare", "start"})
    subscriptions_.push_back(nh_.subscribe<std_msgs::UInt64>(prefix_ + "/" + action, 1,
      [this, action](const std_msgs::UInt64ConstPtr& m)
      { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_); planCommand(action, m->data); }));
  subscriptions_.push_back(nh_.subscribe<geometry_msgs::Vector3Stamped>(prefix_ + "/target", 1,
    [this](const geometry_msgs::Vector3StampedConstPtr& m)
    {
      std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
      if(!perching_locked_ || !motion_.source.empty() || m->header.frame_id != "perching_lock_" + std::to_string(session_)) return;
      const double values[] = {m->vector.x, m->vector.y, m->vector.z};
      for(std::size_t i = 0; i < wire_angles_.size() && i < 3; ++i) coordinateCommand(coordinate_map_.at(wire_angles_[i]), values[i]);
    }));
  for(const std::string key : {"model_valid", "contact_valid", "rezero_active", "rezero_ready", "rezero_failed",
      "pitch_rezero_active", "pitch_rezero_ready", "pitch_rezero_failed", "servo_frozen", "cutting_enabled", "feedback_valid", "secondary_settled"})
    bool_pubs_[key] = nh_.advertise<std_msgs::Bool>(prefix_ + "/" + key, 1, true);
  for(const std::string key : {"servo_measured", "servo_target_nominal", "servo_hold_error", "body_pitch", "body_pitch_rate",
      "passive_alpha_cmd", "passive_alpha_measured", "cut_alpha_min", "cut_alpha_max", "pivot_error", "lock_generation",
      "pitch_joint_measured", "pitch_joint_nominal", "pitch_joint_final", "secondary_joint_measured",
      "secondary_joint_nominal", "pitch_joint_hold_error", "secondary_joint_hold_error", "passive_pitch_delta"})
    number_pubs_[key] = nh_.advertise<std_msgs::Float64>(prefix_ + "/" + key, 1);
}

bool GimbalrotorPerchingNavigator::resolveMechanismModel()
{
  if(!model_) return false;
  ros::NodeHandle pn(nh_, "navigation/perching");
  std::string reason;
  if(!model_->configure(active_joints_, reason))
  { ROS_ERROR_STREAM("Perching model: " << reason); return false; }
  if(!articulated()) { moving_ = {true}; return true; }
  for(const auto& key : {"position_lower", "position_upper", "velocity_caps", "acceleration_caps"})
  {
    std::vector<double> limits;
    if(!pn.hasParam(key)) continue;
    if(!pn.getParam(key, limits) || limits.size() != model_->coordinates().size()) return false;
    for(std::size_t i = 0; i < limits.size(); ++i)
    {
      if(!std::isfinite(limits[i])) return false;
      if(std::string(key) == "position_lower") model_->lower[i] = std::max(model_->lower[i], limits[i]);
      if(std::string(key) == "position_upper") model_->upper[i] = std::min(model_->upper[i], limits[i]);
      if(std::string(key) == "velocity_caps") model_->velocity[i] = std::min(model_->velocity[i], limits[i]);
      if(std::string(key) == "acceleration_caps") model_->acceleration[i] = limits[i];
    }
  }
  if(!model_->lower.allFinite() || !model_->upper.allFinite() || (model_->lower.array() >= model_->upper.array()).any() ||
      (model_->velocity.array() <= 0).any() || (model_->acceleration.array() <= 0).any()) return false;
  moving_.assign(active_joints_.size() + 1, false);
  std::vector<std::string> moving;
  pn.getParam("pitch_coordinates", moving);
  for(const auto& name : moving)
  {
    if(!coordinate_map_.count(name)) return false;
    moving_[coordinate_map_.at(name)] = true;
  }
  if(moving.empty() || !coordinate_map_.count(manual_coordinate_)) return false;
  std::string prefix; nhp_.param<std::string>("tf_prefix", prefix, "");
  pn.param<std::string>("body_frame", body_frame_, tf::resolve(prefix, model_->getBaselinkName()));
  if(positioning_)
  {
    if(!coordinate_map_.count(positioning_joint_) || coordinate_map_.at(positioning_joint_) == 0) return false;
    const int index = coordinate_map_.at(positioning_joint_);
    auto& c = servo_.config;
    c.q_min = model_->lower[index]; c.q_max = model_->upper[index]; c.q_rate = model_->velocity[index];
    c.body_pitch_min = min_body_pitch_; c.body_pitch_max = max_body_pitch_;
    c.feedback_timeout = timeout_; c.future_tolerance = future_tolerance_;
    c.rezero_pitch = rezero_target_; c.rezero_sign = rezero_sign_; c.rezero_rate = rezero_rate_;
    c.rezero_range = rezero_range_; c.rezero_timeout = rezero_timeout_; c.rezero_pitch_tolerance = rezero_tolerance_;
    c.rezero_kp = rezero_kp_; c.rezero_rate_tolerance = rezero_rate_tolerance_; c.settle_duration = rezero_stable_duration_;
    pn.param("pitch_position_tolerance", c.q_hold_error, c.q_hold_error);
    pn.param("pitch_velocity_tolerance", c.joint_settle_rate, c.joint_settle_rate);
    pn.param("pitch_zero_tolerance", c.q_zero_tolerance, c.q_zero_tolerance);
    pn.param("passive_lower_limit", c.passive_min, c.passive_min); pn.param("passive_upper_limit", c.passive_max, c.passive_max);
    pn.param("passive_rate_limit", c.passive_rate, c.passive_rate);
    servo_.model = [this](double q, KDL::Frame& contact, KDL::Vector& cog) { return servoModel(q, contact, cog); };
    if(!c.valid()) return false;
  }
  return true;
}

bool GimbalrotorPerchingNavigator::fresh(double stamp, double receipt) const
{
  const double age = ros::Time::now().toSec() - stamp;
  return stamp > 0 && std::isfinite(age) && age >= -future_tolerance_ && age <= timeout_ &&
      receipt > 0 && (ros::Time::isSimTime() || steadyTime() - receipt <= timeout_);
}

void GimbalrotorPerchingNavigator::jointCallback(const ros::MessageEvent<sensor_msgs::JointState const>& event)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  const auto& msg = *event.getMessage();
  Sample sample;
  sample.stamp = msg.header.stamp.toSec(); sample.receipt = steadyTime();
  if(!fresh(sample.stamp, sample.receipt)) return;
  if(!joint_history_.empty() && sample.stamp <= joint_history_.back().stamp) return;
  const int n = active_joints_.size() + 1;
  sample.q = sample.rate = sample.effort = Vector::Zero(n);
  const auto header = event.getConnectionHeaderPtr();
  const auto producer = header ? header->find("callerid") : ros::M_string::const_iterator();
  sample.effort_valid = effort_source_valid_ && header && producer != header->end() && producer->second == effort_producer_;
  for(int i = 1; i < n; ++i)
  {
    const auto it = std::find(msg.name.begin(), msg.name.end(), active_joints_[i - 1]);
    if(it == msg.name.end() || std::count(msg.name.begin(), msg.name.end(), *it) != 1) return;
    const std::size_t index = std::distance(msg.name.begin(), it);
    if(index >= msg.position.size() || !std::isfinite(msg.position[index])) return;
    sample.q[i] = msg.position[index];
    if(index < msg.velocity.size() && std::isfinite(msg.velocity[index])) sample.rate[i] = msg.velocity[index];
    else if(!joint_history_.empty() && sample.stamp - joint_history_.back().stamp <= max_gap_)
      sample.rate[i] = (sample.q[i] - joint_history_.back().q[i]) / (sample.stamp - joint_history_.back().stamp);
    else sample.rate[i] = std::numeric_limits<double>::quiet_NaN();
    if(index < msg.effort.size() && std::isfinite(msg.effort[index])) sample.effort[i] = msg.effort[index];
    else sample.effort_valid = false;
  }
  joint_history_.push_back(sample);
  while(joint_history_.size() > 64) joint_history_.pop_front();
}

void GimbalrotorPerchingNavigator::bodyCallback(const nav_msgs::OdometryConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  Sample sample;
  sample.stamp = msg->header.stamp.toSec(); sample.receipt = steadyTime();
  auto frame = [](std::string name) { if(!name.empty() && name.front() == '/') name.erase(0, 1); return name; };
  if(!fresh(sample.stamp, sample.receipt) || frame(msg->header.frame_id) != frame(world_) ||
     (!body_frame_.empty() && frame(msg->child_frame_id) != frame(body_frame_))) return;
  if(!body_history_.empty() && sample.stamp <= body_history_.back().stamp) return;
  const auto& p = msg->pose.pose.position; const auto& q = msg->pose.pose.orientation;
  Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
  if(!orientation.coeffs().allFinite() || std::abs(orientation.norm() - 1) > .002) return;
  sample.body.linear() = orientation.normalized().toRotationMatrix(); sample.body.translation() = Eigen::Vector3d(p.x, p.y, p.z);
  const auto& w = msg->twist.twist.angular; sample.omega = Eigen::Vector3d(w.x, w.y, w.z);
  if(!perching_geometry::poseValid(sample.body) || !sample.omega.allFinite()) return;
  body_history_.push_back(sample);
  while(body_history_.size() > 64) body_history_.pop_front();
}

bool GimbalrotorPerchingNavigator::observation(Sample& sample) const
{
  if(!articulated())
  {
    if(!model_ || !model_->locked()) return false;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
    tf::matrixTFToEigen(getCurrentBaselinkRot(), rotation);
    tf::vectorTFToEigen(getCurrentBaselinkPos(), position);
    sample.body.linear() = rotation;
    sample.body.translation() = position;
    tf::vectorTFToEigen(estimator_->getAngularVel(Frame::BASELINK, estimate_mode_), sample.omega);
    sample.stamp = estimator_->getImuLatestTimeStamp();
    sample.receipt = steadyTime();
    sample.q = sample.rate = sample.effort = Vector::Zero(1);
    double pivot, hinge;
    if(!model_->estimate(sample.body, sample.q, pivot, hinge)) return false;
    sample.rate[0] = model_->pivot().linear().col(1).dot(sample.body.linear() * sample.omega);
    sample.effort_valid = true;
    return perching_geometry::poseValid(sample.body) && sample.rate.allFinite() && fresh(sample.stamp, sample.receipt);
  }
  if(body_history_.empty() || joint_history_.empty()) return false;
  const auto& body = body_history_.back(); const auto& joint = joint_history_.back();
  if(!fresh(body.stamp, body.receipt) || !fresh(joint.stamp, joint.receipt) ||
      std::abs(body.stamp - joint.stamp) > sync_tolerance_) return false;
  const double stamp = std::min(body.stamp, joint.stamp);
  auto bracket = [stamp, this](const std::deque<Sample>& history, Sample& result, bool body)
  {
    for(std::size_t i = 0; i < history.size(); ++i)
    {
      if(history[i].stamp == stamp) { result = history[i]; return true; }
      if(i == 0 || history[i - 1].stamp > stamp || history[i].stamp < stamp) continue;
      const auto& a = history[i - 1]; const auto& b = history[i];
      if(b.stamp - a.stamp > max_gap_) return false;
      const double u = (stamp - a.stamp) / (b.stamp - a.stamp);
      result = a; result.stamp = stamp; result.receipt = std::min(a.receipt, b.receipt);
      if(body)
      {
        result.body.translation() = (1 - u) * a.body.translation() + u * b.body.translation();
        result.body.linear() = Eigen::Quaterniond(a.body.linear()).slerp(u, Eigen::Quaterniond(b.body.linear())).toRotationMatrix();
        result.omega = result.body.linear().transpose() * ((1 - u) * a.body.linear() * a.omega + u * b.body.linear() * b.omega);
      }
      else
      {
        result.q = (1 - u) * a.q + u * b.q; result.rate = (1 - u) * a.rate + u * b.rate;
        result.effort = (1 - u) * a.effort + u * b.effort;
        result.effort_valid = a.effort_valid && b.effort_valid;
      }
      return true;
    }
    return false;
  };
  Sample b, j;
  if(!bracket(body_history_, b, true) || !bracket(joint_history_, j, false)) return false;
  sample = j; sample.body = b.body; sample.omega = b.omega; sample.receipt = std::min(b.receipt, j.receipt);
  if(model_->locked())
  {
    double pivot, hinge;
    if(!model_->estimate(sample.body, sample.q, pivot, hinge)) return false;
    Eigen::MatrixXd angular;
    if(!model_->jacobian(sample.q, angular)) return false;
    sample.rate[0] = angular.col(0).dot(sample.omega - angular.rightCols(sample.q.size() - 1) * sample.rate.tail(sample.q.size() - 1));
  }
  return true;
}

bool GimbalrotorPerchingNavigator::motionFeedbackValid() const
{
  if(!perching_locked_ || navi_state_ != HOVER_STATE) return false;
  if(!articulated()) { Sample sample; return observation(sample); }
  Sample sample;
  if(!model_valid_ || !observation(sample) || !sample.q.allFinite() || !sample.rate.allFinite()) return false;
  const double pitch = std::asin(std::clamp(-sample.body.linear()(2, 0), -1.0, 1.0));
  return sample.body.linear()(2, 2) >= std::cos(max_tilt_) && pitch >= min_body_pitch_ && pitch <= max_body_pitch_ &&
      model_->allowed(sample.q, max_tilt_, min_body_pitch_, max_body_pitch_);
}

bool GimbalrotorPerchingNavigator::tryLockPerching(const std::string& reason)
{
  if(!articulated()) return tryLockRigid(reason);
  if(navi_state_ != HOVER_STATE || !model_valid_ || !observation(measured_)) return false;
  if(!resolveMechanismModel()) return false; // Capture held internal geometry/inertias for this lock.
  measured_.q[0] = 0;
  if(!model_->lock(measured_.body, measured_.q) || !model_->allowed(measured_.q, max_tilt_, min_body_pitch_, max_body_pitch_)) return false;
  nominal_ = destination_ = proposed_ = applied_ = locked_q_ = measured_.q;
  nominal_rate_ = proposed_rate_ = applied_rate_ = applied_acceleration_ = Vector::Zero(nominal_.size());
  if(!model_->evaluate(applied_, applied_state_)) return false;
  if(positioning_ && !servo_.enable(servoSample())) return false;
  commitFixedContactLock(tfVector(applied_state_.cog.translation()), tfQuaternion(applied_state_.cog.linear()),
                         tfVector(model_->pivot().translation()));
  session_ = lock_stamp_.toNSec();
  applied_stamp_ = 0; last_update_ = ros::Time::now().toSec();
  phase_ = "CONTACT_LOCKED"; status_ = "locked; collecting unloaded baseline";
  publishDiagnostics();
  return true;
}

void GimbalrotorPerchingNavigator::resetMotion()
{
  motion_.revoke(); session_ = 0;
  body_target_valid_ = velocity_active_ = stopping_ = proposal_valid_ = false;
  rezero_active_ = rezero_ready_ = rezero_failed_ = false;
  body_destination_ = body_progress_ = body_rate_ = 0;
  applied_pitch_offset_ = applied_offset_rate_ = 0;
  applied_stamp_ = last_update_ = 0; rigid_applied_valid_ = false;
  servo_.reset(); phase_ = "FREE_FLIGHT"; prepared_motion_ = paused_ = false;
}

bool GimbalrotorPerchingNavigator::supportsPerchingMode(perching_geometry::Mode mode) const
{
  return mode == perching_geometry::Mode::NORMAL || (!articulated() && mode == perching_geometry::Mode::SLANTED);
}
bool GimbalrotorPerchingNavigator::ownsNavigationTargets() const
{
  return articulated() && perching_locked_ && perching_mode_ != perching_geometry::Mode::DISABLED;
}
std::unique_lock<std::recursive_mutex> GimbalrotorPerchingNavigator::lockNavigationTargets()
{
  return std::unique_lock<std::recursive_mutex>(perching_state_mutex_);
}
void GimbalrotorPerchingNavigator::navigationTargetConflict(const char* source)
{
  status_ = std::string("perching owns reference: ignored ") + source;
}

bool GimbalrotorPerchingNavigator::nominalHeld() const
{
  if(!articulated()) return !velocity_active_;
  return perching_locked_ && !rezero_active_ && nominal_rate_.norm() < 1e-6 && std::abs(body_rate_) < 1e-6 &&
      !velocity_active_ && !stopping_ && (destination_ - nominal_).norm() < 1e-5 &&
      (!body_target_valid_ || std::abs(body_destination_ - body_progress_) < 1e-4);
}

bool GimbalrotorPerchingNavigator::trackingHeld() const
{
  if(!articulated())
  {
    Sample sample;
    return observation(sample) && std::abs(sample.q[0] - rigidAngle()) <= tracking_tolerance_ &&
        std::abs(sample.rate[0]) <= rezero_rate_tolerance_;
  }
  Sample sample;
  Model::State measured;
  return observation(sample) && applied_.size() == sample.q.size() && model_->evaluate(sample.q, measured) &&
      (sample.q - applied_).cwiseAbs().maxCoeff() <= tracking_tolerance_ &&
      perching_geometry::logSO3(measured.body.linear().transpose() * applied_state_.body.linear()).norm() <= body_tracking_tolerance_;
}

void GimbalrotorPerchingNavigator::stopMotion()
{
  motion_.revoke();
  // STOP changes the nominal destination, never the physical correction.
  destination_ = nominal_; body_destination_ = body_progress_;
  velocity_active_ = false; stopping_ = true;
  status_ = "braking nominal motion";
}

void GimbalrotorPerchingNavigator::coordinateCommand(int index, double value)
{
  if(!perching_locked_ || !std::isfinite(value) || index < 0 || index >= nominal_.size() ||
      value < model_->lower[index] || value > model_->upper[index] || rezero_active_) return;
  Vector candidate = destination_;
  candidate[index] = value;
  if(!model_->allowed(candidate, max_tilt_, min_body_pitch_, max_body_pitch_)) return;
  body_target_valid_ = velocity_active_ = false;
  destination_[index] = value; stopping_ = false; rezero_ready_ = false;
  phase_ = "EXECUTING";
}

void GimbalrotorPerchingNavigator::bodyPitchCommand(double angle, double rate, std::uint64_t frame)
{
  if(!perching_locked_ || !std::isfinite(angle) || !std::isfinite(rate) || rate <= 0 || rezero_active_) return;
  const bool new_frame = !body_target_valid_ || frame != body_frame_id_;
  const Vector start = new_frame ? nominal_ : body_start_;
  Vector endpoint;
  if(!model_->pitch(start, angle, moving_, endpoint) || !model_->allowed(endpoint, max_tilt_, min_body_pitch_, max_body_pitch_))
  { status_ = "requested body pitch is infeasible"; return; }
  if(new_frame)
  {
    // Capture the nominal body frame. Loaded applied motion must not change the
    // origin or positive direction of later cumulative destinations/reversals.
    body_start_ = nominal_; body_progress_ = 0; body_frame_id_ = frame;
  }
  rezero_ready_ = false;
  body_destination_ = angle; body_target_valid_ = true; velocity_active_ = false; stopping_ = false;
  destination_ = nominal_; body_rate_limit_ = std::min(rate, rate_limit_); phase_ = "EXECUTING";
}

void GimbalrotorPerchingNavigator::motionCallback(const gimbalrotor::RollYawMotionConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  using Command = gimbalrotor::RollYawMotion;
  const auto& m = *msg;
  if(last_motion_request_ && m.session == last_motion_request_->session && m.epoch == last_motion_request_->epoch &&
      m.source == last_motion_request_->source && m.sequence == last_motion_request_->sequence)
  { ack_pub_.publish(last_motion_ack_); return; }
  std::string why = motion_.rejection(m.epoch, m.source, m.sequence, m.mode == Command::CLAIM);
  if(!perching_locked_ || m.session != session_) why = "invalid contact session";
  if(!std::isfinite(m.value)) why = "nonfinite command";
  if(why.empty() && m.mode == Command::CLAIM && !nominalHeld()) why = "nominal motion is not held";
  if(why.empty())
  {
    if(m.mode == Command::STOP) stopMotion();
    else if(m.mode == Command::CLAIM) { motion_.revoke(); motion_.accept(m.source, m.sequence); body_target_valid_ = false; prepared_motion_ = false; }
    else if(m.mode == Command::RELEASE) motion_.revoke();
    else
    {
      motion_.accept(m.source, m.sequence);
      if(m.mode == Command::BODY_PITCH)
      {
        bodyPitchCommand(m.value, m.rate_limit, m.frame);
        if(!body_target_valid_ || body_frame_id_ != m.frame || body_destination_ != m.value) why = "infeasible body pitch target";
      }
      else if(m.mode == Command::VELOCITY || m.mode == Command::PULSE)
      {
        if(m.value == 0) stopMotion();
        else if(m.mode == Command::PULSE && (!std::isfinite(m.duration) || m.duration <= 0 || m.duration > 8)) why = "invalid pulse duration";
        else
        {
          velocity_active_ = true; body_target_valid_ = false; stopping_ = false;
          velocity_command_ = std::clamp(m.value, -rate_limit_, rate_limit_);
          velocity_expires_ = ros::Time::now().toSec() + (m.mode == Command::PULSE ? m.duration : velocity_duration_);
        }
      }
      else if(m.mode == Command::JOG)
      {
        if(!nominalHeld() || std::abs(m.value) > jog_limit_) why = "jog requires held nominal reference within configured bound";
        else bodyPitchCommand(m.value, rate_limit_, m.sequence);
      }
      else
      {
        const auto name = wire_coordinates_.find(m.mode);
        if(name == wire_coordinates_.end() || !coordinate_map_.count(name->second)) why = "unknown coordinate command";
        else
        {
          const int index = coordinate_map_.at(name->second);
          if(m.value < model_->lower[index] || m.value > model_->upper[index]) why = "coordinate outside limits";
          else
          {
            Vector candidate = destination_;
            candidate[index] = m.value;
            if(!model_->allowed(candidate, max_tilt_, min_body_pitch_, max_body_pitch_)) why = "coordinate target is infeasible";
            else coordinateCommand(index, m.value);
          }
        }
      }
    }
  }
  std::ostringstream out;
  out << std::setprecision(17) << "action=motion session=" << m.session << " epoch=" << m.epoch << " source=" << m.source
      << " sequence=" << m.sequence << " mode=" << int(m.mode) << " accepted=" << (why.empty() ? "true" : "false")
      << " motion_epoch=" << motion_.epoch << " accepted_value=" << m.value << " reason=" << why;
  std_msgs::String ack; ack.data = out.str(); ack_pub_.publish(ack);
  last_motion_request_ = msg; last_motion_ack_ = ack;
  publishDiagnostics();
}

perching_servo::Sample GimbalrotorPerchingNavigator::servoSample() const
{
  perching_servo::Sample s;
  s.now = ros::Time::now().toSec(); s.hover = navi_state_ == HOVER_STATE;
  Sample measured;
  s.model_valid = model_valid_ && observation(measured);
  if(!s.model_valid || !coordinate_map_.count(positioning_joint_)) return s;
  const int i = coordinate_map_.at(positioning_joint_);
  s.joint_stamp = measured.stamp; s.q = measured.q[i]; s.qdot = measured.rate[i];
  s.body = perching_geometry::toKdl(measured.body); s.angular_speed = measured.omega.norm();
  s.pitch_rate = perching_servo::pitchRate(s.body, KDL::Vector(measured.omega.x(), measured.omega.y(), measured.omega.z()));
  return s;
}
bool GimbalrotorPerchingNavigator::servoModel(double value, KDL::Frame& contact, KDL::Vector& cog) const
{
  if(nominal_.size() == 0 || !coordinate_map_.count(positioning_joint_)) return false;
  Vector q = nominal_; q[coordinate_map_.at(positioning_joint_)] = value;
  Pose bc; Eigen::Vector3d center;
  if(!model_->bodyContact(q, bc, center)) return false;
  contact = perching_geometry::toKdl(bc); cog = KDL::Vector(center.x(), center.y(), center.z());
  return true;
}

void GimbalrotorPerchingNavigator::planCommand(const std::string& action, std::uint64_t session)
{
  std::string error;
  if(!perching_locked_ || session != session_) error = "invalid contact session";
  else if(action == "stop" || action == "cancel")
  { stopMotion(); prepared_motion_ = paused_ = false; }
  else if(action == "pause")
  {
    if(!prepared_motion_) error = "no prepared motion";
    else
    {
      paused_destination_ = destination_; paused_body_destination_ = body_destination_; paused_phase_ = phase_;
      stopMotion(); paused_ = true; phase_ = "PAUSED";
    }
  }
  else if(action == "resume")
  {
    if(!paused_ || !nominalHeld() || !motionFeedbackValid()) error = "pause must finish braking before resume";
    else
    { destination_ = paused_destination_; body_destination_ = paused_body_destination_; paused_ = false; phase_ = paused_phase_; }
  }
  else if(action == "prepare")
  {
    if(!nominalHeld() || !trackingHeld() || !motionFeedbackValid() || !motion_.source.empty()) error = "prepare requires unowned held motion feedback";
    else
    {
      ros::NodeHandle request(nh_, "navigation/perching/request");
      double yaw = 0, roll = 0, start = 0, increment = .34906585;
      request.param("yaw", yaw, yaw); request.param("roll", roll, roll); request.param("pitch_start", start, start);
      request.param("pitch_increment", increment, increment);
      Model::State state; Vector goal, end;
      model_->evaluate(nominal_, state);
      const Eigen::Matrix3d desired = state.body.linear() * perching_geometry::expSO3(Eigen::Vector3d::UnitZ() * yaw) *
          perching_geometry::expSO3(Eigen::Vector3d::UnitX() * roll) * perching_geometry::expSO3(Eigen::Vector3d::UnitY() * start);
      if(!model_->orient(nominal_, desired, moving_, goal) || !model_->pitch(goal, increment, moving_, end) ||
          !model_->allowed(goal, max_tilt_, min_body_pitch_, max_body_pitch_) || !model_->allowed(end, max_tilt_, min_body_pitch_, max_body_pitch_))
        error = "requested orientation/pitch is infeasible";
      else
      {
        rezero_ready_ = false; destination_ = goal; prepared_increment_ = increment; prepared_motion_ = true; body_target_valid_ = false;
        velocity_active_ = stopping_ = paused_ = false; phase_ = "PRE_ALIGN";
      }
    }
  }
  else if(action == "start")
  {
    if(!prepared_motion_ || phase_ != "READY" || !nominalHeld() || !trackingHeld() || !motionFeedbackValid()) error = "start requires measured READY";
    else bodyPitchCommand(prepared_increment_, rate_limit_, body_frame_id_ + 1);
  }
  std_msgs::String ack;
  ack.data = "action=" + action + " session=" + std::to_string(session) + " accepted=" + (error.empty() ? "true" : "false") + " reason=" + error;
  ack_pub_.publish(ack); publishDiagnostics();
}

void GimbalrotorPerchingNavigator::workflowCommand(const std::string& action)
{
  if(positioning_)
  {
    auto s = servoSample();
    if(perching_locked_ && applied_pitch_offset_ != 0)
    {
      Sample measured; Vector nominal; Model::State state;
      if(observation(measured) && model_->pitch(measured.q, -applied_pitch_offset_, moving_, nominal) && model_->evaluate(nominal, state))
        s.body = perching_geometry::toKdl(state.body);
    }
    if(action == "confirm_detached_zero") servo_.armZero(s);
    else if(action == "pitch_rezero") servo_.rezero(s);
    else if(action == "relock") servo_.relock(s);
    else if(action == "freeze") servo_.freeze(s);
    else if(action == "unfreeze") servo_.unfreeze(s);
    else if(action == "prepare_unperch") servo_.prepare(s);
    else if(action == "confirm_released") servo_.released(s);
    else if(action == "recover") servo_.recover(s);
    status_ = servo_.status;
    return;
  }
  if(action != "pitch_rezero" || !motionFeedbackValid() || !nominalHeld()) return;
  rezero_active_ = true; rezero_ready_ = rezero_failed_ = false;
  rezero_started_ = ros::Time::now().toSec(); rezero_stable_since_ = 0; rezero_origin_ = nominal_[0];
  destination_ = nominal_; body_target_valid_ = velocity_active_ = false;
}

void GimbalrotorPerchingNavigator::advanceNominal(double dt)
{
  proposed_ = nominal_; proposed_rate_ = nominal_rate_; proposed_progress_ = body_progress_; proposed_body_rate_ = body_rate_;
  previous_servo_ = servo_;
  const double now = ros::Time::now().toSec();
  if(!trackingHeld())
  {
    proposed_rate_.setZero(); proposed_body_rate_ = 0;
    proposal_valid_ = true; status_ = "nominal held for tracking; compliant recovery remains eligible";
    return;
  }
  if(positioning_)
  {
    servo_.tick(servoSample(), dt);
    proposed_[coordinate_map_.at(positioning_joint_)] = servo_.q_target;
    double pivot, hinge;
    if(!model_->estimate(perching_geometry::fromKdl(servo_.target), proposed_, pivot, hinge)) return;
    const Vector goal = proposed_;
    followCoordinates(goal, dt); destination_ = goal; status_ = servo_.status;
  }
  else if(rezero_active_)
  {
    const double pitch = std::asin(std::clamp(-measured_.body.linear()(2, 0), -1.0, 1.0));
    const double error = rezero_target_ - pitch;
    const bool held = (measured_.q.tail(active_joints_.size()) - nominal_.tail(active_joints_.size())).cwiseAbs().maxCoeff() <= hold_tolerance_;
    if(!held || now - rezero_started_ > rezero_timeout_)
    { rezero_active_ = false; rezero_failed_ = true; status_ = "rezero feedback/timeout hold"; return; }
    const double rate = rezero_sign_ * std::clamp(rezero_kp_ * error, -rezero_rate_, rezero_rate_);
    proposed_[0] = std::clamp(nominal_[0] + rate * dt, rezero_origin_ - rezero_range_, rezero_origin_ + rezero_range_);
    const Vector goal = proposed_;
    followCoordinates(goal, dt); destination_ = goal;
    if(std::abs(error) <= rezero_tolerance_ && measured_.omega.norm() <= rezero_rate_tolerance_)
    {
      if(rezero_stable_since_ == 0) rezero_stable_since_ = now;
      if(now - rezero_stable_since_ >= rezero_stable_duration_) { rezero_active_ = false; rezero_ready_ = true; }
    }
    else rezero_stable_since_ = 0;
  }
  else if(body_target_valid_ || velocity_active_ || std::abs(body_rate_) > 1e-9)
  {
    if(velocity_active_ && now >= velocity_expires_) { velocity_active_ = false; stopping_ = true; }
    const double error = body_destination_ - body_progress_;
    double goal_rate = velocity_active_ ? velocity_command_ :
        (body_target_valid_ && !stopping_ ? std::clamp(error / .5, -body_rate_limit_, body_rate_limit_) : 0);
    const double change = acceleration_limit_ * dt;
    proposed_body_rate_ = std::clamp(goal_rate, body_rate_ - change, body_rate_ + change);
    if(std::abs(proposed_body_rate_) < 1e-8) proposed_body_rate_ = 0;
    const double step = .5 * (body_rate_ + proposed_body_rate_) * dt;
    if(!model_->pitch(nominal_, step, moving_, proposed_)) { status_ = "body pitch direction unavailable"; return; }
    proposed_progress_ = body_progress_ + step;
    proposed_rate_ = (proposed_ - nominal_) / dt;
    destination_ = nominal_;
  }
  else followCoordinates(destination_, dt);
  proposal_valid_ = model_->allowed(proposed_, max_tilt_, min_body_pitch_, max_body_pitch_);
  if(!proposal_valid_) status_ = "nominal proposal outside mechanism/body bounds";
}

void GimbalrotorPerchingNavigator::followCoordinates(const Vector& goal, double dt)
{
  for(int i = 0; i < nominal_.size(); ++i)
  {
    const double error = goal[i] - nominal_[i];
    const double desired = stopping_ ? 0 : std::clamp(error / .5, -model_->velocity[i], model_->velocity[i]);
    proposed_rate_[i] = std::clamp(desired, nominal_rate_[i] - .5 * model_->acceleration[i] * dt,
                                 nominal_rate_[i] + .5 * model_->acceleration[i] * dt);
    if(std::abs(proposed_rate_[i]) < 1e-8) proposed_rate_[i] = 0;
    proposed_[i] = nominal_[i] + .5 * (nominal_rate_[i] + proposed_rate_[i]) * dt;
  }
}

bool GimbalrotorPerchingNavigator::candidateAllowed(const Vector& q, double dt) const
{
  if(!model_->allowed(q, max_tilt_, min_body_pitch_, max_body_pitch_)) return false;
  if(applied_stamp_ == 0) return true;
  if(dt <= 0) return (q - applied_).norm() < 1e-8;
  const Vector rate = (q - applied_) / dt;
  if((rate.cwiseAbs().array() > model_->velocity.array() + 1e-6).any() ||
      ((rate - applied_rate_).cwiseAbs().array() > model_->acceleration.array() * dt + 1e-5).any()) return false;
  Model::State state;
  if(!model_->evaluate(q, state)) return false;
  const Eigen::Vector3d velocity = (state.cog.translation() - applied_state_.cog.translation()) / dt;
  return velocity.norm() <= cog_speed_limit_ + 1e-6 &&
      (velocity - reference_velocity_).norm() <= cog_acceleration_limit_ * dt + 1e-5;
}

bool GimbalrotorPerchingNavigator::applyPerchingReference(double pitch_offset, double rate, double acceleration)
{
  if(!perching_locked_)
  {
    // Explicit detached-zero hold owns only the named servo, before a contact
    // exists. It still publishes from the controller's single output phase.
    if(!positioning_ || !servo_.zero_hold_armed || !servo_.feedback(servoSample())) return false;
    return true;
  }
  if(!articulated())
  {
    perching_geometry::Pose pose;
    const tf::Vector3 nominal_position = getTargetPos();
    if(!std::isfinite(pitch_offset) || !std::isfinite(rate) || !std::isfinite(acceleration) ||
       std::abs(active_pitch_delta_ + pitch_offset / arc_pitch_sign_) > max_pitch_delta_ ||
       !perchingAdmittanceTarget(lock_stamp_, pitch_offset, nominal_position, pose))
    {
      if(rigid_applied_valid_)
      {
        double r, p, y; tf::Matrix3x3(rigid_applied_pose_.orientation).getRPY(r, p, y);
        setTargetPos(rigid_applied_pose_.position); setTargetRPY(tf::Vector3(r, p, y));
      }
      return false;
    }
    double roll, pitch, yaw;
    tf::Matrix3x3(pose.orientation).getRPY(roll, pitch, yaw);
    setTargetPos(pose.position); setTargetRPY(tf::Vector3(roll, pitch, yaw));
    applied_pitch_offset_ = pitch_offset; applied_offset_rate_ = rate;
    rigid_applied_pose_ = pose; rigid_applied_pose_.orientation = tf::createQuaternionFromRPY(roll, pitch, yaw); rigid_applied_valid_ = true;
    return true;
  }
  const double now = ros::Time::now().toSec();
  if(applied_stamp_ == now) return false;
  const bool fresh = motionFeedbackValid();
  const Vector& nominal = proposal_valid_ && fresh ? proposed_ : nominal_;
  Vector candidate;
  bool accepted = fresh && std::isfinite(pitch_offset) && std::isfinite(rate) && std::isfinite(acceleration) &&
      model_->pitch(nominal, pitch_offset, moving_, candidate) && candidateAllowed(candidate, cycle_dt_);
  Model::State state;
  if(accepted) accepted = model_->evaluate(candidate, state);
  if(accepted)
  {
    const bool derivative = applied_stamp_ > 0 && cycle_dt_ > 0;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(), omega = Eigen::Vector3d::Zero();
    if(derivative)
    {
      velocity = (state.cog.translation() - applied_state_.cog.translation()) / cycle_dt_;
      omega = perching_geometry::logSO3(applied_state_.cog.linear().transpose() * state.cog.linear()) / cycle_dt_;
      const Vector rate = (candidate - applied_) / cycle_dt_;
      applied_acceleration_ = (rate - applied_rate_) / cycle_dt_;
      applied_rate_ = rate;
    }
    else
    {
      applied_rate_.setZero();
      applied_acceleration_.setZero();
    }
    reference_acceleration_ = derivative ? Eigen::Vector3d((velocity - reference_velocity_) / cycle_dt_) : Eigen::Vector3d::Zero();
    reference_angular_acceleration_ = derivative ? Eigen::Vector3d((omega - reference_omega_) / cycle_dt_) : Eigen::Vector3d::Zero();
    reference_velocity_ = velocity; reference_omega_ = omega;
    applied_ = candidate; applied_state_ = state; applied_pitch_offset_ = pitch_offset; applied_offset_rate_ = rate;
    if(proposal_valid_)
    {
      nominal_ = proposed_; nominal_rate_ = proposed_rate_;
      body_progress_ = proposed_progress_; body_rate_ = proposed_body_rate_;
      if(body_target_valid_ || velocity_active_ || stopping_) destination_ = nominal_;
      if(stopping_ && std::abs(body_rate_) < 1e-8 && nominal_rate_.norm() < 1e-8)
      { stopping_ = false; body_destination_ = body_progress_; destination_ = nominal_; }
    }
    applied_stamp_ = now;
  }
  else
  {
    // Nominal and compliance both roll back to the same last accepted reference.
    // A true feedback loss holds; raw observation remains independent.
    if(positioning_ && proposal_valid_) servo_ = previous_servo_;
    reference_velocity_.setZero(); reference_acceleration_.setZero(); reference_omega_.setZero(); reference_angular_acceleration_.setZero();
    applied_rate_.setZero(); applied_acceleration_.setZero(); nominal_rate_.setZero(); body_rate_ = 0;
    status_ = fresh ? "coordinated reference rejected; holding last feasible output" : "waiting for motion feedback";
  }
  proposal_valid_ = false;
  publishApplied();
  return accepted;
}

void GimbalrotorPerchingNavigator::publishApplied()
{
  double roll, pitch, yaw;
  tf::Matrix3x3(tfQuaternion(applied_state_.cog.linear())).getRPY(roll, pitch, yaw);
  setXyControlMode(POS_CONTROL_MODE);
  setTargetPos(tfVector(applied_state_.cog.translation())); setTargetRPY(tf::Vector3(roll, pitch, yaw));
  setTargetVel(tfVector(reference_velocity_)); setTargetAcc(tfVector(reference_acceleration_));
  setTargetOmega(tfVector(reference_omega_));
  Eigen::Matrix3d current;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), current);
  setTargetAngAcc(tfVector(current.transpose() * applied_state_.cog.linear() * reference_angular_acceleration_));
}

bool GimbalrotorPerchingNavigator::jointCommand(sensor_msgs::JointState& command) const
{
  command.header.stamp = ros::Time::now();
  if(!perching_locked_)
  {
    if(!positioning_ || !servo_.zero_hold_armed || !servo_.feedback(servoSample())) return false;
    command.name = {positioning_joint_};
    command.position = {servo_.q_target};
    return true;
  }
  if(!articulated()) return false;
  command.name = active_joints_;
  for(int i = 1; i < applied_.size(); ++i) command.position.push_back(applied_[i]);
  return true;
}

void GimbalrotorPerchingNavigator::retireCorrection()
{
  // Explicit calibration resets the core while retaining a feasible physical
  // reference. Retire the current nominal plan at that pose, including its sign.
  if(!perching_locked_) return;
  if(articulated())
  {
    nominal_ = destination_ = proposed_ = applied_;
    nominal_rate_.setZero(); applied_acceleration_.setZero(); body_rate_ = 0; body_target_valid_ = velocity_active_ = stopping_ = false;
    proposed_rate_.setZero(); proposal_valid_ = prepared_motion_ = paused_ = rezero_active_ = false;
    if(positioning_ && servo_.ownsPose())
    {
      servo_.q_target = servo_.q_goal = applied_[coordinate_map_.at(positioning_joint_)];
      servo_.alpha_goal = servo_.alpha;
      servo_.target = perching_geometry::toKdl(applied_state_.body);
      servo_.reference = perching_servo::arc(servo_.target, servo_.pivot, servo_.axis, -servo_.alpha);
      KDL::Frame contact;
      KDL::Vector cog;
      if(servoModel(servo_.q_target, contact, cog))
      { servo_.contact = servo_.target * contact; servo_.cog = cog; }
      if(servo_.stage == perching_servo::Stage::PITCH_REZERO_ACTIVE)
        servo_.stage = perching_servo::Stage::CONTACT_LOCKED_PRE_REZERO;
    }
  }
  else
  {
    active_pitch_delta_ += applied_pitch_offset_ / arc_pitch_sign_;
    velocity_active_ = false;
  }
  applied_pitch_offset_ = applied_offset_rate_ = 0;
}

double GimbalrotorPerchingNavigator::pitchAccelerationLimit() const
{
  Vector tangent;
  if(!model_->pitchTangent(nominal_, moving_, tangent)) return 0;
  double limit = std::numeric_limits<double>::infinity();
  for(int i = 0; i < tangent.size(); ++i)
    if(std::abs(tangent[i]) > 1e-8)
    {
      const double nominal_acceleration = cycle_dt_ > 0 && proposal_valid_ ?
          std::abs(proposed_rate_[i] - nominal_rate_[i]) / cycle_dt_ : 0;
      // Reserve curvature/FK margin; final joint and CoG bounds are still checked.
      limit = std::min(limit, .5 * std::max(0.0, model_->acceleration[i] - nominal_acceleration) / std::abs(tangent[i]));
    }
  return std::isfinite(limit) ? limit : 0;
}

void GimbalrotorPerchingNavigator::publishDiagnostics()
{
  if(!articulated())
  {
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    diagnostic_msgs::DiagnosticStatus diagnostic;
    diagnostic.name = nh_.getNamespace() + "/" + status_name_;
    diagnostic.message = perching_locked_ ? "CONTACT_LOCKED" : "FREE_FLIGHT";
    diagnostic_msgs::KeyValue residual;
    residual.key = "hinge_residual";
    residual.value = std::to_string(hinge_error_);
    diagnostic.values.push_back(residual);
    residual.key = "pivot_error";
    residual.value = std::to_string(pivot_error_);
    diagnostic.values.push_back(residual);
    array.status.push_back(diagnostic);
    status_pub_.publish(array);
    std_msgs::String state;
    state.data = diagnostic.message + ": " + status_;
    state_pub_.publish(state);
    return;
  }
  const bool feedback = motionFeedbackValid();
  const bool held = nominalHeld() && feedback && trackingHeld();
  if(held && phase_ == "EXECUTING") phase_ = "COMPLETED_HOLD";
  if(held && phase_ == "PRE_ALIGN") phase_ = "READY";
  std_msgs::UInt64 id; id.data = session_; session_pub_.publish(id); id.data = motion_.epoch; epoch_pub_.publish(id);
  diagnostic_msgs::DiagnosticArray array; array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus diagnostic; diagnostic.name = nh_.getNamespace() + "/" + status_name_;
  const std::string stage = !perching_locked_ ? "FREE_FLIGHT" : rezero_failed_ ? "PITCH_REZERO_FAILED" :
      rezero_active_ ? "PITCH_REZERO_ACTIVE" : rezero_ready_ ? "PITCH_REZERO_HOLD" : prepared_motion_ ? phase_ : "CONTACT_LOCKED";
  diagnostic.message = stage + ": " + status_; diagnostic.level = feedback || !perching_locked_ ? 0 : 1;
  auto values = admittance_diagnostics_;
  auto value = [&](const std::string& key, auto data) { std::ostringstream s; s << std::setprecision(17) << data; values[key] = s.str(); };
  value("session_id", session_); value("motion_epoch", motion_.epoch); value("motion_source", motion_.source);
  value("motion_sequence", motion_.sequence); value("stopping", stopping_); value("feedback_valid", feedback);
  value("body_pitch_target_valid", body_target_valid_); value("body_pitch_target_frame", body_frame_id_);
  value("body_pitch_target_rad", body_destination_); value("body_pitch_target_progress_rad", body_progress_);
  value("body_pitch_target_reference_complete", body_target_valid_ && held);
  value("body_pitch_velocity_active", velocity_active_); value("body_pitch_velocity_limit", rate_limit_);
  value("admittance_hold_eligible", held); value("effective_offset_rad", applied_pitch_offset_);
  value("hold_reason", status_); value("pivot_error", pivot_error_); value("hinge_residual", hinge_error_);
  for(const auto& item : values) { diagnostic_msgs::KeyValue v; v.key = item.first; v.value = item.second; diagnostic.values.push_back(v); }
  array.status.push_back(diagnostic); status_pub_.publish(array);
  auto boolean = [&](const std::string& key, bool data) { std_msgs::Bool m; m.data = data; bool_pubs_.at(key).publish(m); };
  auto number = [&](const std::string& key, double data) { std_msgs::Float64 m; m.data = data; number_pubs_.at(key).publish(m); };
  boolean("model_valid", model_valid_); boolean("contact_valid", perching_locked_ && feedback); boolean("feedback_valid", feedback);
  const auto s = servoSample();
  const bool active = positioning_ ? servo_.stage == perching_servo::Stage::PITCH_REZERO_ACTIVE : rezero_active_;
  const bool ready = positioning_ ? servo_.rezeroReady(s) : rezero_ready_;
  const bool failed = positioning_ ? servo_.stage == perching_servo::Stage::FAULT_HOLD && !servo_.rezero_completed : rezero_failed_;
  for(const auto& prefix : {std::string(""), std::string("pitch_")})
  { boolean(prefix + "rezero_active", active); boolean(prefix + "rezero_ready", ready); boolean(prefix + "rezero_failed", failed); }
  boolean("servo_frozen", positioning_ && servo_.stage == perching_servo::Stage::PASSIVE_CUTTING);
  boolean("cutting_enabled", positioning_ && servo_.stage == perching_servo::Stage::PASSIVE_CUTTING && feedback);
  if(perching_locked_)
  {
    number("body_pitch", std::asin(std::clamp(-measured_.body.linear()(2, 0), -1.0, 1.0)));
    number("body_pitch_rate", measured_.omega.y()); number("pivot_error", pivot_error_);
    number("lock_generation", positioning_ ? servo_.generation : session_);
    number("passive_alpha_cmd", positioning_ ? servo_.alpha : nominal_[0]); number("passive_alpha_measured", measured_.q[0]);
    number("passive_pitch_delta", nominal_[0]); number("cut_alpha_min", servo_.alpha_min); number("cut_alpha_max", servo_.alpha_max);
    if(positioning_)
    {
      const int i = coordinate_map_.at(positioning_joint_);
      number("servo_measured", measured_.q[i]); number("servo_target_nominal", nominal_[i]); number("servo_hold_error", measured_.q[i] - nominal_[i]);
    }
    const auto primary = coordinate_map_.find(manual_coordinate_);
    bool secondary_held = feedback;
    if(primary != coordinate_map_.end())
    {
      const int i = primary->second;
      number("pitch_joint_measured", measured_.q[i]); number("pitch_joint_nominal", destination_[i]); number("pitch_joint_final", applied_[i]);
      number("pitch_joint_hold_error", measured_.q[i] - nominal_[i]);
      for(int j = 1; j < nominal_.size(); ++j) if(j != i)
      {
        number("secondary_joint_measured", measured_.q[j]); number("secondary_joint_nominal", destination_[j]);
        number("secondary_joint_hold_error", measured_.q[j] - nominal_[j]);
        secondary_held &= std::abs(measured_.q[j] - nominal_[j]) <= hold_tolerance_ && std::abs(measured_.rate[j]) <= .05;
      }
    }
    const double now = array.header.stamp.toSec();
    if(!secondary_held) secondary_stable_since_ = 0;
    else if(secondary_stable_since_ == 0) secondary_stable_since_ = now;
    boolean("secondary_settled", secondary_held && now - secondary_stable_since_ >= .3);
    nav_msgs::Odometry pose; pose.header = array.header; pose.header.frame_id = world_; pose.child_frame_id = body_frame_;
    pose.pose.pose = poseMessage(applied_state_.body); reference_pub_.publish(pose);
    pose.header.stamp.fromSec(measured_.stamp); pose.pose.pose = poseMessage(measured_.body); measured_pub_.publish(pose);
    pose.header = array.header; pose.header.frame_id = world_; pose.pose.pose = poseMessage(applied_state_.cog); cog_pub_.publish(pose);
    geometry_msgs::PoseStamped target; target.header = array.header; target.header.frame_id = world_;
    target.pose = poseMessage(applied_state_.body); body_pose_pub_.publish(target);
    auto angles = [&](ros::Publisher& publisher, const Vector& q)
    {
      if(wire_angles_.size() != 3) return;
      geometry_msgs::Vector3Stamped m; m.header = array.header; m.header.frame_id = "perching_lock_" + std::to_string(session_);
      m.vector.x = q[coordinate_map_.at(wire_angles_[0])]; m.vector.y = q[coordinate_map_.at(wire_angles_[1])]; m.vector.z = q[coordinate_map_.at(wire_angles_[2])];
      publisher.publish(m);
    };
    angles(nominal_pub_, destination_); angles(angles_pub_, applied_); angles(lower_pub_, model_->lower); angles(upper_pub_, model_->upper);
  }
  std_msgs::String state;
  if(positioning_)
  {
    std::ostringstream out;
    out << std::setprecision(17) << name(servo_.stage) << "|" << servo_.generation << "|" << s.now << "|" << s.q << "|"
        << servo_.q_target << "|" << perching_servo::pitch(s.body) << "|" << servo_.alpha << "|" << servo_.measuredAlpha(s) << "|"
        << servo_.alpha_min << "|" << servo_.alpha_max << "|" << feedback << "|0|" << pivot_error_ << "|" << servo_.zero_hold_armed << "|" << servo_.status;
    state.data = out.str();
  }
  else state.data = stage + ": " + status_;
  state_pub_.publish(state);
}

PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorPerchingNavigator, aerial_robot_navigation::BaseNavigator)
