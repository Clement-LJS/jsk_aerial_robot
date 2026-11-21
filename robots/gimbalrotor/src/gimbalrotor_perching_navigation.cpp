#include <gimbalrotor/gimbalrotor_perching_navigation.h>
#include <gimbalrotor/model/gimbalrotor_perching_model.h>
#include <pluginlib/class_list_macros.h>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <set>
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

double GimbalrotorPerchingNavigator::steadyTime()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

GimbalrotorPerchingNavigator::GimbalrotorPerchingNavigator():
  GimbalrotorNavigator(),

  perching_mode_(perching_geometry::Mode::DISABLED),
  perching_locked_(false),
  accept_uav_nav_pitch_command_(false),
  command_pitch_sign_(1.0),

  perching_lock_once_(true),

  command_pitch_as_delta_(false),
  use_pitch_command_for_arc_(true),

  active_perching_hold_enable_(true),

  min_valid_radius_(0.05),
  max_pitch_delta_(0.78539816339),  // 45 deg
  arc_pitch_sign_(1.0),

  perching_enable_topic_("perching/enable"),
  perching_slanted_enable_topic_("perching/slanted_enable"),
  locked_pivot_topic_("perching/locked_pivot"),
  relock_topic_("perching/relock"),
  reset_topic_("perching/reset"),
  manual_pitch_delta_topic_("perching/manual_pitch_delta"),

  active_pitch_delta_(0.0),
  locked_radius_(0.0)
{
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
  model_ = boost::dynamic_pointer_cast<GimbalrotorRobotModel>(robot_model);
  readProfile();
  initializeMechanism();

  perching_enable_sub_ = nh_.subscribe<std_msgs::Bool>(perching_enable_topic_, 1,
      [this](const std_msgs::BoolConstPtr& msg) {
        selectPerchingMode(perching_geometry::Mode::NORMAL, msg->data);
      });
  perching_slanted_enable_sub_ = nh_.subscribe<std_msgs::Bool>(perching_slanted_enable_topic_, 1,
      [this](const std_msgs::BoolConstPtr& msg) {
        selectPerchingMode(perching_geometry::Mode::SLANTED, msg->data);
      });
  relock_sub_ = nh_.subscribe(relock_topic_, 1, &GimbalrotorPerchingNavigator::relockCallback, this);
  reset_sub_ = nh_.subscribe<std_msgs::Empty>(reset_topic_, 1,
      [this](const std_msgs::EmptyConstPtr&) {
        std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
        resetPerchingLock();
      });

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

  getParam<std::string>(navi_nh, "perching_enable_topic", perching_enable_topic_, "perching/enable");
  getParam<std::string>(navi_nh, "perching_slanted_enable_topic", perching_slanted_enable_topic_, "perching/slanted_enable");
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
    onNonHover();
    if(perching_mode_ != perching_geometry::Mode::DISABLED) resetPerchingLock();
    perching_mode_ = perching_geometry::Mode::DISABLED;
    last_update_ = 0;
    publishDiagnostics();
    return;
  }
  if(model_ && !model_valid_ && robot_model_->initialized()) model_valid_ = resolveMechanismModel();
  const double now = ros::Time::now().toSec();
  const double elapsed = last_update_ > 0 ? now - last_update_ : 0;
  last_update_ = now;
  if(elapsed < 0)
  {
    onClockRewind();
    resetPerchingLock();
  }
  cycle_dt_ = elapsed > 0 && elapsed <= max_gap_ ? std::min(elapsed, max_step_) : 0;
  // Reference acceptance is called from the active controller cycle, after its
  // lifecycle gate and before PID target capture. Callbacks cannot publish joints.
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

void GimbalrotorPerchingNavigator::relockCallback(const std_msgs::EmptyConstPtr& msg)
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

  if(navi_state_ != HOVER_STATE)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] "
        "perching relock ignored because navigation state is "
        "not HOVER_STATE.");

    return;
  }

  if(!perching_locked_) resetPerchingLock();
  // Model resolution and candidate validation share these caches under this mutex.
  const auto previous_tree = geometry_tree_;
  const auto previous_joints = held_joints_;
  const auto previous_cog_to_body = cog_to_body_;
  const auto previous_pivot = pivot_;
  const bool previous_model_valid = model_valid_;
  if(!tryLockPerching("manual relock", true))
  {
    // Geometry rejection preserves the old lock. A dynamics failure has already
    // reset it; do not restore only half of that retired session.
    if(perching_locked_)
    {
      geometry_tree_ = previous_tree;
      held_joints_ = previous_joints;
      cog_to_body_ = previous_cog_to_body;
      pivot_ = previous_pivot;
      model_valid_ = previous_model_valid;
    }
    ROS_ERROR(
        "[GimbalrotorPerchingNavigator] "
        "manual perching relock failed");
    return;
  }

  active_pitch_delta_ = 0.0;
}

void GimbalrotorPerchingNavigator::resetPerchingLock()
{
  auto dynamics = boost::dynamic_pointer_cast<GimbalrotorPerchingModel>(model_);
  if(dynamics) dynamics->unlock();
  admittance_diagnostics_.clear();
  resetMotion();
  if(perching_locked_ && navi_state_ == HOVER_STATE)
  {
    setTargetVel(tf::Vector3(0, 0, 0));
    setTargetAcc(tf::Vector3(0, 0, 0));
    setTargetOmega(tf::Vector3(0, 0, 0));
    setTargetAngAcc(tf::Vector3(0, 0, 0));
  }
  perching_locked_ = false;
  locked_radius_ = 0.0;
  active_pitch_delta_ = 0.0;
  lock_stamp_ = ros::Time(0);
  pivot_ = Pose::Identity();
  hinge_arc_ = perching_geometry::Lock();
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
  perching_geometry::Pose pose;
  if(!hinge_arc_.target(command_pitch_sign_ * delta, max_pitch_delta_, arc_pitch_sign_, pose))
    return;
  active_pitch_delta_ = pose.delta;
  velocity_active_ = false;
  rigid_command_pending_ = true;
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
  if(perching_mode_ != perching_geometry::Mode::DISABLED && navi_state_ == HOVER_STATE)
  {
    if(applyPerchingConstraint(*msg)) rigid_command_pending_ = true;
    return;
  }

  GimbalrotorNavigator::naviCallback(msg);
}

tf::Vector3 GimbalrotorPerchingNavigator::getCurrentBaselinkPos() const
{
  return estimator_->getPos(Frame::BASELINK, estimate_mode_);
}

tf::Matrix3x3 GimbalrotorPerchingNavigator::getCurrentBaselinkRot() const
{
  return estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
}

bool GimbalrotorPerchingNavigator::applyPerchingConstraint(const aerial_robot_msgs::FlightNav& msg)
{
  if(perching_mode_ == perching_geometry::Mode::DISABLED || navi_state_ != HOVER_STATE) return false;
  if(!perching_locked_ && !tryLockPerching("first perching command")) return false;

  const bool position_command = msg.pos_xy_nav_mode == NAV_MODE_POS ||
      msg.pos_xy_nav_mode == NAV_MODE_POS_VEL ||
      msg.pos_z_nav_mode == NAV_MODE_POS ||
      msg.pos_z_nav_mode == NAV_MODE_POS_VEL;
  const bool velocity_command = msg.pos_xy_nav_mode == NAV_MODE_VEL ||
      msg.pos_xy_nav_mode == NAV_MODE_POS_VEL ||
      msg.pos_z_nav_mode == NAV_MODE_VEL ||
      msg.pos_z_nav_mode == NAV_MODE_POS_VEL;
  const tf::Vector3 desired_velocity = getDesiredVelocity(msg);
  perching_geometry::Pose pose;
  if(use_pitch_command_for_arc_ && hasPitchCommand(msg))
  {
    if(command_pitch_as_delta_)
    {
      if(!hinge_arc_.target(command_pitch_sign_ * msg.target_pitch,
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
      if(!hinge_arc_.targetOrientation(desired, max_pitch_delta_, arc_pitch_sign_, pose)) return false;
    }
    active_pitch_delta_ = pose.delta;
    velocity_active_ = false;
    publishCommandedDebugPose(pose);
    return true;
  }

  if(position_command)
  {
    if(!hinge_arc_.project(getDesiredPosition(msg), max_pitch_delta_, arc_pitch_sign_, pose) &&
       !hinge_arc_.target(active_pitch_delta_, max_pitch_delta_, arc_pitch_sign_, pose)) return false;
    active_pitch_delta_ = pose.delta;
    velocity_active_ = false;
    publishCommandedDebugPose(pose);
  }
  else if(!position_command && !velocity_command)
  {
    if(!activePose(pose)) return false;
    publishCommandedDebugPose(pose);
    return true;
  }

  if(velocity_command)
  {
    if(!hinge_arc_.target(active_pitch_delta_, max_pitch_delta_, arc_pitch_sign_, pose)) return false;
    const tf::Vector3 direction = hinge_arc_.axis.cross(pose.position - hinge_arc_.pivot);
    const tf::Vector3 tangent = hinge_arc_.tangentVelocity(pose.position, desired_velocity);
    velocity_command_ = std::clamp(direction.dot(tangent) / direction.length2(), -rate_limit_, rate_limit_);
    velocity_active_ = std::abs(velocity_command_) > 1e-9;
    velocity_expires_ = ros::Time::now().toSec() + velocity_duration_;
    publishCommandedDebugPose(pose);
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

tf::Vector3 GimbalrotorPerchingNavigator::getCurrentRobotRPY() const
{
  return estimator_->getEuler(Frame::COG, estimate_mode_);
}

tf::Vector3 GimbalrotorPerchingNavigator::getDesiredPosition(const aerial_robot_msgs::FlightNav& nav_msg) const
{
  tf::Vector3 desired_pos = estimator_->getPos(Frame::COG, estimate_mode_);

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

void GimbalrotorPerchingNavigator::publishLockedDebugPose()
{
  geometry_msgs::PoseStamped msg;
  msg.header.stamp = lock_stamp_;
  msg.header.frame_id = world_;

  msg.pose.position.x = locked_robot_pos_world_.x();
  msg.pose.position.y = locked_robot_pos_world_.y();
  msg.pose.position.z = locked_robot_pos_world_.z();

  tf::quaternionTFToMsg(hinge_arc_.orientation, msg.pose.orientation);

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
  if(!perching_locked_ || !hinge_arc_.target(active_pitch_delta_, max_pitch_delta_, arc_pitch_sign_, pose))
    return false;
  return true;
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
  if(perching_mode_ == perching_geometry::Mode::DISABLED || !perching_locked_ || navi_state_ != HOVER_STATE ||
     lock_stamp != lock_stamp_ || !perching_geometry::finite(nominal_position)) return false;
  // Compliance Y is a physical axis angle. Convert to the logical coordinate
  // before the shared helper applies arc_pitch_sign exactly once.
  if(!hinge_arc_.target(active_pitch_delta_ + physical_offset / arc_pitch_sign_,
                       max_pitch_delta_, arc_pitch_sign_, pose)) return false;
  return true;
}

void GimbalrotorPerchingNavigator::readProfile()
{
  ros::NodeHandle pn(nh_, "navigation/perching");
  pn.param<std::string>("debug_prefix", prefix_, "perching");
  pn.param<std::string>("diagnostic_name", status_name_, "perching");
  pn.param<std::string>("world_frame", world_, "world");
  pn.param("pitch_command_sign", command_pitch_sign_, command_pitch_sign_);
  pn.param("tracking_tolerance", tracking_tolerance_, tracking_tolerance_);
  pn.param("feedback_timeout", timeout_, timeout_);
  pn.param("future_tolerance", future_tolerance_, future_tolerance_);
  pn.param("max_execution_step", max_step_, max_step_);
  pn.param("max_execution_gap", max_gap_, max_gap_);
  pn.param("pitch_rate", rate_limit_, rate_limit_);
  pn.param("body_pitch_velocity_command_duration", velocity_duration_, velocity_duration_);
  pn.param("pitch_rezero_rate_tolerance", rezero_rate_tolerance_, rezero_rate_tolerance_);
  const double positive[] = {timeout_, max_step_, max_gap_, rate_limit_, velocity_duration_, rezero_rate_tolerance_};
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
}

bool GimbalrotorPerchingNavigator::configureGeometry(std::string& reason)
{
  reason = "invalid pivot, coordinate map or joint limits";
  if(!kdl_parser::treeFromUrdfModel(model_->getUrdfModel(), geometry_tree_)) return false;
  const auto& tree = geometry_tree_;
  KDL::Chain chain;
  if(!tree.getChain(model_->getBaselinkName(), "perching_pivot", chain)) return false;
  held_joints_ = model_->getJointPositionsSnapshot();
  if(held_joints_.rows() != tree.getNrOfJoints() || !held_joints_.data.allFinite()) return false;
  std::set<std::string> remaining;
  for(unsigned i = 0; i < chain.getNrOfSegments(); ++i)
  {
    const auto& joint = chain.getSegment(i).getJoint();
    if(joint.getType() != KDL::Joint::None) remaining.insert(joint.getName());
  }
  for(const auto& name : active_joints_)
  {
    const auto joint = model_->getUrdfModel().getJoint(name);
    if(!remaining.erase(name) || !model_->getJointIndexMap().count(name) || !joint || joint->mimic ||
       (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::CONTINUOUS) ||
       !joint->limits || !std::isfinite(joint->limits->velocity) || joint->limits->velocity <= 0) return false;
    if(joint->type == urdf::Joint::REVOLUTE && joint->limits->lower >= joint->limits->upper) return false;
  }
  if(!remaining.empty()) return false;
  cog_to_body_ = model_->getCogDesireOrientation<Eigen::Matrix3d>().transpose();
  reason.clear();
  return true;
}

bool GimbalrotorPerchingNavigator::bodyContact(const Vector& q, Pose& contact, Eigen::Vector3d& cog) const
{
  if(q.size() != static_cast<int>(active_joints_.size() + 1) || !q.allFinite()) return false;
  KDL::JntArray joints = held_joints_;
  for(int i = 1; i < q.size(); ++i) joints(model_->getJointIndexMap().at(active_joints_[i - 1])) = q[i];
  // The private URDF tree keeps candidate FK out of the live model's KDL caches.
  std::map<std::string, KDL::Frame> poses;
  KDL::TreeFkSolverPos_recursive fk(geometry_tree_);
  for(const auto& item : geometry_tree_.getSegments())
    if(fk.JntToCart(joints, poses[item.first], item.first) < 0) return false;
  if(!poses.count(model_->getBaselinkName()) || !poses.count("perching_pivot")) return false;
  const auto& body = poses.at(model_->getBaselinkName());
  contact = perching_geometry::fromKdl(body.Inverse() * poses.at("perching_pivot"));
  KDL::RigidBodyInertia total = KDL::RigidBodyInertia::Zero();
  for(const auto& item : model_->getInertiaMap())
  {
    const auto frame = poses.find(item.first);
    if(frame == poses.end()) return false;
    perching_geometry::accumulateLinkInertia(total, frame->second, item.first, item.second, model_->getExtraModuleMap());
  }
  if(!std::isfinite(total.getMass()) || total.getMass() <= 1e-9) return false;
  const auto center = body.Inverse() * total.getCOG();
  cog = Eigen::Vector3d(center.x(), center.y(), center.z());
  return perching_geometry::poseValid(contact) && cog.allFinite();
}

bool GimbalrotorPerchingNavigator::lockGeometry(const Pose& body, const Vector& q, Reference& initial)
{
  Pose contact;
  Eigen::Vector3d cog_body;
  if(!perching_geometry::poseValid(body) || !bodyContact(q, contact, cog_body)) return false;
  Pose rotation = Pose::Identity();
  rotation.linear() = perching_geometry::expSO3(-Eigen::Vector3d::UnitY() * q[0]);
  const Pose pivot = body * contact * rotation;
  const Pose cog_pose = worldCogPose(body, cog_body);
  if(!perching_geometry::poseValid(pivot) || !perching_geometry::poseValid(cog_pose)) return false;

  pivot_ = pivot;
  initial.body = body;
  initial.cog = cog_pose;
  return true;
}

bool GimbalrotorPerchingNavigator::lockDynamicsModel()
{
  auto dynamics = boost::dynamic_pointer_cast<GimbalrotorPerchingModel>(model_);
  if(!dynamics) return true; // Ordinary PID has no constrained dynamics model.
  std::string reason;
  if(dynamics->configure(active_joints_, reason) && dynamics->lock(pivot_, held_joints_, cog_to_body_)) return true;
  ROS_ERROR_STREAM("Perching dynamics lock failed: " << reason);
  resetPerchingLock();
  perching_mode_ = perching_geometry::Mode::DISABLED;
  status_ = "dynamics lock failed; perching reset";
  return false;
}

GimbalrotorPerchingNavigator::Pose GimbalrotorPerchingNavigator::worldCogPose(
    const Pose& body, const Eigen::Vector3d& cog_body) const
{
  Pose cog = Pose::Identity();
  cog.translation() = body * cog_body;
  cog.linear() = body.linear() * cog_to_body_;
  return cog;
}

bool GimbalrotorPerchingNavigator::estimateCoordinates(const Pose& body, Vector& q, double& pivot_error, double& hinge_error) const
{
  Pose contact; Eigen::Vector3d cog;
  if(!perching_geometry::poseValid(body) || !bodyContact(q, contact, cog)) return false;
  const Pose relative = pivot_.inverse() * body * contact;
  Eigen::Quaterniond rotation(relative.linear());
  const double angle = 2 * std::atan2(rotation.vec().dot(Eigen::Vector3d::UnitY()), rotation.w());
  q[0] += std::atan2(std::sin(angle - q[0]), std::cos(angle - q[0]));
  pivot_error = relative.translation().norm();
  hinge_error = perching_geometry::logSO3(perching_geometry::expSO3(-Eigen::Vector3d::UnitY() * q[0]) * relative.linear()).norm();
  return q.allFinite() && std::isfinite(pivot_error) && std::isfinite(hinge_error);
}

bool GimbalrotorPerchingNavigator::resolveMechanismModel()
{
  if(!model_) return false;
  std::string reason;
  if(!configureGeometry(reason))
  { ROS_ERROR_STREAM("Perching model: " << reason); return false; }
  return true;
}

bool GimbalrotorPerchingNavigator::fresh(double stamp, double receipt) const
{
  const double age = ros::Time::now().toSec() - stamp;
  return stamp > 0 && std::isfinite(age) && age >= -future_tolerance_ && age <= timeout_ &&
      receipt > 0 && (ros::Time::isSimTime() || steadyTime() - receipt <= timeout_);
}

bool GimbalrotorPerchingNavigator::observation(Sample& sample) const
{
  if(!model_ || !perching_locked_) return false;
  Eigen::Matrix3d rotation;
  Eigen::Vector3d position;
  tf::matrixTFToEigen(getCurrentBaselinkRot(), rotation);
  tf::vectorTFToEigen(getCurrentBaselinkPos(), position);
  sample.body.linear() = rotation;
  sample.body.translation() = position;
  tf::vectorTFToEigen(estimator_->getAngularVel(Frame::BASELINK, estimate_mode_), sample.omega);
  sample.stamp = estimator_->getImuLatestTimeStamp();
  sample.receipt = steadyTime();
  sample.q = sample.rate = Vector::Zero(1);
  sample.effort = Vector::Zero(1); // The passive hinge has no actuator.
  sample.effort_valid = true;
  double pivot, hinge;
  if(!estimateCoordinates(sample.body, sample.q, pivot, hinge)) return false;
  sample.rate[0] = pivot_.linear().col(1).dot(sample.body.linear() * sample.omega);
  return perching_geometry::poseValid(sample.body) && sample.rate.allFinite() && fresh(sample.stamp, sample.receipt);
}

bool GimbalrotorPerchingNavigator::motionFeedbackValid() const
{
  if(!perching_locked_ || navi_state_ != HOVER_STATE) return false;
  Sample sample;
  return observation(sample);
}

bool GimbalrotorPerchingNavigator::tryLockPerching(const std::string& reason, bool relock)
{
  if(navi_state_ != HOVER_STATE)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorPerchingNavigator] "
        "cannot create perching lock outside HOVER_STATE.");

    return false;
  }

  if(perching_locked_ && perching_lock_once_ && !relock)
  {
    return true;
  }

  if(!model_ || !fresh(estimator_->getImuLatestTimeStamp(), steadyTime())) return false;
  model_valid_ = resolveMechanismModel();
  if(!model_valid_) return false;
  Pose body = Pose::Identity();
  Eigen::Matrix3d body_rotation;
  Eigen::Vector3d body_position;
  tf::matrixTFToEigen(getCurrentBaselinkRot(), body_rotation);
  tf::vectorTFToEigen(getCurrentBaselinkPos(), body_position);
  body.linear() = body_rotation;
  body.translation() = body_position;
  Reference anchored;
  if(!lockGeometry(body, Vector::Zero(1), anchored)) return false;
  const Eigen::Vector3d pivot = pivot_.translation();
  const tf::Vector3 contact_position(pivot.x(), pivot.y(), pivot.z());
  const auto& position = anchored.cog.translation();
  const tf::Vector3 cog_position(position.x(), position.y(), position.z());
  const Eigen::Quaterniond orientation(anchored.cog.linear());
  const tf::Quaternion reference_orientation(orientation.x(), orientation.y(), orientation.z(), orientation.w());
  const Eigen::Vector3d axis = pivot_.linear().col(1);
  perching_geometry::Lock hinge_arc;
  if(!hinge_arc.initialize(cog_position, contact_position, reference_orientation,
                           min_valid_radius_, tf::Vector3(axis.x(), axis.y(), axis.z())))
  {
    ROS_WARN("[GimbalrotorPerchingNavigator] rejected invalid perching geometry");
    return false;
  }
  // Retire the old motion only after the replacement geometry is accepted.
  if(!lockDynamicsModel()) return false;
  if(perching_locked_) resetMotion();
  hinge_arc_ = hinge_arc;
  locked_pivot_world_ = contact_position;
  locked_robot_pos_world_ = cog_position;
  double roll, pitch, yaw;
  tf::Matrix3x3(reference_orientation).getRPY(roll, pitch, yaw);
  reference_locked_rpy_.setValue(roll, pitch, yaw);
  const tf::Vector3 measured_rpy = getCurrentRobotRPY();
  locked_radius_ = hinge_arc_.radial().length();
  active_pitch_delta_ = 0.0;
  rigid_applied_pose_.position = locked_robot_pos_world_;
  rigid_applied_pose_.orientation = reference_orientation;
  rigid_applied_pose_.delta = 0;
  rigid_applied_rate_ = rigid_applied_acceleration_ = 0;
  rigid_applied_valid_ = true;
  last_update_ = ros::Time::now().toSec();
  commitPerchingLockIdentity();

  ROS_WARN("[GimbalrotorPerchingNavigator] perching mode: %s, locked by %s",
           perching_mode_ == perching_geometry::Mode::NORMAL ? "NORMAL" : "SLANTED", reason.c_str());
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
           hinge_arc_.axis.x(), hinge_arc_.axis.y(), hinge_arc_.axis.z());
  ROS_WARN("[GimbalrotorPerchingNavigator] pitch-plane radius: %.3f m",
           locked_radius_);
  publishLockedDebugPose();
  publishLockedPivot();
  perching_geometry::Pose pose;
  if(activePose(pose)) publishCommandedDebugPose(pose);

  return true;
}

void GimbalrotorPerchingNavigator::resetMotion()
{
  velocity_active_ = false;
  applied_pitch_offset_ = applied_offset_rate_ = 0;
  applied_stamp_ = last_update_ = 0;
  rigid_applied_valid_ = false;
  rigid_command_pending_ = false;
  rigid_applied_rate_ = rigid_applied_acceleration_ = 0;
}

bool GimbalrotorPerchingNavigator::supportsPerchingMode(perching_geometry::Mode mode) const
{
  return mode == perching_geometry::Mode::NORMAL || mode == perching_geometry::Mode::SLANTED;
}

bool GimbalrotorPerchingNavigator::ownsNavigationTargets() const
{
  return perching_locked_ && perching_mode_ != perching_geometry::Mode::DISABLED;
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
  return !velocity_active_;
}

bool GimbalrotorPerchingNavigator::trackingHeld() const
{
  Sample sample;
  return observation(sample) && std::abs(sample.q[0] - rigidAngle()) <= tracking_tolerance_ &&
      std::abs(sample.rate[0]) <= rezero_rate_tolerance_;
}

bool GimbalrotorPerchingNavigator::applyPerchingReference(double pitch_offset, double rate, double acceleration)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(navi_state_ != HOVER_STATE || !std::isfinite(pitch_offset) || !std::isfinite(rate) || !std::isfinite(acceleration)) return false;
  if(perching_mode_ == perching_geometry::Mode::DISABLED || !perching_locked_)
  {
    publishDiagnostics();
    return false;
  }
  Sample sample;
  const bool feedback = observation(sample);
  if(feedback) estimateCoordinates(sample.body, sample.q, pivot_error_, hinge_error_);
  const double now = ros::Time::now().toSec();
  if(applied_stamp_ == now) return false;
  if(velocity_active_ && now >= velocity_expires_) velocity_active_ = false;
  const double previous_nominal = active_pitch_delta_;
  if(applied_stamp_ > 0 && velocity_active_ && cycle_dt_ > 0 && feedback)
  {
    perching_geometry::Pose candidate;
    if(hinge_arc_.target(active_pitch_delta_ + velocity_command_ * cycle_dt_ / arc_pitch_sign_,
                        max_pitch_delta_, arc_pitch_sign_, candidate))
    {
      active_pitch_delta_ = candidate.delta;
      rigid_command_pending_ = true;
    }
    if(std::abs(active_pitch_delta_) >= max_pitch_delta_) velocity_active_ = false;
  }
  perching_geometry::Pose pose;
  const tf::Vector3 nominal_position = getTargetPos();
  if(cycle_dt_ <= 0 || !feedback ||
     (applied_stamp_ == 0 && (pitch_offset != 0 || rate != 0 || acceleration != 0)) ||
     ((pitch_offset != 0 || applied_pitch_offset_ != 0) &&
      (std::abs(rate) > rate_limit_ + 1e-6 ||
       std::abs(arc_pitch_sign_ * active_pitch_delta_ + pitch_offset - rigidAngle()) > rate_limit_ * cycle_dt_ + 1e-8)) ||
     std::abs(active_pitch_delta_ + pitch_offset / arc_pitch_sign_) > max_pitch_delta_ ||
     !perchingAdmittanceTarget(lock_stamp_, pitch_offset, nominal_position, pose))
  {
    active_pitch_delta_ = previous_nominal;
    rigid_applied_rate_ = rigid_applied_acceleration_ = applied_offset_rate_ = 0;
    if(rigid_applied_valid_ && (active_perching_hold_enable_ || applied_stamp_ > 0)) publishApplied();
    publishDiagnostics();
    return false;
  }
  const bool first_reference = applied_stamp_ == 0;
  if(first_reference) pose = rigid_applied_pose_;
  const double applied_rate = first_reference ? 0 : arc_pitch_sign_ * (pose.delta - rigid_applied_pose_.delta) / cycle_dt_;
  rigid_applied_acceleration_ = first_reference ? 0 : (applied_rate - rigid_applied_rate_) / cycle_dt_;
  rigid_applied_rate_ = applied_rate;
  const bool correction_changed = pitch_offset != applied_pitch_offset_;
  applied_pitch_offset_ = pitch_offset; applied_offset_rate_ = rate;
  rigid_applied_pose_ = pose;
  rigid_applied_valid_ = true;
  applied_stamp_ = now;
  if(active_perching_hold_enable_ || first_reference || rigid_command_pending_ || correction_changed || pitch_offset != 0 || rate != 0)
  {
    publishApplied();
    // The first cycle installs the lock pose; a staged command still follows it.
    if(!first_reference) rigid_command_pending_ = false;
  }
  publishCommandedDebugPose(pose);
  publishDiagnostics();
  return true;
}

void GimbalrotorPerchingNavigator::publishApplied()
{
  setXyControlMode(POS_CONTROL_MODE);
  Eigen::Matrix3d rotation;
  tf::matrixTFToEigen(tf::Matrix3x3(rigid_applied_pose_.orientation), rotation);
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const Eigen::Matrix3d cog = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  setSynchronizedBaselinkRotationRelativeToCog(perching_geometry::tfQuaternion(cog.transpose() * rotation * cog_to_body_.transpose()));
  setTargetPos(rigid_applied_pose_.position);
  setTargetRPY(tf::Vector3(0, 0, yaw));
  setTargetVel(tf::Vector3(0, 0, 0)); setTargetAcc(tf::Vector3(0, 0, 0));
  setTargetOmega(tf::Vector3(0, 0, 0)); setTargetAngAcc(tf::Vector3(0, 0, 0));
  if(boost::dynamic_pointer_cast<GimbalrotorPerchingModel>(model_))
  {
    const tf::Vector3 radial = rigid_applied_pose_.position - hinge_arc_.pivot;
    const tf::Vector3 tangent = hinge_arc_.axis.cross(radial);
    setTargetVel(tangent * rigid_applied_rate_);
    setTargetAcc(tangent * rigid_applied_acceleration_ + hinge_arc_.axis.cross(tangent) * rigid_applied_rate_ * rigid_applied_rate_);
    const tf::Matrix3x3 target_rotation(tf::createQuaternionFromYaw(yaw));
    setTargetOmega(target_rotation.inverse() * hinge_arc_.axis * rigid_applied_rate_);
    setTargetAngAcc(estimator_->getOrientation(Frame::COG, estimate_mode_).inverse() * hinge_arc_.axis * rigid_applied_acceleration_);
  }
}

void GimbalrotorPerchingNavigator::publishDiagnostics()
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
  for(const auto& key : {"admittance_configured", "admittance_enabled", "baseline_ready", "tare_collecting", "task_torque_valid"})
  {
    residual.key = key;
    residual.value = "0";
    diagnostic.values.push_back(residual);
  }
  residual.key = "admittance_status";
  residual.value = "unavailable: nominal navigation with ordinary flight controller";
  diagnostic.values.push_back(residual);
  for(const auto& item : admittance_diagnostics_)
  {
    auto existing = std::find_if(diagnostic.values.begin(), diagnostic.values.end(),
        [&](const diagnostic_msgs::KeyValue& value) { return value.key == item.first; });
    if(existing != diagnostic.values.end()) existing->value = item.second;
    else { residual.key = item.first; residual.value = item.second; diagnostic.values.push_back(residual); }
  }
  array.status.push_back(diagnostic);
  status_pub_.publish(array);
  std_msgs::String state;
  state.data = diagnostic.message + ": " + status_;
  state_pub_.publish(state);
}

PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorPerchingNavigator, aerial_robot_navigation::BaseNavigator)
