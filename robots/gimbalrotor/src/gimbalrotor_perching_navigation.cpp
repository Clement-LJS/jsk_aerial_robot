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

  min_valid_radius_(0.05),
  max_pitch_delta_(0.78539816339),  // 45 deg
  arc_pitch_sign_(1.0),

  perching_enable_topic_("perching/enable"),
  branch_pose_topic_("perching/branch_pose"),
  perching_point_topic_("perching/point"),
  relock_topic_("perching/relock"),
  reset_topic_("perching/reset"),

  has_branch_pose_(false),
  has_perching_point_(false),

  locked_radius_(0.0),
  locked_y_offset_(0.0),
  locked_x_side_(1.0)
{
  branch_pos_world_.setValue(0.0, 0.0, 0.0);
  perching_point_world_.setValue(0.0, 0.0, 0.0);

  locked_robot_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_rpy_.setValue(0.0, 0.0, 0.0);
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
 
  locked_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("perching/locked_pose", 1, true);
  commanded_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("perching/commanded_pose", 1);

  ROS_WARN("[GimbalrotorPerchingNavigator] initialized");
  ROS_WARN("[GimbalrotorPerchingNavigator] enable topic: %s", perching_enable_topic_.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] branch pose topic: %s", branch_pose_topic_.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] perching point topic: %s", perching_point_topic_.c_str());
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

  getParam<double>(navi_nh, "perching_min_valid_radius", min_valid_radius_, 0.05);
  getParam<double>(navi_nh, "perching_max_pitch_delta", max_pitch_delta_, 0.78539816339);
  getParam<double>(navi_nh, "perching_arc_pitch_sign", arc_pitch_sign_, 1.0);

  getParam<std::string>(navi_nh, "perching_enable_topic", perching_enable_topic_, "perching/enable");
  getParam<std::string>(navi_nh, "perching_branch_pose_topic", branch_pose_topic_, "perching/branch_pose");
  getParam<std::string>(navi_nh, "perching_point_topic", perching_point_topic_, "perching/point");
  getParam<std::string>(navi_nh, "perching_relock_topic", relock_topic_, "perching/relock");
  getParam<std::string>(navi_nh, "perching_reset_topic", reset_topic_, "perching/reset");
}

void GimbalrotorPerchingNavigator::perchingEnableCallback(const std_msgs::BoolConstPtr& msg)
{
  perching_enable_ = msg->data;

  if(perching_enable_)
  {
    ROS_WARN("[GimbalrotorPerchingNavigator] perching ENABLED");

    if(!perching_locked_ || !perching_lock_once_)
    {
      tryLockPerching("enable callback");
    }
  }
  else
  {
    ROS_WARN("[GimbalrotorPerchingNavigator] perching DISABLED");
  }
}

void GimbalrotorPerchingNavigator::branchPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  /*
   * Only use branch position.
   * Do not use branch orientation because a circular branch/pipe orientation can be physically meaningless.
   */
  branch_pos_world_.setValue(msg->pose.position.x,
                             msg->pose.position.y,
                             msg->pose.position.z);

  has_branch_pose_ = true;

  if(!has_perching_point_)
  {
    perching_point_world_ = branch_pos_world_;
  }
}

void GimbalrotorPerchingNavigator::perchingPointCallback(const geometry_msgs::PointStampedConstPtr& msg)
{
  /*
   * Preferred pivot/contact point.
   *
   * Use this when the exact grasp/contact/perching point is not identical to
   * the branch mocap origin.
   */
  perching_point_world_.setValue(msg->point.x,
                                 msg->point.y,
                                 msg->point.z);

  has_perching_point_ = true;
}

void GimbalrotorPerchingNavigator::relockCallback(const std_msgs::EmptyConstPtr& msg)
{
  (void)msg;

  perching_locked_ = false;
  locked_radius_ = 0.0;
  locked_radius_vec_world_.setValue(0.0, 0.0, 0.0);

  tryLockPerching("manual relock");
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
  locked_y_offset_ = 0.0;
  locked_x_side_ = 1.0;

  locked_robot_pos_world_.setValue(0.0, 0.0, 0.0);
  locked_robot_rpy_.setValue(0.0, 0.0, 0.0);
  locked_radius_vec_world_.setValue(0.0, 0.0, 0.0);

  ROS_WARN("[GimbalrotorPerchingNavigator] perching lock reset");
}

bool GimbalrotorPerchingNavigator::tryLockPerching(const std::string& reason)
{
  if(perching_locked_ && perching_lock_once_)
  {
    return true;
  }

  if(require_branch_point_ && !has_branch_pose_ && !has_perching_point_)
  {
    ROS_WARN_THROTTLE(1.0, "[GimbalrotorPerchingNavigator] cannot lock: waiting for branch pose or perching point");
    return false;
  }

  if(!has_perching_point_)
  {
    perching_point_world_ = branch_pos_world_;
  }

  locked_robot_pos_world_ = getCurrentRobotPos();
  locked_robot_rpy_ = getCurrentRobotRPY();

  locked_radius_vec_world_ = locked_robot_pos_world_ - perching_point_world_;
  locked_radius_ = norm3D(locked_radius_vec_world_);

  if(locked_radius_ < min_valid_radius_)
  {
    ROS_WARN_THROTTLE(1.0, "[GimbalrotorPerchingNavigator] cannot lock: invalid radius %.4f", locked_radius_);
    return false;
  }

  locked_y_offset_ = locked_robot_pos_world_.y() - perching_point_world_.y();

  if(locked_robot_pos_world_.x() - perching_point_world_.x() >= 0.0)
  {
    locked_x_side_ = 1.0;
  }
  else
  {
    locked_x_side_ = -1.0;
  }

  perching_locked_ = true;

  ROS_WARN("[GimbalrotorPerchingNavigator] perching locked by %s", reason.c_str());
  ROS_WARN("[GimbalrotorPerchingNavigator] pivot: x %.3f, y %.3f, z %.3f",
           perching_point_world_.x(),
           perching_point_world_.y(),
           perching_point_world_.z());
  ROS_WARN("[GimbalrotorPerchingNavigator] locked robot pos: x %.3f, y %.3f, z %.3f",
           locked_robot_pos_world_.x(),
           locked_robot_pos_world_.y(),
           locked_robot_pos_world_.z());
  ROS_WARN("[GimbalrotorPerchingNavigator] locked rpy deg: roll %.2f, pitch %.2f, yaw %.2f",
           locked_robot_rpy_.x() * 180.0 / PI,
           locked_robot_rpy_.y() * 180.0 / PI,
           locked_robot_rpy_.z() * 180.0 / PI);
  ROS_WARN("[GimbalrotorPerchingNavigator] radius: %.3f m", locked_radius_);

  publishLockedDebugPose();

  return true;
}

void GimbalrotorPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  aerial_robot_msgs::FlightNav nav_msg = *msg;

  if(perching_enable_)
  {
    applyPerchingConstraint(nav_msg);
  }

  aerial_robot_msgs::FlightNavConstPtr nav_msg_ptr(new aerial_robot_msgs::FlightNav(nav_msg));

  BaseNavigator::naviCallback(nav_msg_ptr);

  if(nav_msg.roll_nav_mode == NAV_MODE_POS)
  {
    setTargetRoll(nav_msg.target_roll);
  }

  if(nav_msg.pitch_nav_mode == NAV_MODE_POS)
  {
    setTargetPitch(nav_msg.target_pitch);
  }
}

void GimbalrotorPerchingNavigator::applyPerchingConstraint(aerial_robot_msgs::FlightNav& nav_msg)
{
  if(!perching_enable_) return;

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
   */
  if(use_pitch_command_for_arc_ && hasPitchCommand(nav_msg))
  {
    const double target_pitch = getCommandedPitch(nav_msg);
    const tf::Vector3 target_pos = computeArcPositionFromPitch(target_pitch);

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
   * If no pitch command is given, but perching is enabled, hold the locked
   * branch-relative pose. This avoids falling back to an old free-flight target.
   */
  if(hold_locked_pose_without_pitch_command_ && !hasPositionCommand(nav_msg))
  {
    nav_msg.pos_xy_nav_mode = NAV_MODE_POS;
    nav_msg.pos_z_nav_mode = NAV_MODE_POS;

    nav_msg.target_pos_x = locked_robot_pos_world_.x();
    nav_msg.target_pos_y = locked_robot_pos_world_.y();
    nav_msg.target_pos_z = locked_robot_pos_world_.z();

    nav_msg.target_vel_x = 0.0;
    nav_msg.target_vel_y = 0.0;
    nav_msg.target_vel_z = 0.0;

    nav_msg.pitch_nav_mode = NAV_MODE_POS;
    nav_msg.target_pitch = locked_robot_rpy_.y();

    publishCommandedDebugPose(locked_robot_pos_world_, locked_robot_rpy_.y());

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
    target_pitch = locked_robot_rpy_.y() + nav_msg.target_pitch;
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
   * Branch-pivot pitch arc.
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
   * For now this assumes branch axis roughly world Y, so pitch arc is X-Z.
   */

  double delta_pitch = normalizeAngle(target_pitch - locked_robot_rpy_.y());
  delta_pitch = clamp(delta_pitch, -max_pitch_delta_, max_pitch_delta_);

  const double signed_delta = arc_pitch_sign_ * delta_pitch;

  tf::Matrix3x3 rot(tf::createQuaternionFromRPY(0.0, signed_delta, 0.0));
  tf::Vector3 rotated_radius = rot * locked_radius_vec_world_;

  tf::Vector3 target_pos = perching_point_world_ + rotated_radius;

  /*
   * Keep branch-axis direction locked for first experiment.
   * This prevents sliding sideways along the branch.
   */
  target_pos.setY(perching_point_world_.y() + locked_y_offset_);

  return target_pos;
}

tf::Vector3 GimbalrotorPerchingNavigator::projectPositionToPitchArc(const tf::Vector3& desired_pos) const
{
  const double cx = perching_point_world_.x();
  const double cy = perching_point_world_.y();
  const double cz = perching_point_world_.z();

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
  constrained_pos.setY(cy + locked_y_offset_);
  constrained_pos.setZ(cz + locked_radius_ * dz);

  return constrained_pos;
}

tf::Vector3 GimbalrotorPerchingNavigator::projectVelocityToPitchArcTangent(const tf::Vector3& desired_vel) const
{
  const double cx = perching_point_world_.x();
  const double cz = perching_point_world_.z();

  double rx = getCurrentRobotPos().x() - cx;
  double rz = getCurrentRobotPos().z() - cz;

  double length_xz = norm2D(rx, rz);

  if(length_xz < 1.0e-6)
  {
    return tf::Vector3(0.0, 0.0, 0.0);
  }

  rx /= length_xz;
  rz /= length_xz;

  /*
   * Tangent in X-Z plane.
   */
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
  if(value < min_value) return min_value;
  if(value > max_value) return max_value;
  return value;
}

double GimbalrotorPerchingNavigator::normalizeAngle(double angle) const
{
  while(angle > PI) angle -= 2.0 * PI;
  while(angle < -PI) angle += 2.0 * PI;
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