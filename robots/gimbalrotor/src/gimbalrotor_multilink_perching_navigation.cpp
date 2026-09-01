// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_multilink_perching_navigation.h>

#include <aerial_robot_model/model/aerial_robot_model.h>

#include <pluginlib/class_list_macros.h>

#include <kdl/chain.hpp>

#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <vector>

namespace aerial_robot_navigation
{

GimbalrotorMultilinkPerchingNavigator::
GimbalrotorMultilinkPerchingNavigator()
  : GimbalrotorPerchingNavigator(),
    joint_command_topic_("joints_ctrl"),
    joint_state_topic_("joint_states"),
    secondary_joint_target_topic_(
        "perching/multilink/secondary_joint_target"),
    secondary_position_tolerance_(0.02),
    secondary_velocity_tolerance_(0.05),
    secondary_settle_duration_(0.30),
    joint_state_timeout_(0.15),
    joint_state_future_tolerance_(0.02),
    pitch_axis_alignment_threshold_(0.90),
    secondary_axis_alignment_threshold_(0.90),
    pitch_command_sign_(1.0),
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
    multilink_model_valid_(false),
    multilink_lock_valid_(false),
    locked_contact_world_(KDL::Frame::Identity()),
    locked_pitch_joint_(0.0),
    locked_secondary_joint_(0.0),
    pitch_joint_nominal_target_(0.0),
    secondary_joint_nominal_target_(0.0),
    pitch_admittance_offset_(0.0),
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
}

void GimbalrotorMultilinkPerchingNavigator::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    double loop_du)
{
  GimbalrotorPerchingNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  multilinkRosParamInit();

  multilink_model_valid_ = false;
  const bool robot_model_ready =
      robot_model_ && robot_model_->initialized();

  if(robot_model_ready)
  {
    multilink_model_valid_ = resolveMechanismModel();
  }
  else
  {
    ROS_WARN("[GimbalrotorMultilinkPerchingNavigator] waiting for the robot model to initialize.");
  }

  joint_state_sub_ = nh_.subscribe(
      joint_state_topic_,
      1,
      &GimbalrotorMultilinkPerchingNavigator::jointStateCallback,
      this);
  secondary_joint_target_sub_ = nh_.subscribe(
      secondary_joint_target_topic_,
      1,
      &GimbalrotorMultilinkPerchingNavigator::
          secondaryJointTargetCallback,
      this);

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
  pitch_offset_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/multilink/pitch_joint_admittance_offset", 1);
  pitch_final_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/multilink/pitch_joint_final", 1);
  secondary_measured_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/multilink/secondary_joint_measured", 1);
  secondary_target_pub_ =
      nh_.advertise<std_msgs::Float64>(
          "perching/multilink/secondary_joint_target", 1);
  target_body_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(
          "perching/multilink/target_body_pose", 1);

  std_msgs::Bool valid_msg;
  valid_msg.data = multilink_model_valid_;
  model_valid_pub_.publish(valid_msg);

  if(multilink_model_valid_)
  {
    ROS_WARN(
        "[GimbalrotorMultilinkPerchingNavigator] model ready: "
        "pitch '%s', secondary '%s' (%s), contact '%s'.",
        pitch_joint_name_.c_str(),
        secondary_joint_name_.c_str(),
        secondary_axis_type_ == SECONDARY_AXIS_ROLL ? "roll" : "yaw",
        contact_link_name_.c_str());
  }
  else if(robot_model_ready)
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] multilink model is "
        "not valid. Locking and mechanism commands are disabled.");
  }
}

void GimbalrotorMultilinkPerchingNavigator::multilinkRosParamInit()
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
      pitch_axis_alignment_threshold_,
      0.90);
  multilink_nh.param(
      "secondary_axis_alignment_threshold",
      secondary_axis_alignment_threshold_,
      0.90);
  multilink_nh.param("pitch_command_sign", pitch_command_sign_, 1.0);
  multilink_nh.param("secondary_command_sign", secondary_command_sign_, 1.0);

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

  if(!std::isfinite(pitch_axis_alignment_threshold_) || pitch_axis_alignment_threshold_ <= 0.0 || pitch_axis_alignment_threshold_ > 1.0)
    pitch_axis_alignment_threshold_ = 0.90;
  if(!std::isfinite(secondary_axis_alignment_threshold_) || secondary_axis_alignment_threshold_ <= 0.0 || secondary_axis_alignment_threshold_ > 1.0)
    secondary_axis_alignment_threshold_ = 0.90;

  pitch_command_sign_ =
      std::isfinite(pitch_command_sign_) && pitch_command_sign_ < 0.0 ?
          -1.0 : 1.0;
  secondary_command_sign_ =
      std::isfinite(secondary_command_sign_) && secondary_command_sign_ < 0.0 ?
          -1.0 : 1.0;
}

bool GimbalrotorMultilinkPerchingNavigator::resolveJointLimits(
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
      "[GimbalrotorMultilinkPerchingNavigator] joint '%s' has no "
      "finite position limits and no valid configured fallback limits.",
      joint_name.c_str());
  return false;
}

bool GimbalrotorMultilinkPerchingNavigator::resolveMechanismModel()
{
  if(!robot_model_ || !robot_model_->initialized())
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] robot model is unavailable.");
    return false;
  }

  if(pitch_joint_name_.empty() || secondary_joint_name_.empty() || contact_link_name_.empty())
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] pitch_joint_name, "
        "secondary_joint_name, and contact_link_name must all be configured.");
    return false;
  }

  if(pitch_joint_name_ == secondary_joint_name_)
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] pitch and secondary "
        "joint names must be different.");
    return false;
  }

  const urdf::JointConstSharedPtr pitch_joint = robot_model_->getUrdfModel().getJoint(pitch_joint_name_);
  const urdf::JointConstSharedPtr secondary_joint = robot_model_->getUrdfModel().getJoint(secondary_joint_name_);

  if(!pitch_joint || !secondary_joint)
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] configured mechanism "
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
        "[GimbalrotorMultilinkPerchingNavigator] both configured joints "
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
        "[GimbalrotorMultilinkPerchingNavigator] configured joint axis "
        "is zero or non-finite.");
    return false;
  }

  pitch_axis_local_.normalize();
  secondary_axis_local_.normalize();

  if(std::abs(pitch_axis_local_.y()) < pitch_axis_alignment_threshold_)
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] pitch joint axis is not "
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
        "[GimbalrotorMultilinkPerchingNavigator] secondary joint axis is "
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
        "[GimbalrotorMultilinkPerchingNavigator] contact link '%s' "
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
        "[GimbalrotorMultilinkPerchingNavigator] configured joint is not "
        "a movable KDL model joint.");
    return false;
  }

  pitch_joint_index_ = pitch_index_it->second;
  secondary_joint_index_ = secondary_index_it->second;
  if(pitch_joint_index_ >= robot_model_->getJointPositions().rows() ||
     secondary_joint_index_ >= robot_model_->getJointPositions().rows())
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] configured joint index "
        "is outside the model joint array.");
    return false;
  }

  KDL::Chain chain;
  if(!robot_model_->getTree().getChain(robot_model_->getBaselinkName(), contact_link_name_, chain))
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] no KDL chain exists from "
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
          "[GimbalrotorMultilinkPerchingNavigator] unsupported movable "
          "joint '%s' lies in the baselink-to-contact chain.",
          joint.getName().c_str());
      return false;
    }
    ++movable_order;
  }

  if(pitch_order < 0 || secondary_order < 0 || pitch_order >= secondary_order || movable_order != 2)
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] chain must contain exactly "
        "pitch then secondary as its two movable joints.");
    return false;
  }

  const auto& joint_segment_map = robot_model_->getJointSegmentMap();
  const auto pitch_segments_it = joint_segment_map.find(pitch_joint_name_);
  if(pitch_segments_it == joint_segment_map.end() ||
     pitch_segments_it->second.empty())
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] pitch joint child link "
        "could not be resolved.");
    return false;
  }
  pitch_joint_child_link_name_ = pitch_segments_it->second.front();

  const auto& model_joint_names = robot_model_->getJointNames();
  const auto& parent_link_names = robot_model_->getJointParentLinkNames();
  const auto model_pitch_it = std::find(model_joint_names.begin(), model_joint_names.end(), pitch_joint_name_);
  if(model_pitch_it == model_joint_names.end())
    return false;
  const std::size_t model_pitch_index = std::distance(model_joint_names.begin(), model_pitch_it);
  if(model_pitch_index >= parent_link_names.size())
    return false;
  pitch_joint_parent_link_name_ = parent_link_names.at(model_pitch_index);

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

void GimbalrotorMultilinkPerchingNavigator::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
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
        "[GimbalrotorMultilinkPerchingNavigator] "
        "received stale or invalid joint state measurement "
        "(measurement age %.3f s).",
        measurement_age_at_receive);

    return;
  }

  std::lock_guard<std::mutex> lock(multilink_state_mutex_);

  if(joint_state_received_)
  {
    const double measurement_dt = (measurement_stamp - joint_state_measurement_stamp_).toSec();

    if(!std::isfinite(measurement_dt) || measurement_dt <= 0.0)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[GimbalrotorMultilinkPerchingNavigator] "
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

bool GimbalrotorMultilinkPerchingNavigator::readCurrentMechanismState(
    double& pitch_position,
    double& secondary_position,
    double& pitch_velocity,
    double& secondary_velocity) const
{
  std::lock_guard<std::mutex> lock(multilink_state_mutex_);

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

bool GimbalrotorMultilinkPerchingNavigator::computeBaselinkToContactTransform(
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

bool GimbalrotorMultilinkPerchingNavigator::computeBaselinkToCogVector(
    double pitch_position,
    double secondary_position,
    KDL::Vector& p_B_G) const
{
  if(!multilink_model_valid_ || !std::isfinite(pitch_position) ||
     !std::isfinite(secondary_position))
    return false;

  KDL::JntArray joints = robot_model_->getJointPositions();
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

    total_inertia = total_inertia + frame_it->second * inertia.second;
    for(const auto& extra : robot_model_->getExtraModuleMap())
    {
      if(extra.second.getName() == inertia.first)
      {
        total_inertia = total_inertia +
            frame_it->second *
            (extra.second.getFrameToTip() * extra.second.getInertia());
      }
    }
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

bool GimbalrotorMultilinkPerchingNavigator::tryLockPerching(
    const std::string& reason)
{
  if(getNaviState() != HOVER_STATE || !multilink_model_valid_)
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
        "[GimbalrotorMultilinkPerchingNavigator] cannot lock: named "
        "mechanism joint state is missing, invalid, or stale.");
    return false;
  }

  KDL::Frame T_B_C;
  if(!computeBaselinkToContactTransform(
         pitch_position, secondary_position, T_B_C))
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingNavigator] cannot lock: contact FK "
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
     !std::isfinite(baselink_quaternion.w()))
    return false;

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

  {
    std::lock_guard<std::mutex> lock(multilink_state_mutex_);
    locked_contact_world_ = T_W_C;
    locked_pitch_joint_ = pitch_position;
    locked_secondary_joint_ = secondary_position;
    pitch_joint_nominal_target_ = pitch_position;
    secondary_joint_nominal_target_ = secondary_position;
    pitch_admittance_offset_ = 0.0;
    pitch_joint_final_target_ = pitch_position;
    secondary_joint_final_target_ = secondary_position;
    multilink_lock_valid_ = true;
    secondary_settled_ = false;
    secondary_settle_start_ = ros::Time(0);
    ++mechanism_target_generation_;
  }

  const tf::Vector3 cog_pos = estimator_->getPos(Frame::COG, estimate_mode_);
  const tf::Vector3 cog_rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
  const tf::Vector3 contact_pos(T_W_C.p.x(), T_W_C.p.y(), T_W_C.p.z());
  commitPerchingLockDiagnostics(cog_pos, cog_rpy, contact_pos);

  ROS_WARN(
      "[GimbalrotorMultilinkPerchingNavigator] multilink contact locked "
      "by %s at q=[%.4f, %.4f].",
      reason.c_str(),
      pitch_position,
      secondary_position);
  return true;
}

void GimbalrotorMultilinkPerchingNavigator::resetPerchingLock()
{
  GimbalrotorPerchingNavigator::resetPerchingLock();

  std::lock_guard<std::mutex> lock(multilink_state_mutex_);
  multilink_lock_valid_ = false;
  locked_contact_world_ = KDL::Frame::Identity();
  locked_pitch_joint_ = 0.0;
  locked_secondary_joint_ = 0.0;
  pitch_joint_nominal_target_ = 0.0;
  secondary_joint_nominal_target_ = 0.0;
  pitch_admittance_offset_ = 0.0;
  pitch_joint_final_target_ = 0.0;
  secondary_joint_final_target_ = 0.0;
  secondary_settled_ = false;
  secondary_settle_start_ = ros::Time(0);
  ++mechanism_target_generation_;
}

bool GimbalrotorMultilinkPerchingNavigator::buildFinalJointTarget(
    double& pitch_final,
    double& secondary_final) const
{
  std::lock_guard<std::mutex> lock(multilink_state_mutex_);
  if(!multilink_model_valid_ || !multilink_lock_valid_)
    return false;

  pitch_final = clamp(
      pitch_joint_nominal_target_ + pitch_admittance_offset_,
      pitch_joint_lower_,
      pitch_joint_upper_);
  secondary_final = clamp(
      secondary_joint_nominal_target_,
      secondary_joint_lower_,
      secondary_joint_upper_);

  return std::isfinite(pitch_final) && std::isfinite(secondary_final);
}

void GimbalrotorMultilinkPerchingNavigator::applyActivePerchingTarget()
{
  if(!perchingSessionEnabled() || getNaviState() != HOVER_STATE ||
     !multilink_model_valid_)
    return;

  if(!multilinkLockValid() && !tryLockPerching("active multilink update"))
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
        "[GimbalrotorMultilinkPerchingNavigator] holding the last target: "
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
        "[GimbalrotorMultilinkPerchingNavigator] rejected invalid target FK.");
    return;
  }

  KDL::Frame locked_contact;
  {
    std::lock_guard<std::mutex> lock(multilink_state_mutex_);
    if(!multilink_lock_valid_)
      return;
    locked_contact = locked_contact_world_;
  }

  const KDL::Frame T_W_B_des = locked_contact * T_B_C_final.Inverse();
  const KDL::Vector p_W_G_des = T_W_B_des * p_B_G_final;
  if(!frameFinite(T_W_B_des) ||
     !std::isfinite(p_W_G_des.x()) ||
     !std::isfinite(p_W_G_des.y()) ||
     !std::isfinite(p_W_G_des.z()))
    return;

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
  tf::Quaternion R_W_G = R_W_B * R_C_B.inverse();
  R_W_G.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf::Matrix3x3(R_W_G).getRPY(roll, pitch, yaw);
  if(!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))
    return;

  setTargetPos(tf::Vector3(p_W_G_des.x(), p_W_G_des.y(), p_W_G_des.z()));
  setTargetVel(0.0, 0.0, 0.0);
  setTargetAcc(0.0, 0.0, 0.0);
  setTargetRPY(tf::Vector3(roll, pitch, yaw));
  setTargetOmega(0.0, 0.0, 0.0);
  setTargetAngAcc(0.0, 0.0, 0.0);

  {
    std::lock_guard<std::mutex> lock(multilink_state_mutex_);
    pitch_joint_final_target_ = pitch_final;
    secondary_joint_final_target_ = secondary_final;
  }

  publishJointTarget(pitch_final, secondary_final);

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
}

void GimbalrotorMultilinkPerchingNavigator::applyPerchingConstraint(
    aerial_robot_msgs::FlightNav& nav_msg)
{
  /*
   * While the multilink lock is active, external body commands cannot be
   * projected with the parent's planar arc.  The active update owns the
   * complete body target through fixed-contact FK.
   */
  nav_msg.pos_xy_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  nav_msg.pos_z_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  nav_msg.roll_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  nav_msg.pitch_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
  nav_msg.yaw_nav_mode = aerial_robot_msgs::FlightNav::NO_NAVIGATION;
}

void GimbalrotorMultilinkPerchingNavigator::applyManualPitchDelta(double delta)
{
  std::lock_guard<std::mutex> lock(multilink_state_mutex_);

  if(!multilink_model_valid_ ||
     !multilink_lock_valid_ ||
     !std::isfinite(delta))
    return;

  pitch_joint_nominal_target_ = clamp(locked_pitch_joint_ + pitch_command_sign_ * delta, pitch_joint_lower_, pitch_joint_upper_);
}

void GimbalrotorMultilinkPerchingNavigator::secondaryJointTargetCallback(
    const std_msgs::Float64ConstPtr& msg)
{
  if(getNaviState() != HOVER_STATE || !std::isfinite(msg->data))
    return;

  std::lock_guard<std::mutex> lock(multilink_state_mutex_);
  if(!multilink_model_valid_ || !multilink_lock_valid_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "[GimbalrotorMultilinkPerchingNavigator] secondary target ignored "
        "without a valid multilink perching lock.");
    return;
  }

  const double target = clamp(
      secondary_command_sign_ * msg->data,
      secondary_joint_lower_,
      secondary_joint_upper_);
  if(std::abs(target - secondary_joint_nominal_target_) <= 1.0e-9)
    return;

  secondary_joint_nominal_target_ = target;
  pitch_admittance_offset_ = 0.0;
  secondary_settled_ = false;
  secondary_settle_start_ = ros::Time(0);
  ++mechanism_target_generation_;
}

void GimbalrotorMultilinkPerchingNavigator::updateSecondarySettledState()
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

  std::lock_guard<std::mutex> lock(multilink_state_mutex_);
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

void GimbalrotorMultilinkPerchingNavigator::update()
{
  if(!multilink_model_valid_ &&
     robot_model_ &&
     robot_model_->initialized())
  {
    multilink_model_valid_ = resolveMechanismModel();

    std_msgs::Bool valid_msg;
    valid_msg.data = multilink_model_valid_;
    model_valid_pub_.publish(valid_msg);

    if(multilink_model_valid_)
    {
      ROS_WARN(
          "[GimbalrotorMultilinkPerchingNavigator] "
          "multilink model initialized after receiving robot state.");
    }
    else
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[GimbalrotorMultilinkPerchingNavigator] "
          "robot model is initialized, but the multilink mechanism "
          "configuration is invalid.");
    }
  }

  updateSecondarySettledState();
  GimbalrotorPerchingNavigator::update();
  publishDiagnostics();
}

void GimbalrotorMultilinkPerchingNavigator::publishJointTarget(
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

bool GimbalrotorMultilinkPerchingNavigator::getPitchJointKinematicsWorld(
    Eigen::Vector3d& joint_origin_world,
    Eigen::Vector3d& joint_axis_world) const
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

  KDL::JntArray joints = robot_model_->getJointPositions();
  joints(pitch_joint_index_) = pitch_position;
  joints(secondary_joint_index_) = secondary_position;

  const KDL::Frame T_R_B = robot_model_->forwardKinematics<KDL::Frame>(
      robot_model_->getBaselinkName(), joints);
  const KDL::Frame T_R_P = robot_model_->forwardKinematics<KDL::Frame>(
      pitch_joint_parent_link_name_, joints);
  const KDL::Frame T_R_J = robot_model_->forwardKinematics<KDL::Frame>(
      pitch_joint_child_link_name_, joints);

  const auto joint_segment_it =
      robot_model_->getTree().getSegment(pitch_joint_child_link_name_);
  if(joint_segment_it == robot_model_->getTree().getSegments().end() ||
     !frameFinite(T_R_B) || !frameFinite(T_R_P) || !frameFinite(T_R_J))
    return false;

  const KDL::Segment& pitch_segment =
      GetTreeElementSegment(joint_segment_it->second);
  const KDL::Vector axis_root =
      T_R_P.M * pitch_segment.getJoint().JointAxis();
  const KDL::Vector origin_baselink = T_R_B.Inverse() * T_R_J.p;
  const KDL::Vector axis_baselink = T_R_B.M.Inverse() * axis_root;

  const tf::Vector3 baselink_pos =
      estimator_->getPos(Frame::BASELINK, estimate_mode_);
  const tf::Matrix3x3 baselink_rot =
      estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
  const tf::Vector3 origin_world_tf = baselink_pos + baselink_rot *
      tf::Vector3(
          origin_baselink.x(), origin_baselink.y(), origin_baselink.z());
  const tf::Vector3 axis_world_tf = baselink_rot *
      tf::Vector3(axis_baselink.x(), axis_baselink.y(), axis_baselink.z());

  joint_origin_world = Eigen::Vector3d(
      origin_world_tf.x(), origin_world_tf.y(), origin_world_tf.z());
  joint_axis_world = Eigen::Vector3d(
      axis_world_tf.x(), axis_world_tf.y(), axis_world_tf.z());
  if(!joint_origin_world.allFinite() || !joint_axis_world.allFinite() ||
     joint_axis_world.norm() <= 1.0e-6)
    return false;

  joint_axis_world.normalize();
  return true;
}

void GimbalrotorMultilinkPerchingNavigator::setPitchAdmittanceOffset(
    double offset)
{
  std::lock_guard<std::mutex> lock(multilink_state_mutex_);
  if(!multilink_model_valid_ || !multilink_lock_valid_ ||
     !std::isfinite(offset))
    return;

  const double final_pitch = clamp(
      pitch_joint_nominal_target_ + offset,
      pitch_joint_lower_,
      pitch_joint_upper_);
  pitch_admittance_offset_ = final_pitch - pitch_joint_nominal_target_;
}

void GimbalrotorMultilinkPerchingNavigator::clearPitchAdmittanceOffset()
{
  std::lock_guard<std::mutex> lock(multilink_state_mutex_);
  pitch_admittance_offset_ = 0.0;
}

bool GimbalrotorMultilinkPerchingNavigator::multilinkModelValid() const
{
  std::lock_guard<std::mutex> lock(multilink_state_mutex_);
  return multilink_model_valid_;
}

bool GimbalrotorMultilinkPerchingNavigator::multilinkLockValid() const
{
  std::lock_guard<std::mutex> lock(multilink_state_mutex_);
  return multilink_model_valid_ && multilink_lock_valid_;
}

bool GimbalrotorMultilinkPerchingNavigator::secondaryJointSettled() const
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

  std::lock_guard<std::mutex> lock(multilink_state_mutex_);
  return multilink_model_valid_ && multilink_lock_valid_ &&
         secondary_settled_;
}

std::uint64_t
GimbalrotorMultilinkPerchingNavigator::mechanismTargetGeneration() const
{
  std::lock_guard<std::mutex> lock(multilink_state_mutex_);
  return mechanism_target_generation_;
}

void GimbalrotorMultilinkPerchingNavigator::publishDiagnostics() const
{
  bool settled = false;
  double pitch_measured = 0.0;
  double pitch_nominal = 0.0;
  double pitch_offset = 0.0;
  double pitch_final = 0.0;
  double secondary_measured = 0.0;
  double secondary_target = 0.0;

  {
    std::lock_guard<std::mutex> lock(multilink_state_mutex_);
    settled = secondary_settled_;
    pitch_measured = measured_pitch_joint_;
    pitch_nominal = pitch_joint_nominal_target_;
    pitch_offset = pitch_admittance_offset_;
    pitch_final = pitch_joint_final_target_;
    secondary_measured = measured_secondary_joint_;
    secondary_target = secondary_joint_nominal_target_;
  }

  std_msgs::Bool bool_msg;
  std_msgs::Float64 value_msg;
  bool_msg.data = settled;
  secondary_settled_pub_.publish(bool_msg);
  value_msg.data = pitch_measured;
  pitch_measured_pub_.publish(value_msg);
  value_msg.data = pitch_nominal;
  pitch_nominal_pub_.publish(value_msg);
  value_msg.data = pitch_offset;
  pitch_offset_pub_.publish(value_msg);
  value_msg.data = pitch_final;
  pitch_final_pub_.publish(value_msg);
  value_msg.data = secondary_measured;
  secondary_measured_pub_.publish(value_msg);
  value_msg.data = secondary_target;
  secondary_target_pub_.publish(value_msg);
}

bool GimbalrotorMultilinkPerchingNavigator::frameFinite(
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

double GimbalrotorMultilinkPerchingNavigator::clamp(
    double value,
    double lower,
    double upper)
{
  return std::max(lower, std::min(value, upper));
}

}  // namespace aerial_robot_navigation

PLUGINLIB_EXPORT_CLASS(
    aerial_robot_navigation::GimbalrotorMultilinkPerchingNavigator,
    aerial_robot_navigation::BaseNavigator)
