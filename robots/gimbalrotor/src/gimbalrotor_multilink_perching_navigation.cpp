// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_multilink_perching_navigation.h>

#include <aerial_robot_model/model/aerial_robot_model.h>

#include <pluginlib/class_list_macros.h>

#include <kdl/chain.hpp>

#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

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
    model_resolution_attempted_(false),
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
  resetPitchRezeroState();
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

  // Model topology is fixed; defer resolution only until the first model update.
  model_resolution_attempted_ = robot_model_ready;
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

  if(pitch_rezero_config_valid_)
    pitch_rezero_sub_ = nh_.subscribe(
        pitch_rezero_topic_, 1,
        &GimbalrotorMultilinkPerchingNavigator::pitchRezeroCallback, this);
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
        "[GimbalrotorMultilinkPerchingNavigator] "
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
      "[GimbalrotorMultilinkPerchingNavigator] multilink contact locked "
      "by %s at q=[%.4f, %.4f].",
      reason.c_str(),
      pitch_position,
      secondary_position);
  publishDiagnostics();
  return true;
}

void GimbalrotorMultilinkPerchingNavigator::resetPerchingLock()
{
  GimbalrotorPerchingNavigator::resetPerchingLock();

  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  resetPitchRezeroState();
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

bool GimbalrotorMultilinkPerchingNavigator::buildFinalJointTarget(
    double& pitch_final,
    double& secondary_final) const
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  if(!multilink_model_valid_ || !multilink_lock_valid_)
    return false;

  pitch_final = clamp(
      pitch_joint_nominal_target_,
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
  std::lock_guard<std::recursive_mutex> state_lock(perchingStateMutex());
  if(pitch_rezero_state_ != PitchRezeroState::IDLE)
  {
    applyPitchRezeroTarget();
    return;
  }
  if(perchingMode() != perching_geometry::Mode::NORMAL || getNaviState() != HOVER_STATE ||
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
    std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
    if(!multilink_lock_valid_)
      return;
    locked_contact = locked_contact_world_;
  }

  const KDL::Frame T_W_B_des = locked_contact * T_B_C_final.Inverse();
  if(!commandDesiredBaselinkPose(T_W_B_des, p_B_G_final))
    return;
  pitch_joint_final_target_ = pitch_final;
  secondary_joint_final_target_ = secondary_final;
  publishJointTarget(pitch_final, secondary_final);
}

bool GimbalrotorMultilinkPerchingNavigator::commandDesiredBaselinkPose(
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

bool GimbalrotorMultilinkPerchingNavigator::applyPerchingConstraint(
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

void GimbalrotorMultilinkPerchingNavigator::applyManualPitchDelta(double delta)
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  if(pitch_rezero_state_ != PitchRezeroState::IDLE)
  {
    ROS_WARN_THROTTLE(1.0, "[Multilink] Active joints frozen during pitch rezero/HOLD; relock after ready.");
    return;
  }

  if(!multilink_model_valid_ ||
     !multilink_lock_valid_ ||
     !std::isfinite(delta))
    return;

  const double target = clamp(locked_pitch_joint_ + pitch_command_sign_ * delta,
                              pitch_joint_lower_, pitch_joint_upper_);
  if(std::abs(target - pitch_joint_nominal_target_) <= 1.0e-9)
    return;
  pitch_joint_nominal_target_ = target;
  ++mechanism_target_generation_;
}

void GimbalrotorMultilinkPerchingNavigator::secondaryJointTargetCallback(
    const std_msgs::Float64ConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> state_lock(perchingStateMutex());
  if(pitch_rezero_state_ != PitchRezeroState::IDLE)
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

void GimbalrotorMultilinkPerchingNavigator::update()
{
  std::lock_guard<std::recursive_mutex> state_lock(perchingStateMutex());
  if(!model_resolution_attempted_ &&
     robot_model_ &&
     robot_model_->initialized())
  {
    model_resolution_attempted_ = true;
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

  if(pitch_rezero_state_ != PitchRezeroState::IDLE &&
     (!multilinkLockValid() || perchingMode() != perching_geometry::Mode::NORMAL ||
      getNaviState() != HOVER_STATE))
    resetPitchRezeroState();
  updateBodyPitchFeedback();
  rezero_target_applied_this_update_ = false;
  GimbalrotorPerchingNavigator::update();
  // Rezero owns a continuous hold even if the parent's optional fixed-contact
  // hold is disabled. Avoid integrating twice when the parent called our hook.
  if(pitch_rezero_state_ != PitchRezeroState::IDLE && !rezero_target_applied_this_update_)
    applyPitchRezeroTarget();
  updateSecondarySettledState();
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

bool GimbalrotorMultilinkPerchingNavigator::multilinkModelValid() const
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  return multilink_model_valid_;
}

bool GimbalrotorMultilinkPerchingNavigator::multilinkLockValid() const
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
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

  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  return multilink_model_valid_ && multilink_lock_valid_ &&
         secondary_settled_;
}

std::uint64_t
GimbalrotorMultilinkPerchingNavigator::mechanismTargetGeneration() const
{
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  return mechanism_target_generation_;
}

void GimbalrotorMultilinkPerchingNavigator::publishDiagnostics() const
{
  std::lock_guard<std::recursive_mutex> state_lock(perchingStateMutex());
  std_msgs::Bool rezero_bool;
  rezero_bool.data = pitch_rezero_state_ == PitchRezeroState::ACTIVE;
  pitch_rezero_active_pub_.publish(rezero_bool);
  rezero_bool.data = pitch_rezero_ready_;
  pitch_rezero_ready_pub_.publish(rezero_bool);
  rezero_bool.data = pitch_rezero_failed_;
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

void GimbalrotorMultilinkPerchingNavigator::resetPitchRezeroState()
{
  pitch_rezero_state_ = PitchRezeroState::IDLE;
  pitch_rezero_ready_ = false;
  pitch_rezero_failed_ = false;
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
  rezero_feedback_dt_ = 0.0;
  rezero_pitch_rate_valid_ = false;
  rezero_pivot_world_ = KDL::Vector::Zero();
  rezero_axis_world_ = KDL::Vector::Zero();
  rezero_p_B_G_ = KDL::Vector::Zero();
  rezero_reference_body_world_ = KDL::Frame::Identity();
  rezero_last_body_target_ = KDL::Frame::Identity();
}

bool GimbalrotorMultilinkPerchingNavigator::readBaselinkPose(KDL::Frame& pose) const
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

double GimbalrotorMultilinkPerchingNavigator::currentBaselinkPitch() const
{
  KDL::Frame pose;
  if(!readBaselinkPose(pose)) return std::numeric_limits<double>::quiet_NaN();
  double roll, pitch, yaw;
  pose.M.GetRPY(roll, pitch, yaw);
  return pitch;
}

double GimbalrotorMultilinkPerchingNavigator::wrapAngle(double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

void GimbalrotorMultilinkPerchingNavigator::pitchRezeroCallback(
    const std_msgs::EmptyConstPtr& msg)
{
  (void)msg;
  std::lock_guard<std::recursive_mutex> lock(perchingStateMutex());
  double pitch, secondary, pitch_velocity, secondary_velocity;
  if(perchingMode() != perching_geometry::Mode::NORMAL || getNaviState() != HOVER_STATE ||
     !multilinkLockValid() || !pitch_rezero_config_valid_ ||
     pitch_rezero_state_ != PitchRezeroState::IDLE ||
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
  if(!commandDesiredBaselinkPose(T_W_B, p_B_G))
  {
    ROS_WARN_THROTTLE(1.0, "[Multilink] Rezero rejected: cannot command initial body pose.");
    return;
  }
  resetPitchRezeroState();
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
  pitch_rezero_state_ = PitchRezeroState::ACTIVE;
  publishJointTarget(pitch, secondary);
  publishDiagnostics();
  ROS_WARN("[Multilink] Pitch rezero started about external passive pivot; both active joints frozen.");
}

void GimbalrotorMultilinkPerchingNavigator::updateBodyPitchFeedback()
{
  const ros::Time now = ros::Time::now();
  const double pitch = currentBaselinkPitch();
  rezero_pitch_rate_valid_ = false;
  rezero_feedback_dt_ = 0.0;
  if(!std::isfinite(pitch) || now.isZero())
  {
    rezero_previous_body_pitch_stamp_ = ros::Time(0);
    return;
  }
  if(!rezero_previous_body_pitch_stamp_.isZero())
  {
    rezero_feedback_dt_ = (now - rezero_previous_body_pitch_stamp_).toSec();
    // Long gaps cannot count as continuously observed stabilization.
    if(std::isfinite(rezero_feedback_dt_) && rezero_feedback_dt_ > 0.0 &&
       rezero_feedback_dt_ <= 0.1)
    {
      rezero_measured_pitch_rate_ =
          wrapAngle(pitch - rezero_previous_body_pitch_) / rezero_feedback_dt_;
      rezero_pitch_rate_valid_ = std::isfinite(rezero_measured_pitch_rate_);
    }
  }
  rezero_previous_body_pitch_ = pitch;
  rezero_previous_body_pitch_stamp_ = now;
}

void GimbalrotorMultilinkPerchingNavigator::failPitchRezero(const char* reason)
{
  pitch_rezero_state_ = PitchRezeroState::HOLD;
  pitch_rezero_ready_ = false;
  pitch_rezero_failed_ = true;
  rezero_stable_start_stamp_ = ros::Time(0);
  ROS_WARN("[Multilink] Pitch rezero failed (%s); holding last valid rezero pose and frozen joints. Disable perching and retry safely.", reason);
}

bool GimbalrotorMultilinkPerchingNavigator::verifyPitchRezeroFrozenJoints()
{
  if(pitch_rezero_state_ == PitchRezeroState::IDLE) return true;

  double q_pitch, q_secondary, v_pitch, v_secondary;
  if(!readCurrentMechanismState(q_pitch, q_secondary, v_pitch, v_secondary))
  {
    rezero_pitch_joint_hold_error_ = std::numeric_limits<double>::quiet_NaN();
    rezero_secondary_joint_hold_error_ = std::numeric_limits<double>::quiet_NaN();
    if(!pitch_rezero_failed_)
      failPitchRezero("active-joint state unavailable while joints must remain frozen");
    return false;
  }

  rezero_pitch_joint_hold_error_ = wrapAngle(q_pitch - rezero_frozen_pitch_joint_);
  rezero_secondary_joint_hold_error_ = wrapAngle(q_secondary - rezero_frozen_secondary_joint_);
  if(std::abs(rezero_pitch_joint_hold_error_) > pitch_rezero_pitch_joint_hold_tolerance_)
  {
    if(!pitch_rezero_failed_)
    {
      ROS_WARN("[Multilink] Pitch rezero active pitch joint hold violation: error=%.4f rad, tolerance=%.4f rad.",
               rezero_pitch_joint_hold_error_, pitch_rezero_pitch_joint_hold_tolerance_);
      failPitchRezero("active pitch joint moved outside rezero hold tolerance");
    }
    return false;
  }
  if(std::abs(rezero_secondary_joint_hold_error_) > pitch_rezero_secondary_joint_hold_tolerance_)
  {
    if(!pitch_rezero_failed_)
    {
      ROS_WARN("[Multilink] Pitch rezero active secondary joint hold violation: error=%.4f rad, tolerance=%.4f rad.",
               rezero_secondary_joint_hold_error_, pitch_rezero_secondary_joint_hold_tolerance_);
      failPitchRezero("active secondary joint moved outside rezero hold tolerance");
    }
    return false;
  }
  return true;
}

void GimbalrotorMultilinkPerchingNavigator::updatePitchRezeroCommand()
{
  if(pitch_rezero_state_ != PitchRezeroState::ACTIVE) return;
  const ros::Time now = ros::Time::now();
  const double elapsed = (now - rezero_start_stamp_).toSec();
  const double pitch = currentBaselinkPitch();
  double q_pitch, q_secondary, v_pitch, v_secondary;
  if(!std::isfinite(elapsed) || elapsed < 0.0 || rezero_feedback_dt_ < 0.0 ||
     rezero_feedback_dt_ > 0.5)
    return failPitchRezero("invalid or discontinuous time");
  if(elapsed > pitch_rezero_timeout_)
    return failPitchRezero("timeout before stabilization");
  if(!std::isfinite(pitch) ||
     !readCurrentMechanismState(q_pitch, q_secondary, v_pitch, v_secondary))
    return failPitchRezero("invalid body feedback or stale mechanism state");

  const double error = wrapAngle(pitch_rezero_target_pitch_ - pitch);
  if(rezero_pitch_rate_valid_ && std::abs(error) <= pitch_rezero_pitch_tolerance_ &&
     std::abs(rezero_measured_pitch_rate_) <= pitch_rezero_rate_tolerance_)
  {
    if(rezero_stable_start_stamp_.isZero()) rezero_stable_start_stamp_ = now;
    if((now - rezero_stable_start_stamp_).toSec() >= pitch_rezero_stable_duration_)
    {
      pitch_rezero_state_ = PitchRezeroState::HOLD;
      pitch_rezero_ready_ = true;
      pitch_rezero_failed_ = false;
      ROS_WARN("[Multilink] Pitch rezero ready; body pitch is stable. Publish /perching/relock to commit this pose as the new fixed-contact lock.");
      return;
    }
  }
  else
    rezero_stable_start_stamp_ = ros::Time(0);

  if(!std::isfinite(rezero_feedback_dt_) || rezero_feedback_dt_ <= 0.0) return;
  const double dt = std::min(rezero_feedback_dt_, 0.1);
  const double rate = pitch_rezero_command_sign_ *
      clamp(pitch_rezero_kp_ * error, -pitch_rezero_rate_limit_, pitch_rezero_rate_limit_);
  const double alpha = clamp(rezero_alpha_cmd_ + rate * dt,
                             -pitch_rezero_max_delta_, pitch_rezero_max_delta_);
  const KDL::Rotation rotation = KDL::Rotation::Rot(rezero_axis_world_, alpha);
  const KDL::Frame target(
      rotation * rezero_reference_body_world_.M,
      rezero_pivot_world_ + rotation * (rezero_reference_body_world_.p - rezero_pivot_world_));
  if(!std::isfinite(alpha) || !frameFinite(target) ||
     !commandDesiredBaselinkPose(target, rezero_p_B_G_))
    return failPitchRezero("invalid revolute body target");
  // Commit only a successfully issued target; failures retain the previous pose.
  rezero_alpha_cmd_ = alpha;
  rezero_last_body_target_ = target;
}

void GimbalrotorMultilinkPerchingNavigator::applyPitchRezeroTarget()
{
  rezero_target_applied_this_update_ = true;
  // Monitor ACTIVE and both HOLD outcomes before advancing the passive pivot.
  const bool frozen_joints_valid = verifyPitchRezeroFrozenJoints();
  if(pitch_rezero_state_ == PitchRezeroState::ACTIVE && frozen_joints_valid)
    updatePitchRezeroCommand();
  pitch_joint_nominal_target_ = pitch_joint_final_target_ = rezero_frozen_pitch_joint_;
  secondary_joint_nominal_target_ = secondary_joint_final_target_ = rezero_frozen_secondary_joint_;
  publishJointTarget(rezero_frozen_pitch_joint_, rezero_frozen_secondary_joint_);
  if(!commandDesiredBaselinkPose(rezero_last_body_target_, rezero_p_B_G_) &&
     !pitch_rezero_failed_)
    failPitchRezero("cannot reissue last valid body pose");
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
