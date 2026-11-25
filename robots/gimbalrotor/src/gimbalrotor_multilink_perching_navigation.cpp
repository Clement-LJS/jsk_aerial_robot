#include <gimbalrotor/gimbalrotor_multilink_perching_navigation.h>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <eigen_conversions/eigen_kdl.h>
#include <limits>
#include <stdexcept>
#include <urdf/model.h>
#include <tf_conversions/tf_eigen.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

namespace
{
tf::Vector3 tfVector(const Eigen::Vector3d& v) { return {v.x(), v.y(), v.z()}; }
geometry_msgs::Pose poseMessage(const Eigen::Isometry3d& p)
{
  geometry_msgs::Pose result;
  tf::pointTFToMsg(tfVector(p.translation()), result.position);
  tf::quaternionTFToMsg(perching_geometry::tfQuaternion(p.linear()), result.orientation);
  return result;
}
}

void GimbalrotorMultilinkPerchingNavigator::relockCallback(const std_msgs::EmptyConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(positioning_ && perching_locked_)
  {
    workflowCommand("relock");
    return;
  }
  GimbalrotorPerchingNavigator::relockCallback(msg);
}

void GimbalrotorMultilinkPerchingNavigator::applyManualPitchDelta(double delta)
{
  if(navi_state_ != HOVER_STATE || !perching_locked_)
    return;
  if(positioning_) servo_.cut(delta, servoSample());
  else
  {
    const auto it = coordinate_map_.find(manual_coordinate_);
    if(it != coordinate_map_.end() && coordinateCommand(it->second, locked_q_[it->second] + command_pitch_sign_ * delta)) motion_.revoke();
  }
}

void GimbalrotorMultilinkPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(perching_mode_ != perching_geometry::Mode::DISABLED)
  {
    if(accept_uav_nav_pitch_command_ && hasPitchCommand(*msg)) applyManualPitchDelta(msg->target_pitch);
    return;
  }
  GimbalrotorPerchingNavigator::naviCallback(msg);
}

bool GimbalrotorMultilinkPerchingNavigator::perchingAdmittanceTarget(
    const ros::Time&, double, const tf::Vector3&, perching_geometry::Pose&) const
{
  // A body-only preview cannot commit the coordinated joint reference. The
  // controller uses applyPerchingReference(); no active caller uses this API.
  return false;
}

void GimbalrotorMultilinkPerchingNavigator::readProfile()
{
  GimbalrotorPerchingNavigator::readProfile();
  ros::NodeHandle pn(nh_, "navigation/perching");
  if(!pn.getParam("active_joints", active_joints_) || active_joints_.empty())
    throw std::invalid_argument(pn.resolveName("active_joints") + " must be a nonempty string list");
  for(auto it = active_joints_.begin(); it != active_joints_.end(); ++it)
    if(it->empty() || *it == "alpha" || std::find(active_joints_.begin(), it, *it) != it)
      throw std::invalid_argument(pn.resolveName("active_joints") + ": empty, reserved or duplicate joint name '" + *it + "'");
  perching_geometry::coordinateMap(active_joints_, coordinate_map_);
  pn.param<std::string>("joint_state_topic", joint_topic_, "joint_states");
  std::string effort_source;
  pn.param<std::string>("dynamics/joint_effort/source", effort_source, "");
  if(effort_source == "servo_bridge_joint_states") effort_producer_ = nh_.resolveName("servo_bridge");
  pn.param<std::string>("manual_coordinate", manual_coordinate_, "alpha");
  pn.param<std::string>("positioning_joint", positioning_joint_, "");
  positioning_ = !positioning_joint_.empty();
  pn.param("body_tracking_tolerance", body_tracking_tolerance_, body_tracking_tolerance_);
  pn.param("feedback_sync_tolerance", sync_tolerance_, sync_tolerance_);
  pn.param("body_max_tilt", max_tilt_, max_tilt_);
  pn.param("body_pitch_lower_limit", min_body_pitch_, min_body_pitch_);
  pn.param("body_pitch_upper_limit", max_body_pitch_, max_body_pitch_);
  pn.param("cog_speed_limit", cog_speed_limit_, cog_speed_limit_);
  pn.param("cog_acceleration_limit", cog_acceleration_limit_, cog_acceleration_limit_);
  pn.param("pitch_acceleration", acceleration_limit_, acceleration_limit_);
  pn.param("body_pitch_jog_max_delta", jog_limit_, jog_limit_);
  nh_.param("controller/admittance/perching/recovery_angle_epsilon", jog_tolerance_, jog_tolerance_);
  nh_.param("controller/admittance/perching/cutting_measurement_time", cutting_measurement_time_, cutting_measurement_time_);
  nh_.param("controller/admittance/perching/cutting_hold_duration", cutting_hold_duration_, cutting_hold_duration_);
  pn.param("pitch_rezero_target_pitch", rezero_target_, rezero_target_);
  pn.param("pitch_rezero_command_sign", rezero_sign_, rezero_sign_);
  pn.param("pitch_rezero_rate_limit", rezero_rate_, rezero_rate_);
  pn.param("pitch_rezero_max_delta", rezero_range_, rezero_range_);
  pn.param("pitch_rezero_pitch_tolerance", rezero_tolerance_, rezero_tolerance_);
  pn.param("pitch_rezero_timeout", rezero_timeout_, rezero_timeout_);
  pn.param("pitch_rezero_kp", rezero_kp_, rezero_kp_);
  pn.param("pitch_rezero_stable_duration", rezero_stable_duration_, rezero_stable_duration_);
  if(pn.hasParam("wire_angles"))
  {
    const std::string parameter = pn.resolveName("wire_angles");
    if(!pn.getParam("wire_angles", wire_angles_) || (!wire_angles_.empty() && wire_angles_.size() != 3))
      throw std::invalid_argument(parameter + " must be an empty string list or exactly three coordinate names");
    for(auto it = wire_angles_.begin(); it != wire_angles_.end(); ++it)
      if(!coordinate_map_.count(*it) || std::find(wire_angles_.begin(), it, *it) != it)
        throw std::invalid_argument(parameter + ": unknown or duplicate coordinate '" + *it + "'");
  }
  XmlRpc::XmlRpcValue mapping;
  if(pn.hasParam("wire_coordinates"))
  {
    const std::string parameter = pn.resolveName("wire_coordinates");
    if(!pn.getParam("wire_coordinates", mapping) || mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      throw std::invalid_argument(parameter + " must be a dictionary");
    for(auto it = mapping.begin(); it != mapping.end(); ++it)
    {
      const std::string key = parameter + "/" + it->first;
      if(it->second.getType() != XmlRpc::XmlRpcValue::TypeString)
        throw std::invalid_argument(key + " must name a coordinate as a string");
      std::size_t parsed = 0;
      int mode;
      try
      {
        mode = std::stoi(it->first, &parsed);
      }
      catch(const std::invalid_argument&)
      {
        throw std::invalid_argument(key + ": mode must be an integer");
      }
      catch(const std::out_of_range&)
      {
        throw std::invalid_argument(key + ": mode is out of integer range");
      }
      using Command = gimbalrotor::RollYawMotion;
      if(parsed != it->first.size() || (mode != Command::ALPHA && mode != Command::ROLL && mode != Command::YAW))
        throw std::invalid_argument(key + ": mode must be ALPHA, ROLL or YAW");
      const std::string name = static_cast<std::string>(it->second);
      if(!coordinate_map_.count(name)) throw std::invalid_argument(key + ": unknown coordinate " + name);
      if(!wire_coordinates_.emplace(mode, name).second)
        throw std::invalid_argument(key + ": duplicate numeric mode");
    }
  }
  const double positive[] = {sync_tolerance_, max_tilt_, cog_speed_limit_, cog_acceleration_limit_,
      acceleration_limit_, jog_tolerance_, rezero_rate_, rezero_timeout_, rezero_kp_, rezero_stable_duration_,
      cutting_measurement_time_, cutting_hold_duration_};
  for(double value : positive)
    if(!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid navigation/perching limits or timing");
}

void GimbalrotorMultilinkPerchingNavigator::initializeMechanism()
{
  GimbalrotorPerchingNavigator::initializeMechanism();
  ros::NodeHandle pn(nh_, "navigation/perching");
  std::string body_topic, command_topic;
  pn.param<std::string>("joint_command_topic", command_topic, "joints_ctrl");
  joints_pub_ = nh_.advertise<sensor_msgs::JointState>(command_topic, 1);
  pn.param<std::string>("body_state_topic", body_topic, "uav/baselink/odom");
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
  subscriptions_.push_back(nh_.subscribe(joint_topic_, 1, &GimbalrotorMultilinkPerchingNavigator::jointCallback, this));
  subscriptions_.push_back(nh_.subscribe(body_topic, 1, &GimbalrotorMultilinkPerchingNavigator::bodyCallback, this));
  subscriptions_.push_back(nh_.subscribe(prefix_ + "/motion", 1, &GimbalrotorMultilinkPerchingNavigator::motionCallback, this));
  // Legacy names are data at the ROS boundary; algorithms only see coordinates.
  XmlRpc::XmlRpcValue topics;
  if(pn.hasParam("coordinate_topics"))
  {
    const std::string parameter = pn.resolveName("coordinate_topics");
    if(!pn.getParam("coordinate_topics", topics) || topics.getType() != XmlRpc::XmlRpcValue::TypeStruct)
      throw std::invalid_argument(parameter + " must be a dictionary");
    auto subscribe_coordinate = [&](auto&& self, const XmlRpc::XmlRpcValue& value, const std::string& topic) -> void
    {
      if(value.getType() == XmlRpc::XmlRpcValue::TypeStruct)
      {
        for(auto it = value.begin(); it != value.end(); ++it)
          self(self, it->second, topic.empty() ? it->first : topic + "/" + it->first);
        return;
      }
      if(topic.empty() || value.getType() != XmlRpc::XmlRpcValue::TypeString)
        throw std::invalid_argument(parameter + "/" + topic + " must name a coordinate as a string");
      const std::string name = static_cast<std::string>(value);
      const auto coordinate = coordinate_map_.find(name);
      if(coordinate == coordinate_map_.end())
        throw std::invalid_argument(parameter + "/" + topic + ": unknown coordinate " + name);
      const int index = coordinate->second;
      subscriptions_.push_back(nh_.subscribe<std_msgs::Float64>(topic, 1, [this, index](const std_msgs::Float64ConstPtr& m)
      { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_); if(coordinateCommand(index, m->data)) motion_.revoke(); }));
    };
    subscribe_coordinate(subscribe_coordinate, topics, std::string());
  }
  for(const std::string action : {"confirm_detached_zero", "pitch_rezero", "freeze", "unfreeze", "prepare_unperch", "confirm_released", "recover"})
    subscriptions_.push_back(nh_.subscribe<std_msgs::Empty>(prefix_ + "/" + action, 1,
      [this, action](const std_msgs::EmptyConstPtr&) { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_); workflowCommand(action); }));
  subscriptions_.push_back(nh_.subscribe<std_msgs::Float64>(prefix_ + "/servo_command", 1,
    [this](const std_msgs::Float64ConstPtr& m)
    { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_); if(positioning_) servo_.servo(m->data, servoSample()); }));
  subscriptions_.push_back(nh_.subscribe<std_msgs::Float64>(prefix_ + "/body_pitch_delta", 1,
    [this](const std_msgs::Float64ConstPtr& m)
    {
      std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
      if(navi_state_ != HOVER_STATE || !std::isfinite(m->data)) return;
      const std::uint64_t frame = body_target_valid_ ? body_frame_id_ : body_frame_id_ + 1;
      if(bodyPitchCommand(m->data, rate_limit_, frame)) motion_.revoke();
    }));
  subscriptions_.push_back(nh_.subscribe<std_msgs::Float64>(prefix_ + "/body_pitch_velocity", 1,
    [this](const std_msgs::Float64ConstPtr& m)
    { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
      if(!std::isfinite(m->data)) return;
      if(m->data == 0) stopMotion();
      else if(perching_locked_ && !rezero_active_ && !positioning_)
      {
        motion_.revoke();
        feed_request_.reset();
        velocity_active_ = true; body_target_valid_ = false;
        velocity_command_ = std::clamp(m->data, -rate_limit_, rate_limit_);
        velocity_expires_ = ros::Time::now().toSec() + velocity_duration_;
      }
    }));
  for(const std::string action : {"cancel", "stop", "pause", "resume", "prepare", "start"})
    subscriptions_.push_back(nh_.subscribe<std_msgs::UInt64>(prefix_ + "/" + action, 1,
      [this, action](const std_msgs::UInt64ConstPtr& m)
      { std::lock_guard<std::recursive_mutex> l(perching_state_mutex_); planCommand(action, m->data); }));
  subscriptions_.push_back(nh_.subscribe<geometry_msgs::Vector3Stamped>(prefix_ + "/target", 1,
    [this](const geometry_msgs::Vector3StampedConstPtr& m)
    {
      std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
      if(!perching_locked_ || m->header.frame_id != "perching_lock_" + std::to_string(session_)) return;
      if(wire_angles_.empty() || rezero_active_) return;

      const double values[] = {m->vector.x, m->vector.y, m->vector.z};
      Vector candidate = destination_;
      for(std::size_t i = 0; i < wire_angles_.size() && i < 3; ++i)
      {
        const int index = coordinate_map_.at(wire_angles_[i]);
        if(!std::isfinite(values[i]) || values[i] < model_->lower[index] ||
           values[i] > model_->upper[index]) return;
        candidate[index] = values[i];
      }
      if(!model_->allowed(candidate, max_tilt_, min_body_pitch_, max_body_pitch_)) return;

      motion_.revoke();
      destination_ = candidate;
      feed_request_.reset();
      body_rate_limit_ = INFINITY;
      body_target_valid_ = false;
      velocity_active_ = false;
      stopping_ = false;
      rezero_ready_ = false;
      phase_ = "EXECUTING";
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

bool GimbalrotorMultilinkPerchingNavigator::resolveMechanismModel()
{
  if(!GimbalrotorPerchingNavigator::resolveMechanismModel()) return false;
  ros::NodeHandle pn(nh_, "navigation/perching");
  for(const auto& key : {"position_lower", "position_upper", "velocity_caps", "acceleration_caps"})
  {
    std::vector<double> limits;
    if(!pn.hasParam(key)) continue;
    if(!pn.getParam(key, limits) || limits.size() != active_joints_.size() + 1) return false;
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
  std::vector<std::string> moving;
  pn.getParam("pitch_coordinates", moving);
  if(!perching_geometry::pitchCoordinates(active_joints_.size() + 1, coordinate_map_, moving, manual_coordinate_, moving_))
    return false;
  std::string prefix; nhp_.param<std::string>("tf_prefix", prefix, "");
  std::string body_frame;
  pn.param<std::string>("body_frame", body_frame, tf::resolve(prefix, model_->getBaselinkName()));
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    body_frame_ = body_frame;
  }
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

void GimbalrotorMultilinkPerchingNavigator::jointCallback(const ros::MessageEvent<sensor_msgs::JointState const>& event)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  const auto& msg = *event.getMessage();
  Sample sample;
  sample.stamp = msg.header.stamp.toSec(); sample.receipt = steadyTime();
  if(!fresh(sample.stamp, sample.receipt)) return;
  if(!joint_history_.empty() && sample.stamp <= joint_history_.back().stamp) return;
  const int n = active_joints_.size() + 1;
  sample.q = sample.rate = Vector::Zero(n);
  sample.effort = Vector::Zero(n); // Coordinate zero is the unactuated hinge.
  sample.effort_valid = effort_producer_.empty() || event.getPublisherName() == effort_producer_;
  for(int i = 1; i < n; ++i)
  {
    const auto it = std::find(msg.name.begin(), msg.name.end(), active_joints_[i - 1]);
    if(it == msg.name.end() || std::count(msg.name.begin(), msg.name.end(), *it) != 1) return;
    const std::size_t index = std::distance(msg.name.begin(), it);
    if(index >= msg.position.size() || !std::isfinite(msg.position[index])) return;
    sample.q[i] = msg.position[index];
    if(index < msg.effort.size() && std::isfinite(msg.effort[index])) sample.effort[i] = msg.effort[index];
    else sample.effort_valid = false;
    if(index < msg.velocity.size() && std::isfinite(msg.velocity[index])) sample.rate[i] = msg.velocity[index];
    else if(!joint_history_.empty() && sample.stamp - joint_history_.back().stamp <= max_gap_)
      sample.rate[i] = (sample.q[i] - joint_history_.back().q[i]) / (sample.stamp - joint_history_.back().stamp);
    else sample.rate[i] = std::numeric_limits<double>::quiet_NaN();
  }
  joint_history_.push_back(sample);
  while(joint_history_.size() > 64) joint_history_.pop_front();
}

void GimbalrotorMultilinkPerchingNavigator::bodyCallback(const nav_msgs::OdometryConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
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

bool GimbalrotorMultilinkPerchingNavigator::readSynchronizedSample(Sample& sample) const
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if(body_history_.empty() || joint_history_.empty()) return false;
  const auto& body = body_history_.back(); const auto& joint = joint_history_.back();
  if(!fresh(body.stamp, body.receipt) || !fresh(joint.stamp, joint.receipt)) return false;
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
        result.effort_valid = a.effort_valid && b.effort_valid;
        result.effort = (1 - u) * a.effort + u * b.effort;
      }
      return true;
    }
    return false;
  };
  Sample b, j;
  if(!bracket(body_history_, b, true) || !bracket(joint_history_, j, false)) return false;
  sample = j; sample.body = b.body; sample.omega = b.omega; sample.receipt = std::min(b.receipt, j.receipt);
  return fresh(sample.stamp, sample.receipt);
}

bool GimbalrotorMultilinkPerchingNavigator::observation(Sample& sample) const
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  // All consumers in this navigation cycle use the same synchronized input.
  if(last_update_ > 0 && measurement_cycle_ == last_update_ && measured_.stamp > 0 && fresh(measured_.stamp, measured_.receipt))
  { sample = measured_; return true; }
  Sample measured;
  if(!readSynchronizedSample(measured)) return false;
  if(perching_locked_)
  {
    double pivot, hinge;
    if(!model_->estimate(measured.body, measured.q, pivot, hinge)) return false;
    Eigen::MatrixXd angular;
    if(!model_->jacobian(measured.q, angular)) return false;
    measured.rate[0] = angular.col(0).dot(measured.omega - angular.rightCols(measured.q.size() - 1) * measured.rate.tail(measured.q.size() - 1));
  }
  measured_ = measured;
  measurement_cycle_ = last_update_;
  sample = measured;
  return true;
}

bool GimbalrotorMultilinkPerchingNavigator::motionFeedbackValid() const
{
  if(!perching_locked_ || navi_state_ != HOVER_STATE || force_landing_flag_) return false;
  Sample sample;
  if(!model_valid_ || !observation(sample) || !sample.q.allFinite() || !sample.rate.allFinite()) return false;
  const double pitch = std::asin(std::clamp(-sample.body.linear()(2, 0), -1.0, 1.0));
  return sample.body.linear()(2, 2) >= std::cos(max_tilt_) && pitch >= min_body_pitch_ && pitch <= max_body_pitch_ &&
      model_->allowed(sample.q, max_tilt_, min_body_pitch_, max_body_pitch_);
}

bool GimbalrotorMultilinkPerchingNavigator::tryLockPerching(const std::string& reason, bool relock)
{
  Sample measured;
  // A replacement contact must not depend on reconstruction against the old pivot.
  if(navi_state_ != HOVER_STATE || force_landing_flag_ || !model_valid_ ||
     !(relock && perching_locked_ ? readSynchronizedSample(measured) : observation(measured)) ||
     !measured.q.allFinite() || !measured.rate.allFinite()) return false;
  const double body_pitch = std::asin(std::clamp(-measured.body.linear()(2, 0), -1.0, 1.0));
  if(measured.body.linear()(2, 2) < std::cos(max_tilt_) || body_pitch < min_body_pitch_ || body_pitch > max_body_pitch_) return false;
  const Vector previous_lower = model_->lower, previous_upper = model_->upper;
  const Vector previous_velocity = model_->velocity, previous_acceleration = model_->acceleration;
  const auto previous_moving = moving_;
  const auto previous_body_frame = body_frame_;
  const auto previous_servo = servo_;
  auto candidate_servo = servo_;
  Reference initial;
  const bool accepted = [&]()
  {
    if(!resolveMechanismModel()) return false; // Snapshot the remaining joints for candidate FK.
    measured.q[0] = 0;
    if(!lockGeometry(measured.body, measured.q, initial)) return false;
    if(!model_->allowed(measured.q, max_tilt_, min_body_pitch_, max_body_pitch_)) return false;
    if(positioning_)
    {
      candidate_servo = servo_;
      // The existing servo FK adapter reads nominal coordinates under the same mutex.
      const Vector previous_nominal = nominal_;
      nominal_ = measured.q;
      const bool enabled = candidate_servo.enable(servoSample(&measured));
      nominal_ = previous_nominal;
      if(!enabled) return false;
    }
    return true;
  }();
  if(!accepted)
  {
    // The relock caller restores the shared geometry caches and pivot.
    model_->lower = previous_lower;
    model_->upper = previous_upper;
    model_->velocity = previous_velocity;
    model_->acceleration = previous_acceleration;
    moving_ = previous_moving;
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      body_frame_ = previous_body_frame;
    }
    servo_ = previous_servo;
    return false;
  }
  if(perching_locked_) resetMotion();
  measured_ = measured;
  measurement_cycle_ = -1;
  nominal_ = destination_ = proposed_ = applied_ = locked_q_ = measured_.q;
  nominal_rate_ = proposed_rate_ = applied_rate_ = applied_acceleration_ = Vector::Zero(nominal_.size());
  applied_state_ = initial;
  if(positioning_) servo_ = candidate_servo;
  commitFixedContactLock(tfVector(applied_state_.cog.translation()), perching_geometry::tfQuaternion(applied_state_.cog.linear()),
                         tfVector(pivot_.translation()));
  session_ = lock_stamp_.toNSec();
  applied_stamp_ = 0; last_update_ = ros::Time::now().toSec();
  phase_ = "CONTACT_LOCKED"; status_ = "locked; nominal hold";
  publishDiagnostics();
  return true;
}

void GimbalrotorMultilinkPerchingNavigator::resetMotion()
{
  GimbalrotorPerchingNavigator::resetMotion();
  motion_.revoke(); session_ = 0; measured_ = Sample();
  body_target_valid_ = stopping_ = proposal_valid_ = false;
  rezero_active_ = rezero_ready_ = rezero_failed_ = false;
  body_destination_ = body_progress_ = body_rate_ = 0;
  body_rate_limit_ = INFINITY;
  forward_rate_limit_ = recovery_rate_limit_ = INFINITY;
  feed_request_.reset();
  cutting_state_.reset();
  reference_velocity_.setZero(); reference_acceleration_.setZero();
  reference_omega_.setZero(); reference_angular_acceleration_.setZero();
  servo_.reset(); phase_ = "FREE_FLIGHT"; prepared_motion_ = paused_ = false;
}

bool GimbalrotorMultilinkPerchingNavigator::supportsPerchingMode(perching_geometry::Mode mode) const
{
  return mode == perching_geometry::Mode::NORMAL;
}

bool GimbalrotorMultilinkPerchingNavigator::nominalHeld() const
{
  return perching_locked_ && !rezero_active_ && nominal_rate_.norm() < 1e-6 && std::abs(body_rate_) < 1e-6 &&
      !velocity_active_ && !stopping_ && (destination_ - nominal_).norm() < 1e-5 &&
      (!body_target_valid_ || std::abs(body_destination_ - body_progress_) < 1e-4);
}

bool GimbalrotorMultilinkPerchingNavigator::trackingHeld() const
{
  Sample sample;
  return observation(sample) && applied_.size() == sample.q.size() &&
      (sample.q - applied_).cwiseAbs().maxCoeff() <= tracking_tolerance_ &&
      perching_geometry::logSO3(sample.body.linear().transpose() * applied_state_.body.linear()).norm() <= body_tracking_tolerance_;
}

std::optional<double> GimbalrotorMultilinkPerchingNavigator::pitchFeedStep() const
{
  if(!feed_request_) return std::nullopt;
  // A cancelled feed retains zero forward permission; unrelated navigation has no feed.
  if(feed_request_->session != session_ || feed_request_->epoch != motion_.epoch ||
      feed_request_->source != motion_.source || feed_request_->sequence != motion_.sequence) return 0;
  return feed_request_->value;
}

bool GimbalrotorMultilinkPerchingNavigator::jogReady() const
{
  const double step = pitchFeedStep().value_or(0);
  if(cutting_state_ && step > 0 && (cutting_state_->phase != CuttingState::FEED || cutting_state_->lead > .5 * step)) return false;
  return nominalHeld() && trackingHeld() && (step <= 0 || !std::isfinite(recovery_rate_limit_) ||
      (recovery_rate_limit_ > 0 && std::abs(applied_pitch_offset_) <= std::min(jog_tolerance_, .2 * step)));
}

void GimbalrotorMultilinkPerchingNavigator::stopMotion()
{
  motion_.revoke();
  cutting_state_.reset();
  // Brake from the accepted reference and retire pending servo destinations.
  destination_ = nominal_; body_destination_ = body_progress_;
  rezero_active_ = false;
  if(positioning_)
  {
    if(servo_.stage == perching_servo::Stage::PITCH_REZERO_ACTIVE)
      servo_.stage = perching_servo::Stage::CONTACT_LOCKED_PRE_REZERO;
    servo_.q_goal = servo_.q_target;
    servo_.alpha_goal = servo_.alpha;
  }
  velocity_active_ = false; stopping_ = true;
  status_ = "braking nominal motion";
}

void GimbalrotorMultilinkPerchingNavigator::holdAppliedReference()
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(!perching_locked_ || applied_stamp_ == 0) return;
  // Change the nominal/compliance split without moving the accepted body or joint reference.
  nominal_ = applied_;
  nominal_rate_ = applied_rate_;
  applied_pitch_offset_ = applied_offset_rate_ = 0;
  body_target_valid_ = prepared_motion_ = paused_ = rezero_ready_ = false;
  body_rate_ = 0;
  body_rate_limit_ = rate_limit_;
  if(positioning_)
  {
    servo_.q_target = nominal_[coordinate_map_.at(positioning_joint_)];
    servo_.target = perching_geometry::toKdl(applied_state_.body);
    perching_servo::Sample reference;
    reference.body = servo_.target;
    servo_.alpha = servo_.measuredAlpha(reference);
  }
  stopMotion();
  phase_ = "CONTACT_LOCKED";
}

bool GimbalrotorMultilinkPerchingNavigator::coordinateCommand(int index, double value)
{
  if(!perching_locked_ || !std::isfinite(value) || index < 0 || index >= nominal_.size() ||
      value < model_->lower[index] || value > model_->upper[index] || rezero_active_) return false;
  Vector candidate = destination_;
  candidate[index] = value;
  if(!model_->allowed(candidate, max_tilt_, min_body_pitch_, max_body_pitch_)) return false;
  body_target_valid_ = velocity_active_ = false;
  body_rate_limit_ = INFINITY;
  destination_[index] = value;
  feed_request_.reset();
  stopping_ = false;
  rezero_ready_ = false;
  phase_ = "EXECUTING";
  return true;
}

bool GimbalrotorMultilinkPerchingNavigator::bodyPitchCommand(double angle, double rate, std::uint64_t frame)
{
  if(positioning_ || !perching_locked_ || !std::isfinite(angle) || !std::isfinite(rate) || rate <= 0 || rezero_active_) return false;
  const bool new_frame = !body_target_valid_ || frame != body_frame_id_;
  const Vector start = new_frame ? nominal_ : body_start_;
  Reference origin;
  Vector endpoint;
  if(std::abs(angle) > M_PI || !model_->evaluate(start, origin) ||
      !model_->orient(start, origin.body.linear() * perching_geometry::expSO3(Eigen::Vector3d::UnitY() * angle), moving_, endpoint) ||
      !model_->allowed(endpoint, max_tilt_, min_body_pitch_, max_body_pitch_))
  {
    status_ = "requested body pitch is infeasible";
    return false;
  }
  if(new_frame)
  {
    // Capture the nominal body frame. Loaded applied motion must not change the
    // origin or positive direction of later cumulative destinations/reversals.
    body_start_ = nominal_;
    body_progress_ = 0;
    body_frame_id_ = frame;
  }
  rezero_ready_ = false;
  body_destination_ = angle;
  body_target_valid_ = true;
  velocity_active_ = false;
  stopping_ = false;
  destination_ = nominal_;
  body_rate_limit_ = std::min(rate, rate_limit_);
  feed_request_.reset();
  phase_ = "EXECUTING";
  return true;
}

void GimbalrotorMultilinkPerchingNavigator::motionCallback(const gimbalrotor::RollYawMotionConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  using Command = gimbalrotor::RollYawMotion;
  const auto& m = *msg;
  if(last_motion_request_ && m.session == last_motion_request_->session && m.epoch == last_motion_request_->epoch &&
      m.source == last_motion_request_->source && m.sequence == last_motion_request_->sequence)
  { ack_pub_.publish(last_motion_ack_); return; }
  std::string why = motion_.rejection(m.epoch, m.source, m.sequence);
  if(!perching_locked_ || m.session != session_) why = "invalid contact session";
  if(force_landing_flag_) why = "forced landing is active";
  if(!std::isfinite(m.value)) why = "nonfinite command";
  if(why.empty()) motion_.processed[m.source] = m.sequence;
  if(why.empty() && m.mode == Command::STOP && motion_.source != m.source) why = "request is no longer current";
  if(why.empty() && m.mode == Command::CLAIM && !nominalHeld()) why = "nominal motion is not held";
  if(why.empty() && rezero_active_ && m.value != 0 && (m.mode == Command::VELOCITY || m.mode == Command::PULSE))
    why = "pitch rezero is active";
  if(why.empty() && positioning_ && (m.mode == Command::BODY_PITCH || m.mode == Command::JOG ||
      (m.value != 0 && (m.mode == Command::VELOCITY || m.mode == Command::PULSE))))
    why = "body pitch commands are unavailable for positioning profiles";
  if(why.empty())
  {
    if(m.mode == Command::STOP) stopMotion();
    else if(m.mode == Command::CLAIM) { motion_.revoke(); motion_.accept(m.source, m.sequence); body_target_valid_ = false; prepared_motion_ = false; }
    else if(m.mode == Command::RELEASE)
    { if(motion_.source == m.source) stopMotion(); }
    else
    {
      if(m.mode == Command::BODY_PITCH)
      {
        const bool previous_target = body_target_valid_;
        if(motion_.source != m.source) body_target_valid_ = false;
        if(!bodyPitchCommand(m.value, m.rate_limit, m.frame))
        { body_target_valid_ = previous_target; why = "infeasible body pitch target"; }
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
        if(!(m.value < 0 ? nominalHeld() && trackingHeld() : jogReady()) || std::abs(m.value) > jog_limit_)
          why = "jog requires a settled reference within configured bound";
        else
        {
          const double rate = m.rate_limit == 0 ? rate_limit_ : m.rate_limit;
          Vector goal;
          if(!std::isfinite(rate) || rate <= 0) why = "infeasible jog target";
          else
          {
            // Recover the nominal orientation from the accepted reference, then extend it once.
            const Eigen::Matrix3d target = applied_state_.body.linear() *
                perching_geometry::expSO3(Eigen::Vector3d::UnitY() * (m.value - applied_pitch_offset_));
            if(!model_->orient(nominal_, target, moving_, goal) ||
                !model_->allowed(goal, max_tilt_, min_body_pitch_, max_body_pitch_)) why = "infeasible jog target";
          }
          if(why.empty())
          {
            feed_request_ = msg;
            destination_ = goal;
            body_rate_limit_ = std::min(rate, rate_limit_);
            body_target_valid_ = velocity_active_ = stopping_ = rezero_ready_ = false;
            body_rate_ = 0;
            phase_ = "EXECUTING";
          }
        }
      }
      else
      {
        const auto name = wire_coordinates_.find(m.mode);
        if(name == wire_coordinates_.end() || !coordinate_map_.count(name->second)) why = "unknown coordinate command";
        else
        {
          const int index = coordinate_map_.at(name->second);
          if(!nominalHeld() || !trackingHeld()) why = "motion has not settled; request discarded";
          else if(m.value < model_->lower[index] || m.value > model_->upper[index])
          {
            why = "coordinate outside limits";
          }
          else
          {
            Vector goal = applied_;
            goal[index] = m.value;
            if(applied_pitch_offset_ != 0)
            {
              Reference state;
              Vector nominal_goal;
              if(!model_->evaluate(goal, state) || !model_->orient(goal,
                  state.body.linear() * perching_geometry::expSO3(-Eigen::Vector3d::UnitY() * applied_pitch_offset_), moving_, nominal_goal))
                why = "coordinate target is infeasible under compliance";
              else goal = nominal_goal;
            }
            if(why.empty())
            {
              if(rezero_active_ || !model_->allowed(goal, max_tilt_, min_body_pitch_, max_body_pitch_))
                why = "coordinate target is infeasible";
              else
              {
                destination_ = goal;
                body_rate_limit_ = INFINITY;
                body_target_valid_ = velocity_active_ = stopping_ = rezero_ready_ = false;
                body_rate_ = 0;
                phase_ = "EXECUTING";
              }
            }
          }
        }
      }
      if(why.empty() && motion_.epoch == m.epoch)
      {
        motion_.accept(m.source, m.sequence);
        if(m.mode != Command::JOG) feed_request_.reset();
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

perching_servo::Sample GimbalrotorMultilinkPerchingNavigator::servoSample(const Sample* synchronized) const
{
  perching_servo::Sample s;
  s.now = ros::Time::now().toSec(); s.hover = navi_state_ == HOVER_STATE;
  if(!coordinate_map_.count(positioning_joint_)) return s;
  Sample observed;
  const Sample& measured = synchronized ? *synchronized : observed;
  s.model_valid = model_valid_ && (synchronized || observation(observed));
  if(!s.model_valid) return s;
  const int i = coordinate_map_.at(positioning_joint_);
  s.joint_stamp = measured.stamp; s.q = measured.q[i]; s.qdot = measured.rate[i];
  s.body = perching_geometry::toKdl(measured.body);
  s.angular_speed = measured.omega.norm();
  s.pitch_rate = perching_servo::pitchRate(s.body, KDL::Vector(measured.omega.x(), measured.omega.y(), measured.omega.z()));
  return s;
}

bool GimbalrotorMultilinkPerchingNavigator::servoModel(double value, KDL::Frame& contact, KDL::Vector& cog) const
{
  if(nominal_.size() == 0 || !coordinate_map_.count(positioning_joint_)) return false;
  Vector q = nominal_; q[coordinate_map_.at(positioning_joint_)] = value;
  Pose bc; Eigen::Vector3d center;
  if(!model_->bodyContact(q, bc, center)) return false;
  contact = perching_geometry::toKdl(bc);
  cog = KDL::Vector(center.x(), center.y(), center.z());
  return true;
}

void GimbalrotorMultilinkPerchingNavigator::planCommand(const std::string& action, std::uint64_t session)
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
    if(!nominalHeld() || !trackingHeld() || !motionFeedbackValid()) error = "prepare requires held motion feedback";
    else
    {
      ros::NodeHandle request(nh_, "navigation/perching/request");
      double yaw = 0, roll = 0, start = 0, increment = .34906585;
      request.param("yaw", yaw, yaw); request.param("roll", roll, roll); request.param("pitch_start", start, start);
      request.param("pitch_increment", increment, increment);
      Reference state; Vector goal, end;
      model_->evaluate(nominal_, state);
      const Eigen::Matrix3d desired = state.body.linear() * perching_geometry::expSO3(Eigen::Vector3d::UnitZ() * yaw) *
          perching_geometry::expSO3(Eigen::Vector3d::UnitX() * roll) * perching_geometry::expSO3(Eigen::Vector3d::UnitY() * start);
      if(!model_->orient(nominal_, desired, moving_, goal) || !model_->pitch(goal, increment, moving_, end) ||
          !model_->allowed(goal, max_tilt_, min_body_pitch_, max_body_pitch_) || !model_->allowed(end, max_tilt_, min_body_pitch_, max_body_pitch_))
        error = "requested orientation/pitch is infeasible";
      else
      {
        rezero_ready_ = false; destination_ = goal; body_rate_limit_ = INFINITY; prepared_increment_ = increment; prepared_motion_ = true; body_target_valid_ = false;
        velocity_active_ = stopping_ = paused_ = false; phase_ = "PRE_ALIGN";
      }
    }
  }
  else if(action == "start")
  {
    if(!prepared_motion_ || phase_ != "READY" || !nominalHeld() || !trackingHeld() || !motionFeedbackValid()) error = "start requires measured READY";
    else if(!bodyPitchCommand(prepared_increment_, rate_limit_, body_frame_id_ + 1)) error = "infeasible body pitch target";
  }
  if(error.empty() && (action == "prepare" || action == "start" || action == "resume"))
  { motion_.revoke(); feed_request_.reset(); }
  std_msgs::String ack;
  ack.data = "action=" + action + " session=" + std::to_string(session) + " accepted=" + (error.empty() ? "true" : "false") + " reason=" + error;
  ack_pub_.publish(ack); publishDiagnostics();
}

void GimbalrotorMultilinkPerchingNavigator::workflowCommand(const std::string& action)
{
  if(positioning_)
  {
    auto s = servoSample();
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
  feed_request_.reset();
  rezero_active_ = true; rezero_ready_ = rezero_failed_ = false;
  rezero_started_ = ros::Time::now().toSec(); rezero_stable_since_ = 0; rezero_origin_ = nominal_[0];
  destination_ = nominal_; body_target_valid_ = velocity_active_ = false;
  body_rate_limit_ = INFINITY;
}

void GimbalrotorMultilinkPerchingNavigator::advanceNominal(double dt)
{
  proposed_ = nominal_; proposed_rate_ = nominal_rate_; proposed_progress_ = body_progress_; proposed_body_rate_ = body_rate_;
  const double now = ros::Time::now().toSec();
  if((positioning_ || rezero_active_) && !stopping_ && !trackingHeld())
  {
    proposed_rate_.setZero(); proposed_body_rate_ = 0;
    proposal_valid_ = true; status_ = "nominal held for tracking";
    return;
  }
  if(stopping_ && !body_target_valid_ && !velocity_active_ && std::abs(body_rate_) <= 1e-9)
    followCoordinates(destination_, dt);
  else if(positioning_)
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
  }
  else if(body_target_valid_ || velocity_active_ || std::abs(body_rate_) > 1e-9)
  {
    if(velocity_active_ && now >= velocity_expires_) { velocity_active_ = false; stopping_ = true; }
    const double error = body_destination_ - body_progress_;
    const double acceleration = .5 * acceleration_limit_, change = acceleration * dt;
    const bool tracking = trackingHeld();
    // Reserve stopping distance and finish the last sub-step instead of approaching forever.
    const double stopping_rate = std::sqrt(change * change + 2 * acceleration * std::abs(error)) - change;
    double goal_rate = velocity_active_ ? velocity_command_ :
        (body_target_valid_ && !stopping_ ? std::copysign(std::min(body_rate_limit_, stopping_rate), error) : 0);
    if(!tracking) { goal_rate = 0; status_ = "braking for tracking"; }
    proposed_body_rate_ = std::clamp(goal_rate, body_rate_ - change, body_rate_ + change);
    if(std::abs(proposed_body_rate_) < 1e-8) proposed_body_rate_ = 0;
    double step = .5 * (body_rate_ + proposed_body_rate_) * dt;
    if(body_target_valid_ && !stopping_ && !velocity_active_ && tracking &&
        std::abs(error) <= .5 * change * dt && std::abs(body_rate_) <= change)
    { step = error; proposed_body_rate_ = 0; }
    if(!model_->pitch(nominal_, step, moving_, proposed_)) { status_ = "body pitch direction unavailable"; return; }
    proposed_progress_ = body_progress_ + step;
    proposed_rate_ = (proposed_ - nominal_) / dt;
    destination_ = nominal_;
  }
  else followCoordinates(destination_, dt);
  proposal_valid_ = model_->allowed(proposed_, max_tilt_, min_body_pitch_, max_body_pitch_);
  if(!proposal_valid_) status_ = "nominal proposal outside mechanism/body bounds";
}

void GimbalrotorMultilinkPerchingNavigator::followCoordinates(const Vector& goal, double dt)
{
  const bool tracking = positioning_ || rezero_active_ || trackingHeld();
  if(std::isfinite(body_rate_limit_))
  {
    Vector direction = goal - nominal_;
    const double distance = direction.norm();
    if(distance > 1e-12) direction /= distance;
    else if(nominal_rate_.norm() > 1e-12) direction = nominal_rate_.normalized();
    else { proposed_ = goal; proposed_rate_.setZero(); return; }
    Eigen::MatrixXd angular;
    if(!model_->jacobian(nominal_, angular)) return;
    double speed = INFINITY, acceleration = INFINITY;
    for(int i = 0; i < direction.size(); ++i)
    {
      if(std::abs(direction[i]) < 1e-12) continue;
      speed = std::min(speed, model_->velocity[i] / std::abs(direction[i]));
      acceleration = std::min(acceleration, model_->acceleration[i] / std::abs(direction[i]));
    }
    const double angular_scale = (angular * direction).norm();
    if(angular_scale > 1e-12)
    {
      speed = std::min(speed, body_rate_limit_ / angular_scale);
      if(pitchFeedStep().value_or(0) > 0) speed = std::min(speed, forward_rate_limit_ / angular_scale);
      acceleration = std::min(acceleration, acceleration_limit_ / angular_scale);
    }
    // A single path speed keeps the joint contributions coordinated; retain braking margin.
    acceleration *= .5;
    const double current_rate = direction.dot(nominal_rate_), change = acceleration * dt;
    const double stopping_rate = std::sqrt(change * change + 2 * acceleration * distance) - change;
    const double desired = stopping_ || !tracking ? 0 : std::min(speed, stopping_rate);
    double next_rate = std::clamp(desired, current_rate - change, current_rate + change);
    if(std::abs(next_rate) < 1e-8) next_rate = 0;
    proposed_ = nominal_ + direction * (.5 * (current_rate + next_rate) * dt);
    proposed_rate_ = direction * next_rate;
    if(!stopping_ && tracking && distance <= std::min(.5 * change, speed) * dt && std::abs(current_rate) <= change)
    { proposed_ = goal; proposed_rate_.setZero(); }
    return;
  }
  const bool direct = !positioning_ && !rezero_active_ && applied_pitch_offset_ == 0 && applied_offset_rate_ == 0;
  for(int i = 0; i < nominal_.size(); ++i)
  {
    const double acceleration = (direct ? 1 : .5) * model_->acceleration[i];
    const double speed = model_->velocity[i];
    const double error = goal[i] - nominal_[i], change = acceleration * dt;
    const double stopping_rate = std::sqrt(change * change + 2 * acceleration * std::abs(error)) - change;
    const double desired = stopping_ || !tracking ? 0 : std::copysign(std::min(speed, stopping_rate), error);
    const double current_rate = direct ? applied_rate_[i] : nominal_rate_[i];
    proposed_rate_[i] = std::clamp(desired, current_rate - change, current_rate + change);
    if(std::abs(proposed_rate_[i]) < 1e-8) proposed_rate_[i] = 0;
    // Without compliance, bound the actual discrete output velocity directly, including variable dt.
    proposed_[i] = nominal_[i] + (direct ? proposed_rate_[i] : .5 * (current_rate + proposed_rate_[i])) * dt;
    if(!stopping_ && tracking && std::abs(error) <= .5 * change * dt && std::abs(current_rate) <= change &&
        (!direct || (std::abs(error / dt) <= speed && std::abs(error / dt - current_rate) <= change)))
    { proposed_[i] = goal[i]; proposed_rate_[i] = direct ? error / dt : 0; }
  }
}

bool GimbalrotorMultilinkPerchingNavigator::candidateAllowed(const Vector& q, double dt) const
{
  if(!model_->allowed(q, max_tilt_, min_body_pitch_, max_body_pitch_)) return false;
  if(applied_stamp_ == 0) return true;
  if(dt <= 0) return (q - applied_).norm() < 1e-8;
  const Vector rate = (q - applied_) / dt;
  if((rate.cwiseAbs().array() > model_->velocity.array() + 1e-6).any() ||
      ((rate - applied_rate_).cwiseAbs().array() > model_->acceleration.array() * dt + 1e-5).any()) return false;
  Reference state;
  if(!model_->evaluate(q, state)) return false;
  const Eigen::Vector3d velocity = (state.cog.translation() - applied_state_.cog.translation()) / dt;
  return velocity.norm() <= cog_speed_limit_ + 1e-6 &&
      (velocity - reference_velocity_).norm() <= cog_acceleration_limit_ * dt + 1e-5;
}

double GimbalrotorMultilinkPerchingNavigator::appliedPitchRate() const
{
  return applied_state_.body.linear().col(1).dot(applied_state_.cog.linear() * reference_omega_);
}

bool GimbalrotorMultilinkPerchingNavigator::applyPerchingReference(double pitch_offset, double rate, double acceleration,
    double forward_rate_limit, double recovery_rate_limit, const std::optional<CuttingMotion>& cutting)
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  if(navi_state_ != HOVER_STATE || force_landing_flag_ || !std::isfinite(pitch_offset) || !std::isfinite(rate) || !std::isfinite(acceleration)) return false;
  if(perching_mode_ == perching_geometry::Mode::DISABLED || !perching_locked_)
  {
    sensor_msgs::JointState command;
    if(cycle_dt_ > 0 && jointCommand(command)) joints_pub_.publish(command);
    publishDiagnostics();
    return false;
  }

  const double now = ros::Time::now().toSec();
  if(applied_stamp_ == now) return false;
  forward_rate_limit_ = forward_rate_limit;
  recovery_rate_limit_ = recovery_rate_limit;
  proposal_valid_ = false;
  const auto previous_servo = servo_;
  const Vector previous_destination = destination_;
  const bool previous_rezero_active = rezero_active_, previous_rezero_failed = rezero_failed_;
  const bool previous_velocity_active = velocity_active_, previous_stopping = stopping_;
  const double previous_offset_rate = applied_offset_rate_;
  auto restoreNominal = [&]()
  {
    servo_ = previous_servo;
    destination_ = previous_destination;
    rezero_active_ = previous_rezero_active; rezero_failed_ = previous_rezero_failed;
    velocity_active_ = previous_velocity_active; stopping_ = previous_stopping;
    proposed_ = nominal_; proposed_rate_ = nominal_rate_;
    proposed_progress_ = body_progress_; proposed_body_rate_ = body_rate_;
    proposal_valid_ = false;
  };
  if(observation(measured_)) model_->estimate(measured_.body, measured_.q, pivot_error_, hinge_error_);
  const bool fresh = motionFeedbackValid();
  if(fresh) status_ = "tracking accepted reference";
  // Cutting allocates one total increment below instead of advancing nominal independently.
  if(!cutting && applied_stamp_ > 0 && fresh && cycle_dt_ > 0) advanceNominal(cycle_dt_);
  if(applied_stamp_ > 0 && !proposal_valid_) restoreNominal();
  const Vector planned_nominal = proposal_valid_ && fresh ? proposed_ : nominal_;
  Vector nominal = planned_nominal;
  Vector candidate;
  Reference state;
  const bool correcting = pitch_offset != 0 || rate != 0 || acceleration != 0 ||
      applied_pitch_offset_ != 0 || applied_offset_rate_ != 0 || std::isfinite(forward_rate_limit_);
  const double acceleration_limit = std::min(acceleration_limit_, pitch_offset_acceleration_limit_);
  Reference cutting_origin, cutting_endpoint;
  Vector cutting_tangent;
  double cutting_rate = 0, remaining = 0;
  bool relief_constrained = false;
  const double previous_pitch_rate = appliedPitchRate();
  const bool feeding = cutting && pitchFeedStep().value_or(0) > 0;
  if(!feeding) cutting_state_.reset();
  const bool tracking = !cutting || trackingHeld();
  const bool cutting_geometry = !cutting || (model_->evaluate(nominal_, cutting_origin) &&
      model_->evaluate(destination_, cutting_endpoint) && model_->pitchTangent(applied_, moving_, cutting_tangent));
  if(cutting && cutting_geometry && cycle_dt_ > 0)
  {
    remaining = perching_geometry::logSO3(cutting_origin.body.linear().transpose() * cutting_endpoint.body.linear()).y() - applied_pitch_offset_;
    double requested_rate = cutting->rate;
    double retreat_remaining = INFINITY, feed_remaining = INFINITY;
    if(feeding && fresh && cutting->valid)
    {
      if(!cutting_state_)
      {
        cutting_state_.emplace(CuttingState{});
        cutting_state_->frame = applied_state_.body.linear();
        cutting_state_->measured = perching_geometry::logSO3(cutting_state_->frame.transpose() * measured_.body.linear()).y();
        // Preserve the initial holding error, but never absorb subsequent feed lag into it.
        cutting_state_->initial_lead = -cutting_state_->measured;
      }
      auto& motion = *cutting_state_;
      const double measured_pitch = perching_geometry::logSO3(motion.frame.transpose() * measured_.body.linear()).y();
      motion.measured += cycle_dt_ / (cutting_measurement_time_ + cycle_dt_) * (measured_pitch - motion.measured);
      const double applied_pitch = perching_geometry::logSO3(motion.frame.transpose() * applied_state_.body.linear()).y();
      motion.lead = applied_pitch - motion.measured - motion.initial_lead;
      feed_remaining = std::max(0.0, *pitchFeedStep() - motion.lead);
      const bool contact_onset = cutting->contact && !motion.contact;
      if(cutting->contact) motion.contact = true;
      // Unloading during our own retreat/recovery must not trigger another first-contact backoff.
      else if(cutting->released && motion.phase == CuttingState::FEED && nominalHeld() &&
          std::abs(applied_pitch_offset_) <= jog_tolerance_) motion.contact = false;
      if((contact_onset || cutting->relief) && motion.phase == CuttingState::FEED)
      {
        motion.phase = CuttingState::RETREAT;
        motion.measured_anchor = motion.measured;
        // Both raw and averaged measured pitch bound the anchor; no roll/yaw error is imported.
        motion.anchor = std::min(applied_pitch, std::min(measured_pitch, motion.measured));
        motion.goal = motion.anchor - cutting->retreat;
        motion.withdrawal = cutting->retreat;
        motion.hold_elapsed = 0;
      }
      if(motion.phase != CuttingState::FEED)
      {
        if(motion.anchor - cutting->retreat < motion.goal - 1e-9 && (cutting->contact || cutting->relief))
        {
          motion.goal = motion.anchor - cutting->retreat;
          motion.withdrawal = cutting->retreat;
          motion.phase = CuttingState::RETREAT;
          motion.hold_elapsed = 0;
        }
        retreat_remaining = std::max(0.0, applied_pitch - motion.goal);
        relief_constrained = applied_pitch_offset_ - retreat_remaining < -pitch_offset_limit_;
        if(motion.phase == CuttingState::HOLD && cutting->relief)
        { motion.phase = CuttingState::RETREAT; motion.hold_elapsed = 0; }
        if(motion.phase == CuttingState::RETREAT)
        {
          if(retreat_remaining > 1e-6) requested_rate = -pitch_offset_rate_limit_;
          else if(cutting->relief)
          {
            // The initial displacement is not a spring equilibrium under persistent overload.
            requested_rate = std::min(cutting->rate, 0.0);
            retreat_remaining = INFINITY;
          }
          else
          {
            requested_rate = 0;
            if(std::abs(previous_pitch_rate) < 1e-6 && motion.measured <= motion.measured_anchor - .5 * motion.withdrawal)
            { motion.phase = CuttingState::HOLD; motion.hold_elapsed = 0; }
          }
        }
        if(motion.phase == CuttingState::HOLD)
        {
          requested_rate = 0;
          if(motion.measured <= motion.measured_anchor - .5 * motion.withdrawal) motion.hold_elapsed += cycle_dt_;
          else motion.hold_elapsed = 0;
          if(motion.hold_elapsed >= cutting_hold_duration_) motion.phase = CuttingState::FEED;
        }
      }
    }
    else if(cutting->relief && fresh)
    {
      // Cancellation retains bounded load relief, without retaining a recovery destination.
      const double measured_progress = perching_geometry::logSO3(applied_state_.body.linear().transpose() * measured_.body.linear()).y();
      requested_rate = std::min(0.0, measured_progress) / cycle_dt_ + std::min(requested_rate, -acceleration_limit * cycle_dt_);
    }
    if(!cutting->valid || !fresh)
    {
      requested_rate = 0;
      if(cutting_state_) cutting_state_->hold_elapsed = 0;
    }
    if(!tracking || !feeding) requested_rate = std::min(requested_rate, 0.0);
    const double change = acceleration_limit * cycle_dt_;
    double positive_limit = forward_rate_limit_, negative_limit = pitch_offset_rate_limit_;
    if(feeding) positive_limit = std::min(positive_limit, std::sqrt(change * change + 2 * acceleration_limit * std::max(0.0, remaining)) - change);
    positive_limit = std::min(positive_limit, std::sqrt(change * change + 2 * acceleration_limit * feed_remaining) - change);
    negative_limit = std::min(negative_limit, std::sqrt(change * change + 2 * acceleration_limit * retreat_remaining) - change);
    negative_limit = std::min(negative_limit, std::sqrt(change * change +
        2 * acceleration_limit * std::max(0.0, pitch_offset_limit_ + applied_pitch_offset_)) - change);
    cutting_rate = std::clamp(std::clamp(requested_rate, -negative_limit, positive_limit), previous_pitch_rate - change, previous_pitch_rate + change);
  }
  auto preview = [&](double& offset, double& offset_rate, double& offset_acceleration)
  {
    candidate = nominal;
    Reference origin;
    double positive_rate = recovery_rate_limit_, forward_rate = INFINITY;
    if(cutting)
    {
      if(!cutting_geometry) return false;
      double increment = cutting_rate * cycle_dt_;
      const double change = acceleration_limit * cycle_dt_;
      const bool finish = feeding && tracking && cutting->valid && cutting_state_ && cutting_state_->phase == CuttingState::FEED &&
          !cutting->relief && cutting->rate > 0 && forward_rate_limit_ > 0 &&
          remaining >= 0 && remaining <= std::min(.5 * change, forward_rate_limit_) * cycle_dt_ && std::abs(previous_pitch_rate) <= change;
      if(finish) increment = remaining;
      if((feeding && increment > std::max(0.0, remaining) + 1e-9) || applied_pitch_offset_ + increment < -pitch_offset_limit_ - 1e-9) return false;
      const double progress = applied_pitch_offset_ + increment;
      // Recover negative offset first; only the remainder advances nominal toward its retained endpoint.
      const bool advance = progress >= 0;
      // A tangent seed advances even steps smaller than the orientation solver's tolerance.
      candidate = applied_ + cutting_tangent * increment;
      if(finish) candidate = destination_;
      else if(!model_->orient(candidate, cutting_origin.body.linear() *
          perching_geometry::expSO3(Eigen::Vector3d::UnitY() * progress), moving_, candidate)) return false;
      if(!model_->evaluate(candidate, state)) return false;
      offset = advance || finish ? 0 : perching_geometry::logSO3(cutting_origin.body.linear().transpose() * state.body.linear()).y();
      if(std::abs(offset) > pitch_offset_limit_ + 1e-9) return false;
      offset_rate = (offset - applied_pitch_offset_) / cycle_dt_;
      offset_acceleration = (offset_rate - applied_offset_rate_) / cycle_dt_;
      proposed_ = advance || finish ? candidate : nominal_;
      proposed_rate_ = (proposed_ - nominal_) / cycle_dt_;
      proposed_progress_ = body_progress_; proposed_body_rate_ = 0;
      proposal_valid_ = true;
      forward_rate = std::max(forward_rate_limit_, previous_pitch_rate - change);
      status_ = cutting->relief ? "measured-pitch relief" :
          (offset < 0 ? "recovering compliant reference" : "feeding retained endpoint");
      if(relief_constrained) status_ = "relief constrained by offset limit";
      else if(cutting_state_ && cutting_state_->phase == CuttingState::RETREAT) status_ = "executing measured-pitch retreat";
      else if(cutting_state_ && cutting_state_->phase == CuttingState::HOLD) status_ = "holding after measured withdrawal";
      else if(!tracking && !cutting->relief) status_ = "cutting feed held for tracking";
      else if(forward_rate_limit_ == 0 && !cutting->relief) status_ = "cutting forward motion held";
      else if(cutting_state_ && cutting_state_->lead > .5 * pitchFeedStep().value_or(0) && remaining <= jog_tolerance_)
        status_ = "reference reached; awaiting measured cutting progress";
    }
    else if(correcting)
    {
      if(!model_->evaluate(nominal, origin)) return false;
      if(std::isfinite(forward_rate_limit_) && applied_stamp_ > 0)
      {
        const Eigen::Vector3d axis = origin.body.linear().col(1);
        const Eigen::Vector3d previous_omega = applied_state_.cog.linear() * reference_omega_;
        const Eigen::Matrix3d previous_nominal = applied_state_.body.linear() *
            perching_geometry::expSO3(-Eigen::Vector3d::UnitY() * applied_pitch_offset_);
        const double nominal_rate = axis.dot(perching_geometry::logSO3(origin.body.linear() * previous_nominal.transpose())) / cycle_dt_;
        // Share the forward budget between nominal travel and recovery, with bounded braking.
        forward_rate = std::max(forward_rate_limit_, axis.dot(previous_omega) - acceleration_limit_ * cycle_dt_);
        positive_rate = std::min(positive_rate, forward_rate - nominal_rate);
      }
      if(applied_stamp_ > 0 && !limitPitchOffset(offset, offset_rate, offset_acceleration,
          positive_rate, std::isfinite(recovery_rate_limit_) ? 0 : INFINITY)) return false;
      if(offset != 0 && !model_->orient(nominal,
          origin.body.linear() * perching_geometry::expSO3(Eigen::Vector3d::UnitY() * offset), moving_, candidate)) return false;
    }
    // These checks bound the combined nominal + compliant joint/CoG motion.
    if(!candidateAllowed(candidate, cycle_dt_)) return false;
    if(!cutting)
    {
      if(correcting && offset == 0) state = origin;
      else if(!model_->evaluate(candidate, state)) return false;
    }
    if((correcting || std::isfinite(body_rate_limit_)) && applied_stamp_ > 0)
    {
      const Eigen::Vector3d omega_world = perching_geometry::logSO3(state.body.linear() * applied_state_.body.linear().transpose()) / cycle_dt_;
      const Eigen::Vector3d previous_omega_world = applied_state_.cog.linear() * reference_omega_;
      const Eigen::Vector3d pitch_axis = state.body.linear().col(1);
      const bool body_jog = std::isfinite(body_rate_limit_);
      const double speed = body_jog ? omega_world.norm() : std::abs(pitch_axis.dot(omega_world));
      const double change = body_jog ? (omega_world - previous_omega_world).norm() : std::abs(pitch_axis.dot(omega_world - previous_omega_world));
      const double speed_limit = cutting ? (pitch_axis.dot(omega_world) < 0 ?
          std::max(pitch_offset_rate_limit_, -previous_pitch_rate - acceleration_limit * cycle_dt_) : forward_rate) : rate_limit_;
      if(speed > speed_limit + 1e-6 || change > acceleration_limit_ * cycle_dt_ + 1e-6) return false;
      if(pitch_axis.dot(omega_world) > forward_rate + 1e-6) return false;
    }
    return true;
  };
  const bool eligible = fresh && (applied_stamp_ == 0 || cycle_dt_ > 0) &&
      (applied_stamp_ > 0 || (!cutting && pitch_offset == 0 && rate == 0 && acceleration == 0));
  bool accepted = eligible && preview(pitch_offset, rate, acceleration);
  if(eligible && !accepted && correcting && applied_stamp_ > 0)
  {
    const double requested_acceleration = std::clamp(acceleration, -acceleration_limit, acceleration_limit);
    const double requested_cutting_rate = cutting_rate;
    for(int attempt = 0; attempt < 7; ++attempt)
    {
      if(cutting) cutting_rate = attempt == 6 ? previous_pitch_rate +
          std::clamp(-previous_pitch_rate, -acceleration_limit * cycle_dt_, acceleration_limit * cycle_dt_) :
          previous_pitch_rate + (requested_cutting_rate - previous_pitch_rate) * std::pow(.5, attempt + 1);
      double next_acceleration = attempt == 6 ? std::clamp(-applied_offset_rate_ / cycle_dt_, -acceleration_limit, acceleration_limit) :
          requested_acceleration * std::pow(.5, attempt);
      double next_rate = applied_offset_rate_ + next_acceleration * cycle_dt_;
      double next_offset = applied_pitch_offset_ + next_rate * cycle_dt_;
      nominal = planned_nominal;
      bool feasible = preview(next_offset, next_rate, next_acceleration);
      if(!cutting && !feasible && (nominal - nominal_).norm() > 1e-12)
      {
        nominal = nominal_;
        feasible = preview(next_offset, next_rate, next_acceleration);
        if(feasible) restoreNominal();
      }
      if(!feasible) continue;
      pitch_offset = next_offset; rate = next_rate; acceleration = next_acceleration;
      accepted = true;
      if(!cutting) status_ = proposal_valid_ ? "bounded compliant step accepted" : "bounded compliant step; nominal progress held";
      break;
    }
  }
  if(accepted)
  {
    // Ordinary fully actuated flight commands zero CoG roll/pitch. The desired
    // body attitude is carried by desire_coordinate, as in GimbalrotorNavigator.
    const double yaw = std::atan2(state.cog.linear()(1, 0), state.cog.linear()(0, 0));
    state.cog.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const bool braking = stopping_;
    const bool derivative = applied_stamp_ > 0 && cycle_dt_ > 0;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(), omega = Eigen::Vector3d::Zero();
    if(derivative)
    {
      velocity = (state.cog.translation() - applied_state_.cog.translation()) / cycle_dt_;
      omega = state.cog.linear().transpose() *
          perching_geometry::logSO3(state.body.linear() * applied_state_.body.linear().transpose()) / cycle_dt_;
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
    reference_angular_acceleration_ = derivative ?
        Eigen::Vector3d((omega - state.cog.linear().transpose() * applied_state_.cog.linear() * reference_omega_) / cycle_dt_) :
        Eigen::Vector3d::Zero();
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
    if(positioning_)
    {
      // Keep servo workflow progress nominal; compliance must not accumulate.
      Reference nominal_state = applied_state_;
      if(pitch_offset != 0) model_->evaluate(nominal_, nominal_state);
      servo_.q_target = nominal_[coordinate_map_.at(positioning_joint_)];
      servo_.target = perching_geometry::toKdl(nominal_state.body);
      perching_servo::Sample reference;
      reference.body = servo_.target;
      if(servo_.stage == perching_servo::Stage::PASSIVE_CUTTING || servo_.stage == perching_servo::Stage::PITCH_REZERO_ACTIVE)
        servo_.alpha = servo_.measuredAlpha(reference);
      if(braking)
      {
        servo_.q_goal = servo_.q_target;
        servo_.alpha_goal = servo_.alpha;
        // Servo FK must retain the adopted hinge angle after braking.
        servo_.contact = perching_geometry::toKdl(model_->pivot()) * KDL::Frame(KDL::Rotation::RotY(nominal_[0]));
      }
    }
    if(rezero_active_)
    {
      const double body_pitch = std::asin(std::clamp(-measured_.body.linear()(2, 0), -1.0, 1.0));
      if(std::abs(rezero_target_ - body_pitch) <= rezero_tolerance_ && measured_.omega.norm() <= rezero_rate_tolerance_)
      {
        if(rezero_stable_since_ == 0) rezero_stable_since_ = now;
        if(now - rezero_stable_since_ >= rezero_stable_duration_) { rezero_active_ = false; rezero_ready_ = true; }
      }
      else rezero_stable_since_ = 0;
    }
  }
  else
  {
    // Reject the body/joint pair together, including servo workflow progress.
    restoreNominal();
    nominal_rate_.setZero(); body_rate_ = 0;
    applied_rate_.setZero(); applied_acceleration_.setZero(); applied_offset_rate_ = 0;
    reference_velocity_.setZero(); reference_acceleration_.setZero();
    reference_omega_.setZero(); reference_angular_acceleration_.setZero();
    rezero_stable_since_ = 0;
    status_ = fresh ? "coordinated reference rejected; holding last feasible output" : "waiting for motion feedback";
  }
  proposal_valid_ = false;
  if(applied_stamp_ > 0) publishApplied();
  if(accepted)
  {
    sensor_msgs::JointState command;
    if(jointCommand(command)) joints_pub_.publish(command);
  }
  if(fresh && nominalHeld() && trackingHeld())
  {
    if(phase_ == "EXECUTING" && jogReady()) phase_ = "COMPLETED_HOLD";
    if(phase_ == "PRE_ALIGN") phase_ = "READY";
  }
  if(!admittance_diagnostics_.empty())
  {
    admittance_diagnostics_["reference_accepted"] = accepted ? "1" : "0";
    admittance_diagnostics_["offset_rad"] = std::to_string(applied_pitch_offset_);
    admittance_diagnostics_["offset_rate_rad_s"] = std::to_string(applied_offset_rate_);
    if(accepted && cycle_dt_ > 0)
      admittance_diagnostics_["offset_acceleration_rad_s2"] = std::to_string((applied_offset_rate_ - previous_offset_rate) / cycle_dt_);
  }
  publishDiagnostics();
  return accepted;
}

void GimbalrotorMultilinkPerchingNavigator::publishApplied()
{
  setXyControlMode(POS_CONTROL_MODE);
  const Eigen::Matrix3d relative = applied_state_.cog.linear().transpose() * applied_state_.body.linear();
  setSynchronizedBaselinkRotationRelativeToCog(perching_geometry::tfQuaternion(relative));
  double roll, pitch, yaw;
  tf::Matrix3x3(perching_geometry::tfQuaternion(applied_state_.cog.linear())).getRPY(roll, pitch, yaw);
  setTargetPos(tfVector(applied_state_.cog.translation())); setTargetRPY(tf::Vector3(roll, pitch, yaw));
  setTargetVel(tfVector(reference_velocity_)); setTargetAcc(tfVector(reference_acceleration_));
  setTargetOmega(tfVector(reference_omega_));
  Eigen::Matrix3d current;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::BASELINK, estimate_mode_), current);
  current = (current * relative.transpose()).eval();
  setTargetAngAcc(tfVector(current.transpose() * applied_state_.cog.linear() * reference_angular_acceleration_));
}

bool GimbalrotorMultilinkPerchingNavigator::jointCommand(sensor_msgs::JointState& command) const
{
  if(navi_state_ != HOVER_STATE) return false;
  command.header.stamp = ros::Time::now();
  if(!perching_locked_)
  {
    if(!positioning_ || !servo_.zero_hold_armed || !servo_.feedback(servoSample())) return false;
    command.name = {positioning_joint_};
    command.position = {servo_.q_target};
    return true;
  }
  if(applied_stamp_ == 0) return false;
  command.name = active_joints_;
  for(int i = 1; i < applied_.size(); ++i) command.position.push_back(applied_[i]);
  return true;
}

void GimbalrotorMultilinkPerchingNavigator::publishDiagnostics()
{
  const bool feedback = motionFeedbackValid();
  const bool held = applied_stamp_ > 0 && nominalHeld() && feedback && trackingHeld();
  std_msgs::UInt64 id; id.data = session_; session_pub_.publish(id); id.data = motion_.epoch; epoch_pub_.publish(id);
  diagnostic_msgs::DiagnosticArray array; array.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus diagnostic; diagnostic.name = nh_.getNamespace() + "/" + status_name_;
  const std::string stage = !perching_locked_ ? "FREE_FLIGHT" : rezero_failed_ ? "PITCH_REZERO_FAILED" :
      rezero_active_ ? "PITCH_REZERO_ACTIVE" : rezero_ready_ ? "PITCH_REZERO_HOLD" : prepared_motion_ ? phase_ : "CONTACT_LOCKED";
  diagnostic.message = stage + ": " + status_; diagnostic.level = feedback || !perching_locked_ ? 0 : 1;
  std::map<std::string, std::string> values;
  for(const auto& key : {"admittance_configured", "admittance_enabled", "admittance_enable_requested",
      "baseline_ready", "tare_collecting", "tare_generation", "tare_request_sequence", "enable_request_sequence",
      "task_torque_valid", "offset_rad", "offset_rate_rad_s", "offset_acceleration_rad_s2"}) values[key] = "0";
  values["admittance_status"] = "unavailable: nominal navigation with ordinary flight controller";
  for(const auto& item : admittance_diagnostics_) values[item.first] = item.second;
  auto value = [&](const std::string& key, auto data) { std::ostringstream s; s << std::setprecision(17) << data; values[key] = s.str(); };
  value("session_id", session_); value("motion_epoch", motion_.epoch); value("motion_source", motion_.source);
  value("motion_sequence", motion_.sequence); value("stopping", stopping_); value("feedback_valid", feedback);
  value("body_pitch_target_valid", body_target_valid_); value("body_pitch_target_frame", body_frame_id_);
  value("body_pitch_target_rad", body_destination_); value("body_pitch_target_progress_rad", body_progress_);
  value("body_pitch_target_reference_complete", body_target_valid_ && held);
  value("body_pitch_velocity_active", velocity_active_); value("body_pitch_velocity_limit", rate_limit_);
  value("admittance_hold_eligible", held);
  value("jog_ready", held && jogReady()); value("effective_offset_rad", applied_pitch_offset_);
  value("hold_reason", status_); value("pivot_error", pivot_error_); value("hinge_residual", hinge_error_);
  if(cutting_state_)
  {
    value("cutting_phase", static_cast<int>(cutting_state_->phase));
    value("cutting_measured_progress_rad", cutting_state_->measured);
    value("cutting_forward_lead_rad", cutting_state_->lead);
    value("cutting_initial_lead_rad", cutting_state_->initial_lead);
    value("cutting_retreat_goal_rad", cutting_state_->goal);
    value("cutting_measured_withdrawal_rad", cutting_state_->measured_anchor - cutting_state_->measured);
  }
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

void GimbalrotorMultilinkPerchingNavigator::onNonHover()
{
  servo_.zero_hold_armed = false;
}

void GimbalrotorMultilinkPerchingNavigator::onClockRewind()
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  body_history_.clear();
  joint_history_.clear();
}

PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorMultilinkPerchingNavigator, aerial_robot_navigation::BaseNavigator)
