#include <aerial_robot_motion/core/motion_core.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <aerial_robot_motion/core/ros_conversions.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <XmlRpcValue.h>
#include <set>
#include <sstream>
#include <algorithm>

namespace aerial_robot_motion
{
namespace
{
bool fresh(const ros::Time& stamp, const ros::Time& now, double timeout)
{
  const double age = (now - stamp).toSec();
  return !stamp.isZero() && age >= -0.02 && age <= timeout;
}
template<class Base>
std::vector<boost::shared_ptr<Base>> loadPlugins(ros::NodeHandle pnh, const std::string& list,
    const std::string& config, pluginlib::ClassLoader<Base>& loader, const ModelInfo& info)
{
  XmlRpc::XmlRpcValue entries;
  if (!pnh.getParam(list, entries) || entries.getType() != XmlRpc::XmlRpcValue::TypeArray)
    throw std::invalid_argument("missing or invalid plugin list: " + list);
  std::vector<boost::shared_ptr<Base>> result;
  std::set<std::string> names;
  for (int i = 0; i < entries.size(); ++i)
  {
    auto& entry = entries[i];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeStruct || !entry.hasMember("name") ||
        !entry.hasMember("type") || entry["name"].getType() != XmlRpc::XmlRpcValue::TypeString ||
        entry["type"].getType() != XmlRpc::XmlRpcValue::TypeString)
      throw std::invalid_argument(list + " requires entries with name and type");
    std::string name = entry["name"], type = entry["type"];
    if (name.empty() || name.find('/') != std::string::npos || !names.insert(name).second)
      throw std::invalid_argument("invalid or duplicate plugin instance name: " + name);
    auto plugin = loader.createInstance(type);
    plugin->initialize(ros::NodeHandle(pnh, config + "/" + name), info);
    result.push_back(plugin);
  }
  return result;
}
bool finiteTwist(const geometry_msgs::Twist& twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) && std::isfinite(twist.linear.z) &&
         std::isfinite(twist.angular.x) && std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}
}

MotionCore::MotionCore(ros::NodeHandle nh, ros::NodeHandle pnh)
  : nh_(nh), pnh_(pnh), cost_loader_("aerial_robot_motion", "aerial_robot_motion::cost::Base"),
    constraint_loader_("aerial_robot_motion", "aerial_robot_motion::constraint::Base"), tf_listener_(tf_)
{
  pnh_.param<std::string>("world_frame", world_, "world"); world_ = cleanFrame(world_);
  if (world_.empty()) throw std::invalid_argument("world_frame is empty");
  rate_ = nonnegativeParam(pnh_, "update_rate", 50);
  timeout_ = nonnegativeParam(pnh_, "state_timeout", 0.2);
  max_dt_ = nonnegativeParam(pnh_, "max_dt", 0.1);
  max_skew_ = nonnegativeParam(pnh_, "state_max_skew", 0.05);
  regularization_ = nonnegativeParam(pnh_, "qp_regularization", 1e-6);
  contact_tolerance_ = nonnegativeParam(pnh_, "contact_position_tolerance", 0.005);
  contact_angle_tolerance_ = nonnegativeParam(pnh_, "contact_angle_tolerance", 0.02);
  if (rate_ <= 0 || timeout_ <= 0 || max_dt_ < 1 / rate_ || max_dt_ > 1 || regularization_ <= 0)
    throw std::invalid_argument("invalid timing or QP regularization");
  pnh_.param("publish_commands", publish_commands_, false);
  pnh_.param("hover_state", hover_state_, 5);
  pnh_.param<std::string>("hinge_axis_frame", hinge_frame_, "contact");
  if (hinge_frame_ != "contact" && hinge_frame_ != "world")
    throw std::invalid_argument("hinge_axis_frame must be contact or world");
  hinge_axis_ = vectorParam(pnh_, "hinge_axis", 3, 0, false);
  if (hinge_axis_.norm() < 1e-9) throw std::invalid_argument("hinge_axis must be nonzero");
  hinge_axis_.normalize();
  model_.initialize(nh_, pnh_);
  costs_ = loadPlugins(pnh_, "cost_plugins", "costs", cost_loader_, model_.info());
  constraints_ = loadPlugins(pnh_, "constraint_plugins", "constraints", constraint_loader_, model_.info());
  if (costs_.empty()) throw std::invalid_argument("at least one cost plugin is required");
  // The perching service requires the contact constraint to be in the list.
  XmlRpc::XmlRpcValue entries; pnh_.getParam("constraint_plugins", entries);
  for (int i = 0; i < entries.size(); ++i)
    if (static_cast<std::string>(entries[i]["type"]) == "aerial_robot_motion/RevoluteContact") has_contact_plugin_ = true;
  solver_.configure(ros::NodeHandle(pnh_, "solver"));
  commands_.initialize(pnh_, tf_, world_);
  admittance_.initialize(pnh_, tf_, world_);
  bridge_.initialize(nh_, pnh_, model_.info(), world_);
  std::string topic;
  pnh_.param<std::string>("odom_topic", topic, "uav/baselink/odom");
  odom_sub_ = nh_.subscribe(topic, 1, &MotionCore::odometry, this, ros::TransportHints().tcpNoDelay());
  pnh_.param<std::string>("cog_odometry_topic", topic, "uav/cog/odom");
  cog_sub_ = nh_.subscribe(topic, 1, &MotionCore::cogOdometry, this);
  pnh_.param<std::string>("joint_state_topic", topic, "joint_states");
  joint_sub_ = nh_.subscribe(topic, 1, &MotionCore::jointState, this, ros::TransportHints().tcpNoDelay());
  pnh_.param<std::string>("flight_state_topic", topic, "flight_state");
  flight_sub_ = nh_.subscribe<std_msgs::UInt8>(topic, 1, [this](const std_msgs::UInt8ConstPtr& msg) {
    flight_state_ = msg->data; flight_stamp_ = ros::Time::now();
  });
  pnh_.param<std::string>("command_inhibit_topic", topic, "");
  if (!topic.empty()) inhibit_sub_ = nh_.subscribe<std_msgs::Bool>(topic, 1,
      [this](const std_msgs::BoolConstPtr& msg) { inhibited_ = msg->data; });
  diagnostics_ = pnh_.advertise<diagnostic_msgs::DiagnosticArray>("status", 1, true);
  perching_service_ = pnh_.advertiseService("perching/enable", &MotionCore::perching, this);
  reset_service_ = pnh_.advertiseService("command/reset", &MotionCore::resetTarget, this);
  timer_ = nh_.createTimer(ros::Duration(1 / rate_), &MotionCore::update, this);
  ROS_INFO("Motion core ready: %d variables, %zu link joints, commands %s", model_.info().dimension(),
           model_.info().joint_names.size(), publish_commands_ ? "enabled (HOVER gate)" : "disabled (preview)");
}

void MotionCore::odometry(const nav_msgs::OdometryConstPtr& msg) { odom_ = *msg; have_odom_ = true; }
void MotionCore::cogOdometry(const nav_msgs::OdometryConstPtr& msg) { cog_ = *msg; have_cog_ = true; }
void MotionCore::jointState(const sensor_msgs::JointStateConstPtr& msg) { joints_ = *msg; have_joints_ = true; }

bool MotionCore::measurements(MotionState& state, std::string& error)
{
  const auto now = ros::Time::now();
  if (!have_odom_ || !fresh(odom_.header.stamp, now, timeout_))
    { error = "missing, future, or stale baselink odometry"; return false; }
  if (!finiteTwist(odom_.twist.twist)) { error = "nonfinite odometry twist"; return false; }
  const std::string child = cleanFrame(odom_.child_frame_id), base = model_.baselink();
  if (child != base && (child.size() <= base.size() ||
      child.substr(child.size() - base.size() - 1) != "/" + base))
    { error = "odometry child_frame_id must identify the configured model baselink"; return false; }
  Eigen::Isometry3d world_T_base = fromPose(odom_.pose.pose);
  const auto worldPose = [this](const nav_msgs::Odometry& odom, const Eigen::Isometry3d& pose) {
    const std::string frame = cleanFrame(odom.header.frame_id);
    if (frame.empty()) throw std::runtime_error("odometry world frame is empty");
    if (frame == world_) return pose;
    return fromTransform(tf_.lookupTransform(world_, frame, odom.header.stamp).transform) * pose;
  };
  world_T_base = worldPose(odom_, world_T_base);
  if (publish_commands_ || have_cog_)
  {
    if (!have_cog_ || !fresh(cog_.header.stamp, now, timeout_) ||
        std::abs((cog_.header.stamp - odom_.header.stamp).toSec()) > max_skew_)
      { error = "missing, stale, or unsynchronized CoG odometry"; return false; }
    auto world_T_cog = worldPose(cog_, fromPose(cog_.pose.pose));
    model_.setCogDesiredOrientation(world_T_cog.linear().transpose() * world_T_base.linear());
  }
  if (model_.needsJointState() && (!have_joints_ || !fresh(joints_.header.stamp, now, timeout_) ||
      std::abs((joints_.header.stamp - odom_.header.stamp).toSec()) > max_skew_))
    { error = "missing, stale, or unsynchronized joint state"; return false; }
  KDL::JntArray full;
  if (!model_.readJoints(have_joints_ ? joints_ : sensor_msgs::JointState{}, full, error)) return false;
  state = model_.measured(world_T_base, full, odom_.header.stamp);
  return true;
}

bool MotionCore::perching(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  try
  {
    if (!req.data)
    {
      perched_ = false; solver_.reset(); res.success = true; res.message = "contact released"; return true;
    }
    if (!has_contact_plugin_) throw std::runtime_error("RevoluteContact plugin is not configured");
    if (perched_) { res.success = true; res.message = "contact remains locked"; return true; }
    MotionState state; std::string error;
    if (!measurements(state, error)) throw std::runtime_error(error);
    locked_position_ = state.contact.translation();
    locked_axis_ = hinge_frame_ == "world" ? hinge_axis_ : state.contact.linear() * hinge_axis_;
    hinge_local_ = state.contact.linear().transpose() * locked_axis_;
    perched_ = true;
    commands_.resetTarget(state.tool); admittance_.reset(); solver_.reset();
    res.success = true; res.message = "contact locked from current measured state; target reset to current tool";
  }
  catch (const std::exception& e) { res.success = false; res.message = e.what(); }
  return true;
}

bool MotionCore::resetTarget(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res)
{
  try
  {
    MotionState state; std::string error;
    if (!measurements(state, error)) throw std::runtime_error(error);
    commands_.resetTarget(state.tool); admittance_.reset(); solver_.reset();
    res.success = true; res.message = "target reset to measured tool";
  }
  catch (const std::exception& e) { res.success = false; res.message = e.what(); }
  return true;
}

void MotionCore::update(const ros::TimerEvent&)
{
  try
  {
    MotionContext ctx; std::string error;
    if (!measurements(ctx.state, error)) { diagnostic(false, error); return; }
    commands_.setMeasuredPose(ctx.state.tool, ctx.state.stamp);
    const auto now = ros::Time::now();
    if (publish_commands_ && (inhibited_ || flight_state_ != hover_state_ || !fresh(flight_stamp_, now, timeout_)))
    {
      last_solved_stamp_ = ros::Time{}; admittance_.reset();
      diagnostic(false, inhibited_ ? "legacy motion inhibits commands" : "waiting for fresh HOVER flight state"); return;
    }
    // No second solve on the same sensor sample, including after infeasibility.
    if (ctx.state.stamp == last_solved_stamp_) return;
    ctx.dt = last_solved_stamp_.isZero() ? 1 / rate_ : (ctx.state.stamp - last_solved_stamp_).toSec();
    last_solved_stamp_ = ctx.state.stamp;
    if (!validStep(ctx.dt) || ctx.dt > max_dt_)
    {
      admittance_.reset(); solver_.reset(); diagnostic(false, "invalid state update dt; holding reference"); return;
    }
    ctx.target = admittance_.update(commands_.target(), ctx.state.tool, ctx.dt, now, ctx.state.stamp);
    ctx.target_twist = admittance_.targetTwist();
    ctx.contact_active = perched_; ctx.locked_contact_position = locked_position_;
    ctx.locked_hinge_world = locked_axis_; ctx.hinge_contact = hinge_local_;
    QPProblem problem; problem.reset(model_.info().dimension(), regularization_);
    for (const auto& cost : costs_) if (!cost->update(ctx, problem)) throw std::runtime_error(cost->name() + " update failed");
    for (const auto& constraint : constraints_) if (!constraint->update(ctx, problem)) throw std::runtime_error(constraint->name() + " update failed");
    if (!bridge_.fullAttitude())
    {
      // Ordinary xyz/yaw navigators cannot track independently requested body
      // tilt. Include joint contributions when baselink is not the KDL root.
      const auto jac = model_.frameJacobian(ctx.state, model_.baselink(), Eigen::Isometry3d::Identity());
      problem.appendConstraints(jac.block(3, 0, 2, problem.g.size()), Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero());
    }
    Eigen::VectorXd velocity; std::string status;
    const auto start = ros::WallTime::now();
    const bool valid = solver_.solve(problem, velocity, status);  // Exactly one QP call.
    const double seconds = (ros::WallTime::now() - start).toSec();
    if (seconds > 1 / rate_) ROS_WARN_THROTTLE(2.0, "QP exceeded configured update period: %.4f s", seconds);
    if (!valid) { diagnostic(false, status, &ctx, &problem, nullptr, seconds); return; }
    WholeBodyReference reference;
    reference.state = model_.integrate(ctx.state, velocity, ctx.dt, ctx.state.stamp + ros::Duration(ctx.dt));
    reference.velocity = velocity; reference.valid = true; reference.status = status;
    const auto& info = model_.info();
    if ((reference.state.joints.array() < info.lower.array() - 1e-8).any() ||
        (reference.state.joints.array() > info.upper.array() + 1e-8).any())
      throw std::runtime_error("integrated reference exceeds mechanical joint limits");
    if (perched_)
    {
      const double pos_error = (reference.state.contact.translation() - locked_position_).norm();
      const double angle_error = std::acos(std::clamp((reference.state.contact.linear() * hinge_local_).dot(locked_axis_), -1.0, 1.0));
      if (pos_error > contact_tolerance_ || angle_error > contact_angle_tolerance_)
        throw std::runtime_error("integrated reference exceeds nonlinear contact tolerance");
    }
    bridge_.publish(reference, publish_commands_);
    diagnostic(true, status, &ctx, &problem, &velocity, seconds);
  }
  catch (const std::exception& e)
  {
    solver_.reset(); diagnostic(false, e.what());
  }
}

void MotionCore::diagnostic(bool valid, const std::string& text, const MotionContext* ctx,
    const QPProblem* problem, const Eigen::VectorXd* solution, double seconds)
{
  diagnostic_msgs::DiagnosticArray msg; msg.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus status;
  status.name = pnh_.getNamespace(); status.hardware_id = model_.info().root_link;
  status.level = valid ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::WARN;
  status.message = text;
  const auto add = [&status](const std::string& key, const auto& value) {
    std::ostringstream stream; stream << value;
    diagnostic_msgs::KeyValue entry; entry.key = key; entry.value = stream.str(); status.values.push_back(entry);
  };
  add("valid", valid); add("commands_enabled", publish_commands_); add("variables", model_.info().dimension());
  add("constraints", problem ? problem->A.rows() : 0); add("solve_seconds", seconds);
  add("perched", perched_); add("admittance_enabled", admittance_.enabled());
  std::string costs, constraints;
  for (const auto& p : costs_) costs += p->name() + " ";
  for (const auto& p : constraints_) constraints += p->name() + " ";
  add("cost_plugins", costs); add("constraint_plugins", constraints);
  if (ctx)
  {
    add("tool_error", poseError(ctx->target, ctx->state.tool).transpose());
    if (solution)
    {
      add("achieved_tool_twist", (ctx->state.tool_jacobian * *solution).transpose());
      for (const auto& cost : costs_)
      {
        const auto residual = cost->residual(*ctx, *solution);
        if (residual.size()) add(cost->name() + "/task_residual", residual.transpose());
      }
    }
    if (perched_)
    {
      add("locked_contact_world", locked_position_.transpose()); add("locked_hinge_world", locked_axis_.transpose());
      add("contact_position_error", (ctx->state.contact.translation() - locked_position_).norm());
      add("contact_angle_error", std::acos(std::clamp((ctx->state.contact.linear() * hinge_local_).dot(locked_axis_), -1.0, 1.0)));
    }
  }
  msg.status.push_back(status); diagnostics_.publish(msg);
  if (!valid) ROS_WARN_THROTTLE(2.0, "Motion reference held: %s", text.c_str());
}
}
