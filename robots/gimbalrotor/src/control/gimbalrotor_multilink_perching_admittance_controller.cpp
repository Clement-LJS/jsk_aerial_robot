#include <gimbalrotor/control/gimbalrotor_multilink_perching_admittance_controller.h>
#include <pluginlib/class_list_macros.h>
#include <stdexcept>

namespace aerial_robot_control
{
void GimbalrotorMultilinkPerchingAdmittanceController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double rate)
{
  GimbalrotorPerchingAdmittanceController::initialize(nh, nhp, model, estimator, navigator, rate);
  articulated_navigator_ = dynamic_cast<aerial_robot_navigation::GimbalrotorMultilinkPerchingNavigator*>(navigator.get());
  if(!articulated_navigator_) throw std::invalid_argument("articulated controller requires the multilink perching navigator");
  articulated_model_ = boost::dynamic_pointer_cast<GimbalrotorMultilinkPerchingModel>(model);
  if(!articulated_model_) throw std::invalid_argument("articulated controller requires the shared articulated model");
  nh_.param("navigation/perching/dynamics/observer_gain", observer_gain_, observer_gain_);
  nh_.param("navigation/perching/feedback_timeout", feedback_timeout_, feedback_timeout_);
  if(!std::isfinite(observer_gain_) || observer_gain_ <= 0 || !std::isfinite(feedback_timeout_) || feedback_timeout_ <= 0)
    throw std::invalid_argument("invalid articulated observer gain/timeout");
  std::string topic, effort_topic, joint_topic;
  nh_.param<std::string>("controller/command_wrench_topic", topic, "perching/roll_yaw/command_wrench_estimate");
  nh_.param<std::string>("navigation/perching/joint_state_topic", joint_topic, "joint_states");
  nh_.param<std::string>("navigation/perching/dynamics/joint_effort/topic", effort_topic, joint_topic);
  if(nh_.resolveName(effort_topic) != nh_.resolveName(joint_topic))
    throw std::invalid_argument("observer effort topic must match synchronized joint feedback");
  command_wrench_sub_ = nh_.subscribe(topic, 64, &GimbalrotorMultilinkPerchingAdmittanceController::commandWrenchCallback, this);
}

void GimbalrotorMultilinkPerchingAdmittanceController::commandWrenchCallback(const ros::MessageEvent<geometry_msgs::WrenchStamped const>& event)
{
  if(event.getPublisherName() != ros::this_node::getName()) return;
  const auto& msg = event.getMessage();
  CommandSample sample;
  sample.stamp = msg->header.stamp.toSec();
  sample.receipt = ros::SteadyTime::now().toSec();
  sample.wrench << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
      msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
  const double now = ros::Time::now().toSec();
  if(msg->header.frame_id != world_ || !sample.wrench.allFinite() || sample.stamp <= 0 ||
      now - sample.stamp < -future_tolerance_ || now - sample.stamp > feedback_timeout_) return;
  std::lock_guard<std::mutex> lock(command_mutex_);
  if(!command_history_.empty() && command_history_.back().stamp > now + future_tolerance_) command_history_.clear();
  if(!command_history_.empty() && sample.stamp <= command_history_.back().stamp) return;
  command_history_.push_back(sample);
  while(command_history_.size() > 64) command_history_.pop_front();
}

bool GimbalrotorMultilinkPerchingAdmittanceController::externalLoad(Eigen::VectorXd& load, Eigen::VectorXd& tangent, double& stamp)
{
  const auto session = articulated_navigator_->perchingSession();
  const double now = ros::Time::now().toSec();
  reason_ = "awaiting locked articulated model";
  if(session.lock_stamp.isZero() || !articulated_model_->locked()) return false;
  if(observer_lock_stamp_ != session.lock_stamp || observation_stamp_ > now + future_tolerance_)
  {
    observation_filter_.reset();
    observer_.reset();
    observation_stamp_ = published_observer_stamp_ = 0;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_history_.clear();
    }
    if(!articulated_model_->dynamics(dynamics_, reason_)) return false;
    observer_lock_stamp_ = session.lock_stamp;
  }
  aerial_robot_navigation::GimbalrotorPerchingNavigator::Sample sample;
  reason_ = "awaiting fresh synchronized measured body/joints";
  if(!articulated_navigator_->observation(sample) || sample.stamp < session.lock_stamp.toSec() ||
      sample.stamp < observation_stamp_ || sample.q.size() != static_cast<int>(dynamics_.coordinates().size()) ||
      !sample.q.allFinite() || !sample.rate.allFinite() || !sample.omega.allFinite()) return false;
  reason_ = "missing/nonfinite synchronized nominal joint effort";
  if(!sample.effort_valid || sample.effort.size() != sample.q.size() || !sample.effort.allFinite()) return false;
  CommandSample command;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    for(auto it = command_history_.rbegin(); it != command_history_.rend(); ++it)
      if(it->stamp <= sample.stamp) { command = *it; break; }
  }
  reason_ = "missing/stale world/CoG command estimate preceding observation";
  const double receipt_age = ros::SteadyTime::now().toSec() - command.receipt;
  if(command.stamp < session.lock_stamp.toSec() || command.receipt <= 0 ||
      sample.stamp - command.stamp > observer_max_interval_ || now - command.stamp > feedback_timeout_ ||
      receipt_age < 0 || (!ros::Time::isSimTime() && receipt_age > feedback_timeout_)) return false;
  if(sample.stamp > observation_stamp_)
  {
    observation_stamp_ = sample.stamp;
    if(!observation_filter_.observe(sample.q, sample.body.linear(), sample.omega, sample.effort,
                                    command.wrench, sample.stamp, observer_max_interval_))
    {
      observer_.reset("observer warmup: measured derivatives settling after initialization/gap");
      reason_ = observer_.reason();
      return false;
    }
    GimbalrotorPerchingModel::State state;
    reason_ = "invalid filtered articulated geometry";
    if(!articulated_model_->evaluate(observation_filter_.q, state))
    { observer_.reset(reason_); return false; }
    const Eigen::Vector3d cog_offset_body = state.body.linear().transpose() * (state.cog.translation() - state.body.translation());
    perching_dynamics::MultilinkModel::Evaluation evaluation;
    // evaluate shifts this CoG command to the body material origin exactly once.
    if(!dynamics_.evaluate(observation_filter_.q, observation_filter_.rate, observation_filter_.effort,
                          observation_filter_.wrench.head<3>(), observation_filter_.wrench.tail<3>(),
                          cog_offset_body, evaluation, reason_))
    { observer_.reset(reason_); return false; }
    observer_.observe(evaluation.momentum, evaluation.predicted_rate, sample.stamp, observer_gain_, observer_max_interval_);
  }
  reason_ = observer_.reason();
  if(!observer_.ready() || observer_.stamp() != sample.stamp) return false;
  // Model evaluation must not turn an admitted measurement into a stale output.
  reason_ = "stale articulated observer result";
  if(ros::Time::now().toSec() - sample.stamp > feedback_timeout_ ||
      (!ros::Time::isSimTime() && ros::SteadyTime::now().toSec() - command.receipt > feedback_timeout_)) return false;
  raw_load_valid_ = true;
  load = observer_.load();
  stamp = observer_.stamp();
  if(stamp > published_observer_stamp_)
  {
    sensor_msgs::JointState msg;
    msg.header.stamp.fromSec(stamp);
    msg.name = dynamics_.coordinates();
    msg.effort.assign(load.data(), load.data() + load.size());
    generalized_load_pub_.publish(msg);
    published_observer_stamp_ = stamp;
  }
  reason_ = "body-local +Y pitch tangent unavailable; generalized observer ready";
  if(!articulated_navigator_->pitchTangent(observation_filter_.q, articulated_navigator_->movingCoordinates(), tangent) ||
      tangent.size() != load.size() || !tangent.allFinite()) return false;
  reason_ = "articulated momentum observer ready (commanded actuation estimate)";
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorMultilinkPerchingAdmittanceController,
                       aerial_robot_control::ControlBase)
