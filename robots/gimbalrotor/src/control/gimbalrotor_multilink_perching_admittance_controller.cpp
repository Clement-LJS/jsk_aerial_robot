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
  std::string effort_topic, joint_topic;
  nh_.param<std::string>("navigation/perching/joint_state_topic", joint_topic, "joint_states");
  nh_.param<std::string>("navigation/perching/dynamics/joint_effort/topic", effort_topic, joint_topic);
  if(nh_.resolveName(effort_topic) != nh_.resolveName(joint_topic))
    throw std::invalid_argument("observer effort topic must match synchronized joint feedback");
}

void GimbalrotorMultilinkPerchingAdmittanceController::reset()
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  GimbalrotorPerchingAdmittanceController::reset();
  ++observation_epoch_;
  observation_worker_.cancel();
  observation_filter_.reset();
  observer_.reset();
  observer_lock_stamp_ = ros::Time();
  observation_stamp_ = published_observer_stamp_ = 0;
  observation_model_.reset();
  observation_result_.reset();
}

bool GimbalrotorMultilinkPerchingAdmittanceController::externalLoad(Eigen::VectorXd& load, Eigen::VectorXd& tangent, double& stamp)
{
  const auto session = articulated_navigator_->perchingSession();
  const double now = ros::Time::now().toSec();
  reason_ = "awaiting locked articulated model";
  if(session.lock_stamp.isZero() || !articulated_model_->locked()) return false;
  if(!observation_model_ || observer_lock_stamp_ != session.lock_stamp || observation_stamp_ > now + future_tolerance_)
  {
    ++observation_epoch_;
    observation_worker_.cancel();
    observation_filter_.reset();
    observer_.reset();
    observer_lock_stamp_ = ros::Time();
    observation_stamp_ = published_observer_stamp_ = 0;
    observation_model_.reset();
    observation_result_.reset();
    auto model = std::make_shared<perching_dynamics::MultilinkModel>();
    if(!articulated_model_->dynamics(*model, reason_)) return false;
    observation_model_ = std::move(model);
    observer_lock_stamp_ = session.lock_stamp;
  }
  const auto fresh = [this](const ObservationResult& result)
  {
    const double now = ros::Time::now().toSec();
    const double receipt_age = ros::SteadyTime::now().toSec() - result.receipt;
    return std::isfinite(result.stamp) && result.stamp > 0 && result.command_stamp > 0 &&
        result.command_stamp <= result.stamp && now - result.stamp >= -future_tolerance_ &&
        now - result.stamp <= feedback_timeout_ && result.stamp - result.command_stamp <= observer_max_interval_ &&
        result.receipt > 0 && receipt_age >= 0 && (ros::Time::isSimTime() || receipt_age <= feedback_timeout_);
  };
  // Consume before submitting, so continuous input cannot starve completed work.
  if(auto result = observation_worker_.take())
  {
    reason_ = result->ok ? "stale/obsolete articulated observer result" : result->reason;
    if(result->ok && result->lock_stamp == session.lock_stamp && result->epoch == observation_epoch_ &&
        std::isfinite(result->stamp) && result->stamp >= session.lock_stamp.toSec() && result->stamp > observer_.stamp() &&
        result->stamp <= now + future_tolerance_ && result->command_stamp > 0 && result->command_stamp <= result->stamp &&
        result->stamp - result->command_stamp <= observer_max_interval_ &&
        result->q.size() == static_cast<int>(observation_model_->coordinates().size()) && result->q.allFinite() &&
        result->evaluation.momentum.size() == result->q.size() && result->evaluation.predicted_rate.size() == result->q.size() &&
        result->evaluation.momentum.allFinite() && result->evaluation.predicted_rate.allFinite())
    {
      // Preserve chronological observer history; freshness still gates use of the resulting load below.
      if(observer_.observe(result->evaluation.momentum, result->evaluation.predicted_rate, result->stamp,
                           observer_gain_, observer_max_interval_)) observation_result_ = std::move(result);
      else observation_result_.reset();
    }
  }
  aerial_robot_navigation::GimbalrotorPerchingNavigator::Sample sample;
  reason_ = "awaiting fresh synchronized measured body/joints and joint effort";
  const bool measured = articulated_navigator_->observation(sample) && sample.stamp >= session.lock_stamp.toSec() &&
      sample.stamp > observation_stamp_ && sample.q.size() == static_cast<int>(observation_model_->coordinates().size()) &&
      sample.q.allFinite() && sample.omega.allFinite() && sample.effort_valid &&
      sample.effort.size() == sample.q.size() && sample.effort.allFinite();
  CommandSample command;
  for(auto it = command_history_.rbegin(); it != command_history_.rend(); ++it)
    if(it->stamp <= sample.stamp) { command = *it; break; }
  if(measured) reason_ = "missing/stale world/CoG command estimate preceding observation";
  const double receipt_age = ros::SteadyTime::now().toSec() - command.receipt;
  if(measured && command.stamp >= session.lock_stamp.toSec() && command.receipt > 0 &&
      sample.stamp - command.stamp <= observer_max_interval_ && now - command.stamp <= feedback_timeout_ &&
      receipt_age >= 0 && (ros::Time::isSimTime() || receipt_age <= feedback_timeout_))
  {
    observation_stamp_ = sample.stamp;
    if(!observation_filter_.observe(sample.q, sample.body.linear(), sample.omega, sample.effort,
                                    command.wrench, sample.stamp, observer_max_interval_))
    {
      // A derivative warmup/gap retires old work, without touching completed tare.
      ++observation_epoch_;
      observation_worker_.cancel();
      observation_result_.reset();
      observer_.reset("observer warmup: measured derivatives settling after initialization/gap");
      reason_ = observer_.reason();
      return false;
    }
    GimbalrotorPerchingModel::State state;
    reason_ = "invalid filtered articulated geometry";
    if(!articulated_model_->evaluate(observation_filter_.q, state))
    { observation_result_.reset(); return false; }
    const Eigen::Vector3d cog_offset_body = state.body.linear().transpose() * (state.cog.translation() - state.body.translation());
    ObservationResult input;
    input.lock_stamp = session.lock_stamp;
    input.epoch = observation_epoch_;
    input.stamp = sample.stamp;
    input.command_stamp = command.stamp;
    input.receipt = sample.receipt;
    input.q = observation_filter_.q;
    observation_worker_.submit([input = std::move(input), model = observation_model_, rate = observation_filter_.rate,
                                effort = observation_filter_.effort, wrench = observation_filter_.wrench, cog_offset_body]() mutable
    {
      // Modeled world/CoG actuation; evaluate shifts its moment to the body once.
      input.ok = model->evaluate(input.q, rate, effort, wrench.head<3>(), wrench.tail<3>(), cog_offset_body,
                                 input.evaluation, input.reason);
      return std::move(input);
    });
    reason_ = observer_.reason();
  }
  if(!observation_result_ || !observer_.ready() || observer_.stamp() != observation_result_->stamp) return false;
  const bool tangent_valid = articulated_model_->pitchTangent(observation_result_->q, articulated_navigator_->movingCoordinates(), tangent) &&
      tangent.size() == observer_.load().size() && tangent.allFinite();
  // Recheck after live geometry work, before publishing or using this residual.
  reason_ = "stale articulated observer result";
  if(!fresh(*observation_result_)) return false;
  raw_load_valid_ = true;
  load = observer_.load();
  stamp = observer_.stamp();
  if(stamp > published_observer_stamp_)
  {
    sensor_msgs::JointState msg;
    msg.header.stamp.fromSec(stamp);
    msg.name = observation_model_->coordinates();
    msg.effort.assign(load.data(), load.data() + load.size());
    generalized_load_pub_.publish(msg);
    published_observer_stamp_ = stamp;
  }
  reason_ = "body-local +Y pitch tangent unavailable; generalized observer ready";
  if(!tangent_valid) return false;
  reason_ = "articulated momentum observer ready (commanded actuation estimate)";
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorMultilinkPerchingAdmittanceController,
                       aerial_robot_control::ControlBase)
