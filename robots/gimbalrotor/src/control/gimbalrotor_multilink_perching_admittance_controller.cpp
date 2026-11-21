#include <gimbalrotor/control/gimbalrotor_multilink_perching_admittance_controller.h>
#include <pluginlib/class_list_macros.h>
#include <chrono>

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
  std::string command_topic;
  nh_.param<std::string>("controller/command_wrench_topic", command_topic, "perching/command_wrench_estimate");
  // This input must be effective actuation at CoG in world axes, including
  // Spinal feedback/saturation. Commanded acceleration is not this telemetry.
  // Leave the observer input disconnected until that producer and synchronized
  // calibrated joint torque are available. Nominal PID needs neither input.
  ROS_WARN_STREAM("Multilink admittance disabled: no verified effective world/CoG wrench producer on " << command_topic
                  << " or calibrated synchronized actuator torque. Nominal PID remains available.");
}

void GimbalrotorMultilinkPerchingAdmittanceController::reset()
{
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  GimbalrotorPerchingAdmittanceController::reset();
  observer_.reset();
  observation_filter_.reset();
  filtered_stamp_ = observation_start_ = 0;
  observation_tangent_.resize(0);
  commands_.clear();
  dynamics_valid_ = false;
}

void GimbalrotorMultilinkPerchingAdmittanceController::controlCore()
{
  std::lock_guard<std::recursive_mutex> navigation_lock(perching_navigator_->perchingStateMutex());
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  const auto session = perching_navigator_->perchingSession();
  if(session.mode != session_.mode || session.lock_stamp != session_.lock_stamp || ros::Time::now() < prev_admittance_time_)
  {
    observer_.reset();
    observation_filter_.reset();
    filtered_stamp_ = 0;
    observation_start_ = ros::Time::now().toSec();
    commands_.clear();
    dynamics_valid_ = !session.lock_stamp.isZero() && articulated_model_->locked() && articulated_model_->dynamics(dynamics_, reason_);
  }
  // The parent advances tare/filter/scalar admittance and accepts one reference,
  // then runs the ordinary PID/allocation. update() dispatches once.
  GimbalrotorPerchingAdmittanceController::controlCore();
}

void GimbalrotorMultilinkPerchingAdmittanceController::commandWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
  if(msg->header.frame_id != world_ || msg->header.stamp.isZero()) return;
  Command command;
  command.stamp = msg->header.stamp.toSec();
  command.receipt = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  const auto& f = msg->wrench.force; const auto& m = msg->wrench.torque;
  command.wrench << f.x, f.y, f.z, m.x, m.y, m.z;
  if(!commands_.empty() && ros::Time::now().toSec() < commands_.back().stamp) commands_.clear();
  if(!command.wrench.allFinite() || (!commands_.empty() && command.stamp <= commands_.back().stamp)) return;
  commands_.push_back(command); while(commands_.size() > 64) commands_.pop_front();
}

bool GimbalrotorMultilinkPerchingAdmittanceController::externalLoad(Eigen::VectorXd& load, Eigen::VectorXd& tangent, double& stamp)
{
  aerial_robot_navigation::GimbalrotorPerchingNavigator::Sample sample;
  auto fail = [&](const std::string& reason)
  { reason_ = reason; observer_.reset(reason); return false; };
  if(!command_sub_) return fail("contact admittance disabled: effective CoG wrench and synchronized calibrated joint torque unavailable");
  if(!dynamics_valid_ || !articulated_model_->locked() || !perching_navigator_->observation(sample) || !sample.effort_valid)
    return fail("awaiting synchronized motion and named actuator efforts");
  if(sample.stamp < observation_start_) return false;
  const Command* command = nullptr;
  for(const auto& candidate : commands_) if(candidate.stamp <= sample.stamp) command = &candidate;
  if(!command || command->stamp < observation_start_ || sample.stamp - command->stamp > synchronization_tolerance_)
    return fail("awaiting effective world/CoG wrench; current Spinal bridge has no producer");
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if(!ros::Time::isSimTime() && wall - command->receipt > external_wrench_timeout_) return fail("stale command wrench receipt");
  if(sample.stamp > filtered_stamp_)
  {
    filtered_stamp_ = sample.stamp;
    if(!observation_filter_.observe(sample.q, sample.body.linear(), sample.omega, sample.effort, command->wrench,
                                   sample.stamp, observer_max_interval_)) return fail("conditioning measured coordinate rates");
    perching_dynamics::MultilinkModel::Evaluation evaluation;
    GimbalrotorPerchingModel::State state;
    if(!articulated_model_->evaluate(observation_filter_.q, state)) return fail("invalid synchronized wrench origin");
    // The input is a world wrench at CoG. Freeze B->CoG at the same filtered
    // configuration as momentum; evaluate() shifts its moment to B once.
    const Eigen::Vector3d offset = state.body.linear().transpose() * (state.cog.translation() - state.body.translation());
    if(!dynamics_.evaluate(observation_filter_.q, observation_filter_.rate, observation_filter_.effort,
        observation_filter_.wrench.head<3>(), observation_filter_.wrench.tail<3>(), offset, evaluation, reason_))
      return fail(reason_);
    if(!observer_.observe(evaluation.momentum, evaluation.predicted_rate, sample.stamp, observer_gain_, observer_max_interval_))
    { reason_ = observer_.reason(); return false; }
    // Projection and momentum use the same filtered observation configuration.
    if(!articulated_navigator_->pitchTangent(observation_filter_.q,
        articulated_navigator_->movingCoordinates(), observation_tangent_)) observation_tangent_.resize(0);
    sensor_msgs::JointState raw;
    raw.header.stamp.fromSec(observer_.stamp()); raw.header.frame_id = "perching_generalized";
    raw.name = articulated_model_->coordinates();
    raw.position.assign(observation_filter_.q.data(), observation_filter_.q.data() + observation_filter_.q.size());
    raw.effort.assign(observer_.load().data(), observer_.load().data() + observer_.load().size());
    generalized_load_pub_.publish(raw);
  }
  if(!observer_.ready() || ros::Time::now().toSec() - observer_.stamp() > external_wrench_timeout_) return false;
  if(observation_tangent_.size() != observer_.load().size() || !observation_tangent_.allFinite())
  { reason_ = "body pitch projection unavailable"; return false; }
  // t = dq/ds for body-local +Y pitch: t^T Q is N.m per radian of body
  // pitch. The shared contact gate applies the configured torque sign once.
  load = observer_.load(); tangent = observation_tangent_; stamp = observer_.stamp();
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorMultilinkPerchingAdmittanceController,
                       aerial_robot_control::ControlBase)
