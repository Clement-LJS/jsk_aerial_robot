#include <gimbalrotor/control/gimbalrotor_multilink_perching_admittance_controller.h>
#include <pluginlib/class_list_macros.h>
#include <Eigen/SVD>

namespace aerial_robot_control
{
void GimbalrotorMultilinkPerchingAdmittanceController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double rate)
{
  GimbalrotorPerchingAdmittanceController::initialize(nh, nhp, model, estimator, navigator, rate);
  articulated_model_ = boost::dynamic_pointer_cast<GimbalrotorMultilinkPerchingModel>(model);
  if(!articulated_model_) throw std::invalid_argument("articulated controller requires the shared articulated model");
  std::string command_topic, joint_topic;
  nh_.param<std::string>("controller/command_wrench_topic", command_topic, "perching/command_wrench_estimate");
  nh_.param<std::string>("navigation/perching/joint_command_topic", joint_topic, "joints_ctrl");
  command_sub_ = nh_.subscribe(command_topic, 20, &GimbalrotorMultilinkPerchingAdmittanceController::commandWrenchCallback, this);
  joints_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_topic, 1);
}

void GimbalrotorMultilinkPerchingAdmittanceController::reset()
{
  GimbalrotorPerchingAdmittanceController::reset();
  observer_.reset();
  observation_filter_.reset();
  dynamics_valid_ = false;
  joint_pids_.clear();
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
    dynamics_valid_ = !session.lock_stamp.isZero() && articulated_model_->dynamics(dynamics_, reason_);
    const auto count = perching_navigator_->movingCoordinates().size();
    joint_pids_.assign(count > 0 ? count - 1 : 0, pid_controllers_.at(PITCH));
    for(auto& pid : joint_pids_)
    {
      pid.reset();
      if(pitch_i_limit_suppressed_) pid.setLimitI(normal_pitch_i_limit_);
    }
  }
  // The parent advances tare/filter/scalar admittance once, then invokes our
  // coordinated feedback and the common allocator. update() dispatches once.
  GimbalrotorPerchingAdmittanceController::controlCore();
}

void GimbalrotorMultilinkPerchingAdmittanceController::sendCmd()
{
  std::lock_guard<std::recursive_mutex> lock(perching_navigator_->perchingStateMutex());
  sensor_msgs::JointState command;
  if(perching_navigator_->jointCommand(command)) joints_pub_.publish(command);
  GimbalrotorPerchingAdmittanceController::sendCmd();
}

bool GimbalrotorMultilinkPerchingAdmittanceController::feedbackWrench(Eigen::Matrix<double, 6, 1>& wrench)
{
  aerial_robot_navigation::GimbalrotorPerchingNavigator::Sample measured;
  if(!perching_navigator_->observation(measured) || !measured.effort_valid || !measured.rate.allFinite()) return false;
  const auto& reference = perching_navigator_->appliedCoordinates();
  const auto& reference_rate = perching_navigator_->appliedRates();
  const auto& reference_acceleration = perching_navigator_->appliedAccelerations();
  Eigen::MatrixXd angular;
  if(reference.size() != measured.q.size() || reference_rate.size() != reference.size() ||
     reference_acceleration.size() != reference.size() ||
     joint_pids_.size() + 1 != static_cast<std::size_t>(reference.size()) || !articulated_model_->jacobian(measured.q, angular)) return false;
  const double elapsed = ros::Time::now().toSec() - control_timestamp_;
  const double dt = elapsed > 0 && elapsed <= .1 ? elapsed : 0;
  // Form errors in permitted coordinates first: supported Cartesian residuals
  // never enter any PID. Row zero is body-local pitch; the other rows hold or
  // move the named actuators, including coordinates outside the pitch task.
  Eigen::MatrixXd task = Eigen::MatrixXd::Identity(reference.size(), reference.size());
  task.row(0) = angular.row(1);
  const Eigen::VectorXd error = task * (reference - measured.q);
  const Eigen::VectorXd rate_error = task * (reference_rate - measured.rate);
  Eigen::VectorXd acceleration(reference.size());
  auto& pitch = pid_controllers_.at(PITCH);
  pitch.update(error[0], dt, rate_error[0]);
  acceleration[0] = pitch.result();
  for(int i = 1; i < reference.size(); ++i)
  {
    joint_pids_[i - 1].update(error[i], dt, rate_error[i]);
    acceleration[i] = joint_pids_[i - 1].result();
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(task, Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(1e-6);
  const Eigen::VectorXd coordinated = svd.solve(acceleration);
  if((task * coordinated - acceleration).norm() > 1e-5) return false;
  return articulated_model_->effort(measured.q, measured.rate, coordinated + reference_acceleration, measured.effort, wrench);
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
  const auto body = estimator_->getPos(Frame::BASELINK, estimate_mode_);
  const auto cog = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::vectorTFToEigen(estimator_->getOrientation(Frame::BASELINK, estimate_mode_).inverse() * (cog - body), command.offset);
  if(!commands_.empty() && ros::Time::now().toSec() < commands_.back().stamp) commands_.clear();
  if(!command.wrench.allFinite() || !command.offset.allFinite() || (!commands_.empty() && command.stamp <= commands_.back().stamp)) return;
  commands_.push_back(command); while(commands_.size() > 64) commands_.pop_front();
}

bool GimbalrotorMultilinkPerchingAdmittanceController::externalLoad(Eigen::VectorXd& load, Eigen::VectorXd& tangent, double& stamp)
{
  aerial_robot_navigation::GimbalrotorPerchingNavigator::Sample sample;
  auto fail = [&](const std::string& reason)
  { reason_ = reason; observer_.reset(reason); return false; };
  if(!dynamics_valid_ || !perching_navigator_->observation(sample) || !sample.effort_valid)
    return fail("awaiting synchronized motion and named actuator efforts");
  if(sample.stamp < observation_start_) return false;
  const Command* command = nullptr;
  for(const auto& candidate : commands_) if(candidate.stamp <= sample.stamp) command = &candidate;
  if(!command || command->stamp < observation_start_ || sample.stamp - command->stamp > synchronization_tolerance_)
    return fail("awaiting synchronized command wrench");
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  if(!ros::Time::isSimTime() && wall - command->receipt > external_wrench_timeout_) return fail("stale command wrench receipt");
  if(sample.stamp > filtered_stamp_)
  {
    filtered_stamp_ = sample.stamp;
    if(!observation_filter_.observe(sample.q, sample.body.linear(), sample.omega, sample.effort, command->wrench,
                                   sample.stamp, observer_max_interval_)) return fail("conditioning measured coordinate rates");
    perching_dynamics::MultilinkModel::Evaluation evaluation;
    if(!dynamics_.evaluate(observation_filter_.q, observation_filter_.rate, observation_filter_.effort,
        observation_filter_.wrench.head<3>(), observation_filter_.wrench.tail<3>(), command->offset, evaluation, reason_))
      return fail(reason_);
    if(!observer_.observe(evaluation.momentum, evaluation.predicted_rate, sample.stamp, observer_gain_, observer_max_interval_)) return false;
    // Projection and momentum use the same filtered observation configuration.
    if(!perching_navigator_->mechanism().pitchTangent(observation_filter_.q,
        perching_navigator_->movingCoordinates(), observation_tangent_)) observation_tangent_.resize(0);
    sensor_msgs::JointState raw;
    raw.header.stamp.fromSec(observer_.stamp()); raw.header.frame_id = "perching_generalized";
    raw.name = perching_navigator_->mechanism().coordinates();
    raw.position.assign(observation_filter_.q.data(), observation_filter_.q.data() + observation_filter_.q.size());
    raw.effort.assign(observer_.load().data(), observer_.load().data() + observer_.load().size());
    generalized_load_pub_.publish(raw);
  }
  if(!observer_.ready() || ros::Time::now().toSec() - observer_.stamp() > external_wrench_timeout_) return false;
  load = observer_.load(); tangent = observation_tangent_; stamp = observer_.stamp();
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorMultilinkPerchingAdmittanceController,
                       aerial_robot_control::ControlBase)
