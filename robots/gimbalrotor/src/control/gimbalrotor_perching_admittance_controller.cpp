#include <gimbalrotor/control/gimbalrotor_perching_admittance_controller.h>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <iomanip>
#include <stdexcept>

namespace aerial_robot_control
{
void GimbalrotorPerchingAdmittanceController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double rate)
{
  GimbalrotorAdmittanceController::initialize(nh, nhp, model, estimator, navigator, rate);
  perching_navigator_ = dynamic_cast<aerial_robot_navigation::GimbalrotorPerchingNavigator*>(navigator.get());
  if(!perching_navigator_) throw std::invalid_argument("perching admittance requires the perching navigator");
  perching_model_ = boost::dynamic_pointer_cast<GimbalrotorPerchingModel>(model);
  if(!perching_model_) throw std::invalid_argument("perching controller requires the shared perching robot model");
  ros::NodeHandle pn(nh_, "controller/admittance/perching");
  pn.param("contact_torque_filter_alpha", contact_torque_filter_alpha_, contact_torque_filter_alpha_);
  pn.param("contact_on_threshold", contact_on_threshold_, contact_on_threshold_);
  pn.param("contact_off_threshold", contact_off_threshold_, contact_off_threshold_);
  pn.param("contact_on_duration", contact_on_duration_, contact_on_duration_);
  pn.param("contact_off_duration", contact_off_duration_, contact_off_duration_);
  pn.param("recovery_angle_epsilon", recovery_angle_epsilon_, recovery_angle_epsilon_);
  pn.param("recovery_rate_epsilon", recovery_rate_epsilon_, recovery_rate_epsilon_);
  pn.param("equilibrium_wrench_required_samples", equilibrium_wrench_required_samples_, equilibrium_wrench_required_samples_);
  pn.param("perching_pitch_torque_sign", perching_pitch_torque_sign_, perching_pitch_torque_sign_);
  pn.param("pitch_acceleration_limit", pitch_acceleration_limit_, pitch_acceleration_limit_);
  if(perching_navigator_->articulated())
  {
    pn.param("cutting_working_torque", cutting_working_torque_, cutting_working_torque_);
    pn.param("cutting_max_torque", cutting_max_torque_, cutting_max_torque_);
    pn.param("cutting_recovery_rate", cutting_recovery_rate_, cutting_recovery_rate_);
    pn.param("cutting_force_integral_time", cutting_force_integral_time_, cutting_force_integral_time_);
  }
  nh_.param("navigation/perching/observer_max_interval", observer_max_interval_, observer_max_interval_);
  nh_.param("navigation/perching/future_tolerance", future_tolerance_, future_tolerance_);
  world_ = perching_navigator_->worldFrame();
  nh_.param<std::string>("navigation/perching_pivot_wrench_frame_id", pivot_frame_, "perching_locked_wrench");
  pn.param<std::string>("perching_pivot_wrench_frame_id", pivot_frame_, pivot_frame_);
  std::string frame_prefix;
  nhp_.param<std::string>("tf_prefix", frame_prefix, "");
  pivot_frame_ = tf::resolve(frame_prefix, pivot_frame_);
  if(pivot_frame_ == tf::resolve(frame_prefix, "perching_pivot"))
    throw std::invalid_argument("perching_pivot is reserved for the robot-attached URDF marker");
  std::string enable_topic, tare_topic;
  pn.param<std::string>("perching_admittance_enable_topic", enable_topic, "perching/admittance_enable");
  pn.param<std::string>("tare_topic", tare_topic, perching_navigator_->prefix() + "/admittance_tare");
  perching_admittance_enable_sub_ = nh_.subscribe(enable_topic, 10,
      &GimbalrotorPerchingAdmittanceController::perchingAdmittanceEnableCallback, this);
  tare_sub_ = nh_.subscribe<std_msgs::Empty>(tare_topic, 10, [this](const std_msgs::EmptyConstPtr&)
  {
    std::lock_guard<std::recursive_mutex> navigation_lock(perching_navigator_->perchingStateMutex());
    std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
    ++tare_request_sequence_;
    if(!perching_active_ || !perching_navigator_->nominalHeld() || !perching_navigator_->trackingHeld())
    { reason_ = "tare refused: hold the locked nominal reference unloaded"; return; }
    resetCalibration();
  });
  // A single owner for ordinary-flight enabling too; remove the base subscription.
  admittance_enable_sub_.shutdown();
  normal_enable_sub_ = nh_.subscribe<std_msgs::Bool>(admittance_enable_topic_, 1, [this](const std_msgs::BoolConstPtr& m)
  {
    std::lock_guard<std::recursive_mutex> navigation_lock(perching_navigator_->perchingStateMutex());
    std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
    if(perching_navigator_->perchingSession().mode != perching_geometry::Mode::DISABLED)
      perchingAdmittanceEnableCallback(m);
    else normal_admittance_enabled_ = m->data;
  });
  contact_active_pub_ = nh_.advertise<std_msgs::Bool>("perching/contact_admittance/contact_active", 1);
  recovery_active_pub_ = nh_.advertise<std_msgs::Bool>("perching/contact_admittance/recovery_active", 1);
  pivot_torque_raw_pub_ = nh_.advertise<std_msgs::Float64>("perching/contact_admittance/pivot_torque_raw", 1);
  pivot_torque_filtered_pub_ = nh_.advertise<std_msgs::Float64>("perching/contact_admittance/pivot_torque_filtered", 1);
  pitch_offset_pub_ = nh_.advertise<std_msgs::Float64>("perching/contact_admittance/pitch_offset", 1);
  pivot_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("perching/estimated_external_wrench_pivot", 1);
  generalized_load_pub_ = nh_.advertise<sensor_msgs::JointState>(perching_navigator_->prefix() + "/generalized_external_load", 1);
  generalized_valid_pub_ = nh_.advertise<std_msgs::Bool>(perching_navigator_->prefix() + "/generalized_external_load_valid", 1);
  const double positive[] = {admittance_config_.rot_virtual_inertia.y(), admittance_config_.rot_damping.y(),
      admittance_config_.rot_stiffness.y(), admittance_config_.torque_limit.y(), admittance_config_.angle_offset_limit.y(),
      admittance_config_.angular_vel_offset_limit.y(), admittance_config_.torque_lpf_alpha, contact_torque_filter_alpha_,
      pitch_acceleration_limit_, observer_max_interval_, external_wrench_timeout_, recovery_angle_epsilon_, recovery_rate_epsilon_};
  for(double value : positive) if(!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid perching admittance configuration");
  if(!std::isfinite(future_tolerance_) || future_tolerance_ < 0 || contact_torque_filter_alpha_ > 1 ||
      admittance_config_.torque_lpf_alpha > 1 || equilibrium_wrench_required_samples_ < 1 ||
      !std::isfinite(contact_on_threshold_) || !std::isfinite(contact_off_threshold_) ||
      contact_off_threshold_ < 0 || contact_on_threshold_ <= contact_off_threshold_ ||
      !std::isfinite(contact_on_duration_) || contact_on_duration_ < 0 || !std::isfinite(contact_off_duration_) || contact_off_duration_ < 0 ||
      std::abs(perching_pitch_torque_sign_) != 1 || !std::isfinite(admittance_config_.torque_ref.y()) ||
      std::abs(admittance_config_.torque_ref.y()) > 1e-9)
    throw std::invalid_argument("invalid perching contact/tare parameters");
  if(cutting_working_torque_ != 0 || cutting_max_torque_ != 0 || cutting_recovery_rate_ != 0)
  {
    if(!std::isfinite(cutting_working_torque_) || cutting_working_torque_ < contact_on_threshold_ ||
        !std::isfinite(cutting_max_torque_) || cutting_max_torque_ <= cutting_working_torque_ ||
        !std::isfinite(cutting_recovery_rate_) || cutting_recovery_rate_ <= 0)
      throw std::invalid_argument("invalid articulated cutting load/rate parameters");
  }
  if(!std::isfinite(cutting_force_integral_time_) || cutting_force_integral_time_ < 0)
    throw std::invalid_argument("invalid cutting force integral time");
}

void GimbalrotorPerchingAdmittanceController::resetCalibration()
{
  perching_admittance_enabled_ = equilibrium_wrench_ready_ = contact_active_ = recovery_active_ = false;
  input_valid_ = output_accepted_ = raw_load_valid_ = false;
  cutting_force_bias_ = 0;
  cutting_motion_.reset();
  equilibrium_wrench_sample_count_ = 0; baseline_.resize(0); baseline_sum_.resize(0);
  contact_on_timer_ = contact_off_timer_ = residual_pitch_torque_raw_ = residual_pitch_torque_filtered_ = 0;
  prepared_perching_admittance_wrench_world_.setZero();
  previous_contact_gate_time_ = ros::Time(); last_tare_stamp_ = last_load_stamp_;
  reset_requested_ = true; ++tare_generation_; reason_ = "collecting unloaded baseline";
}

void GimbalrotorPerchingAdmittanceController::reset()
{
  std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
  if(pitch_i_limit_suppressed_) updatePitchIntegralPolicy(pid_controllers_.at(PITCH), false);
  GimbalrotorAdmittanceController::reset();
  resetCalibration();
  perching_active_ = false;
  session_ = {perching_geometry::Mode::DISABLED, ros::Time()};
}

void GimbalrotorPerchingAdmittanceController::perchingAdmittanceEnableCallback(const std_msgs::BoolConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> navigation_lock(perching_navigator_->perchingStateMutex());
  std::lock_guard<std::recursive_mutex> l(perching_state_mutex_);
  ++enable_request_sequence_;
  if(!msg->data) { resetCalibration(); return; }
  const auto session = perching_navigator_->perchingSession();
  if(!perching_active_ || !equilibrium_wrench_ready_ || reset_requested_ ||
     session.mode != session_.mode || session.lock_stamp != session_.lock_stamp || !perching_model_->locked() ||
     !perching_navigator_->motionFeedbackValid() ||
     !admittance_config_.use_admittance || admittance_config_.rot_enable.y() < .5)
  { reason_ = "enable refused: valid lock and completed unloaded tare required; enable explicitly later"; ROS_WARN_STREAM(reason_); return; }
  perching_admittance_enabled_ = true;
  reason_ = "armed; waiting for contact";
}

bool GimbalrotorPerchingAdmittanceController::externalLoad(Eigen::VectorXd& load, Eigen::VectorXd& tangent, double& stamp)
{
  const auto sample = getExternalWrenchSnapshot();
  const double now = ros::Time::now().toSec();
  stamp = sample.measurement_stamp.toSec();
  reason_ = "awaiting fresh world/CoG external wrench";
  if(!perching_model_->locked() || !sample.available || sample.measurement_stamp.isZero() || sample.receive_stamp.isZero() ||
      sample.frame_id != world_ || external_wrench_frame_ != world_ || !sample.raw_wrench.allFinite() ||
      stamp < session_.lock_stamp.toSec() || stamp < last_load_stamp_ ||
      now - sample.receive_stamp.toSec() < 0 || now - sample.receive_stamp.toSec() > external_wrench_timeout_ ||
      now - stamp < -future_tolerance_ || now - stamp > external_wrench_timeout_) return false;
  const auto& geometry = perching_navigator_->rigidLock();
  if(!geometry.valid) return false;
  const Eigen::Vector3d pivot = tfVector3ToEigen(geometry.pivot);
  const Eigen::Vector3d cog = tfVector3ToEigen(estimator_->getPos(Frame::COG, estimate_mode_));
  if(!cog.allFinite()) return false;
  load = sample.raw_wrench;
  load.tail<3>() = perching_dynamics::shiftMoment(load.tail<3>(), cog - pivot, load.head<3>());
  tangent = Eigen::VectorXd::Zero(6);
  tangent.tail<3>() = tfVector3ToEigen(geometry.axis);
  stamp = sample.measurement_stamp.toSec();
  const bool raw_valid = load.allFinite() && tangent.allFinite();
  if(raw_valid && stamp > last_load_stamp_)
  {
    geometry_msgs::WrenchStamped raw;
    raw.header.stamp.fromSec(stamp); raw.header.frame_id = pivot_frame_;
    Eigen::Matrix3d rotation;
    rotation.col(0) = tfVector3ToEigen(geometry.radial().normalized());
    rotation.col(1) = tfVector3ToEigen(geometry.axis);
    rotation.col(2) = rotation.col(0).cross(rotation.col(1));
    const Eigen::Matrix<double, 6, 1> wrench = perching_dynamics::rotateWrench(rotation.transpose(), load);
    raw.wrench.force.x = wrench[0]; raw.wrench.force.y = wrench[1]; raw.wrench.force.z = wrench[2];
    raw.wrench.torque.x = wrench[3]; raw.wrench.torque.y = wrench[4]; raw.wrench.torque.z = wrench[5];
    const Eigen::Quaterniond orientation(rotation);
    pivot_tf_.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()), geometry.pivot),
                                                 raw.header.stamp, world_, pivot_frame_));
    pivot_wrench_pub_.publish(raw);
  }
  raw_load_valid_ = raw_valid;
  if(raw_valid) reason_ = "world/CoG wrench projected at locked pivot";
  return raw_valid;
}

void GimbalrotorPerchingAdmittanceController::preparePerchingAdmittanceInput()
{
  prepared_perching_admittance_wrench_world_.setZero(); input_valid_ = false;
  Eigen::VectorXd load, tangent;
  double stamp = 0;
  raw_load_valid_ = false;
  const bool raw_valid = externalLoad(load, tangent, stamp);
  const bool fresh_load = raw_valid && stamp > last_load_stamp_;
  std_msgs::Bool valid; valid.data = perching_navigator_->articulated() && raw_load_valid_; generalized_valid_pub_.publish(valid);
  const double dt = previous_contact_gate_time_.isZero() ? 0 : stamp - previous_contact_gate_time_.toSec();
  if(fresh_load) previous_contact_gate_time_.fromSec(stamp);
  if(!raw_valid) previous_contact_gate_time_ = ros::Time();
  if(raw_valid)
  {
    last_load_stamp_ = std::max(last_load_stamp_, stamp);
    if(fresh_load && !perching_admittance_enabled_ && !equilibrium_wrench_ready_ && stamp > last_tare_stamp_ &&
        std::abs(admittance_output_.angle_offset_compliance.y()) <= recovery_angle_epsilon_ &&
        std::abs(admittance_output_.angular_vel_offset_compliance.y()) <= recovery_rate_epsilon_ &&
        perching_navigator_->nominalHeld() && perching_navigator_->trackingHeld() && perching_navigator_->motionFeedbackValid())
    {
      if(baseline_sum_.size() != load.size()) baseline_sum_ = Eigen::VectorXd::Zero(load.size());
      baseline_sum_ += load; ++equilibrium_wrench_sample_count_; last_tare_stamp_ = stamp;
      if(equilibrium_wrench_sample_count_ >= equilibrium_wrench_required_samples_)
      {
        baseline_ = baseline_sum_ / equilibrium_wrench_sample_count_;
        equilibrium_wrench_ready_ = baseline_.allFinite();
        if(equilibrium_wrench_ready_) ROS_INFO("Perching unloaded baseline ready; enable explicitly to arm");
      }
    }
    if(equilibrium_wrench_ready_ && baseline_.size() == load.size() && tangent.size() == load.size() && tangent.allFinite())
    {
      residual_pitch_torque_raw_ = perching_pitch_torque_sign_ * tangent.dot(load - baseline_);
      input_valid_ = std::isfinite(residual_pitch_torque_raw_);
    }
  }
  if(!perching_admittance_enabled_)
  {
    if(raw_valid) reason_ += equilibrium_wrench_ready_ ? "; unloaded baseline ready; awaiting explicit enable" : "; collecting unloaded baseline while held";
    return;
  }
  if(!input_valid_)
  {
    contact_active_ = false; contact_on_timer_ = contact_off_timer_ = 0; residual_pitch_torque_filtered_ = 0;
    recovery_active_ = true;
    reason_ += cutting_max_torque_ > 0 && perching_navigator_->pitchFeedStep() ?
        "; forward recovery held, tare and arming retained" : "; zero-input recovery, tare and arming retained";
    return;
  }
  if(fresh_load && dt > 0 && dt <= observer_max_interval_)
  {
    residual_pitch_torque_filtered_ = contact_torque_filter_alpha_ * residual_pitch_torque_raw_ +
        (1 - contact_torque_filter_alpha_) * residual_pitch_torque_filtered_;
    updateContactGate(residual_pitch_torque_filtered_, dt);
  }
  else if(fresh_load)
  {
    contact_on_timer_ = contact_off_timer_ = 0;
    contact_active_ = false;
    // Reacquired valid load must not look unloaded to the cutting speed limit.
    residual_pitch_torque_filtered_ = cutting_max_torque_ > 0 ? residual_pitch_torque_raw_ : 0;
  }
  if(contact_active_) prepared_perching_admittance_wrench_world_[4] = residual_pitch_torque_filtered_;
  if(cutting_max_torque_ > 0 && perching_navigator_->pitchFeedStep())
    reason_ += contact_active_ ? "; cutting load control; contact" : "; cutting load control; below contact gate";
  else reason_ += contact_active_ ? "; contact" : "; armed; zero-input recovery/hold";
}

void GimbalrotorPerchingAdmittanceController::controlCore()
{
  std::lock_guard<std::recursive_mutex> navigation_lock(perching_navigator_->perchingStateMutex());
  std::lock_guard<std::recursive_mutex> lock(perching_state_mutex_);
  const auto session = perching_navigator_->perchingSession();
  if(session.mode != session_.mode || session.lock_stamp != session_.lock_stamp || ros::Time::now() < prev_admittance_time_)
  {
    session_ = session;
    resetCalibration();
    admittance_output_ = AdmittanceCoreOutput();
    admittance_output_.valid = true;
    prev_admittance_time_ = ros::Time::now();
    last_load_stamp_ = last_tare_stamp_ = 0;
    pid_controllers_.at(PITCH).reset();
  }
  perching_active_ = session.mode != perching_geometry::Mode::DISABLED && !session.lock_stamp.isZero() &&
      navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE && !navigator_->getForceLandingFlag() && perching_model_->locked();
  const auto feed = perching_navigator_->pitchFeedStep();
  cutting_motion_.reset();
  if(perching_active_ && perching_admittance_enabled_ && cutting_max_torque_ > 0 && feed && *feed >= 0)
    cutting_motion_.emplace();
  if(!cutting_motion_ || *feed <= 0 || cutting_force_integral_time_ == 0 || perching_navigator_->jogReady() ||
      (perching_navigator_->nominalHeld() && perching_navigator_->appliedPitchOffset() == 0)) cutting_force_bias_ = 0;
  if(!perching_active_)
  {
    updatePitchIntegralPolicy(pid_controllers_.at(PITCH), false);
    admittance_enabled_ = normal_admittance_enabled_;
    if(reset_requested_) { admittance_core_.reset(); admittance_output_ = AdmittanceCoreOutput(); reset_requested_ = false; }
    admittance_core_.setConfig(admittance_config_);
    reason_ = "free flight; perching admittance disarmed";
    raw_load_valid_ = false;
    std_msgs::Bool valid; valid.data = false; generalized_valid_pub_.publish(valid);
    publishContactAdmittanceDiagnostics();
    GimbalrotorAdmittanceController::controlCore();
    return;
  }
  if(reset_requested_)
  {
    admittance_core_.reset();
    admittance_output_.angle_offset_compliance.y() = perching_navigator_->appliedPitchOffset();
    admittance_output_.angular_vel_offset_compliance.y() = perching_navigator_->appliedPitchOffsetRate();
    admittance_output_.angular_acc_offset_compliance.setZero();
    admittance_core_.acceptRotationalState(admittance_output_);
    reset_requested_ = false;
  }
  preparePerchingAdmittanceInput();
  admittance_enabled_ = perching_admittance_enabled_ && equilibrium_wrench_ready_;
  recovery_active_ = !contact_active_ &&
      (std::abs(perching_navigator_->appliedPitchOffset()) > recovery_angle_epsilon_ ||
       std::abs(perching_navigator_->appliedPitchOffsetRate()) > recovery_rate_epsilon_);
  publishContactAdmittanceDiagnostics();
  const double dt = (ros::Time::now() - prev_admittance_time_).toSec();
  if(std::isfinite(dt) && dt > 0 && dt <= observer_max_interval_ && perching_navigator_->motionFeedbackValid())
    updateAdmittance(); // Exactly one scalar integration per eligible control tick.
  else { prev_admittance_time_ = ros::Time::now(); admittance_output_.valid = false; }
  // Articulated PID tracks the accepted compliant attitude; retain its normal holding integral.
  const bool suppress = !perching_navigator_->articulated() && (contact_active_ || recovery_active_ ||
      std::abs(admittance_output_.angle_offset_compliance.y()) > recovery_angle_epsilon_);
  auto& pitch_pid = pid_controllers_.at(PITCH);
  updatePitchIntegralPolicy(pitch_pid, suppress);
  // Bypass the free-flight admittance integration; the pre-PID hook accepts
  // this cycle's scalar correction before the ordinary PID and allocation.
  GimbalrotorController::controlCore();
  if(suppress) pitch_pid.setErrI(0);
  recovery_active_ = !contact_active_ &&
      (std::abs(admittance_output_.angle_offset_compliance.y()) > recovery_angle_epsilon_ ||
       std::abs(admittance_output_.angular_vel_offset_compliance.y()) > recovery_rate_epsilon_);
  publishAdmittanceStateIfChanged();
}

void GimbalrotorPerchingAdmittanceController::preparePerchingReference()
{
  if(!perching_active_)
  { GimbalrotorController::preparePerchingReference(); return; }
  const double previous_rate = perching_navigator_->appliedPitchOffsetRate();
  const double dt = perching_navigator_->referenceTimeStep();
  const double limit = std::min(pitch_acceleration_limit_, perching_navigator_->pitchAccelerationLimit());
  // Even a rejected scalar proposal enters navigation once, allowing a bounded
  // brake preview and nominal progress without committing an invalid scalar.
  auto proposal = admittance_output_;
  if(!cutting_motion_ && perching_navigator_->articulated() && proposal.valid && !contact_active_ && dt > 0 &&
      std::abs(perching_navigator_->appliedPitchOffset()) <= std::min(recovery_angle_epsilon_, .5 * limit * dt * dt) &&
      std::abs(previous_rate) <= std::min(recovery_rate_epsilon_, .5 * limit * dt))
  {
    // Finish only when the last displacement and its subsequent stop fit the acceleration bound.
    proposal.angle_offset_compliance.y() = 0;
    proposal.angular_vel_offset_compliance.y() = 0;
    proposal.angular_acc_offset_compliance.y() = 0;
  }
  if(!proposal.valid && dt > 0)
  {
    proposal.angular_acc_offset_compliance.y() = std::clamp(-previous_rate / dt, -limit, limit);
  }
  applyAdmittanceOutputToNavigator(navigator_->getTargetPos(), navigator_->getTargetRPY(), proposal);
  if(!perching_navigator_->articulated())
  {
    // Preserve the rigid admittance feedforward without changing ordinary rigid navigation.
    const auto& geometry = perching_navigator_->rigidLock();
    const tf::Vector3 radial = navigator_->getTargetPos() - geometry.pivot;
    const tf::Vector3 tangent = geometry.axis.cross(radial);
    const double rate = perching_navigator_->rigidRate(), acceleration = perching_navigator_->rigidAcceleration();
    navigator_->setTargetVel(tangent * rate);
    navigator_->setTargetAcc(tangent * acceleration + geometry.axis.cross(tangent) * rate * rate);
    const tf::Matrix3x3 rotation(tf::createQuaternionFromYaw(navigator_->getTargetRPY().z()));
    navigator_->setTargetOmega(rotation.inverse() * geometry.axis * rate);
    navigator_->setTargetAngAcc(estimator_->getOrientation(Frame::COG, estimate_mode_).inverse() * geometry.axis * acceleration);
  }
  const double accepted_rate = perching_navigator_->appliedPitchOffsetRate();
  admittance_output_.angle_offset_compliance.y() = perching_navigator_->appliedPitchOffset();
  admittance_output_.angular_vel_offset_compliance.y() = accepted_rate;
  admittance_output_.angular_acc_offset_compliance.y() = dt > 0 ? (accepted_rate - previous_rate) / dt : 0;
  admittance_output_.valid = true;
  auto accepted = admittance_output_;
  if(cutting_motion_)
  {
    // The cutting core filters total motion; public diagnostics still describe the offset.
    accepted.angular_vel_offset_compliance.y() = perching_navigator_->appliedPitchRate();
    accepted.angular_acc_offset_compliance.y() = 0;
    const double step = perching_navigator_->pitchFeedStep().value_or(0);
    if(step <= 0 || perching_navigator_->jogReady() ||
        (perching_navigator_->nominalHeld() && perching_navigator_->appliedPitchOffset() == 0)) cutting_force_bias_ = 0;
    else if(proposal.valid && input_valid_ && output_accepted_ && contact_active_ &&
        residual_pitch_torque_filtered_ < 0 && cutting_force_integral_time_ > 0 && dt > 0)
    {
      const double increment = (cutting_working_torque_ + residual_pitch_torque_filtered_) * dt / cutting_force_integral_time_;
      const double blocked_rate = cutting_motion_->rate - accepted.angular_vel_offset_compliance.y();
      // Compare with the already-filtered proposal, not the steady PI demand (J/D lag is intentional).
      if((increment > 0 && !cutting_motion_->relief && blocked_rate <= 1e-6) || (increment < 0 && blocked_rate >= -1e-6))
      {
        const double bound = admittance_config_.rot_damping.y() * std::min(perching_navigator_->pitchRateLimit(), cutting_recovery_rate_);
        cutting_force_bias_ = std::clamp(cutting_force_bias_ + increment, 0.0, bound);
      }
    }
  }
  admittance_core_.acceptRotationalState(accepted);
  if(!output_accepted_) reason_ = "reference held; accepted scalar state retained";
  std_msgs::Float64 offset;
  offset.data = admittance_output_.angle_offset_compliance.y();
  pitch_offset_pub_.publish(offset);
}

void GimbalrotorPerchingAdmittanceController::updateAdmittance()
{
  const ros::Time now = ros::Time::now();
  const double elapsed = (now - prev_admittance_time_).toSec();
  prev_admittance_time_ = now;
  const double dt = perching_navigator_->referenceTimeStep();
  if(!std::isfinite(elapsed) || elapsed <= 0 || elapsed > observer_max_interval_ ||
      !std::isfinite(dt) || dt <= 0 || dt > observer_max_interval_)
  { admittance_output_.valid = false; return; }
  auto config = admittance_config_;
  config.trans_enable.setZero();
  config.rot_enable = Eigen::Vector3d::UnitY();
  if(cutting_motion_)
  {
    if(!input_valid_) { admittance_output_.valid = false; return; }
    const double bound = config.rot_damping.y() * std::min(perching_navigator_->pitchRateLimit(), cutting_recovery_rate_);
    cutting_force_bias_ = std::clamp(cutting_force_bias_, 0.0, bound);
    if(-residual_pitch_torque_filtered_ >= cutting_max_torque_) cutting_force_bias_ = 0;
    // J*v_dot + D*v = T_work - load + bias; v is total applied body-local pitch rate.
    config.rot_stiffness.y() = 0;
    config.torque_ref.y() = -(cutting_working_torque_ + cutting_force_bias_);
  }
  else if(cutting_max_torque_ > 0 && perching_admittance_enabled_ && input_valid_ && perching_navigator_->pitchFeedStep())
    config.torque_ref.y() = -cutting_working_torque_;
  admittance_core_.setConfig(config);
  // Seed every cycle from accepted state, including entry/exit between total-rate and offset-rate modes.
  auto accepted = admittance_output_;
  accepted.angle_offset_compliance.y() = perching_navigator_->appliedPitchOffset();
  accepted.angular_vel_offset_compliance.y() = cutting_motion_ ? perching_navigator_->appliedPitchRate() : perching_navigator_->appliedPitchOffsetRate();
  accepted.angular_acc_offset_compliance.setZero();
  admittance_core_.acceptRotationalState(accepted);
  AdmittanceCoreInput input;
  input.dt = dt;
  // Disarming removes the input torque; the applied offset still recovers through the spring/damper.
  input.enabled = true;
  input.R_world_compliance = perching_model_->pivot().linear();
  if(perching_navigator_->articulated())
  {
    aerial_robot_navigation::GimbalrotorPerchingNavigator::Sample sample;
    if(!perching_navigator_->observation(sample)) { admittance_output_.valid = false; return; }
    input.R_world_compliance = sample.body.linear();
  }
  // The generalized task torque acts along compliance +Y, not world +Y.
  const double torque = cutting_motion_ ? residual_pitch_torque_filtered_ : prepared_perching_admittance_wrench_world_[4];
  input.external_wrench_world.tail<3>() = input.R_world_compliance.col(1) * torque;
  admittance_output_ = admittance_core_.update(input);
  if(cutting_motion_ && admittance_output_.valid)
  {
    cutting_motion_->rate = admittance_output_.angular_vel_offset_compliance.y();
    // Velocity can remain negative while a nonnegative force drive brakes the retreat.
    const double drive = torque - config.torque_ref.y();
    cutting_motion_->relief = -residual_pitch_torque_filtered_ >= cutting_max_torque_ || drive < 0;
    // Offset output remains an offset; the distinct total-rate proposal travels in CuttingMotion.
    admittance_output_.angle_offset_compliance.y() = perching_navigator_->appliedPitchOffset();
    admittance_output_.angular_vel_offset_compliance.y() = perching_navigator_->appliedPitchOffsetRate();
    admittance_output_.angular_acc_offset_compliance.y() = 0;
  }
}

void GimbalrotorPerchingAdmittanceController::applyAdmittanceOutputToNavigator(
    const tf::Vector3& position, const tf::Vector3& rpy, const AdmittanceCoreOutput& output)
{
  if(!perching_active_)
  { GimbalrotorAdmittanceController::applyAdmittanceOutputToNavigator(position, rpy, output); return; }
  double forward_rate = INFINITY, recovery_rate = INFINITY;
  const auto feed = perching_navigator_->pitchFeedStep();
  if(perching_admittance_enabled_ && cutting_max_torque_ > 0 && feed)
  {
    // Cancellation or withdrawal retires forward recovery, without disarming load relief.
    recovery_rate = 0;
    const double step = *feed;
    if(step > 0 && input_valid_ && output.valid)
    {
      forward_rate = std::min(perching_navigator_->pitchRateLimit(), cutting_recovery_rate_);
      // Do not acknowledge a recovered step while relief is still requested.
      recovery_rate = cutting_motion_ && cutting_motion_->relief ? 0 : forward_rate;
    }
    else forward_rate = 0;
  }
  output_accepted_ = perching_navigator_->applyPerchingReference(output.angle_offset_compliance.y(),
      output.angular_vel_offset_compliance.y(), output.angular_acc_offset_compliance.y(), forward_rate, recovery_rate, cutting_motion_);
}
void GimbalrotorPerchingAdmittanceController::publishContactAdmittanceDiagnostics()
{
  std_msgs::Bool boolean; boolean.data = contact_active_; contact_active_pub_.publish(boolean);
  boolean.data = recovery_active_; recovery_active_pub_.publish(boolean);
  std_msgs::Float64 number; number.data = residual_pitch_torque_raw_; pivot_torque_raw_pub_.publish(number);
  number.data = residual_pitch_torque_filtered_; pivot_torque_filtered_pub_.publish(number);
  if(!perching_active_) { number.data = admittance_output_.angle_offset_compliance.y(); pitch_offset_pub_.publish(number); }
  std::map<std::string, std::string> values;
  auto value = [&](const std::string& key, auto data) { std::ostringstream s; s << std::setprecision(17) << data; values[key] = s.str(); };
  value("admittance_configured", admittance_config_.use_admittance && admittance_config_.rot_enable.y() >= .5);
  value("admittance_enabled", perching_admittance_enabled_); value("admittance_enable_requested", perching_admittance_enabled_);
  value("baseline_samples", equilibrium_wrench_sample_count_);
  value("raw_load_valid", raw_load_valid_);
  value("contact_active", contact_active_); value("recovery_active", recovery_active_);
  value("baseline_ready", equilibrium_wrench_ready_); value("tare_collecting", perching_active_ && !perching_admittance_enabled_ && !equilibrium_wrench_ready_);
  value("tare_generation", tare_generation_); value("tare_request_sequence", tare_request_sequence_); value("enable_request_sequence", enable_request_sequence_);
  value("reference_accepted", output_accepted_);
  value("cutting_force_bias", cutting_force_bias_);
  value("task_torque_valid", input_valid_); value("admittance_status", reason_);
  value("task_load_source", perching_navigator_->articulated() ? "articulated_momentum: commanded world/CoG actuation estimate" :
      "world/CoG wrench: locked pivot hinge projection");
  value("generalized_observer_status", perching_navigator_->articulated() ?
      (raw_load_valid_ ? "ready" : reason_) : "not applicable: rigid pivot wrench");
  value("offset_rad", admittance_output_.angle_offset_compliance.y()); value("offset_rate_rad_s", admittance_output_.angular_vel_offset_compliance.y());
  value("offset_acceleration_rad_s2", admittance_output_.angular_acc_offset_compliance.y());
  perching_navigator_->setAdmittanceDiagnostics(values);
}
void GimbalrotorPerchingAdmittanceController::updateContactGate(
    double residual_pitch_torque,
    double dt)
{
  const double magnitude = std::abs(residual_pitch_torque);

  if(!contact_active_)
  {
    contact_off_timer_ = 0.0;

    if(magnitude > contact_on_threshold_)
    {
      contact_on_timer_ += dt;

      if(contact_on_timer_ >= contact_on_duration_)
      {
        contact_active_ = true;
        recovery_active_ = false;
        contact_on_timer_ = 0.0;

        ROS_WARN(
            "[GimbalrotorPerchingAdmittanceController] "
            "Contact admittance active: filtered residual pitch torque %.4f Nm.",
            residual_pitch_torque);
      }
    }
    else
    {
      contact_on_timer_ = 0.0;
    }

    return;
  }

  contact_on_timer_ = 0.0;

  if(magnitude < contact_off_threshold_)
  {
    contact_off_timer_ += dt;

    if(contact_off_timer_ >= contact_off_duration_)
    {
      contact_active_ = false;
      recovery_active_ = true;
      contact_off_timer_ = 0.0;

      ROS_WARN(
          "[GimbalrotorPerchingAdmittanceController] "
          "Contact released: recovering pitch admittance offset.");
    }
  }
  else
  {
    contact_off_timer_ = 0.0;
  }
}

void GimbalrotorPerchingAdmittanceController::updatePitchIntegralPolicy(PID& pitch_pid, bool suppress_pitch_i_limit)
{
  if(suppress_pitch_i_limit)
  {
    /*
     * This runs once when perching admittance becomes enabled.
     */
    if(!pitch_i_limit_suppressed_)
    {
      /*
       * Save the current normal limit before changing it.
       *
       * Normally this is the YAML value 10.0.
       */
      normal_pitch_i_limit_ = pitch_pid.getLimitI();

      pitch_i_limit_suppressed_ = true;

      ROS_WARN(
          "[GimbalrotorPerchingAdmittanceController] "
          "Perching admittance enabled: pitch limit_i changed from %.6f to 0.0.",
          normal_pitch_i_limit_);
    }

    /*
     * Force the integral output limit to zero.
     */
    pitch_pid.setLimitI(0.0);

    /*
     * limit_i = 0 only forces the resulting I term to zero.
     * The stored integral error would otherwise continue
     * accumulating, so clear it before PID::update().
     */
    pitch_pid.setErrI(0.0);
  }
  else if(pitch_i_limit_suppressed_)
  {
    /*
     * Perching admittance was disabled.
     *
     * Restore the normal limit and restart the integrator from zero.
     */
    pitch_pid.setErrI(0.0);
    pitch_pid.setLimitI(normal_pitch_i_limit_);
    pitch_i_limit_suppressed_ = false;

    ROS_WARN(
        "[GimbalrotorPerchingAdmittanceController] "
        "Perching admittance disabled: pitch limit_i restored to %.6f.",
        normal_pitch_i_limit_);
  }
}


}
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorPerchingAdmittanceController, aerial_robot_control::ControlBase)
