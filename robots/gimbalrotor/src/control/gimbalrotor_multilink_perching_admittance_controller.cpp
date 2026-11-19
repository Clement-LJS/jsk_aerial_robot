#include <gimbalrotor/control/gimbalrotor_multilink_perching_admittance_controller.h>
#include <gimbalrotor/control/gimbalrotor_multilink_perching_observer.h>
#include <pluginlib/class_list_macros.h>
#include <map>
#include <deque>
#include <boost/weak_ptr.hpp>
#include <algorithm>
#include <cmath>
#include <tuple>
#include <stdexcept>
#include <Eigen/SVD>

namespace aerial_robot_control
{
// The controller owns this runtime. Navigation holds only a weak handle and
// serializes all calls (including ROS callbacks) with its contact-session mutex.
class RollYawInteractionRuntime final : public RollYawInteraction,
                                      private RollYawInteractionDiagnostics,
                                      public std::enable_shared_from_this<RollYawInteractionRuntime>
{
public:
  using Input = RollYawInteractionInput;
  using Output = RollYawInteractionOutput;
  using InteractionSource = RollYawInteractionDiagnostics::InteractionSource;
  using Dispatch = std::function<void(const std::function<void(const Input&)>&, bool, const std::string&)>;
  void initialize(const Input&, ros::NodeHandle, boost::shared_ptr<aerial_robot_model::RobotModel>, Dispatch);
  const RollYawInteractionDiagnostics& diagnostics() const override { return *this; }
  bool articulated() const override { return interaction_source_ == InteractionSource::ARTICULATED_MOMENTUM; }
  bool observerReady() const override { return momentum_observer_.ready(); }
  double observerStamp() const override { return momentum_observer_.stamp(); }
  const Eigen::VectorXd& observerLoad() const override { return momentum_observer_.load(); }
  void captureModel(const Input&) override;
  void clearModel() override
  {
    resetCalibration();
    articulated_model_valid_ = false;
    ++observation_epoch_;
    observation_worker_.cancel();
    (void)observation_worker_.take();
    observation_pending_ = false;
    observation_model_.reset();
  }
  void captureDynamicsJoints(const Input&, const sensor_msgs::JointState&, const std::string& producer, double receipt) override;
  bool observeArticulatedSample(const Input&, const roll_yaw::Feedback&) override;
  bool articulatedLoadValid(const Input&, const roll_yaw::Feedback&) const override;
  void publishArticulatedLoad(const Input&) override;
  void resetArticulatedSession(const Input&, const std::string&) override;
  void invalidateArticulated(const Input&, const std::string&) override;
  void resetCompliance() override;
  void stopped() override;
  bool motionAllowed(const Input&) const override;
  Output updateInteraction(const Input&, const roll_yaw::Feedback&) override;
  double reconcileReference(bool accepted, bool paused, double applied_relief) override;
  double holdReference(double applied_relief) override;
  void waitForFeedback(double now) override { compliance_time_ = now; }

private:
  struct SourceSettings
  {
    std::string source, contract, effort_source, topic, producer, units, sign, joint_topic, controller, description;
    double gain = 10;
  };
  SourceSettings readInteractionSource() const;
  bool effortSettingsValid(const SourceSettings&) const;
  bool loadInteractionSource(const Input&, ros::Subscriber& retired, const SourceSettings&);
  bool interactionConfigurationValid() const;
  void commandWrenchCallback(const Input&, const geometry_msgs::WrenchStampedConstPtr&,
                            const Eigen::Vector3d& offset, double receipt);
  void invalidateInteraction(); // Current task sample/filter only; never owns calibration.
  void resetCalibration(); // Explicit lock/model/source change or admitted operator tare.
  void rejectSample(const Input&, const std::string&);
  bool quietContact(const Input&) const override;
  bool heldContact(const Input&) const override;
  bool correctionReleased() const;
  bool observeInteractionSample(const Input&, const roll_yaw::Feedback&, double stamp, bool held);
  void admittanceCallback(const Input&, const std_msgs::BoolConstPtr&);
  void externalWrenchCallback(const Input&, const geometry_msgs::WrenchStampedConstPtr&, double receipt);
  struct TareRequest
  {
    std::uint64_t session = 0, generation = 0;
    double sign = 0, scale = 0;
    std::string frame, point;
    SourceSettings source;
  };
  void tareCallback(const Input&, const TareRequest&, ros::Subscriber& retired);
  void requestTare();
  void subscribeCommandWrench();
  ros::NodeHandle nh_;
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  Dispatch dispatch_;
  std::atomic<std::uint64_t> identity_session_{0}, identity_generation_{0};
  void updateIdentity(const Input& c)
  {
    identity_generation_.store(c.generation);
    identity_session_.store(c.session);
  }
  std::vector<ros::Subscriber> subscriptions_;
  ros::Subscriber command_wrench_sub_;
  ros::Publisher generalized_load_pub_, generalized_load_valid_pub_;
  double observer_gain_ = 10;
  perching_dynamics::ObservationFilter observation_filter_;
  double observer_settling_until_ = 0;
  struct BodySample { roll_yaw::Feedback feedback; };
  struct JointSample { double stamp; Eigen::Vector3d q, effort; };
  struct CommandSample { double stamp, receipt; Eigen::Matrix<double, 6, 1> wrench; Eigen::Vector3d offset; };
  std::deque<BodySample> body_history_;
  std::deque<JointSample> joint_history_;
  std::deque<CommandSample> command_history_;
  double filtered_stamp_ = 0, observation_start_stamp_ = 0;
  Eigen::VectorXd baseline_, baseline_sum_;
  double baseline_stamp_ = 0;
  std::uint64_t baseline_session_ = 0;
  perching_dynamics::MultilinkModel articulated_model_;
  std::shared_ptr<const perching_dynamics::MultilinkModel> observation_model_;
  struct Observation
  {
    bool ok = false;
    std::uint64_t session = 0, epoch = 0;
    double body_stamp = 0, joint_stamp = 0, command_stamp = 0, receipt = 0, wall_seconds = 0;
    std::uint64_t tree_calls = 0;
    Eigen::Matrix3d jacobian = Eigen::Matrix3d::Zero();
    perching_dynamics::MultilinkModel::Evaluation value;
    std::string reason;
  };
  roll_yaw::Worker<Observation> observation_worker_;
  bool observation_pending_ = false;
  std::uint64_t observation_epoch_ = 0, articulated_session_ = 0;
  perching_dynamics::MomentumObserver momentum_observer_;
  bool articulated_model_valid_ = false, articulated_valid_ = false;
  double articulated_receipt_ = 0, articulated_body_stamp_ = 0, articulated_published_stamp_ = 0;
  Eigen::Matrix3d articulated_jacobian_ = Eigen::Matrix3d::Zero(); // Same sample as the residual.
  struct DynamicsJoint
  {
    double position = 0, rate = 0, effort = std::numeric_limits<double>::quiet_NaN();
    double stamp = 0, receipt = 0;
    bool rate_valid = false;
  };
  std::map<std::string, DynamicsJoint> dynamics_joints_;
  geometry_msgs::WrenchStamped command_wrench_;
  Eigen::Vector3d command_cog_offset_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double command_wrench_receipt_ = 0, articulated_joint_stamp_ = 0, articulated_command_stamp_ = 0;
  double compliance_time_ = 0;
  geometry_msgs::WrenchStamped external_wrench_;
  double previous_offset_ = 0, previous_rate_ = 0, previous_acceleration_ = 0;
};

namespace
{
std::string frameName(std::string s)
{
  if(!s.empty() && s.front() == '/')
    s.erase(0, 1);
  return s;
}
}
void GimbalrotorMultilinkPerchingAdmittanceController::initialize(
    ros::NodeHandle nh, ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate)
{
  GimbalrotorController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  auto* host = dynamic_cast<RollYawInteractionHost*>(navigator.get());
  if(!host)
    throw std::runtime_error("roll-yaw admittance requires a roll-yaw interaction navigator");
  boost::weak_ptr<aerial_robot_navigation::BaseNavigator> weak_navigator(navigator);
  auto dispatch = [weak_navigator](const std::function<void(const RollYawInteractionInput&)>& call, bool diagnostics, const std::string& key)
  {
    if(auto navigator = weak_navigator.lock())
      if(auto* host = dynamic_cast<RollYawInteractionHost*>(navigator.get()))
        host->withRollYawInteraction(call, diagnostics, key);
  };
  auto interaction = std::make_shared<RollYawInteractionRuntime>();
  interaction_ = interaction;
  if(!host->attachRollYawInteraction(interaction, [&](const RollYawInteractionInput& c)
      { interaction->initialize(c, nh, robot_model, dispatch); }))
    throw std::runtime_error("roll-yaw interaction controller registration rejected");
}

Eigen::Vector3d GimbalrotorMultilinkPerchingAdmittanceController::angularFeedforward() const
{
  const auto* host = dynamic_cast<const RollYawInteractionHost*>(navigator_.get());
  if(!host || !host->rollYawReferenceActive()) return Eigen::Vector3d::Zero();
  Eigen::Vector3d rate(target_omega_.x(), target_omega_.y(), target_omega_.z());
  Eigen::Vector3d acceleration(target_ang_acc_.x(), target_ang_acc_.y(), target_ang_acc_.z());
  Eigen::Vector3d gain = Eigen::Vector3d::Zero(), limit = gain;
  for(int i = 0; i < 2; ++i)
  {
    const auto& pid = pid_controllers_.at(ROLL + i);
    gain[i] = pid.getDGain();
    limit[i] = std::min(pid.getLimitD(), pid.getLimitSum());
  }
  return roll_yaw::referenceFeedforward(rate, acceleration, gain, limit);
}

void RollYawInteractionRuntime::initialize(const Input& c, ros::NodeHandle nh,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model, Dispatch dispatch)
{
  nh_ = nh;
  robot_model_ = std::move(robot_model);
  dispatch_ = std::move(dispatch);
  ros::NodeHandle pn(nh, "navigation/roll_yaw_perching");
  ros::NodeHandle an(pn, "admittance");
  auto& a = admittance_.config;
  an.param("inertia", a.inertia, a.inertia);
  an.param("damping", a.damping, a.damping);
  an.param("stiffness", a.stiffness, a.stiffness);
  an.param("torque_limit", a.torque_limit, a.torque_limit);
  an.param("angle_limit", a.angle_limit, a.angle_limit);
  an.param("rate_limit", a.rate_limit, a.rate_limit);
  an.param("acceleration_limit", a.acceleration_limit, a.acceleration_limit);
  an.param("reference_torque", a.reference_torque, a.reference_torque);
  an.param("filter_time", a.filter_time, a.filter_time);
  an.param("on_threshold", a.on_threshold, a.on_threshold);
  an.param("off_threshold", a.off_threshold, a.off_threshold);
  an.param("on_dwell", a.on_dwell, a.on_dwell);
  an.param("off_dwell", a.off_dwell, a.off_dwell);
  an.param("baseline_samples", a.baseline_samples, a.baseline_samples);
  an.param<std::string>("external_wrench_topic", interaction_topic_, "estimated_external_wrench");
  an.param<std::string>("wrench_frame", interaction_frame_, "world");
  an.param<std::string>("wrench_point", interaction_point_, "cog");
  an.param("wrench_sign", interaction_sign_, interaction_topic_ == "estimated_external_wrench" ? 1.0 : 0.0);
  an.param("wrench_scale", interaction_scale_, interaction_topic_ == "estimated_external_wrench" ? 1.0 : 0.0);
  ros::Subscriber retired; // No previous subscription during initialization.
  loadInteractionSource(c, retired, readInteractionSource());
  interaction_configured_ = interactionConfigurationValid();
  generalized_load_pub_ = nh.advertise<sensor_msgs::JointState>(c.prefix + "/generalized_external_load", 1);
  generalized_load_valid_pub_ = nh.advertise<std_msgs::Bool>(c.prefix + "/generalized_external_load_valid", 1);
  const std::weak_ptr<RollYawInteractionRuntime> weak = shared_from_this();
  subscriptions_.push_back(nh.subscribe<std_msgs::Bool>("perching/admittance_enable", 1,
      [weak](const std_msgs::BoolConstPtr& msg)
      {
        if(auto self = weak.lock())
          self->dispatch_([self, msg](const Input& c) { self->admittanceCallback(c, msg); }, true, "enable");
      }));
  subscriptions_.push_back(nh.subscribe<geometry_msgs::WrenchStamped>(interaction_topic_, 1,
      [weak](const geometry_msgs::WrenchStampedConstPtr& msg)
      {
        if(auto self = weak.lock())
        {
          const double receipt = roll_yaw::steadySeconds();
          self->dispatch_([self, msg, receipt](const Input& c)
              { self->externalWrenchCallback(c, msg, receipt); }, false, "external_wrench");
        }
      }));
  subscriptions_.push_back(nh.subscribe<std_msgs::Empty>(c.prefix + "/admittance_tare", 1,
      [weak](const std_msgs::EmptyConstPtr&)
      {
        if(auto self = weak.lock())
          self->requestTare();
      }));
}

void RollYawInteractionRuntime::subscribeCommandWrench()
{
  const std::weak_ptr<RollYawInteractionRuntime> weak = shared_from_this();
  command_wrench_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>("perching/roll_yaw/command_wrench_estimate", 1,
      [weak](const geometry_msgs::WrenchStampedConstPtr& msg)
      {
        if(auto self = weak.lock())
        {
          const double receipt = roll_yaw::steadySeconds();
          Eigen::Vector3d offset = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
          if(self->robot_model_ && self->robot_model_->initialized())
          {
            const auto frames = self->robot_model_->getSegmentsTf();
            const auto body = frames.find(self->robot_model_->getBaselinkName());
            if(body != frames.end())
              offset = roll_yaw::fromKdl(body->second.Inverse() * self->robot_model_->getCog<KDL::Frame>()).translation();
          }
          self->dispatch_([self, msg, offset, receipt](const Input& c)
              { self->commandWrenchCallback(c, msg, offset, receipt); }, false, "command_wrench");
        }
      });
}

void RollYawInteractionRuntime::requestTare()
{
  TareRequest request;
  request.session = identity_session_.load();
  request.generation = identity_generation_.load();
  request.source = readInteractionSource();
  ros::NodeHandle an(nh_, "navigation/roll_yaw_perching/admittance");
  an.param("wrench_sign", request.sign, interaction_topic_ == "estimated_external_wrench" ? 1.0 : 0.0);
  an.param("wrench_scale", request.scale, interaction_topic_ == "estimated_external_wrench" ? 1.0 : 0.0);
  an.param<std::string>("wrench_frame", request.frame, "world");
  an.param<std::string>("wrench_point", request.point, "cog");
  // Parameters are read on the callback thread. The owned request is accepted
  // by the same ordered update owner as contact and motion commands.
  const auto self = shared_from_this();
  dispatch_([self, request](const Input& c)
  {
    ros::Subscriber retired;
    self->tareCallback(c, request, retired);
    // Ingestion never waits on the session lock, including subscription shutdown.
  }, false, "tare");
}

void RollYawInteractionRuntime::captureModel(const Input& c)
{
  updateIdentity(c);
  resetCalibration(); // Recapturing geometry changes the generalized-coordinate reference.
  articulated_model_valid_ = false;
  if(articulated() && c.dynamics_factory)
    articulated_model_valid_ = c.dynamics_factory(c.model, articulated_model_, articulated_reason_);
  ++observation_epoch_;
  observation_worker_.cancel();
  (void)observation_worker_.take();
  observation_pending_ = false;
  observation_model_ = articulated_model_valid_ ?
      std::make_shared<const perching_dynamics::MultilinkModel>(articulated_model_) : nullptr;
}

void RollYawInteractionRuntime::stopped()
{
  if(!admittance_.baseline_ready)
  {
    resetCompliance(); // An unfinished or invalidated tare cannot survive a stop.
    return;
  }
  admittance_reason_ = admittance_.enabled
      ? "enabled: nominal stopped; completed tare retained, freshness still required"
      : "disabled/ready: nominal stopped; completed tare retained";
}

bool RollYawInteractionRuntime::motionAllowed(const Input& c) const
{
  const auto& current = c.feedback;
  return !(admittance_.enabled && (!interaction_valid_ || !admittance_.baseline_ready ||
           current.now - interaction_stamp_ > c.feedback_config.timeout ||
           current.now - interaction_stamp_ < -c.feedback_config.future_tolerance ||
           (articulated() ? !articulatedLoadValid(c, current) :
            std::abs(interaction_stamp_ - current.body_stamp) > c.synchronization_tolerance) ||
           roll_yaw::steadySeconds() - interaction_receipt_ > c.feedback_config.timeout));
}

double RollYawInteractionRuntime::holdReference(double applied_relief)
{
  admittance_.offset = admittance_.enabled || std::abs(admittance_.offset) > 1e-10
                           ? std::clamp(applied_relief, -admittance_.config.angle_limit, admittance_.config.angle_limit) : 0;
  admittance_.rate = admittance_.acceleration = 0;
  return admittance_.offset;
}

double RollYawInteractionRuntime::reconcileReference(bool accepted, bool paused, double applied_relief)
{
  if(!accepted)
  {
    // Keep fresh observation/filter history, but commit correction dynamics
    // only with an accepted reference step (clock/bootstrap returns included).
    admittance_.offset = previous_offset_;
    admittance_.rate = previous_rate_;
    admittance_.acceleration = previous_acceleration_;
  }
  if(paused)
  {
    // Reconcile against the actual held on-path reference. No hidden spring
    // displacement or velocity is accumulated across a long pause.
    return holdReference(applied_relief);
  }
  return admittance_.offset;
}

RollYawInteractionRuntime::SourceSettings RollYawInteractionRuntime::readInteractionSource() const
{
  SourceSettings source;
  ros::NodeHandle an(nh_, "navigation/roll_yaw_perching/admittance");
  ros::NodeHandle dn(nh_, "navigation/roll_yaw_perching/dynamics");
  ros::NodeHandle en(dn, "joint_effort");
  an.param<std::string>("source", source.source, "legacy_wrench");
  dn.param("observer_gain", source.gain, 10.0);
  dn.param<std::string>("joint_effort_contract", source.contract, "unavailable");
  en.param<std::string>("source", source.effort_source, "unavailable");
  en.param<std::string>("topic", source.topic, "joint_states");
  en.param<std::string>("producer", source.producer, "");
  en.param<std::string>("units", source.units, "Nm");
  en.param<std::string>("sign", source.sign, "urdf");
  ros::NodeHandle(nh_, "navigation/roll_yaw_perching").param<std::string>("joint_state_topic", source.joint_topic, "joint_states");
  if(source.effort_source != "servo_bridge_joint_states")
  {
    nh_.getParam("joint_state_controller/type", source.controller);
    nh_.getParam("robot_description", source.description);
  }
  return source;
}

bool RollYawInteractionRuntime::effortSettingsValid(const SourceSettings& settings) const
{
  const bool matching_topic = !settings.topic.empty() &&
      nh_.resolveName(settings.topic) == nh_.resolveName(settings.joint_topic);
  if(settings.effort_source == "servo_bridge_joint_states") return matching_topic;
  return settings.effort_source == "gazebo_ode_joint_states" && ros::Time::isSimTime() &&
      settings.contract == "joint_side_actuation_nm" && settings.units == "Nm" && settings.sign == "urdf" &&
      matching_topic && !settings.producer.empty() && settings.producer.front() == '/' &&
      settings.controller == "joint_state_controller/JointStateController" &&
      settings.description.find("gazebo_ros_control/AerialRobotHWSim") != std::string::npos;
}

bool RollYawInteractionRuntime::loadInteractionSource(const Input& c, ros::Subscriber& retired,
                                                       const SourceSettings& settings)
{
  const auto previous = std::make_tuple(interaction_source_name_, observer_gain_, effort_contract_,
                                         effort_source_, effort_topic_, effort_producer_, effort_source_valid_);
  const std::string previous_reason = articulated_reason_;
  auto finish = [&]()
  {
    const bool changed = previous != std::make_tuple(interaction_source_name_, observer_gain_, effort_contract_,
                                                    effort_source_, effort_topic_, effort_producer_, effort_source_valid_);
    if(changed)
    {
      resetCalibration();
      resetArticulatedSession(c, "interaction source/configuration changed; awaiting coherent dynamics sample");
    }
    else
      articulated_reason_ = previous_reason;
    return changed;
  };
  interaction_source_name_ = settings.source;
  interaction_source_ = interaction_source_name_ == "legacy_wrench" ? InteractionSource::LEGACY_WRENCH :
                        interaction_source_name_ == "articulated_momentum" ? InteractionSource::ARTICULATED_MOMENTUM :
                        InteractionSource::INVALID;
  if(interaction_source_ == InteractionSource::ARTICULATED_MOMENTUM)
  {
    if(!command_wrench_sub_)
      subscribeCommandWrench();
  }
  else
  {
    retired = command_wrench_sub_;
    command_wrench_sub_ = ros::Subscriber();
  }
  observer_gain_ = settings.gain;
  effort_contract_ = settings.contract;
  effort_source_ = settings.effort_source;
  effort_topic_ = settings.topic;
  effort_producer_ = settings.producer;
  if(effort_source_ == "servo_bridge_joint_states")
  {
    // ServoBridge already applies torque_scale * angle_sgn to the raw load.
    // Use those nominal current-derived joint efforts directly; independent
    // torque calibration is neither assumed nor required by this adapter.
    effort_producer_ = nh_.resolveName("servo_bridge");
    effort_contract_ = "servo_bridge_nominal_joint_effort";
    effort_source_valid_ = effortSettingsValid(settings);
    articulated_reason_ = effort_source_valid_ ? "waiting for synchronized ServoBridge joint effort" :
        "ServoBridge effort topic must match the configured joint-state topic";
    return finish();
  }
  // Audited local simulation path: AerialRobotHWSim delegates read/writeSim
  // to DefaultRobotHWSim; JointStateController publishes its JointStateHandle.
  // For the shipped ODE world and EffortJointInterface, effort is GetForce(0):
  // ODE saves the limited SetForce torque, NOT contact reaction/GetForceTorque
  // or the position PID target. The installed ODE implementation was inspected.
  // No motor constant, gearbox conversion or second URDF sign is applied here.
  effort_source_valid_ = effortSettingsValid(settings);
  articulated_reason_ = effort_source_valid_ ? "waiting for synchronized Gazebo ODE joint-side torque" :
      "unsupported effort source or invalid Gazebo ODE producer/topic/units/sign configuration";
  return finish();
}

bool RollYawInteractionRuntime::interactionConfigurationValid() const
{
  if(!admittance_.config.valid())
    return false;
  if(interaction_source_ == InteractionSource::ARTICULATED_MOMENTUM)
    return std::isfinite(observer_gain_) && observer_gain_ > 0;
  return interaction_source_ == InteractionSource::LEGACY_WRENCH &&
         (interaction_sign_ == 1 || interaction_sign_ == -1) && std::isfinite(interaction_scale_) &&
         interaction_scale_ > 0 && (interaction_frame_ == "world" || interaction_frame_ == "body") &&
         (interaction_point_ == "cog" || interaction_point_ == "body" || interaction_point_ == "world_origin");
}

void RollYawInteractionRuntime::captureDynamicsJoints(const Input& c, const sensor_msgs::JointState& msg,
                                                        const std::string& producer, double receipt)
{
  // Irrelevant-only messages (hand/lock/gimbals) can share joint_states, including from another
  // publisher. They are not reduced dynamics inputs and cannot refresh/reset it.
  if(std::find(msg.name.begin(), msg.name.end(), c.roll) == msg.name.end() &&
     std::find(msg.name.begin(), msg.name.end(), c.yaw) == msg.name.end())
    return;
  if(!effort_source_valid_ || producer != effort_producer_ ||
     (effort_source_ == "gazebo_ode_joint_states" && !ros::Time::isSimTime()))
  {
    dynamics_joints_.clear();
    rejectSample(c, "joint effort producer unavailable or differs from the selected source");
    return;
  }
  // ServoBridge and the supported JointStateController publish the complete set.
  // A partial roll/yaw sample cannot retain an older apparently valid torque.
  if(std::count(msg.name.begin(), msg.name.end(), c.roll) != 1 ||
     std::count(msg.name.begin(), msg.name.end(), c.yaw) != 1)
  {
    dynamics_joints_.clear();
    rejectSample(c, "verified joint-state producer omitted or duplicated roll/yaw");
    return;
  }
  const double stamp = msg.header.stamp.toSec(), now = ros::Time::now().toSec();
  for(std::size_t i = 0; i < msg.name.size(); ++i)
  {
    const auto& name = msg.name[i];
    if(name != c.roll && name != c.yaw)
      continue;
    auto& previous = dynamics_joints_[name];
    if(stamp <= 0 || now - stamp > c.feedback_config.timeout || now - stamp < -c.feedback_config.future_tolerance ||
       std::count(msg.name.begin(), msg.name.end(), name) != 1 || msg.position.size() != msg.name.size() ||
       !std::isfinite(msg.position[i]) || (!msg.velocity.empty() &&
       (msg.velocity.size() != msg.name.size() || !std::isfinite(msg.velocity[i]))))
    {
      previous = DynamicsJoint();
      rejectSample(c, "invalid dynamics joint timestamp/name/position/rate array");
      continue;
    }
    if((name == c.roll || name == c.yaw) &&
       (msg.effort.size() != msg.name.size() || !std::isfinite(msg.effort[i])))
    {
      previous = DynamicsJoint();
      rejectSample(c, "missing/nonfinite joint-side actuation torque");
      continue;
    }
    if(previous.stamp > 0 && (stamp < previous.stamp || stamp - previous.stamp > c.feedback_config.timeout))
    {
      previous = DynamicsJoint();
      rejectSample(c, "discontinuous dynamics joint timestamp; awaiting coherent samples");
    }
    if(stamp == previous.stamp)
      continue; // Duplicate samples never refresh receipt or observer time.
    DynamicsJoint sample;
    sample.position = msg.position[i];
    sample.stamp = stamp;
    sample.receipt = receipt;
    if(!msg.velocity.empty())
    {
      sample.rate = msg.velocity[i];
      sample.rate_valid = std::isfinite(sample.rate);
    }
    else if(previous.stamp > 0 && stamp - previous.stamp > 1e-4 && stamp - previous.stamp <= c.feedback_config.timeout)
    {
      sample.rate = (sample.position - previous.position) / (stamp - previous.stamp);
      sample.rate_valid = std::isfinite(sample.rate);
    }
    if(msg.effort.size() == msg.name.size())
      sample.effort = msg.effort[i]; // Already converted; never apply torque_scale or angle_sgn again.
    previous = sample;
    if(!sample.rate_valid)
      rejectSample(c, "joint rate unavailable; awaiting advancing valid samples");
  }
  if(dynamics_joints_.count(c.roll) && dynamics_joints_.count(c.yaw))
  {
    const auto& roll = dynamics_joints_.at(c.roll);
    const auto& yaw = dynamics_joints_.at(c.yaw);
    if(roll.stamp == yaw.stamp && (joint_history_.empty() || roll.stamp > joint_history_.back().stamp))
    {
      joint_history_.push_back({roll.stamp, Eigen::Vector3d(0, roll.position, yaw.position),
                                Eigen::Vector3d(0, roll.effort, yaw.effort)});
      while(joint_history_.size() > 32) joint_history_.pop_front();
    }
  }
}

void RollYawInteractionRuntime::commandWrenchCallback(const Input& c, const geometry_msgs::WrenchStampedConstPtr& msg,
                                                      const Eigen::Vector3d& offset, double receipt)
{
  if(interaction_source_ != InteractionSource::ARTICULATED_MOMENTUM)
    return;
  const double stamp = msg->header.stamp.toSec(), now = ros::Time::now().toSec();
  const auto& w = msg->wrench;
  if(stamp <= 0 || now - stamp > c.feedback_config.timeout || now - stamp < -c.feedback_config.future_tolerance ||
     frameName(msg->header.frame_id) != frameName(c.world) ||
     !Eigen::Vector3d(w.force.x, w.force.y, w.force.z).allFinite() ||
     !Eigen::Vector3d(w.torque.x, w.torque.y, w.torque.z).allFinite())
  {
    command_wrench_receipt_ = 0;
    rejectSample(c, "invalid command wrench timestamp/frame/value");
    return;
  }
  if(stamp < command_wrench_.header.stamp.toSec())
  {
    command_wrench_receipt_ = 0;
    command_wrench_ = geometry_msgs::WrenchStamped();
    rejectSample(c, "command wrench timestamp moved backwards; awaiting coherent samples");
    return;
  }
  if(stamp == command_wrench_.header.stamp.toSec())
    return;
  // The input point and receipt were copied with this source message, not
  // reconstructed after deferred dispatch from a newer live robot posture.
  if(!offset.allFinite())
  {
    command_wrench_receipt_ = 0;
    rejectSample(c, "command wrench application point unavailable");
    return;
  }
  command_wrench_ = *msg;
  command_cog_offset_ = offset;
  command_wrench_receipt_ = receipt;
  Eigen::Matrix<double, 6, 1> input;
  input << w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z;
  command_history_.push_back({stamp, receipt, input, offset});
  while(command_history_.size() > 32) command_history_.pop_front();
}

void RollYawInteractionRuntime::rejectSample(const Input& c, const std::string& reason)
{
  articulated_valid_ = false;
  interaction_valid_ = false;
  articulated_reason_ = reason;
  invalidateInteraction(); // Even a prolonged outage retains the same-session tare.
  publishArticulatedLoad(c);
}

void RollYawInteractionRuntime::invalidateArticulated(const Input& c, const std::string& reason)
{
  ++observation_epoch_;
  observation_worker_.cancel();
  (void)observation_worker_.take();
  observation_pending_ = false;
  momentum_observer_.reset(reason);
  observer_settling_until_ = 0;
  articulated_reason_ = reason;
  articulated_valid_ = false;
  articulated_receipt_ = 0;
  articulated_session_ = 0;
  articulated_jacobian_.setZero();
  invalidateInteraction();
  publishArticulatedLoad(c); // Promptly retire a bad sample, including callback-time faults/relocks.
}

void RollYawInteractionRuntime::resetArticulatedSession(const Input& c, const std::string& reason)
{
  updateIdentity(c);
  if(!c.lock_valid || (baseline_session_ != 0 && baseline_session_ != c.session))
    resetCalibration();
  observation_start_stamp_ = c.feedback.now; // Clock epoch, independent of immutable lock identity.
  articulated_worker_wall_ = 0;
  articulated_worker_tree_calls_ = 0;
  dynamics_joints_.clear();
  body_history_.clear(); joint_history_.clear(); command_history_.clear(); observation_filter_.reset();
  filtered_stamp_ = 0; observer_rates_ready_ = false; observer_gyro_disagreement_.setZero();
  articulated_body_stamp_ = articulated_joint_stamp_ = articulated_command_stamp_ = 0;
  articulated_published_stamp_ = 0;
  command_wrench_ = geometry_msgs::WrenchStamped();
  command_cog_offset_.setConstant(std::numeric_limits<double>::quiet_NaN());
  command_wrench_receipt_ = 0;
  invalidateArticulated(c, reason);
}

void RollYawInteractionRuntime::invalidateInteraction()
{
  interaction_valid_ = false;
  interaction_stamp_ = interaction_receipt_ = 0;
  // Discard stale scalar forcing/dwell/history while preserving the measured
  // calibration and the bounded correction state needed for hold/braking.
  const auto previous = admittance_;
  admittance_.reset();
  admittance_.baseline_ready = previous.baseline_ready;
  admittance_.samples = previous.samples;
  admittance_.bias = previous.bias;
  admittance_.enabled = previous.enabled;
  admittance_.offset = previous.offset;
  admittance_.rate = previous.rate;
  admittance_.acceleration = previous.acceleration;
}

void RollYawInteractionRuntime::resetCalibration()
{
  tare_requested_ = false;
  baseline_.resize(0); baseline_sum_.resize(0); baseline_stamp_ = 0;
  baseline_session_ = 0;
  admittance_.baseline_ready = false;
  admittance_.samples = 0;
  admittance_.bias = 0;
  invalidateInteraction();
}

bool RollYawInteractionRuntime::articulatedLoadValid(const Input& c, const roll_yaw::Feedback& f) const
{
  const double stamp = momentum_observer_.stamp();
  const double wall_age = roll_yaw::steadySeconds() - articulated_receipt_;
  return interaction_source_ == InteractionSource::ARTICULATED_MOMENTUM && articulated_valid_ &&
         c.lock_valid && c.session == articulated_session_ && f.hover &&
         articulated_model_valid_ && momentum_observer_.ready() &&
         momentum_observer_.load().size() == 3 && momentum_observer_.load().allFinite() &&
         stamp > 0 && f.now - stamp >= -c.feedback_config.future_tolerance &&
         f.now - stamp <= c.feedback_config.timeout &&
         articulated_receipt_ > 0 && wall_age >= 0 && wall_age <= c.feedback_config.timeout &&
         c.last_body_message_stamp == f.body_stamp && c.last_joint_message_stamp == f.joint_stamp &&
         c.contact_safety_rejection(f).empty();
}

void RollYawInteractionRuntime::publishArticulatedLoad(const Input& c)
{
  if(!generalized_load_pub_ || !generalized_load_valid_pub_)
    return; // Source configuration is loaded before publishers are initialized.
  std_msgs::Bool valid;
  valid.data = articulatedLoadValid(c, c.feedback);
  generalized_load_valid_pub_.publish(valid);
  if(!valid.data || momentum_observer_.stamp() == articulated_published_stamp_)
    return;
  sensor_msgs::JointState load;
  load.header.stamp.fromSec(momentum_observer_.stamp());
  load.name = {"alpha", c.roll, c.yaw};
  for(int i = 0; i < 3; ++i)
    load.effort.push_back(momentum_observer_.load()[i]);
  // Generalized N.m in [alpha, roll, yaw] order; one publication per observation.
  // Consumers must require both validity and a fresh observation timestamp.
  generalized_load_pub_.publish(load);
  articulated_published_stamp_ = momentum_observer_.stamp();
}

bool RollYawInteractionRuntime::observeArticulatedSample(const Input& c, const roll_yaw::Feedback& f)
{
  auto fail = [this, &c](const std::string& reason) { rejectSample(c, reason); return false; };
  auto invalid = [this, &c](const std::string& reason) { invalidateArticulated(c, reason); return false; };
  if(!c.lock_valid || !f.hover)
    return invalid("requires valid synchronized attached HOVER state");
  if(c.model_identity_changed)
  {
    resetCalibration();
    articulated_model_valid_ = false; // Remains unusable until an explicit model capture.
    return invalid("robot topology/inertia/extra modules changed since lock; recapture required");
  }
  const auto contact_error = c.contact_safety_rejection(f);
  if(!contact_error.empty())
    return fail("invalid contact/feedback: " + contact_error);
  if(c.last_body_message_stamp != f.body_stamp || c.last_joint_message_stamp != f.joint_stamp)
    return fail("latest body/joint measurement was rejected; awaiting valid inputs");
  if(!articulated_model_valid_)
    return invalid("invalid locked dynamics snapshot; recapture with valid inertias and geometry");
  if(!c.dynamics_snapshot_current || !c.dynamics_snapshot_current())
    return invalid("locked dynamics snapshot temporarily unavailable");
  if(!effort_source_valid_ || (effort_source_ == "gazebo_ode_joint_states" && !ros::Time::isSimTime()))
    return invalid("selected joint effort source is unavailable or incorrectly configured");
  const double wall = roll_yaw::steadySeconds();
  if(f.now - f.body_stamp > c.feedback_config.timeout)
    return fail("dynamics observation timestamp exceeds maximum age");
  auto coherent = [&](double stamp, double receipt)
  {
    return stamp > 0 && receipt > 0 && wall - receipt <= c.feedback_config.timeout &&
           f.now - stamp >= -c.feedback_config.future_tolerance && f.now - stamp <= c.feedback_config.timeout &&
           std::abs(stamp - f.body_stamp) <= c.synchronization_tolerance &&
           std::abs(stamp - f.joint_stamp) <= c.synchronization_tolerance;
  };
  double oldest = std::min(f.body_stamp, f.joint_stamp), newest = std::max(f.body_stamp, f.joint_stamp);
  Eigen::Vector3d joint_load = Eigen::Vector3d::Zero();
  for(int i = 1; i < 3; ++i)
  {
    const auto it = dynamics_joints_.find(i == 1 ? c.roll : c.yaw);
    if(it == dynamics_joints_.end() || !coherent(it->second.stamp, it->second.receipt) ||
       it->second.stamp != f.joint_stamp || !std::isfinite(it->second.effort) || !it->second.rate_valid)
      return fail("missing/nonfinite/stale synchronized roll/yaw effort or rate");
    joint_load[i] = it->second.effort;
  }
  const double input_stamp = command_wrench_.header.stamp.toSec();
  const double session_stamp = observation_start_stamp_;
  if(f.body_stamp < session_stamp || f.joint_stamp < session_stamp || input_stamp < session_stamp)
    return fail("awaiting body/joint/command samples from the current contact session");
  if(!coherent(input_stamp, command_wrench_receipt_))
    return fail("missing/stale/unsynchronized world command wrench about CoG");
  if(!command_cog_offset_.allFinite())
    return fail("invalid command wrench CoG application point");
  oldest = std::min(oldest, input_stamp);
  newest = std::max(newest, input_stamp);
  if(newest - oldest > c.synchronization_tolerance)
    return fail("full dynamics sample exceeds feedback synchronization tolerance");
  if(auto result = observation_worker_.take())
  {
    observation_pending_ = false;
    if(result->epoch != observation_epoch_) return articulatedLoadValid(c, f);
    if(result->session != c.session || f.now - result->body_stamp > c.feedback_config.timeout ||
       wall - result->receipt > c.feedback_config.timeout || result->body_stamp > f.now + c.feedback_config.future_tolerance)
      return fail("obsolete/stale dynamics worker output");
    articulated_worker_wall_ = result->wall_seconds;
    articulated_worker_tree_calls_ = result->tree_calls;
    if(!result->ok) return fail(result->reason);
    const bool ready = momentum_observer_.observe(result->value.momentum, result->value.predicted_rate,
        result->body_stamp, observer_gain_, c.feedback_config.observer_max_interval);
    articulated_reason_ = momentum_observer_.reason();
    articulated_session_ = result->session;
    articulated_body_stamp_ = result->body_stamp;
    articulated_joint_stamp_ = result->joint_stamp;
    articulated_command_stamp_ = result->command_stamp;
    articulated_jacobian_ = result->jacobian;
    articulated_receipt_ = result->receipt; // Original source capture, never completion time.
    if(!ready) observer_settling_until_ = result->body_stamp + 3 / observer_gain_;
    articulated_valid_ = ready && result->body_stamp >= observer_settling_until_;
    if(ready && !articulated_valid_) articulated_reason_ = "momentum observer settling; contact baseline retained";
    if(!ready && momentum_observer_.stamp() == 0)
      articulated_reason_ = "observer rebase after sample gap; contact baseline retained";
  }
  if(body_history_.empty() || f.body_stamp > body_history_.back().feedback.body_stamp)
  {
    body_history_.push_back({f});
    while(body_history_.size() > 32) body_history_.pop_front();
  }
  if(observation_pending_) return articulatedLoadValid(c, f);
  // Interpolate measured pose/gyro and joint position to one observation time.
  // The applied command is held between publications, never extrapolated from
  // a future input. All subsequent filtering uses this same advancing clock.
  if(body_history_.size() < 2 || joint_history_.size() < 2 || command_history_.empty())
    return fail("awaiting synchronized observation history");
  const double sample_stamp = std::min({f.body_stamp, joint_history_.back().stamp, command_history_.back().stamp});
  if(sample_stamp <= filtered_stamp_)
    return articulatedLoadValid(c, f);
  auto aligned = f;
  bool body_found = false, joints_found = false;
  for(std::size_t i = 1; i < body_history_.size(); ++i)
  {
    const auto& a = body_history_[i - 1].feedback;
    const auto& b = body_history_[i].feedback;
    if(a.body_stamp <= sample_stamp && sample_stamp <= b.body_stamp)
    {
      const double u = (sample_stamp - a.body_stamp) / (b.body_stamp - a.body_stamp);
      aligned.body.translation() = (1 - u) * a.body.translation() + u * b.body.translation();
      aligned.body.linear() = Eigen::Quaterniond(a.body.linear()).slerp(u, Eigen::Quaterniond(b.body.linear())).toRotationMatrix();
      aligned.body_omega = aligned.body.linear().transpose() *
          ((1 - u) * a.body.linear() * a.body_omega + u * b.body.linear() * b.body_omega);
      body_found = b.body_stamp - a.body_stamp <= c.feedback_config.observer_max_interval;
      break;
    }
  }
  for(std::size_t i = 1; i < joint_history_.size(); ++i)
  {
    const auto& a = joint_history_[i - 1];
    const auto& b = joint_history_[i];
    if(a.stamp <= sample_stamp && sample_stamp <= b.stamp)
    {
      const double u = (sample_stamp - a.stamp) / (b.stamp - a.stamp);
      aligned.eta = (1 - u) * a.q + u * b.q;
      joint_load = (1 - u) * a.effort + u * b.effort;
      joints_found = b.stamp - a.stamp <= c.feedback_config.observer_max_interval;
      break;
    }
  }
  const CommandSample* command = nullptr;
  for(const auto& sample : command_history_)
    if(sample.stamp <= sample_stamp) command = &sample;
  if(!body_found || !joints_found || !command || sample_stamp - command->stamp > c.synchronization_tolerance)
    return fail("awaiting bracketing body/joint and applied-command samples");
  roll_yaw::Estimate estimate;
  Eigen::Vector3d q = aligned.eta;
  if(!c.model.estimate(aligned.body, q, estimate)) return fail("passive hinge position unavailable");
  q[0] = estimate.alpha;
  // Signed lock-relative alpha and encoder roll/yaw, all radians. The physical
  // intervals stay inside the angle wrap; never differentiate desired targets.
  if((q.array() < c.planning_config.limits.lower.array()).any() ||
     (q.array() > c.planning_config.limits.upper.array()).any())
    return fail("synchronized measured coordinates outside operating limits");
  filtered_stamp_ = sample_stamp;
  observer_rates_ready_ = observation_filter_.observe(q, aligned.body.linear(), aligned.body_omega,
      joint_load, command->wrench, sample_stamp, c.feedback_config.observer_max_interval);
  observer_gyro_disagreement_ = observation_filter_.gyro - observation_filter_.omega;
  if(!observer_rates_ready_) return fail("observer measured-coordinate derivatives settling after initialization or a measurement gap");
  q = observation_filter_.q;
  const Eigen::Vector3d rate = observation_filter_.rate;
  Eigen::Matrix3d j;
  if(!c.model.jacobian(q, j)) return invalid("body Jacobian unavailable");
  // J(q) maps the filtered measured-coordinate derivative to body-frame rad/s.
  // omega independently differentiates synchronized measured body poses and is
  // filtered/transported into the aligned measured body's axes at sample_stamp.
  // Reduced-model disagreement is a quality diagnostic, not planner validation.
  angular_rate_residual_ = (j * rate - observation_filter_.omega).norm();
  if(!q.allFinite() || !rate.allFinite() || !std::isfinite(angular_rate_residual_))
    return fail("nonfinite measured generalized state or body-rate diagnostic");
  joint_load = observation_filter_.effort;
  const Eigen::Vector3d force = observation_filter_.wrench.head<3>();
  const Eigen::Vector3d moment = observation_filter_.wrench.tail<3>();
  if(!observation_model_) return invalid("immutable dynamics model unavailable");
  const auto model = observation_model_;
  const auto offset = command->offset;
  const auto session = c.session, epoch = observation_epoch_;
  const double body_stamp = sample_stamp, joint_stamp = sample_stamp, command_stamp = command->stamp;
  const double receipt = std::min({command->receipt, dynamics_joints_.at(c.roll).receipt, dynamics_joints_.at(c.yaw).receipt});
  observation_pending_ = observation_worker_.submit(
      [model, q, rate, joint_load, force, moment, offset, session, epoch, body_stamp, joint_stamp, command_stamp, receipt, j]
      (const std::function<bool()>& cancelled)
      {
        Observation result;
        result.session = session;
        result.epoch = epoch;
        result.body_stamp = body_stamp;
        result.joint_stamp = joint_stamp;
        result.command_stamp = command_stamp;
        result.receipt = receipt;
        result.jacobian = j;
        const double started = roll_yaw::steadySeconds();
        roll_yaw::workCounters() = roll_yaw::WorkCounters();
        result.ok = !cancelled() && model->evaluate(q, rate, joint_load, force, moment, offset, result.value, result.reason);
        result.wall_seconds = roll_yaw::steadySeconds() - started;
        result.tree_calls = roll_yaw::workCounters().full_tree;
        return result;
      });
  return articulatedLoadValid(c, f); // Bootstrap and stale output never authorize admittance.

}

void RollYawInteractionRuntime::resetCompliance()
{
  // Motion ownership may reset correction dynamics, but does not own tare.
  const auto previous = admittance_;
  admittance_.reset();
  if(previous.baseline_ready)
  {
    admittance_.baseline_ready = true;
    admittance_.samples = previous.samples;
    admittance_.bias = previous.bias;
  }
  else
    resetCalibration(); // Cancel incomplete collection on a motion reset.
  tare_requested_ = false;
  external_wrench_ = geometry_msgs::WrenchStamped();
  invalidateInteraction();
  compliance_time_ = 0;
  admittance_reason_ = admittance_.baseline_ready
      ? "disabled/ready: motion reset; completed session tare retained"
      : "disabled: hold contact, tare unloaded, then enable and send pitch commands";
}

bool RollYawInteractionRuntime::quietContact(const Input& c) const
{
  return c.lock_valid && c.feedback.hover && c.reference_held && !c.stopping &&
      c.contact_safety_rejection(c.feedback).empty() &&
      (c.plan ? c.plan->ok && !c.plan->goal.align_only && c.stage == roll_yaw::Stage::READY &&
                c.preparation_rejection(c.feedback).empty() : c.quiet_contact);
}

bool RollYawInteractionRuntime::heldContact(const Input& c) const
{
  return c.lock_valid && c.feedback.hover && c.nominal_held && !c.stopping && c.tracking_held &&
      c.contact_safety_rejection(c.feedback).empty() &&
      (!c.plan || (c.plan->ok && !c.plan->goal.align_only && c.stage == roll_yaw::Stage::READY));
}

bool RollYawInteractionRuntime::correctionReleased() const
{
  return std::abs(admittance_.offset) <= 1e-8 && std::abs(admittance_.rate) <= 1e-8 &&
      std::abs(admittance_.acceleration) <= 1e-8;
}

void RollYawInteractionRuntime::admittanceCallback(const Input& c, const std_msgs::BoolConstPtr& msg)
{
  ++enable_request_sequence_;
  enable_requested_ = msg->data;
  if(!msg->data)
  {
    admittance_.enabled = false;
    admittance_reason_ = "disabled; bounded coordinated correction release";
  }
  else if(!interaction_configured_ || !admittance_.config.valid())
  {
    admittance_reason_ =
        "unavailable: invalid source, observer gain, admittance dynamics or legacy wrench mapping";
  }
  else if(interaction_source_ == InteractionSource::ARTICULATED_MOMENTUM && !articulatedLoadValid(c, c.feedback))
  {
    admittance_reason_ = articulated_valid_ && momentum_observer_.ready()
        ? "unavailable: waiting for a fresh valid articulated result"
        : "unavailable: articulated observer: " + articulated_reason_;
  }
  else if(!heldContact(c) || (!admittance_.enabled && !correctionReleased()))
  {
    admittance_reason_ =
        "unavailable: require held nominal contact, measured tracking and released correction";
  }
  else if(!admittance_.baseline_ready)
  {
    admittance_reason_ = tare_requested_
        ? "unavailable: tare collecting; wait for the unloaded baseline to complete"
        : "unavailable: no completed baseline; tare at held unloaded contact with a fresh synchronized load";
  }
  else if(!interaction_valid_ || interaction_stamp_ <= 0 ||
          ros::Time::now().toSec() - interaction_stamp_ > c.feedback_config.timeout ||
          roll_yaw::steadySeconds() - interaction_receipt_ > c.feedback_config.timeout ||
          (articulated() ? c.feedback.now - interaction_stamp_ < -c.feedback_config.future_tolerance :
           std::abs(interaction_stamp_ - c.feedback.body_stamp) > c.synchronization_tolerance))
  {
    admittance_reason_ = articulated()
        ? "unavailable: waiting for a fresh projected articulated interaction"
        : "unavailable: external wrench invalid, stale or unsynchronized with body feedback";
  }
  else
  {
    admittance_.enabled = true;
    admittance_reason_ = "enabled/ready: approximate path interaction; assumed no-cut tare";
  }
  ROS_INFO_STREAM("[RollYaw] " << admittance_reason_);
}
void RollYawInteractionRuntime::tareCallback(const Input& c, const TareRequest& request, ros::Subscriber& retired)
{
  ++tare_request_sequence_;
  const auto session = request.session, generation = request.generation;
  const auto sign = request.sign, scale = request.scale;
  const auto& frame = request.frame;
  const auto& point = request.point;
  if(session != c.session || generation != c.generation || admittance_.enabled ||
     !correctionReleased() || !heldContact(c))
  {
    admittance_reason_ =
        "tare rejected: disable, wait for released correction and held nominal contact; retry after state change";
    return;
  }
  const bool requested_articulated = request.source.source == "articulated_momentum";
  const bool valid_mapping = (sign == 1 || sign == -1) && std::isfinite(scale) && scale > 0 &&
      (frame == "world" || frame == "body") && (point == "cog" || point == "body" || point == "world_origin");
  if(!admittance_.config.valid() ||
     (requested_articulated ? !std::isfinite(request.source.gain) || request.source.gain <= 0 ||
                             !effortSettingsValid(request.source)
                           : request.source.source != "legacy_wrench" || !valid_mapping))
  {
    admittance_reason_ = "tare rejected: invalid source, observer gain, admittance dynamics or legacy wrench mapping";
    return; // Refused requests must leave the completed calibration untouched.
  }
  const bool source_changed = loadInteractionSource(c, retired, request.source);
  if(source_changed && interaction_source_ == InteractionSource::ARTICULATED_MOMENTUM && c.dynamics_factory && c.lock_valid)
    articulated_model_valid_ = c.dynamics_factory(c.model, articulated_model_, articulated_reason_);
  ++observation_epoch_;
  observation_worker_.cancel();
  (void)observation_worker_.take();
  observation_pending_ = false;
  observation_model_ = articulated_model_valid_ ?
      std::make_shared<const perching_dynamics::MultilinkModel>(articulated_model_) : nullptr;
  interaction_sign_ = sign;
  interaction_scale_ = scale;
  interaction_frame_ = frame;
  interaction_point_ = point;
  interaction_configured_ = interactionConfigurationValid();
  resetCalibration();
  admittance_.reset();
  baseline_session_ = c.session;
  tare_requested_ = true;
  ++tare_generation_;
  interaction_stamp_ = interaction_source_ == InteractionSource::LEGACY_WRENCH ? external_wrench_.header.stamp.toSec()
                                                                            : momentum_observer_.stamp();
  interaction_receipt_ = 0;
  admittance_reason_ = "taring: operator asserts no cutting contact; averaging fresh unloaded samples";
}
void RollYawInteractionRuntime::externalWrenchCallback(const Input& c, const geometry_msgs::WrenchStampedConstPtr& msg,
                                                       double receipt)
{
  if(interaction_source_ != InteractionSource::LEGACY_WRENCH)
    return;
  const double t = msg->header.stamp.toSec(), now = ros::Time::now().toSec();
  const auto& force = msg->wrench.force;
  const auto& moment = msg->wrench.torque;
  const std::string expected = interaction_frame_ == "world" ? c.world : c.body_frame;
  if(!std::isfinite(t) || t <= 0 || now - t > c.feedback_config.timeout ||
     now - t < -c.feedback_config.future_tolerance || frameName(msg->header.frame_id) != frameName(expected) ||
     !Eigen::Vector3d(force.x, force.y, force.z).allFinite() ||
     !Eigen::Vector3d(moment.x, moment.y, moment.z).allFinite())
  {
    interaction_receipt_ = 0;
    interaction_valid_ = false;
    admittance_reason_ = "unavailable: external wrench frame/stamp/values invalid";
    return;
  }
  if(t <= external_wrench_.header.stamp.toSec())
    return;
  external_wrench_ = *msg;
  interaction_receipt_ = receipt;
}
bool RollYawInteractionRuntime::observeInteractionSample(const Input& c, const roll_yaw::Feedback& f, double stamp, bool held)
{
  const double direction = c.plan && c.plan->goal.pitch_end < c.plan->goal.pitch_start ? -1 : 1;
  Eigen::VectorXd load;
  Eigen::Vector3d force = Eigen::Vector3d::Zero(), moment = force;
  Eigen::Vector3d q = f.eta;
  bool fresh = false;
  const bool articulated = interaction_source_ == InteractionSource::ARTICULATED_MOMENTUM;
  if(articulated)
  {
    fresh = articulatedLoadValid(c, f);
    stamp = momentum_observer_.stamp();
    if(fresh) { load = momentum_observer_.load(); interaction_receipt_ = articulated_receipt_; }
  }
  else if(interaction_source_ == InteractionSource::LEGACY_WRENCH)
  {
    roll_yaw::Estimate estimate;
    Eigen::Isometry3d body, cog;
    fresh = c.model.estimate(f.body, q, estimate);
    q[0] = estimate.alpha;
    fresh = fresh && c.model.evaluate(q, body, cog);
    if(fresh)
    {
      const auto& w = external_wrench_.wrench;
      force << w.force.x, w.force.y, w.force.z;
      moment << w.torque.x, w.torque.y, w.torque.z;
      if(interaction_frame_ == "body") { force = f.body.linear() * force; moment = f.body.linear() * moment; }
      Eigen::Vector3d source = Eigen::Vector3d::Zero();
      if(interaction_point_ == "body") source = f.body.translation();
      if(interaction_point_ == "cog") source = f.body * (body.inverse() * cog).translation();
      load.resize(6);
      load << force, (moment + source.cross(force)).eval(); // World origin, independent of later pitch targets.
      load *= interaction_sign_ * interaction_scale_;
    }
  }
  if(!fresh || !load.allFinite()) return false;
  if(tare_requested_ && held && stamp > baseline_stamp_)
  {
    if(baseline_sum_.size() != load.size() || (baseline_stamp_ > 0 && stamp - baseline_stamp_ > 2.0))
    { baseline_sum_ = Eigen::VectorXd::Zero(load.size()); admittance_.samples = 0; }
    baseline_sum_ += load;
    if(!baseline_sum_.allFinite()) return false;
    baseline_stamp_ = stamp;
    if(++admittance_.samples >= admittance_.config.baseline_samples)
    {
      baseline_ = baseline_sum_ / admittance_.samples;
      admittance_.baseline_ready = true;
      tare_requested_ = false;
      admittance_reason_ = "disabled/ready: unloaded contact baseline complete; enable explicitly";
    }
    else
      admittance_reason_ = "taring: averaging fresh unloaded samples at held nominal contact";
  }
  if(admittance_.baseline_ready)
  {
    if(baseline_session_ != c.session || baseline_.size() != load.size())
    {
      resetCalibration(); // Changed identity/dimension is not a measurement dropout.
      return false;
    }
    load -= baseline_;
  }
  else
    return false; // Raw loads remain available; task torque requires an applicable tare.
  if(articulated)
  {
    const auto& j = articulated_jacobian_;
    if(!j.allFinite()) return false;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(j, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if(svd.singularValues().minCoeff() < c.planning_config.min_singular_value) return false;
    const Eigen::Vector3d tangent = svd.solve(direction * Eigen::Vector3d::UnitY());
    interaction_torque_ = tangent.dot(load);
    fresh = tangent.allFinite() && (j * tangent - direction * Eigen::Vector3d::UnitY()).norm() <= 1e-8;
  }
  else
    fresh = roll_yaw::pathInteractionTorque(c.model, q, direction, c.planning_config.min_singular_value,
        load.head<3>(), load.tail<3>(), Eigen::Vector3d::Zero(), interaction_torque_);
  fresh = fresh && std::isfinite(interaction_torque_);
  if(fresh)
  {
    // Articulated vector tare owns its sample count and gap policy. The scalar filter's
    // pre-baseline gap reset must not reset that count while retaining its sum.
    if(stamp > interaction_stamp_ && (!articulated || admittance_.baseline_ready))
      admittance_.observe(interaction_torque_, stamp, false, !c.plan);
    interaction_stamp_ = stamp;
  }
  return fresh;
}

RollYawInteractionRuntime::Output RollYawInteractionRuntime::updateInteraction(const Input& c, const roll_yaw::Feedback& f)
{
  updateIdentity(c);
  if((baseline_session_ != 0 && baseline_session_ != c.session) || !c.lock_valid || c.model_identity_changed)
    resetCalibration();
  previous_offset_ = admittance_.offset;
  previous_rate_ = admittance_.rate;
  previous_acceleration_ = admittance_.acceleration;
  Output out;
  const double dt = compliance_time_ > 0 ? f.now - compliance_time_ : 0;
  compliance_time_ = f.now;
  if(tare_requested_ && (!c.nominal_held || c.stopping ||
      (c.plan && (!c.plan->ok || c.plan->goal.align_only || c.stage != roll_yaw::Stage::READY))))
  {
    resetCalibration();
    admittance_reason_ = "tare cancelled: nominal maneuver changed the calibration hold; tare unloaded again";
  }
  if(!c.lock_valid || (c.plan && (!c.plan->ok || c.plan->goal.align_only)))
  {
    invalidateInteraction();
    return out;
  }
  const bool articulated = interaction_source_ == InteractionSource::ARTICULATED_MOMENTUM;
  const double stamp = articulated ? momentum_observer_.stamp() : external_wrench_.header.stamp.toSec();
  bool scalar_failed = false, projection_failed = false;
  bool fresh = interaction_configured_ && c.contact_safety_rejection(f).empty();
  if(articulated)
    // Input synchronization was checked before worker submission. Consume the
    // load/Jacobian pair by its original stamp/receipt, not the newest pose.
    fresh = fresh && articulatedLoadValid(c, f);
  else
    fresh = fresh && stamp > 0 && f.now - stamp >= -c.feedback_config.future_tolerance &&
            f.now - stamp <= c.feedback_config.timeout &&
            std::abs(stamp - f.body_stamp) <= c.synchronization_tolerance && interaction_receipt_ > 0 &&
            roll_yaw::steadySeconds() - interaction_receipt_ <= c.feedback_config.timeout;
  const bool held = heldContact(c);
  if(fresh && !articulated && stamp == interaction_stamp_)
    fresh = interaction_valid_;
  if(fresh && (articulated || stamp > interaction_stamp_))
  {
    fresh = observeInteractionSample(c, f, stamp, held);
    projection_failed = articulated && admittance_.baseline_ready && !fresh && articulatedLoadValid(c, f);
  }
  if(articulated && admittance_.enabled && !admittance_.baseline_ready)
    fresh = false; // Only an explicit calibration/session change requires a new tare.
  if(tare_requested_ && (!fresh || !held))
  {
    admittance_reason_ = "tare waiting for held tracking and a fresh synchronized unloaded source";
  }
  if(admittance_.enabled && !fresh)
    admittance_reason_ =
        "stale/unavailable interaction: bounded path stop, correction braking/held; original goal retained";
  else if(admittance_.enabled)
    admittance_reason_ = "enabled/ready: approximate external work, resistance reduces penetration";
  else if(fresh && admittance_.baseline_ready)
    admittance_reason_ = "disabled/ready: unloaded contact baseline complete; enable explicitly";
  if(articulated && !fresh)
  {
    const bool raw_available = articulatedLoadValid(c, f);
    admittance_reason_ = projection_failed ? "signed path projection unavailable; retrying with retained tare"
        : raw_available && !admittance_.baseline_ready
            ? (tare_requested_ ? "task torque unavailable: collecting unloaded tare"
                               : "task torque unavailable: no completed baseline")
        : "articulated observer unavailable: " + articulated_reason_;
    if(tare_requested_ && (!raw_available || !held))
      admittance_reason_ += "; tare collection paused";
    else if(admittance_.baseline_ready)
      admittance_reason_ += "; completed tare retained";
    else if(admittance_.enabled)
      admittance_reason_ += "; no completed baseline; disarm and tare unloaded";
    if(admittance_.enabled)
      admittance_reason_ += "; feed held with correction braking/held";
  }
  if(!fresh) invalidateInteraction(); // Pause task forcing only; retain raw load and tare.
  // Observation/filtering continues at a hold; integration never advances an
  // invisible correction while explicitly PAUSED, aligning or READY.
  if(((c.plan && c.stage == roll_yaw::Stage::EXECUTING) || !c.plan) && !c.stopping &&
     (admittance_.enabled || std::abs(admittance_.offset) > 1e-10 || std::abs(admittance_.rate) > 1e-10))
  {
    double available = admittance_.config.angle_limit;
    bool mapped = true;
    if(c.plan)
    {
      const auto& plan = *c.plan;
      double angle = 0, cross = 0;
      const Eigen::Matrix3d pre = roll_yaw::cuttingOrientation(plan.neutral, plan.goal, 0, 0, 0).rotation;
      mapped = roll_yaw::taskPitch(pre, plan.cut.desired(c.nominal_progress).rotation, plan.goal.pitch_start, angle, cross);
      available = std::min(c.relief_available,
          std::max(0.0, (plan.goal.pitch_end >= plan.goal.pitch_start ? 1 : -1) * (angle - plan.goal.pitch_start)));
    }
    const double elapsed = c.plan ? dt : c.local_dt;
    const int steps = c.feedback_config.integrationSteps(elapsed);
    auto candidate = admittance_;
    if(!c.plan)
    {
      candidate.config.rate_limit = std::min(candidate.config.rate_limit, c.local_rate_limit);
      candidate.config.acceleration_limit = std::min(candidate.config.acceleration_limit, c.local_acceleration_limit);
    }
    bool integrated = mapped && steps >= 0;
    for(int i = 0; integrated && i < steps; ++i)
      integrated = candidate.step(elapsed / steps, fresh, c.plan != nullptr, available);
    if(integrated)
    {
      candidate.config = admittance_.config; // Physical limits constrain this step, not the configured law.
      admittance_ = candidate;
    }
    else
    {
      fresh = false;
      scalar_failed = true;
      admittance_reason_ = "scalar state unavailable; bounded feed hold";
    }
  }
  // A constrained integration step is not a changed contact calibration.
  // The follower brakes and reconciles its applied correction before retrying.

  interaction_valid_ = fresh;
  out.updated = true;
  out.relief = c.plan ? std::min(0.0, admittance_.offset) : admittance_.offset;
  out.interaction_hold = (admittance_.enabled && !fresh) || scalar_failed;
  out.resisting_load = admittance_.enabled && fresh && admittance_.contact &&
      (c.plan || c.pitch_rate * admittance_.filtered < 0);
  out.relief_enabled = admittance_.enabled;
  out.relief_angle_limit = admittance_.config.angle_limit;
  out.relief_rate_limit = admittance_.config.rate_limit;
  out.relief_acceleration_limit = admittance_.config.acceleration_limit;
  return out;
}
} // namespace aerial_robot_control
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorMultilinkPerchingAdmittanceController,
                       aerial_robot_control::ControlBase);
