// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <gimbalrotor/model/gimbalrotor_multilink_perching_model.h>
#include <gimbalrotor/gimbalrotor_perching.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <functional>
#include <memory>

namespace aerial_robot_control
{
// Borrowed, immutable inputs, valid only during main-owner dispatch.
// No executor or mutable navigation target crosses this interface.
struct RollYawInteractionInput
{
  const roll_yaw::Kinematics& model;
  const roll_yaw::ExecutionConfig& feedback_config;
  const roll_yaw::PlanningConfig& planning_config;
  roll_yaw::Feedback feedback;
  const roll_yaw::Plan* plan;
  roll_yaw::Stage stage;
  bool stopping, lock_valid;
  double nominal_progress, relief_available, synchronization_tolerance;
  double last_body_message_stamp, last_joint_message_stamp;
  std::uint64_t session, generation;
  const std::string& roll;
  const std::string& yaw;
  const std::string& world;
  const std::string& body_frame;
  const std::string& prefix;
  const std::function<bool(const roll_yaw::Kinematics&, perching_dynamics::MultilinkModel&, std::string&)>& dynamics_factory;
  const std::function<bool()>& dynamics_snapshot_current;
  // Essential feedback, finite geometry and physical limits; excludes model residual tolerances.
  std::function<std::string(const roll_yaw::Feedback&)> contact_safety_rejection, preparation_rejection;
  bool reference_held = false, quiet_contact = false;
  double pitch_rate = 0, local_dt = 0, local_rate_limit = 0, local_acceleration_limit = 0;
  // Nominal command completion and measured tracking, independent of quietness
  // and of the enabled compliant correction's position/rate.
  bool nominal_held = false, tracking_held = false;
  // A verified identity mismatch, distinct from a temporarily unavailable model.
  bool model_identity_changed = false;
};
struct RollYawInteractionOutput
{
  bool updated = false;
  double relief = 0;
  bool interaction_hold = false, resisting_load = false, relief_enabled = false;
  double relief_angle_limit = 0, relief_rate_limit = 0, relief_acceleration_limit = 0;
};
struct RollYawInteractionDiagnostics
{
  roll_yaw::PitchAdmittance admittance_;
  // Source is selected at startup or a disarmed READY tare, never mid-motion.
  enum class InteractionSource { LEGACY_WRENCH, ARTICULATED_MOMENTUM, INVALID };
  InteractionSource interaction_source_ = InteractionSource::LEGACY_WRENCH;
  std::string interaction_source_name_ = "legacy_wrench", effort_contract_ = "unavailable";
  // Existing ServoBridge nominal effort or the audited Gazebo/ODE JointState source.
  std::string effort_source_ = "unavailable", effort_topic_, effort_producer_;
  bool effort_source_valid_ = false;
  std::string articulated_reason_ = "no locked dynamics snapshot";
  double articulated_worker_wall_ = 0;
  double angular_rate_residual_ = 0;
  Eigen::Vector3d observer_gyro_disagreement_ = Eigen::Vector3d::Zero();
  bool observer_rates_ready_ = false;
  std::uint64_t articulated_worker_tree_calls_ = 0;
  std::string admittance_reason_ = "disarmed";
  std::uint64_t tare_generation_ = 0;
  std::uint64_t tare_request_sequence_ = 0, enable_request_sequence_ = 0;
  bool interaction_configured_ = false, tare_requested_ = false;
  bool enable_requested_ = false;
  bool interaction_valid_ = false;
  std::string interaction_frame_, interaction_point_, interaction_topic_;
  double interaction_sign_ = 0, interaction_scale_ = 0, interaction_torque_ = 0, interaction_stamp_ = 0,
         interaction_receipt_ = 0;
};
class RollYawInteraction;
class RollYawInteractionHost
{
public:
  virtual ~RollYawInteractionHost() = default;
  virtual bool rollYawReferenceActive() const = 0;
  virtual bool attachRollYawInteraction(const std::shared_ptr<RollYawInteraction>&,
                                       const std::function<void(const RollYawInteractionInput&)>& initialize) = 0;
  // Deferred to the update owner, coalesced by key. Callbacks MUST own all
  // captures; the borrowed Input exists only for the duration of invocation.
  virtual void withRollYawInteraction(const std::function<void(const RollYawInteractionInput&)>&,
                                     bool publish_diagnostics = false, const std::string& key = "interaction") = 0;
};

// Navigation holds only a weak interface; the controller owns its implementation.
// Calls and const diagnostic views are valid only under the host session mutex.
class RollYawInteraction
{
public:
  using Input = RollYawInteractionInput;
  using Output = RollYawInteractionOutput;
  using InteractionSource = RollYawInteractionDiagnostics::InteractionSource;
  virtual ~RollYawInteraction() = default;
  virtual const RollYawInteractionDiagnostics& diagnostics() const = 0;
  virtual bool articulated() const = 0;
  virtual bool observerReady() const = 0;
  virtual double observerStamp() const = 0;
  virtual const Eigen::VectorXd& observerLoad() const = 0;
  virtual void captureModel(const Input&) = 0;
  virtual void clearModel() = 0;
  virtual void captureDynamicsJoints(const Input&, const sensor_msgs::JointState&, const std::string& producer, double receipt) = 0;
  virtual bool observeArticulatedSample(const Input&, const roll_yaw::Feedback&) = 0;
  virtual bool articulatedLoadValid(const Input&, const roll_yaw::Feedback&) const = 0;
  virtual void publishArticulatedLoad(const Input&) = 0;
  virtual void resetArticulatedSession(const Input&, const std::string&) = 0;
  virtual void invalidateArticulated(const Input&, const std::string&) = 0;
  virtual void resetCompliance() = 0;
  virtual void stopped() = 0;
  virtual bool motionAllowed(const Input&) const = 0;
  virtual bool quietContact(const Input&) const = 0;
  virtual bool heldContact(const Input&) const = 0;
  virtual Output updateInteraction(const Input&, const roll_yaw::Feedback&) = 0;
  virtual double reconcileReference(bool accepted, bool paused, double applied_relief) = 0;
  virtual double holdReference(double applied_relief) = 0;
  virtual void waitForFeedback(double now) = 0;
};

class GimbalrotorMultilinkPerchingAdmittanceController : public GimbalrotorController
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                  double ctrl_loop_rate) override;
private:
  Eigen::Vector3d angularFeedforward() const override;
  std::shared_ptr<RollYawInteraction> interaction_;
};
} // namespace aerial_robot_control
