// -*- mode: c++ -*-

#include <gimbalrotor/control/gimbalrotor_interaction_controller.h>

#include <aerial_robot_estimation/sensor/imu.h>

#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_eigen.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace aerial_robot_control
{

GimbalrotorInteractionController::GimbalrotorInteractionController()
  : GimbalrotorController()
{
  // Reasonable defaults for the current pitch-perching experiment.
  Vector6d admittance_mass;
  admittance_mass << 5.0, 5.0, 5.0, 0.08, 0.08, 0.08;

  Vector6d admittance_damping;
  admittance_damping << 35.0, 35.0, 35.0, 0.3, 0.3, 0.3;

  Vector6d admittance_stiffness;
  admittance_stiffness << 25.0, 25.0, 25.0, 0.8, 0.8, 0.8;

  interaction_config_.admittance.virtual_mass = diagonalMatrix(admittance_mass);
  interaction_config_.admittance.damping = diagonalMatrix(admittance_damping);
  interaction_config_.admittance.stiffness = diagonalMatrix(admittance_stiffness);

  Vector6d wrench_error_limit;
  wrench_error_limit << 3.0, 3.0, 3.0, 0.8, 0.8, 0.8;
  interaction_config_.admittance.wrench_error_limit = wrench_error_limit;

  Vector6d position_limit;
  position_limit << 0.05, 0.05, 0.05, 0.15, 0.15, 0.15;
  interaction_config_.admittance.position_offset_limit = position_limit;

  Vector6d velocity_limit;
  velocity_limit << 0.10, 0.10, 0.10, 0.50, 0.50, 0.50;
  interaction_config_.admittance.velocity_offset_limit = velocity_limit;

  interaction_config_.admittance.acceleration_offset_limit = Vector6d::Constant(10.0);
  interaction_config_.admittance.enabled_dof = Vector6d::Ones();
  interaction_config_.admittance.wrench_filter_time_constant = 0.03;

  interaction_config_.impedance.inertia = interaction_config_.admittance.virtual_mass;
  interaction_config_.impedance.damping = interaction_config_.admittance.damping;
  interaction_config_.impedance.stiffness = interaction_config_.admittance.stiffness;
  interaction_config_.impedance.enabled_dof = Vector6d::Ones();
  interaction_config_.impedance.wrench_limit = wrench_error_limit;

  Vector6d observer_gain;
  observer_gain << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0;
  interaction_config_.observer.gain = diagonalMatrix(observer_gain);
}

void GimbalrotorInteractionController::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate)
{
  GimbalrotorController::initialize(
      nh,
      nhp,
      robot_model,
      estimator,
      navigator,
      ctrl_loop_rate);

  // GimbalrotorController::initialize() explicitly calls its own rosParamInit,
  // so load this derived class's parameters here.
  GimbalrotorInteractionController::rosParamInit();

  interaction_controller_.configure(interaction_config_);

  interaction_enable_sub_ = nh_.subscribe(interaction_enable_topic_, 1, &GimbalrotorInteractionController::interactionEnableCallback, this);
  estimated_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("estimated_external_wrench", 1);
  constraint_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("interaction/constraint_wrench", 1);

  previous_observer_time_ = ros::Time::now();

  ROS_WARN("[GimbalrotorInteractionController] initialized; mode=%s; enable topic=%s", mode_name_.c_str(), interaction_enable_topic_.c_str());
}

void GimbalrotorInteractionController::reset()
{
  GimbalrotorController::reset();

  interaction_enabled_ = false;
  interaction_controller_.reset();
  estimated_wrench_world_cog_.setZero();
  estimated_wrench_valid_ = false;
  previous_perching_mode_active_ = false;
  previous_observer_time_ = ros::Time::now();
}

void GimbalrotorInteractionController::rosParamInit()
{
  // Safe even though the parent already loaded these parameters.
  GimbalrotorController::rosParamInit();

  ros::NodeHandle interaction_nh(nh_, "controller/interaction");
  ros::NodeHandle admittance_nh(interaction_nh, "admittance");
  ros::NodeHandle impedance_nh(interaction_nh, "impedance");
  ros::NodeHandle observer_nh(interaction_nh, "observer");

  getParam<std::string>(interaction_nh, "mode", mode_name_, std::string("admittance"));

  if(mode_name_ == "impedance")
  {
    mode_ = Mode::IMPEDANCE;
  }
  else
  {
    mode_ = Mode::ADMITTANCE;
    mode_name_ = "admittance";
  }

  getParam<std::string>(interaction_nh, "enable_topic", interaction_enable_topic_, std::string("cutting_enable"));
  getParam<std::string>(interaction_nh, "compliance_frame", compliance_frame_, std::string("world"));
  getParam<bool>(observer_nh, "enabled", observer_enabled_, true);
  getParam<bool>(observer_nh, "only_in_hover_land", observer_only_in_hover_land_, true);

  loadVector6Param(interaction_nh, "free_flight_active_dof", free_flight_active_dof_);

  Vector6d value;

  value = interaction_config_.admittance.virtual_mass.diagonal();
  if(loadVector6Param(admittance_nh, "virtual_mass", value))
  {
    interaction_config_.admittance.virtual_mass = diagonalMatrix(value);
  }

  value = interaction_config_.admittance.damping.diagonal();
  if(loadVector6Param(admittance_nh, "damping", value))
  {
    interaction_config_.admittance.damping = diagonalMatrix(value);
  }

  value = interaction_config_.admittance.stiffness.diagonal();
  if(loadVector6Param(admittance_nh, "stiffness", value))
  {
    interaction_config_.admittance.stiffness = diagonalMatrix(value);
  }

  loadVector6Param(admittance_nh, "reference_wrench", interaction_config_.admittance.reference_wrench);
  loadVector6Param(admittance_nh, "wrench_error_limit", interaction_config_.admittance.wrench_error_limit);
  loadVector6Param(admittance_nh, "position_offset_limit", interaction_config_.admittance.position_offset_limit);
  loadVector6Param(admittance_nh, "velocity_offset_limit", interaction_config_.admittance.velocity_offset_limit);
  loadVector6Param(admittance_nh, "acceleration_offset_limit", interaction_config_.admittance.acceleration_offset_limit);
  getParam<double>(admittance_nh, "wrench_filter_time_constant", interaction_config_.admittance.wrench_filter_time_constant, 0.03);
  getParam<double>(admittance_nh, "maximum_dt", interaction_config_.admittance.maximum_dt, 0.1);

  value = interaction_config_.impedance.inertia.diagonal();
  if(loadVector6Param(impedance_nh, "inertia", value))
  {
    interaction_config_.impedance.inertia = diagonalMatrix(value);
  }

  value = interaction_config_.impedance.damping.diagonal();
  if(loadVector6Param(impedance_nh, "damping", value))
  {
    interaction_config_.impedance.damping = diagonalMatrix(value);
  }

  value = interaction_config_.impedance.stiffness.diagonal();
  if(loadVector6Param(impedance_nh, "stiffness", value))
  {
    interaction_config_.impedance.stiffness = diagonalMatrix(value);
  }

  loadVector6Param(impedance_nh, "wrench_limit", interaction_config_.impedance.wrench_limit);
  getParam<bool>(impedance_nh, "use_inertia_feedforward", interaction_config_.impedance.use_inertia_feedforward, true);

  value = interaction_config_.observer.gain.diagonal();
  if(loadVector6Param(observer_nh, "gain", value))
  {
    interaction_config_.observer.gain = diagonalMatrix(value);
  }

  getParam<double>(observer_nh, "maximum_dt", interaction_config_.observer.maximum_dt, 0.1);
}

void GimbalrotorInteractionController::controlCore()
{
  const ros::Time now = ros::Time::now();
  const double observer_dt = (now - previous_observer_time_).toSec();
  previous_observer_time_ = now;

  /*
   * This reads the command saved by the PREVIOUS control cycle, estimates the
   * current external wrench, and then executes the normal controller. During
   * the normal controller call, modifyControlTarget() uses this estimate.
   */
  updateExternalWrenchEstimate(observer_dt);

  GimbalrotorController::controlCore();
}

void GimbalrotorInteractionController::modifyControlTarget(double dt)
{
  if(!interaction_enabled_ || !estimated_wrench_valid_)
  {
    AdmittanceInput disabled_input;
    disabled_input.enabled = false;
    interaction_controller_.calculateAdmittance(disabled_input);
    return;
  }

  auto perching_navigator = boost::dynamic_pointer_cast<aerial_robot_navigation::GimbalrotorPerchingNavigator>(navigator_);

  const bool perching_mode_active = perching_navigator && perching_navigator->isPerchingEnabled() && perching_navigator->isPerchingLocked();

  // Admittance position/velocity offsets belong to one coordinate system.
  // Reset them when changing between free-flight and perching coordinates.
  if(perching_mode_active != previous_perching_mode_active_)
  {
    interaction_controller_.resetAdmittance();
    previous_perching_mode_active_ = perching_mode_active;
  }

  if(mode_ == Mode::ADMITTANCE)
  {
    if(perching_mode_active)
    {
      applyPerchingAdmittance(*perching_navigator, dt);
    }
    else
    {
      applyFreeFlightAdmittance(dt);
    }
    return;
  }

  if(perching_navigator && perching_navigator->isPerchingEnabled())
  {
    ROS_WARN_THROTTLE(1.0, "[GimbalrotorInteractionController] perching impedance mapping is not enabled in this first version; using PID target only");
    return;
  }

  applyFreeFlightImpedance();
}

void GimbalrotorInteractionController::interactionEnableCallback(const std_msgs::BoolConstPtr& msg)
{
  const bool was_enabled = interaction_enabled_;
  interaction_enabled_ = msg->data;

  if(was_enabled && !interaction_enabled_)
  {
    interaction_controller_.resetAdmittance();
  }

  ROS_WARN("[GimbalrotorInteractionController] interaction enabled: %d", static_cast<int>(interaction_enabled_));
}

void GimbalrotorInteractionController::updateExternalWrenchEstimate(double dt)
{
  estimated_wrench_valid_ = false;

  const bool flight_state_valid = !observer_only_in_hover_land_ || (navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE) || (navigator_->getNaviState() == aerial_robot_navigation::LAND_STATE);

  const Eigen::VectorXd previous_command_dynamic = getTargetWrenchAccCog();

  WrenchObserverInput input;
  input.dt = dt;
  input.enabled = observer_enabled_ && flight_state_valid && previous_command_dynamic.size() == 6;

  if(!input.enabled)
  {
    interaction_controller_.estimateExternalWrench(input);
    return;
  }

  auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu>(estimator_->getImuHandler(0));

  if(!imu_handler)
  {
    ROS_WARN_THROTTLE(1.0, "[GimbalrotorInteractionController] IMU handler is unavailable");
    return;
  }

  Eigen::Vector3d linear_velocity_world;
  Eigen::Vector3d angular_velocity_cog;

  tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), linear_velocity_world);
  tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), angular_velocity_cog);

  const Eigen::Matrix3d rotation_world_cog = getCurrentCogRotationWorld();

  const double mass = gimbalrotor_robot_model_->getMass();
  const Eigen::Matrix3d inertia_cog = gimbalrotor_robot_model_->getInertia<Eigen::Matrix3d>();

  input.momentum.head<3>() = mass * linear_velocity_world;
  input.momentum.tail<3>() = inertia_cog * angular_velocity_cog;

  const Vector6d previous_command = previous_command_dynamic.head<6>();

  // Linear command is converted from COG coordinates to world coordinates.
  input.commanded_effort.head<3>() = rotation_world_cog * (mass * previous_command.head<3>());

  // Rotational observer states remain in COG coordinates, matching the inertia and angular velocity above.
  input.commanded_effort.tail<3>() = inertia_cog * previous_command.tail<3>();

  const Eigen::VectorXd gravity_generalized = mass * robot_model_->getGravity();

  if(gravity_generalized.size() >= 3)
  {
    input.model_bias.head<3>() = gravity_generalized.head<3>();
  }

  input.model_bias.tail<3>() = angular_velocity_cog.cross(inertia_cog * angular_velocity_cog);

  const WrenchObserverOutput observer_output = interaction_controller_.estimateExternalWrench(input);

  if(!observer_output.valid)
  {
    return;
  }

  // The translational observer is in world coordinates. The rotational
  // observer is in COG coordinates, so rotate its result into world before
  // treating the result as a spatial wrench.
  estimated_wrench_world_cog_.head<3>() = observer_output.external_effort.head<3>();
  estimated_wrench_world_cog_.tail<3>() = rotation_world_cog * observer_output.external_effort.tail<3>();

  estimated_wrench_valid_ = estimated_wrench_world_cog_.allFinite();

  if(estimated_wrench_valid_)
  {
    publishEstimatedWrench();
  }
}

void GimbalrotorInteractionController::publishEstimatedWrench() const
{
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
  msg.header.frame_id = "world";

  msg.wrench.force.x = estimated_wrench_world_cog_(0);
  msg.wrench.force.y = estimated_wrench_world_cog_(1);
  msg.wrench.force.z = estimated_wrench_world_cog_(2);
  msg.wrench.torque.x = estimated_wrench_world_cog_(3);
  msg.wrench.torque.y = estimated_wrench_world_cog_(4);
  msg.wrench.torque.z = estimated_wrench_world_cog_(5);

  estimated_wrench_pub_.publish(msg);
}

void GimbalrotorInteractionController::publishConstraintWrench(const Vector6d& wrench_constraint) const
{
  geometry_msgs::WrenchStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "perching_constraint";

  msg.wrench.force.x = wrench_constraint(0);
  msg.wrench.force.y = wrench_constraint(1);
  msg.wrench.force.z = wrench_constraint(2);
  msg.wrench.torque.x = wrench_constraint(3);
  msg.wrench.torque.y = wrench_constraint(4);
  msg.wrench.torque.z = wrench_constraint(5);

  constraint_wrench_pub_.publish(msg);
}

void GimbalrotorInteractionController::applyFreeFlightAdmittance(double dt)
{
  const Eigen::Matrix3d rotation_world_cog = getCurrentCogRotationWorld();

  Vector6d measured_wrench = estimated_wrench_world_cog_;
  Eigen::Matrix3d rotation_world_compliance = Eigen::Matrix3d::Identity();

  if(compliance_frame_ == "cog" ||
     compliance_frame_ == "body" ||
     compliance_frame_ == "base_link")
  {
    measured_wrench = SpatialConstraint::rotateWrench(measured_wrench, rotation_world_cog.transpose());
    rotation_world_compliance = rotation_world_cog;
  }

  AdmittanceInput input;
  input.measured_wrench = measured_wrench;
  input.active_dof = free_flight_active_dof_;
  input.dt = dt;
  input.enabled = true;

  const AdmittanceOutput output = interaction_controller_.calculateAdmittance(input);

  if(!output.valid)
  {
    return;
  }

  const Eigen::Vector3d position_offset_world = rotation_world_compliance * output.position_offset.head<3>();
  const Eigen::Vector3d velocity_offset_world = rotation_world_compliance * output.velocity_offset.head<3>();
  const Eigen::Vector3d acceleration_offset_world = rotation_world_compliance * output.acceleration_offset.head<3>();

  target_pos_ += eigenToTf(position_offset_world);
  target_vel_ += eigenToTf(velocity_offset_world);
  target_acc_ += eigenToTf(acceleration_offset_world);

  const Eigen::Vector3d rotation_offset_world = rotation_world_compliance * output.position_offset.tail<3>();

  tf::Matrix3x3 nominal_rotation_tf;
  nominal_rotation_tf.setRPY(
      target_rpy_.x(),
      target_rpy_.y(),
      target_rpy_.z());

  Eigen::Matrix3d nominal_rotation_world;
  tf::matrixTFToEigen(
      nominal_rotation_tf,
      nominal_rotation_world);

  const Eigen::Matrix3d corrected_rotation_world =
      SpatialConstraint::expSO3(rotation_offset_world)
      * nominal_rotation_world;

  tf::Matrix3x3 corrected_rotation_tf;
  tf::matrixEigenToTF(
      corrected_rotation_world,
      corrected_rotation_tf);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  corrected_rotation_tf.getRPY(roll, pitch, yaw);
  target_rpy_.setValue(roll, pitch, yaw);

  const Eigen::Vector3d angular_velocity_offset_world =
      rotation_world_compliance
      * output.velocity_offset.tail<3>();
  const Eigen::Vector3d angular_acceleration_offset_world =
      rotation_world_compliance
      * output.acceleration_offset.tail<3>();

  target_omega_ += eigenToTf(rotation_world_cog.transpose() * angular_velocity_offset_world);
  target_ang_acc_ += eigenToTf(rotation_world_cog.transpose() * angular_acceleration_offset_world);
}

void GimbalrotorInteractionController::applyPerchingAdmittance(
    aerial_robot_navigation::GimbalrotorPerchingNavigator& navigator,
    double dt)
{
  const SpatialConstraint& constraint =
      navigator.getSpatialConstraint();

  /*
   * The wrench observer is physically defined about COG.
   *
   * Shifting that wrench from COG to the pivot is correct and independent
   * from the base_link arc geometry.
   */
  const Vector6d generalized_effort =
      constraint.generalizedEffortFromWorldCogWrench(
          estimated_wrench_world_cog_,
          getCurrentCogPositionWorld());

  publishConstraintWrench(
      generalized_effort);

  AdmittanceInput input;

  input.measured_wrench =
      generalized_effort;

  input.active_dof =
      constraint.getAllowedDof();

  input.dt =
      dt;

  input.enabled =
      true;

  const AdmittanceOutput output =
      interaction_controller_.calculateAdmittance(
          input);

  const int dof =
      constraint.getSpatialDofIndex();

  if(!output.valid ||
     dof < 0)
  {
    return;
  }

  const double corrected_coordinate =
      navigator.getActiveConstraintCoordinate()
      +
      output.position_offset(dof);

  /*
   * This result is the corrected constrained BASE_LINK target.
   */
  const SpatialConstraintTarget baselink_target =
      navigator.calculateConstrainedTarget(
          corrected_coordinate,
          output.velocity_offset(dof),
          output.acceleration_offset(dof));

  if(!baselink_target.valid)
  {
    return;
  }

  /*
   * Convert only after the base_link constraint has been applied.
   */
  const SpatialConstraintTarget cog_target =
      navigator.convertBaselinkTargetToCogTarget(
          baselink_target);

  if(!cog_target.valid)
  {
    return;
  }

  /*
   * Translational PID tracks COG.
   */
  target_pos_ =
      eigenToTf(
          cog_target.position_world);

  target_vel_ =
      eigenToTf(
          cog_target.linear_velocity_world);

  target_acc_ =
      eigenToTf(
          cog_target.linear_acceleration_world);

  /*
   * Attitude target is the constrained base_link orientation.
   */
  tf::Matrix3x3 target_baselink_rotation_tf;

  tf::matrixEigenToTF(
      baselink_target.rotation_world,
      target_baselink_rotation_tf);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  target_baselink_rotation_tf.getRPY(
      roll,
      pitch,
      yaw);

  target_rpy_.setValue(
      roll,
      pitch,
      yaw);

  /*
   * Angular velocity is the same rigid-body angular motion regardless of
   * whether the reference origin is base_link or COG.
   *
   * Convert the world angular values into the controller's COG coordinates.
   */
  const Eigen::Matrix3d rotation_world_cog =
      getCurrentCogRotationWorld();

  target_omega_ =
      eigenToTf(
          rotation_world_cog.transpose()
          * baselink_target.angular_velocity_world);

  target_ang_acc_ =
      eigenToTf(
          rotation_world_cog.transpose()
          * baselink_target.angular_acceleration_world);
}

Eigen::Matrix3d
GimbalrotorInteractionController::getCurrentCogRotationWorld() const
{
  Eigen::Matrix3d rotation;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), rotation);
  return rotation;
}

Eigen::Vector3d
GimbalrotorInteractionController::getCurrentCogPositionWorld() const
{
  Eigen::Vector3d position;
  tf::vectorTFToEigen(estimator_->getPos(Frame::COG, estimate_mode_), position);
  return position;
}

bool GimbalrotorInteractionController::loadVector6Param(const ros::NodeHandle& nh, const std::string& name, Vector6d& value)
{
  std::vector<double> data;
  if(!nh.getParam(name, data))
  {
    return false;
  }

  if(data.size() != 6)
  {
    ROS_ERROR("[GimbalrotorInteractionController] %s must contain 6 values", name.c_str());
    return false;
  }

  for(int i = 0; i < 6; ++i)
  {
    value(i) = data.at(i);
  }
  return true;
}

Matrix6d GimbalrotorInteractionController::diagonalMatrix(const Vector6d& diagonal)
{
  return diagonal.asDiagonal();
}

tf::Vector3 GimbalrotorInteractionController::eigenToTf(const Eigen::Vector3d& vector)
{
  return tf::Vector3(vector.x(), vector.y(), vector.z());
}

Eigen::Vector3d GimbalrotorInteractionController::tfToEigen(const tf::Vector3& vector)
{
  return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

}  // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(
    aerial_robot_control::GimbalrotorInteractionController,
    aerial_robot_control::ControlBase)
