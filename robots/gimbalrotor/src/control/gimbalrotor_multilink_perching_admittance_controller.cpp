// -*- mode: c++ -*-

#include <gimbalrotor/control/gimbalrotor_multilink_perching_admittance_controller.h>

#include <pluginlib/class_list_macros.h>

#include <cmath>

namespace aerial_robot_control
{

GimbalrotorMultilinkPerchingAdmittanceController::
GimbalrotorMultilinkPerchingAdmittanceController()
  : GimbalrotorPerchingAdmittanceController(),
    multilink_navigator_valid_(false),
    last_seen_mechanism_generation_(INVALID_GENERATION),
    last_tared_mechanism_generation_(INVALID_GENERATION),
    pitch_external_torque_(0.0),
    pitch_equilibrium_torque_(0.0),
    pitch_residual_torque_(0.0)
{
}

void GimbalrotorMultilinkPerchingAdmittanceController::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate)
{
  GimbalrotorPerchingAdmittanceController::initialize(
      nh,
      nhp,
      robot_model,
      estimator,
      navigator,
      ctrl_loop_rate);

  multilink_navigator_ = boost::dynamic_pointer_cast<
      aerial_robot_navigation::GimbalrotorMultilinkPerchingNavigator>(
          navigator);
  multilink_navigator_valid_ = static_cast<bool>(multilink_navigator_);

  pitch_external_torque_pub_ = nh_.advertise<std_msgs::Float64>(
      "perching/multilink/pitch_joint_external_torque", 1);
  pitch_equilibrium_torque_pub_ = nh_.advertise<std_msgs::Float64>(
      "perching/multilink/pitch_joint_equilibrium_torque", 1);
  pitch_residual_torque_pub_ = nh_.advertise<std_msgs::Float64>(
      "perching/multilink/pitch_joint_residual_torque", 1);

  if(multilink_navigator_valid_)
  {
    last_seen_mechanism_generation_ =
        multilink_navigator_->mechanismTargetGeneration();
    ROS_WARN(
        "[GimbalrotorMultilinkPerchingAdmittanceController] initialized.");
  }
  else
  {
    ROS_ERROR(
        "[GimbalrotorMultilinkPerchingAdmittanceController] requires "
        "GimbalrotorMultilinkPerchingNavigator. Multilink admittance "
        "will remain inactive.");
  }
}

void GimbalrotorMultilinkPerchingAdmittanceController::reset()
{
  if(multilink_navigator_)
    multilink_navigator_->clearPitchAdmittanceOffset();

  GimbalrotorPerchingAdmittanceController::reset();

  const std::uint64_t current_generation = multilink_navigator_ ?
      multilink_navigator_->mechanismTargetGeneration() :
      INVALID_GENERATION;
  std::lock_guard<std::mutex> lock(multilink_controller_mutex_);
  last_seen_mechanism_generation_ = current_generation;
  last_tared_mechanism_generation_ = INVALID_GENERATION;
  pitch_external_torque_ = 0.0;
  pitch_equilibrium_torque_ = 0.0;
  pitch_residual_torque_ = 0.0;
}

void GimbalrotorMultilinkPerchingAdmittanceController::controlCore()
{
  bool invalidate_tare = false;
  bool clear_pitch_offset = false;
  std::uint64_t current_generation = INVALID_GENERATION;

  if(multilink_navigator_valid_)
  {
    current_generation =
        multilink_navigator_->mechanismTargetGeneration();
    const bool secondary_settled =
        multilink_navigator_->secondaryJointSettled();

    std::lock_guard<std::mutex> lock(multilink_controller_mutex_);
    if(current_generation != last_seen_mechanism_generation_)
    {
      last_seen_mechanism_generation_ = current_generation;
      last_tared_mechanism_generation_ = INVALID_GENERATION;
      invalidate_tare = true;
      clear_pitch_offset = true;
    }
    else if(last_tared_mechanism_generation_ != INVALID_GENERATION &&
            !secondary_settled)
    {
      last_tared_mechanism_generation_ = INVALID_GENERATION;
      invalidate_tare = true;
      clear_pitch_offset = true;
    }
  }

  if(invalidate_tare)
  {
    invalidatePerchingAdmittanceTare();
    ROS_WARN(
        "[GimbalrotorMultilinkPerchingAdmittanceController] mechanism "
        "configuration changed; pitch admittance was disarmed and a "
        "fresh equilibrium tare is required.");
  }

  if(clear_pitch_offset && multilink_navigator_)
    multilink_navigator_->clearPitchAdmittanceOffset();

  GimbalrotorPerchingAdmittanceController::controlCore();

  if(multilink_navigator_valid_ &&
     multilink_navigator_->multilinkModelValid() &&
     multilink_navigator_->multilinkLockValid() &&
     multilink_navigator_->secondaryJointSettled() &&
     perchingEquilibriumTareReady())
  {
    const std::uint64_t generation_after_control =
        multilink_navigator_->mechanismTargetGeneration();
    std::lock_guard<std::mutex> lock(multilink_controller_mutex_);
    if(generation_after_control == last_seen_mechanism_generation_)
      last_tared_mechanism_generation_ = generation_after_control;
  }

  if(!multilink_navigator_valid_ ||
     !multilink_navigator_->multilinkLockValid() ||
     multilink_navigator_->getNaviState() !=
         aerial_robot_navigation::HOVER_STATE)
  {
    if(multilink_navigator_)
      multilink_navigator_->clearPitchAdmittanceOffset();
  }

  publishJointTorqueDiagnostics();
}

bool GimbalrotorMultilinkPerchingAdmittanceController::
allowPerchingEquilibriumTareCollection() const
{
  return multilink_navigator_valid_ &&
         multilink_navigator_->getNaviState() ==
             aerial_robot_navigation::HOVER_STATE &&
         multilink_navigator_->multilinkModelValid() &&
         multilink_navigator_->multilinkLockValid() &&
         multilink_navigator_->secondaryJointSettled();
}

bool GimbalrotorMultilinkPerchingAdmittanceController::
allowPerchingAdmittanceArming() const
{
  if(!allowPerchingEquilibriumTareCollection())
    return false;

  const std::uint64_t generation =
      multilink_navigator_->mechanismTargetGeneration();
  std::lock_guard<std::mutex> lock(multilink_controller_mutex_);
  return generation == last_seen_mechanism_generation_ &&
         generation == last_tared_mechanism_generation_;
}

bool GimbalrotorMultilinkPerchingAdmittanceController::
computeResidualPerchingPitchTorque(
    const Eigen::Matrix<double, 6, 1>& wrench_cog_world,
    const Eigen::Matrix<double, 6, 1>& wrench_pivot_world,
    const Eigen::Matrix<double, 6, 1>& equilibrium_wrench_pivot_world,
    const Eigen::Vector3d& cog_pos_world,
    const Eigen::Vector3d& pivot_pos_world,
    const Eigen::Matrix3d& R_world_constraint,
    const Eigen::Vector3d& constraint_axis_world,
    double& residual_pitch_torque,
    Eigen::Vector3d& pitch_axis_world) const
{
  (void)wrench_pivot_world;
  (void)R_world_constraint;
  (void)constraint_axis_world;

  Eigen::Vector3d pitch_origin_world;
  if(!multilink_navigator_valid_ ||
     !multilink_navigator_->getPitchJointKinematicsWorld(
         pitch_origin_world, pitch_axis_world))
    return false;

  if(!wrench_cog_world.allFinite() ||
     !equilibrium_wrench_pivot_world.allFinite() ||
     !cog_pos_world.allFinite() || !pivot_pos_world.allFinite() ||
     !pitch_origin_world.allFinite() || !pitch_axis_world.allFinite() ||
     pitch_axis_world.norm() <= 1.0e-6)
    return false;

  pitch_axis_world.normalize();

  /*
   * This is a kinematic projection of the existing rigid-body wrench
   * estimate, not a full multibody external-wrench observer.
   */
  const Eigen::Vector3d force_world = wrench_cog_world.head<3>();
  const Eigen::Vector3d torque_pitch_world =
      wrench_cog_world.tail<3>() +
      (cog_pos_world - pitch_origin_world).cross(force_world);
  const double external_torque =
      pitch_axis_world.dot(torque_pitch_world);

  const Eigen::Vector3d equilibrium_force_world =
      equilibrium_wrench_pivot_world.head<3>();
  const Eigen::Vector3d equilibrium_torque_pitch_world =
      equilibrium_wrench_pivot_world.tail<3>() +
      (pivot_pos_world - pitch_origin_world)
          .cross(equilibrium_force_world);
  const double equilibrium_torque =
      pitch_axis_world.dot(equilibrium_torque_pitch_world);

  residual_pitch_torque =
      perchingPitchTorqueSign() *
      (external_torque - equilibrium_torque);

  if(!std::isfinite(external_torque) ||
     !std::isfinite(equilibrium_torque) ||
     !std::isfinite(residual_pitch_torque))
    return false;

  {
    std::lock_guard<std::mutex> lock(multilink_controller_mutex_);
    pitch_external_torque_ = external_torque;
    pitch_equilibrium_torque_ = equilibrium_torque;
    pitch_residual_torque_ = residual_pitch_torque;
  }

  return true;
}

Eigen::Matrix3d GimbalrotorMultilinkPerchingAdmittanceController::
getComplianceToWorldRotation() const
{
  Eigen::Vector3d origin_world;
  Eigen::Vector3d y_axis;
  if(!multilink_navigator_valid_ ||
     !multilink_navigator_->getPitchJointKinematicsWorld(
         origin_world, y_axis) ||
     !y_axis.allFinite() || y_axis.norm() <= 1.0e-6)
    return Eigen::Matrix3d::Identity();

  y_axis.normalize();
  Eigen::Vector3d reference = Eigen::Vector3d::UnitZ();
  if(std::abs(reference.dot(y_axis)) > 0.90)
    reference = Eigen::Vector3d::UnitX();

  Eigen::Vector3d x_axis = reference.cross(y_axis);
  if(!x_axis.allFinite() || x_axis.norm() <= 1.0e-6)
    return Eigen::Matrix3d::Identity();
  x_axis.normalize();

  Eigen::Vector3d z_axis = x_axis.cross(y_axis);
  if(!z_axis.allFinite() || z_axis.norm() <= 1.0e-6)
    return Eigen::Matrix3d::Identity();
  z_axis.normalize();

  Eigen::Matrix3d rotation;
  rotation.col(0) = x_axis;
  rotation.col(1) = y_axis;
  rotation.col(2) = z_axis;
  if(!rotation.allFinite() ||
     std::abs(rotation.determinant() - 1.0) > 1.0e-3)
    return Eigen::Matrix3d::Identity();
  return rotation;
}

void GimbalrotorMultilinkPerchingAdmittanceController::
applyAdmittanceOutputToNavigator(
    const tf::Vector3& original_target_pos,
    const tf::Vector3& original_target_rpy,
    const AdmittanceCoreOutput& output)
{
  if(!multilink_navigator_valid_)
    return;

  if(!multilink_navigator_->multilinkLockValid())
  {
    GimbalrotorAdmittanceController::applyAdmittanceOutputToNavigator(
        original_target_pos, original_target_rpy, output);
    return;
  }

  if(multilink_navigator_->getNaviState() !=
         aerial_robot_navigation::HOVER_STATE ||
     !multilink_navigator_->secondaryJointSettled() ||
     !perchingPitchOutputFinite(output))
  {
    multilink_navigator_->clearPitchAdmittanceOffset();
    return;
  }

  multilink_navigator_->setPitchAdmittanceOffset(
      output.angle_offset_compliance(1));
}

void GimbalrotorMultilinkPerchingAdmittanceController::
publishJointTorqueDiagnostics() const
{
  double external_torque = 0.0;
  double equilibrium_torque = 0.0;
  double residual_torque = 0.0;
  {
    std::lock_guard<std::mutex> lock(multilink_controller_mutex_);
    external_torque = pitch_external_torque_;
    equilibrium_torque = pitch_equilibrium_torque_;
    residual_torque = pitch_residual_torque_;
  }

  std_msgs::Float64 msg;
  msg.data = external_torque;
  pitch_external_torque_pub_.publish(msg);
  msg.data = equilibrium_torque;
  pitch_equilibrium_torque_pub_.publish(msg);
  msg.data = residual_torque;
  pitch_residual_torque_pub_.publish(msg);
}

}  // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(
    aerial_robot_control::GimbalrotorMultilinkPerchingAdmittanceController,
    aerial_robot_control::ControlBase)
