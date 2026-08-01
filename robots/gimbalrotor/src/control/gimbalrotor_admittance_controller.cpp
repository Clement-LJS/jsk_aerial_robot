#include <gimbalrotor/control/gimbalrotor_admittance_controller.h>

#include <pluginlib/class_list_macros.h>

namespace aerial_robot_control
{

GimbalrotorAdmittanceController::GimbalrotorAdmittanceController()
  : GimbalrotorController(),
    external_wrench_topic_("estimated_external_wrench"),
    admittance_enable_topic_("admittance_enable"),
    admittance_state_topic_("admittance/active"),
    external_wrench_frame_("world"),
    compliance_frame_("world"),
    admittance_enabled_(false),
    last_reported_admittance_enabled_(false),
    has_external_wrench_(false),
    external_wrench_timeout_(0.15)
{
  raw_external_wrench_.setZero();
}

void GimbalrotorAdmittanceController::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate)
{
  /*
   * Use the normal gimbalrotor controller.
   *
   * This preserves:
   *   - normal pose PID
   *   - normal gimbalrotor allocation
   *   - normal gimbal command
   *   - normal four-axis command
   *
   * This class only inserts admittance correction before normal controlCore().
   */
  GimbalrotorController::initialize(
      nh,
      nhp,
      robot_model,
      estimator,
      navigator,
      ctrl_loop_rate);

  /*
   * GimbalrotorController::initialize() explicitly calls
   * GimbalrotorController::rosParamInit(), not virtual rosParamInit().
   * Therefore, call this class param initialization here.
   */
  GimbalrotorAdmittanceController::rosParamInit();

  admittance_core_.setConfig(admittance_config_);

  /*
   * Initialize the wrench snapshot before subscribing.
   *
   * A callback must never be able to write a valid sample and then have initialize() overwrite it with an empty state.
   */
  {
    std::lock_guard<std::mutex> lock(external_wrench_mutex_);

    raw_external_wrench_.setZero();

    last_external_wrench_receive_time_ = ros::Time(0);
    external_wrench_measurement_stamp_ = ros::Time(0);
    external_wrench_receive_stamp_ = ros::Time(0);

    external_wrench_sequence_ = 0;
    has_external_wrench_ = false;
  }

  prev_admittance_time_ = ros::Time::now();

  external_wrench_sub_ =
      nh_.subscribe(
          external_wrench_topic_,
          1,
          &GimbalrotorAdmittanceController::externalWrenchCallback,
          this);

  admittance_enable_sub_ =
      nh_.subscribe(
          admittance_enable_topic_,
          1,
          &GimbalrotorAdmittanceController::admittanceEnableCallback,
          this);

  admittance_state_pub_ =
      nh_.advertise<std_msgs::Bool>(
          admittance_state_topic_,
          1,
          true);

  ROS_WARN_STREAM("[" << controllerName() << "] initialized.");
  ROS_WARN_STREAM("[" << controllerName() << "] external_wrench_topic: "
                  << external_wrench_topic_);
  ROS_WARN_STREAM("[" << controllerName() << "] admittance_enable_topic: "
                  << admittance_enable_topic_);
  ROS_WARN_STREAM("[" << controllerName() << "] admittance_state_topic: "
                  << admittance_state_topic_);
  ROS_WARN_STREAM("[" << controllerName() << "] external_wrench_frame: "
                  << external_wrench_frame_);
  ROS_WARN_STREAM("[" << controllerName() << "] compliance_frame: "
                  << compliance_frame_);

  publishAdmittanceStateIfChanged(true);
}

void GimbalrotorAdmittanceController::reset()
{
  GimbalrotorController::reset();

  admittance_enabled_ = false;

  {
    std::lock_guard<std::mutex> lock(external_wrench_mutex_);

    raw_external_wrench_.setZero();

    last_external_wrench_receive_time_ = ros::Time(0);
    external_wrench_measurement_stamp_ = ros::Time(0);
    external_wrench_receive_stamp_ = ros::Time(0);

    external_wrench_sequence_ = 0;

    has_external_wrench_ = false;
  }

  admittance_core_.reset();

  admittance_output_ = AdmittanceCoreOutput();

  prev_admittance_time_ = ros::Time::now();
  publishAdmittanceStateIfChanged(true);

  ROS_WARN_STREAM("[" << controllerName() << "] reset.");
}

void GimbalrotorAdmittanceController::rosParamInit()
{
  /*
   * Load normal gimbalrotor params first.
   *
   * This is safe even if already called inside GimbalrotorController::initialize().
   * It only reads parameters.
   */
  GimbalrotorController::rosParamInit();

  ros::NodeHandle imp_nh(nh_, "controller/admittance");

  getParam<bool>(
      imp_nh,
      "use_admittance",
      admittance_config_.use_admittance,
      true);

  getParam<std::string>(
      imp_nh,
      "external_wrench_topic",
      external_wrench_topic_,
      std::string("estimated_external_wrench"));

  getParam<std::string>(
      imp_nh,
      "admittance_enable_topic",
      admittance_enable_topic_,
      std::string("admittance_enable"));

  getParam<std::string>(
      imp_nh,
      "admittance_state_topic",
      admittance_state_topic_,
      std::string("admittance/active"));

  getParam<std::string>(
      imp_nh,
      "external_wrench_frame",
      external_wrench_frame_,
      std::string("world"));

  getParam<std::string>(
      imp_nh,
      "compliance_frame",
      compliance_frame_,
      std::string("world"));

  getParam<double>(
      imp_nh,
      "force_lpf_alpha",
      admittance_config_.force_lpf_alpha,
      0.2);

  getParam<double>(
      imp_nh,
      "torque_lpf_alpha",
      admittance_config_.torque_lpf_alpha,
      0.2);

  getParam<double>(
      imp_nh,
      "enable_x",
      admittance_config_.trans_enable(0),
      1.0);

  getParam<double>(
      imp_nh,
      "enable_y",
      admittance_config_.trans_enable(1),
      1.0);

  getParam<double>(
      imp_nh,
      "enable_z",
      admittance_config_.trans_enable(2),
      1.0);

  getParam<double>(
      imp_nh,
      "enable_roll",
      admittance_config_.rot_enable(0),
      1.0);

  getParam<double>(
      imp_nh,
      "enable_pitch",
      admittance_config_.rot_enable(1),
      1.0);

  getParam<double>(
      imp_nh,
      "enable_yaw",
      admittance_config_.rot_enable(2),
      1.0);

  getParam<double>(
      imp_nh,
      "mass_x",
      admittance_config_.trans_virtual_mass(0),
      5.0);

  getParam<double>(
      imp_nh,
      "mass_y",
      admittance_config_.trans_virtual_mass(1),
      5.0);

  getParam<double>(
      imp_nh,
      "mass_z",
      admittance_config_.trans_virtual_mass(2),
      5.0);

  getParam<double>(
      imp_nh,
      "damping_x",
      admittance_config_.trans_damping(0),
      30.0);

  getParam<double>(
      imp_nh,
      "damping_y",
      admittance_config_.trans_damping(1),
      30.0);

  getParam<double>(
      imp_nh,
      "damping_z",
      admittance_config_.trans_damping(2),
      30.0);

  getParam<double>(
      imp_nh,
      "stiffness_x",
      admittance_config_.trans_stiffness(0),
      60.0);

  getParam<double>(
      imp_nh,
      "stiffness_y",
      admittance_config_.trans_stiffness(1),
      60.0);

  getParam<double>(
      imp_nh,
      "stiffness_z",
      admittance_config_.trans_stiffness(2),
      60.0);

  getParam<double>(
      imp_nh,
      "force_ref_x",
      admittance_config_.force_ref(0),
      0.0);

  getParam<double>(
      imp_nh,
      "force_ref_y",
      admittance_config_.force_ref(1),
      0.0);

  getParam<double>(
      imp_nh,
      "force_ref_z",
      admittance_config_.force_ref(2),
      0.0);

  getParam<double>(
      imp_nh,
      "force_limit_x",
      admittance_config_.force_limit(0),
      3.0);

  getParam<double>(
      imp_nh,
      "force_limit_y",
      admittance_config_.force_limit(1),
      3.0);

  getParam<double>(
      imp_nh,
      "force_limit_z",
      admittance_config_.force_limit(2),
      3.0);

  getParam<double>(
      imp_nh,
      "pos_offset_limit_x",
      admittance_config_.pos_offset_limit(0),
      0.05);

  getParam<double>(
      imp_nh,
      "pos_offset_limit_y",
      admittance_config_.pos_offset_limit(1),
      0.05);

  getParam<double>(
      imp_nh,
      "pos_offset_limit_z",
      admittance_config_.pos_offset_limit(2),
      0.05);

  getParam<double>(
      imp_nh,
      "vel_offset_limit_x",
      admittance_config_.vel_offset_limit(0),
      0.10);

  getParam<double>(
      imp_nh,
      "vel_offset_limit_y",
      admittance_config_.vel_offset_limit(1),
      0.10);

  getParam<double>(
      imp_nh,
      "vel_offset_limit_z",
      admittance_config_.vel_offset_limit(2),
      0.10);

  getParam<double>(
      imp_nh,
      "inertia_roll",
      admittance_config_.rot_virtual_inertia(0),
      0.08);

  getParam<double>(
      imp_nh,
      "inertia_pitch",
      admittance_config_.rot_virtual_inertia(1),
      0.08);

  getParam<double>(
      imp_nh,
      "inertia_yaw",
      admittance_config_.rot_virtual_inertia(2),
      0.08);

  getParam<double>(
      imp_nh,
      "rot_damping_roll",
      admittance_config_.rot_damping(0),
      0.35);

  getParam<double>(
      imp_nh,
      "rot_damping_pitch",
      admittance_config_.rot_damping(1),
      0.35);

  getParam<double>(
      imp_nh,
      "rot_damping_yaw",
      admittance_config_.rot_damping(2),
      0.35);

  getParam<double>(
      imp_nh,
      "rot_stiffness_roll",
      admittance_config_.rot_stiffness(0),
      0.8);

  getParam<double>(
      imp_nh,
      "rot_stiffness_pitch",
      admittance_config_.rot_stiffness(1),
      0.8);

  getParam<double>(
      imp_nh,
      "rot_stiffness_yaw",
      admittance_config_.rot_stiffness(2),
      0.8);

  getParam<double>(
      imp_nh,
      "torque_ref_roll",
      admittance_config_.torque_ref(0),
      0.0);

  getParam<double>(
      imp_nh,
      "torque_ref_pitch",
      admittance_config_.torque_ref(1),
      0.0);

  getParam<double>(
      imp_nh,
      "torque_ref_yaw",
      admittance_config_.torque_ref(2),
      0.0);

  getParam<double>(
      imp_nh,
      "torque_limit_roll",
      admittance_config_.torque_limit(0),
      0.8);

  getParam<double>(
      imp_nh,
      "torque_limit_pitch",
      admittance_config_.torque_limit(1),
      0.8);

  getParam<double>(
      imp_nh,
      "torque_limit_yaw",
      admittance_config_.torque_limit(2),
      0.8);

  getParam<double>(
      imp_nh,
      "angle_offset_limit_roll",
      admittance_config_.angle_offset_limit(0),
      0.15);

  getParam<double>(
      imp_nh,
      "angle_offset_limit_pitch",
      admittance_config_.angle_offset_limit(1),
      0.15);

  getParam<double>(
      imp_nh,
      "angle_offset_limit_yaw",
      admittance_config_.angle_offset_limit(2),
      0.15);

  getParam<double>(
      imp_nh,
      "angular_vel_offset_limit_roll",
      admittance_config_.angular_vel_offset_limit(0),
      0.50);

  getParam<double>(
      imp_nh,
      "angular_vel_offset_limit_pitch",
      admittance_config_.angular_vel_offset_limit(1),
      0.50);

  getParam<double>(
      imp_nh,
      "angular_vel_offset_limit_yaw",
      admittance_config_.angular_vel_offset_limit(2),
      0.50);

  getParam<double>(
      imp_nh,
      "external_wrench_timeout",
      external_wrench_timeout_,
      0.15);
  
  admittance_core_.setConfig(admittance_config_);

  ROS_INFO_STREAM("[" << controllerName() << "] use_admittance: "
                  << admittance_config_.use_admittance);

  ROS_INFO_STREAM("[" << controllerName() << "] enable xyz: "
                  << admittance_config_.trans_enable(0) << ", "
                  << admittance_config_.trans_enable(1) << ", "
                  << admittance_config_.trans_enable(2));

  ROS_INFO_STREAM("[" << controllerName() << "] enable rpy: "
                  << admittance_config_.rot_enable(0) << ", "
                  << admittance_config_.rot_enable(1) << ", "
                  << admittance_config_.rot_enable(2));
}

void GimbalrotorAdmittanceController::controlCore()
{
  const ros::Time now = ros::Time::now();
  const double dt = (now - prev_admittance_time_).toSec();
  prev_admittance_time_ = now;

  AdmittanceCoreInput input;
  input.external_wrench_world = getExternalWrenchWorld();
  input.R_world_compliance = getComplianceToWorldRotation();
  input.dt = dt;
  input.enabled = admittance_enabled_;

  admittance_output_ = admittance_core_.update(input);

  /*
   * Important architecture:
   *
   * AdmittanceCore does NOT touch navigator.
   * This wrapper temporarily injects the offset only because the existing
   * PoseLinearController reads targets from navigator_ inside controlCore().
   *
   * After normal GimbalrotorController::controlCore(), original navigation
   * target is restored immediately.
   */
  const tf::Vector3 original_target_pos =
      navigator_->getTargetPos();

  const tf::Vector3 original_target_rpy =
      navigator_->getTargetRPY();

  if(admittance_output_.valid)
    {
      applyAdmittanceOutputToNavigator(
          original_target_pos,
          original_target_rpy,
          admittance_output_);
    }

  GimbalrotorController::controlCore();

  /*
   * Restore navigation target so admittance does not become navigation logic.
   * Navigation remains navigation.
   * Admittance remains only a correction used during this controller cycle.
   */
  if(admittance_output_.valid)
    {
      navigator_->setTargetPos(original_target_pos);
      navigator_->setTargetRPY(original_target_rpy);
    }

  publishAdmittanceStateIfChanged();
}

void GimbalrotorAdmittanceController::applyAdmittanceOutputToNavigator(
    const tf::Vector3& original_target_pos,
    const tf::Vector3& original_target_rpy,
    const AdmittanceCoreOutput& output)
{
  const Eigen::Vector3d original_target_pos_eigen =
      tfVector3ToEigen(original_target_pos);

  const Eigen::Vector3d modified_target_pos_eigen =
      original_target_pos_eigen + output.pos_offset_world;

  tf::Vector3 modified_target_rpy = original_target_rpy;
  modified_target_rpy.setX(
      modified_target_rpy.x() + output.rpy_offset_world(0));
  modified_target_rpy.setY(
      modified_target_rpy.y() + output.rpy_offset_world(1));
  modified_target_rpy.setZ(
      modified_target_rpy.z() + output.rpy_offset_world(2));

  navigator_->setTargetPos(
      eigenToTfVector3(modified_target_pos_eigen));

  navigator_->setTargetRPY(modified_target_rpy);
}

void GimbalrotorAdmittanceController::externalWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(external_wrench_mutex_);

  raw_external_wrench_(0) = msg->wrench.force.x;
  raw_external_wrench_(1) = msg->wrench.force.y;
  raw_external_wrench_(2) = msg->wrench.force.z;
  raw_external_wrench_(3) = msg->wrench.torque.x;
  raw_external_wrench_(4) = msg->wrench.torque.y;
  raw_external_wrench_(5) = msg->wrench.torque.z;

  external_wrench_measurement_stamp_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  external_wrench_receive_stamp_ = ros::Time::now();
  last_external_wrench_receive_time_ = external_wrench_receive_stamp_;

  ++external_wrench_sequence_;
  has_external_wrench_ = true;
}

void GimbalrotorAdmittanceController::admittanceEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
  const bool previous_enabled = admittance_enabled_;

  if(previous_enabled == msg->data)
  {
    /*
     * Ignore repeated copies of the same command.
     * The current state has already been published through the latched admittance_state_pub_.
     */
    return;
  }

  admittance_enabled_ = msg->data;

  /*
   * Reset only on a real enabled -> disabled transition.
   */
  if(previous_enabled && !admittance_enabled_)
  {
    admittance_core_.reset();
    admittance_output_ = AdmittanceCoreOutput();

    prev_admittance_time_ = ros::Time::now();
  }

  publishAdmittanceStateIfChanged();
}

void GimbalrotorAdmittanceController::publishAdmittanceStateIfChanged(bool force)
{
  if(!force && admittance_enabled_ == last_reported_admittance_enabled_) return;

  std_msgs::Bool state_msg;
  state_msg.data = admittance_enabled_;
  admittance_state_pub_.publish(state_msg);

  last_reported_admittance_enabled_ = admittance_enabled_;

  ROS_WARN(
      "[%s] admittance %s",
      controllerName(),
      admittance_enabled_ ? "enabled" : "disabled");
}

GimbalrotorAdmittanceController::ExternalWrenchSnapshot
GimbalrotorAdmittanceController::getExternalWrenchSnapshot() const
{
  ExternalWrenchSnapshot snapshot;

  std::lock_guard<std::mutex> lock(external_wrench_mutex_);

  snapshot.raw_wrench = raw_external_wrench_;
  snapshot.measurement_stamp = external_wrench_measurement_stamp_;
  snapshot.receive_stamp = external_wrench_receive_stamp_;
  snapshot.sequence = external_wrench_sequence_;
  snapshot.available = has_external_wrench_;

  return snapshot;
}

Eigen::Matrix<double, 6, 1>
GimbalrotorAdmittanceController::transformExternalWrenchToWorld(const Eigen::Matrix<double, 6, 1>& raw_wrench) const
{
  /*
   * The momentum observer used for the experiment publishes force and torque components in world axes.
   */
  if(external_wrench_frame_ == "world" || external_wrench_frame_ == "map" || external_wrench_frame_ == "odom")
  {
    return raw_wrench;
  }

  /*
   * Preserve support for another source that publishes force and torque components in the robot body/CoG axes.
   */
  if(external_wrench_frame_ == "cog" || external_wrench_frame_ == "body" || external_wrench_frame_ == "base_link")
  {
    const tf::Matrix3x3 R_world_cog_tf = estimator_->getOrientation(Frame::COG, estimate_mode_);

    Eigen::Matrix3d R_world_cog;

    tf::matrixTFToEigen(R_world_cog_tf, R_world_cog);

    Eigen::Matrix<double, 6, 1> wrench_world;

    wrench_world.head<3>() = R_world_cog * raw_wrench.head<3>();
    wrench_world.tail<3>() = R_world_cog * raw_wrench.tail<3>();

    return wrench_world;
  }

  ROS_WARN_THROTTLE(
      1.0,
      "[%s] Unknown external_wrench_frame [%s]. "
      "Treating the input as world frame.",
      controllerName(),
      external_wrench_frame_.c_str());

  return raw_wrench;
}

Eigen::Matrix<double, 6, 1>
GimbalrotorAdmittanceController::getExternalWrenchWorld() const
{
  const ExternalWrenchSnapshot snapshot = getExternalWrenchSnapshot();

  if(!snapshot.available)
  {
    ROS_WARN_THROTTLE(1.0, "[%s] No external wrench has been received.", controllerName());

    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  const double wrench_age = (ros::Time::now() - snapshot.receive_stamp).toSec();

  if(!std::isfinite(wrench_age) || wrench_age < 0.0 || wrench_age > external_wrench_timeout_)
  {
    ROS_WARN_THROTTLE(1.0, "[%s] External wrench is stale: %.3f s.", controllerName(), wrench_age);

    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  const Eigen::Matrix<double, 6, 1> wrench_world = transformExternalWrenchToWorld(snapshot.raw_wrench);

  if(!wrench_world.allFinite())
  {
    ROS_ERROR_THROTTLE(1.0, "[%s] Rejected non-finite external wrench.", controllerName());

    return Eigen::Matrix<double, 6, 1>::Zero();
  }

  return wrench_world;
}

Eigen::Matrix3d
GimbalrotorAdmittanceController::getComplianceToWorldRotation() const
{
  if(compliance_frame_ == "world" ||
     compliance_frame_ == "map" ||
     compliance_frame_ == "odom")
    {
      return Eigen::Matrix3d::Identity();
    }

  if(compliance_frame_ == "cog" ||
     compliance_frame_ == "body" ||
     compliance_frame_ == "base_link")
    {
      tf::Matrix3x3 R_world_cog_tf =
          estimator_->getOrientation(Frame::COG, estimate_mode_);

      Eigen::Matrix3d R_world_cog;
      tf::matrixTFToEigen(R_world_cog_tf, R_world_cog);

      return R_world_cog;
    }

  ROS_WARN_THROTTLE(
      1.0,
      "[%s] Unknown compliance_frame [%s]. Use world frame.",
      controllerName(),
      compliance_frame_.c_str());

  return Eigen::Matrix3d::Identity();
}

tf::Vector3 GimbalrotorAdmittanceController::eigenToTfVector3(
    const Eigen::Vector3d& v) const
{
  return tf::Vector3(v(0), v(1), v(2));
}

Eigen::Vector3d GimbalrotorAdmittanceController::tfVector3ToEigen(
    const tf::Vector3& v) const
{
  return Eigen::Vector3d(v.x(), v.y(), v.z());
}

} // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(
    aerial_robot_control::GimbalrotorAdmittanceController,
    aerial_robot_control::ControlBase)
