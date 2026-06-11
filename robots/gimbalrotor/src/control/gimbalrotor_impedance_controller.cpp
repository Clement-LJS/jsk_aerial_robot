#include <gimbalrotor/control/gimbalrotor_impedance_controller.h>

#include <pluginlib/class_list_macros.h>

namespace aerial_robot_control
{

GimbalrotorImpedanceController::GimbalrotorImpedanceController()
  : GimbalrotorController(),
    external_wrench_topic_("estimated_external_wrench"),
    impedance_enable_topic_("impedance_enable"),
    external_wrench_frame_("world"),
    compliance_frame_("world"),
    impedance_enabled_(false)
{
  raw_external_wrench_.setZero();
}

void GimbalrotorImpedanceController::initialize(
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
   * This class only inserts impedance correction before normal controlCore().
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
  GimbalrotorImpedanceController::rosParamInit();

  impedance_core_.setConfig(impedance_config_);

  external_wrench_sub_ =
      nh_.subscribe(
          external_wrench_topic_,
          1,
          &GimbalrotorImpedanceController::externalWrenchCallback,
          this);

  impedance_enable_sub_ =
      nh_.subscribe(
          impedance_enable_topic_,
          1,
          &GimbalrotorImpedanceController::impedanceEnableCallback,
          this);

  prev_impedance_time_ = ros::Time::now();

  ROS_WARN_STREAM("[GimbalrotorImpedanceController] initialized.");
  ROS_WARN_STREAM("[GimbalrotorImpedanceController] external_wrench_topic: "
                  << external_wrench_topic_);
  ROS_WARN_STREAM("[GimbalrotorImpedanceController] impedance_enable_topic: "
                  << impedance_enable_topic_);
  ROS_WARN_STREAM("[GimbalrotorImpedanceController] external_wrench_frame: "
                  << external_wrench_frame_);
  ROS_WARN_STREAM("[GimbalrotorImpedanceController] compliance_frame: "
                  << compliance_frame_);
}

void GimbalrotorImpedanceController::reset()
{
  GimbalrotorController::reset();

  impedance_enabled_ = false;
  raw_external_wrench_.setZero();

  impedance_core_.reset();
  impedance_output_ = ImpedanceCoreOutput();

  prev_impedance_time_ = ros::Time::now();

  ROS_WARN("[GimbalrotorImpedanceController] reset.");
}

void GimbalrotorImpedanceController::rosParamInit()
{
  /*
   * Load normal gimbalrotor params first.
   *
   * This is safe even if already called inside GimbalrotorController::initialize().
   * It only reads parameters.
   */
  GimbalrotorController::rosParamInit();

  ros::NodeHandle imp_nh(nh_, "controller/impedance");

  getParam<bool>(imp_nh, "use_impedance", impedance_config_.use_impedance, true);

  getParam<std::string>(imp_nh, "external_wrench_topic", external_wrench_topic_, std::string("estimated_external_wrench"));
  getParam<std::string>(imp_nh, "impedance_enable_topic", impedance_enable_topic_, std::string("impedance_enable"));
  getParam<std::string>(imp_nh, "external_wrench_frame", external_wrench_frame_, std::string("world"));
  getParam<std::string>(imp_nh, "compliance_frame", compliance_frame_, std::string("world"));

  getParam<double>(imp_nh, "force_lpf_alpha", impedance_config_.force_lpf_alpha, 0.2);
  getParam<double>(imp_nh, "torque_lpf_alpha", impedance_config_.torque_lpf_alpha, 0.2);

  getParam<double>(imp_nh, "enable_x", impedance_config_.trans_enable(0), 1.0);
  getParam<double>(imp_nh, "enable_y", impedance_config_.trans_enable(1), 1.0);
  getParam<double>(imp_nh, "enable_z", impedance_config_.trans_enable(2), 1.0);
  getParam<double>(imp_nh, "enable_roll", impedance_config_.rot_enable(0), 1.0);
  getParam<double>(imp_nh, "enable_pitch", impedance_config_.rot_enable(1), 1.0);
  getParam<double>(imp_nh, "enable_yaw", impedance_config_.rot_enable(2), 1.0);

  getParam<double>(imp_nh, "mass_x", impedance_config_.trans_virtual_mass(0), 5.0);
  getParam<double>(imp_nh, "mass_y", impedance_config_.trans_virtual_mass(1), 5.0);
  getParam<double>(imp_nh, "mass_z", impedance_config_.trans_virtual_mass(2), 5.0);

  getParam<double>(imp_nh, "damping_x", impedance_config_.trans_damping(0), 30.0);
  getParam<double>(imp_nh, "damping_y", impedance_config_.trans_damping(1), 30.0);
  getParam<double>(imp_nh, "damping_z", impedance_config_.trans_damping(2), 30.0);

  getParam<double>(imp_nh, "stiffness_x", impedance_config_.trans_stiffness(0), 60.0);
  getParam<double>(imp_nh, "stiffness_y", impedance_config_.trans_stiffness(1), 60.0);
  getParam<double>(imp_nh, "stiffness_z", impedance_config_.trans_stiffness(2), 60.0);

  getParam<double>(imp_nh, "force_ref_x", impedance_config_.force_ref(0), 0.0);
  getParam<double>(imp_nh, "force_ref_y", impedance_config_.force_ref(1), 0.0);
  getParam<double>(imp_nh, "force_ref_z", impedance_config_.force_ref(2), 0.0);

  getParam<double>(imp_nh, "force_limit_x", impedance_config_.force_limit(0), 3.0);
  getParam<double>(imp_nh, "force_limit_y", impedance_config_.force_limit(1), 3.0);
  getParam<double>(imp_nh, "force_limit_z", impedance_config_.force_limit(2), 3.0);

  getParam<double>(imp_nh, "pos_offset_limit_x", impedance_config_.pos_offset_limit(0), 0.05);
  getParam<double>(imp_nh, "pos_offset_limit_y", impedance_config_.pos_offset_limit(1), 0.05);
  getParam<double>(imp_nh, "pos_offset_limit_z", impedance_config_.pos_offset_limit(2), 0.05);

  getParam<double>(imp_nh, "vel_offset_limit_x", impedance_config_.vel_offset_limit(0), 0.10);
  getParam<double>(imp_nh, "vel_offset_limit_y", impedance_config_.vel_offset_limit(1), 0.10);
  getParam<double>(imp_nh, "vel_offset_limit_z", impedance_config_.vel_offset_limit(2), 0.10);

  getParam<double>(imp_nh, "inertia_roll", impedance_config_.rot_virtual_inertia(0), 0.08);
  getParam<double>(imp_nh, "inertia_pitch", impedance_config_.rot_virtual_inertia(1), 0.08);
  getParam<double>(imp_nh, "inertia_yaw", impedance_config_.rot_virtual_inertia(2), 0.08);

  getParam<double>(imp_nh, "rot_damping_roll", impedance_config_.rot_damping(0), 0.35);
  getParam<double>(imp_nh, "rot_damping_pitch", impedance_config_.rot_damping(1), 0.35);
  getParam<double>(imp_nh, "rot_damping_yaw", impedance_config_.rot_damping(2), 0.35);

  getParam<double>(imp_nh, "rot_stiffness_roll", impedance_config_.rot_stiffness(0), 0.8);
  getParam<double>(imp_nh, "rot_stiffness_pitch", impedance_config_.rot_stiffness(1), 0.8);
  getParam<double>(imp_nh, "rot_stiffness_yaw", impedance_config_.rot_stiffness(2), 0.8);

  getParam<double>(imp_nh, "torque_ref_roll", impedance_config_.torque_ref(0), 0.0);
  getParam<double>(imp_nh, "torque_ref_pitch", impedance_config_.torque_ref(1), 0.0);
  getParam<double>(imp_nh, "torque_ref_yaw", impedance_config_.torque_ref(2), 0.0);

  getParam<double>(imp_nh, "torque_limit_roll", impedance_config_.torque_limit(0), 0.8);
  getParam<double>(imp_nh, "torque_limit_pitch", impedance_config_.torque_limit(1), 0.8);
  getParam<double>(imp_nh, "torque_limit_yaw", impedance_config_.torque_limit(2), 0.8);

  getParam<double>(imp_nh, "angle_offset_limit_roll", impedance_config_.angle_offset_limit(0), 0.15);
  getParam<double>(imp_nh, "angle_offset_limit_pitch", impedance_config_.angle_offset_limit(1), 0.15);
  getParam<double>(imp_nh, "angle_offset_limit_yaw", impedance_config_.angle_offset_limit(2), 0.15);

  getParam<double>(imp_nh, "angular_vel_offset_limit_roll", impedance_config_.angular_vel_offset_limit(0), 0.50);
  getParam<double>(imp_nh, "angular_vel_offset_limit_pitch", impedance_config_.angular_vel_offset_limit(1), 0.50);
  getParam<double>(imp_nh, "angular_vel_offset_limit_yaw", impedance_config_.angular_vel_offset_limit(2), 0.50);

  impedance_core_.setConfig(impedance_config_);

  ROS_INFO_STREAM("[GimbalrotorImpedanceController] use_impedance: "
                  << impedance_config_.use_impedance);

  ROS_INFO_STREAM("[GimbalrotorImpedanceController] enable xyz: "
                  << impedance_config_.trans_enable(0) << ", "
                  << impedance_config_.trans_enable(1) << ", "
                  << impedance_config_.trans_enable(2));

  ROS_INFO_STREAM("[GimbalrotorImpedanceController] enable rpy: "
                  << impedance_config_.rot_enable(0) << ", "
                  << impedance_config_.rot_enable(1) << ", "
                  << impedance_config_.rot_enable(2));
}

void GimbalrotorImpedanceController::controlCore()
{
  const ros::Time now = ros::Time::now();
  const double dt = (now - prev_impedance_time_).toSec();
  prev_impedance_time_ = now;

  ImpedanceCoreInput input;
  input.external_wrench_world = getExternalWrenchWorld();
  input.R_world_compliance = getComplianceToWorldRotation();
  input.dt = dt;
  input.enabled = impedance_enabled_;

  impedance_output_ = impedance_core_.update(input);

  /*
   * Important architecture:
   *
   * ImpedanceCore does NOT touch navigator.
   * This wrapper temporarily injects the offset only because the existing
   * PoseLinearController reads targets from navigator_ inside controlCore().
   *
   * After normal GimbalrotorController::controlCore(), original navigation
   * target is restored immediately.
   */
  const tf::Vector3 original_target_pos = navigator_->getTargetPos();

  const tf::Vector3 original_target_rpy = navigator_->getTargetRPY();

  if(impedance_output_.valid)
    {
      const Eigen::Vector3d original_target_pos_eigen = tfVector3ToEigen(original_target_pos);
      const Eigen::Vector3d modified_target_pos_eigen = original_target_pos_eigen + impedance_output_.pos_offset_world;

      tf::Vector3 modified_target_rpy = original_target_rpy;
      modified_target_rpy.setX(modified_target_rpy.x() + impedance_output_.rpy_offset_world(0));
      modified_target_rpy.setY(modified_target_rpy.y() + impedance_output_.rpy_offset_world(1));
      modified_target_rpy.setZ(modified_target_rpy.z() + impedance_output_.rpy_offset_world(2));

      navigator_->setTargetPos(eigenToTfVector3(modified_target_pos_eigen));

      navigator_->setTargetRPY(modified_target_rpy);
    }

  GimbalrotorController::controlCore();

  /*
   * Restore navigation target so impedance does not become navigation logic.
   * Navigation remains navigation.
   * Impedance remains only a correction used during this controller cycle.
   */
  if(impedance_output_.valid)
    {
      navigator_->setTargetPos(original_target_pos);
      navigator_->setTargetRPY(original_target_rpy);
    }

  if(impedance_output_.valid)
    {
      ROS_WARN_THROTTLE(
          0.5,
          "[Gimbalrotor Impedance Adapter] "
          "F_comp: %.3f %.3f %.3f | "
          "T_comp: %.3f %.3f %.3f | "
          "dx_world: %.4f %.4f %.4f | "
          "drpy_world: %.4f %.4f %.4f",
          impedance_output_.force_compliance(0),
          impedance_output_.force_compliance(1),
          impedance_output_.force_compliance(2),
          impedance_output_.torque_compliance(0),
          impedance_output_.torque_compliance(1),
          impedance_output_.torque_compliance(2),
          impedance_output_.pos_offset_world(0),
          impedance_output_.pos_offset_world(1),
          impedance_output_.pos_offset_world(2),
          impedance_output_.rpy_offset_world(0),
          impedance_output_.rpy_offset_world(1),
          impedance_output_.rpy_offset_world(2));
    }
}

void GimbalrotorImpedanceController::externalWrenchCallback(
    const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  raw_external_wrench_(0) = msg->wrench.force.x;
  raw_external_wrench_(1) = msg->wrench.force.y;
  raw_external_wrench_(2) = msg->wrench.force.z;

  raw_external_wrench_(3) = msg->wrench.torque.x;
  raw_external_wrench_(4) = msg->wrench.torque.y;
  raw_external_wrench_(5) = msg->wrench.torque.z;
}

void GimbalrotorImpedanceController::impedanceEnableCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
  const bool previous_enabled = impedance_enabled_;
  impedance_enabled_ = msg->data;

  if(previous_enabled && !impedance_enabled_)
    {
      impedance_core_.reset();
      impedance_output_ = ImpedanceCoreOutput();
    }

  ROS_WARN(
      "[GimbalrotorImpedanceController] impedance_enabled: %d",
      static_cast<int>(impedance_enabled_));
}

Eigen::Matrix<double, 6, 1>
GimbalrotorImpedanceController::getExternalWrenchWorld() const
{
  Eigen::Matrix<double, 6, 1> wrench_world = raw_external_wrench_;

  if(external_wrench_frame_ == "world" ||
     external_wrench_frame_ == "map" ||
     external_wrench_frame_ == "odom")
    {
      return wrench_world;
    }

  if(external_wrench_frame_ == "cog" ||
     external_wrench_frame_ == "body" ||
     external_wrench_frame_ == "base_link")
    {
      tf::Matrix3x3 R_world_cog_tf = estimator_->getOrientation(Frame::COG, estimate_mode_);

      Eigen::Matrix3d R_world_cog;
      tf::matrixTFToEigen(R_world_cog_tf, R_world_cog);

      wrench_world.segment<3>(0) = R_world_cog * raw_external_wrench_.segment<3>(0);
      wrench_world.segment<3>(3) = R_world_cog * raw_external_wrench_.segment<3>(3);

      return wrench_world;
    }

  ROS_WARN_THROTTLE(
      1.0,
      "[GimbalrotorImpedanceController] Unknown external_wrench_frame [%s]. "
      "Use world frame.",
      external_wrench_frame_.c_str());

  return wrench_world;
}

Eigen::Matrix3d
GimbalrotorImpedanceController::getComplianceToWorldRotation() const
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
      tf::Matrix3x3 R_world_cog_tf = estimator_->getOrientation(Frame::COG, estimate_mode_);

      Eigen::Matrix3d R_world_cog;
      tf::matrixTFToEigen(R_world_cog_tf, R_world_cog);

      return R_world_cog;
    }

  ROS_WARN_THROTTLE(
      1.0,
      "[GimbalrotorImpedanceController] Unknown compliance_frame [%s]. "
      "Use world frame.",
      compliance_frame_.c_str());

  return Eigen::Matrix3d::Identity();
}

tf::Vector3 GimbalrotorImpedanceController::eigenToTfVector3(const Eigen::Vector3d& v) const
{
  return tf::Vector3(v(0), v(1), v(2));
}

Eigen::Vector3d GimbalrotorImpedanceController::tfVector3ToEigen(const tf::Vector3& v) const
{
  return Eigen::Vector3d(v.x(), v.y(), v.z());
}

} // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(
    aerial_robot_control::GimbalrotorImpedanceController,
    aerial_robot_control::ControlBase)
