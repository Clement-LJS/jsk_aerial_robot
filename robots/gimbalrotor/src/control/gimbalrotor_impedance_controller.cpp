#include <gimbalrotor/control/gimbalrotor_impedance_controller.h>

#include <cmath>

#include <pluginlib/class_list_macros.h>

namespace aerial_robot_control
{

GimbalrotorImpedanceController::GimbalrotorImpedanceController()
  : GimbalrotorMultilinkController(),
    use_impedance_(true),
    interaction_mode_(NORMAL_FLIGHT),
    interaction_mode_topic_("interaction_mode"),
    pitch_joint_compliance_offset_topic_("pitch_joint_compliance_offset"),
    is_cutting_(false),
    prev_is_cutting_(false),
    external_wrench_topic_("estimated_external_wrench"),
    is_cutting_topic_("isCutting"),
    external_wrench_frame_("world"),
    tool_link_name_("saw"),
    tool_frame_roll_(0.0),
    tool_frame_pitch_(0.0),
    tool_frame_yaw_(0.0),
    prev_modified_target_valid_(false),
    target_override_threshold_(0.05),
    force_lpf_alpha_(0.2),

    use_pitch_impedance_(true),
    pitch_virtual_inertia_(0.08),
    pitch_damping_(0.35),
    pitch_stiffness_(0.8),
    pitch_torque_ref_(0.0),
    pitch_torque_limit_(0.8),
    pitch_torque_lpf_alpha_(0.15),
    pitch_torque_deadband_(0.02),
    pitch_torque_lpf_(0.0),
    pitch_angle_offset_limit_(0.20),
    pitch_rate_offset_limit_(0.60),
    pitch_ang_acc_correction_limit_(2.0),
    pitch_angle_offset_(0.0),
    pitch_rate_offset_(0.0),
    pitch_ang_acc_offset_(0.0)
{
  est_external_wrench_ = Eigen::VectorXd::Zero(6);

  force_raw_.setZero();
  force_world_lpf_.setZero();
  force_tool_lpf_.setZero();

  dx_tool_.setZero();
  dx_dot_tool_.setZero();
  dx_ddot_tool_.setZero();

  prev_dx_world_.setZero();
  prev_modified_target_world_.setZero();

  /*
   * Tool-frame axis convention:
   *
   *   X_tool = cutting feed direction
   *   Y_tool = lateral direction
   *   Z_tool = blade normal direction
   */
  axis_enable_ << 1.0, 1.0, 1.0;

  mass_ << 5.0, 5.0, 5.0;
  damping_ << 20.0, 60.0, 60.0;
  stiffness_ << 20.0, 100.0, 100.0;
  force_ref_tool_ << 0.0, 0.0, 0.0;
  force_limit_tool_ << 3.0, 2.0, 2.0;
  displacement_limit_tool_ << 0.08, 0.015, 0.015;
  velocity_limit_tool_ << 0.15, 0.05, 0.05;
}

void GimbalrotorImpedanceController::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate)
{
  GimbalrotorMultilinkController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  gimbalrotor_multilink_robot_model_for_impedance_ = boost::dynamic_pointer_cast<GimbalrotorMultilinkRobotModel>(robot_model);

  if(!gimbalrotor_multilink_robot_model_for_impedance_)
    {
      ROS_ERROR("[GimbalrotorImpedanceController] Failed to cast robot_model to GimbalrotorMultilinkRobotModel.");
      ROS_ERROR("[GimbalrotorImpedanceController] Check robot_model_plugin_name. It should be gimbalrotor_multilink_robot_model.");
    }
  else
    {
      ROS_INFO("[GimbalrotorImpedanceController] Using GimbalrotorMultilinkRobotModel.");
    }

  rosParamInit();

  external_wrench_sub_ = nh_.subscribe(external_wrench_topic_, 1, &GimbalrotorImpedanceController::externalWrenchCallback, this);
  is_cutting_sub_ = nh_.subscribe(is_cutting_topic_, 1, &GimbalrotorImpedanceController::isCuttingCallback, this);
  interaction_mode_sub_ = nh_.subscribe(interaction_mode_topic_, 1, &GimbalrotorImpedanceController::interactionModeCallback, this);
  pitch_joint_compliance_offset_pub_ = nh_.advertise<std_msgs::Float64>(pitch_joint_compliance_offset_topic_, 1);

  resetImpedanceMemory();

  prev_time_ = ros::Time::now();

  ROS_WARN_STREAM("[GimbalrotorImpedanceController] initialized.");
  ROS_WARN_STREAM("[GimbalrotorImpedanceController] trigger topic: " << is_cutting_topic_);
  ROS_WARN_STREAM("[GimbalrotorImpedanceController] tool_link_name: " << tool_link_name_);
  ROS_WARN_STREAM("[GimbalrotorImpedanceController] external_wrench_topic: " << external_wrench_topic_);
  ROS_WARN_STREAM("[GimbalrotorImpedanceController] external_wrench_frame: " << external_wrench_frame_);
}

void GimbalrotorImpedanceController::reset()
{
  GimbalrotorMultilinkController::reset();

  interaction_mode_ = NORMAL_FLIGHT;

  is_cutting_ = false;
  prev_is_cutting_ = false;

  resetImpedanceMemory();

  prev_time_ = ros::Time::now();

  ROS_WARN("[GimbalrotorImpedanceController] reset.");
}

void GimbalrotorImpedanceController::rosParamInit()
{
  GimbalrotorMultilinkController::rosParamInit();

  ros::NodeHandle imp_nh(nh_, "controller/impedance");

  getParam<bool>(imp_nh, "use_impedance", use_impedance_, true);

  getParam<std::string>(imp_nh, "external_wrench_topic", external_wrench_topic_, std::string("estimated_external_wrench"));

  /* With nh_.subscribe("isCutting"), actual topic is: /gimbalrotor/isCutting */
  getParam<std::string>(imp_nh, "is_cutting_topic", is_cutting_topic_, std::string("isCutting"));
  getParam<std::string>(imp_nh, "interaction_mode_topic", interaction_mode_topic_, std::string("interaction_mode"));
  getParam<std::string>(imp_nh, "pitch_joint_compliance_offset_topic", pitch_joint_compliance_offset_topic_, std::string("pitch_joint_compliance_offset"));
  getParam<std::string>(imp_nh, "external_wrench_frame", external_wrench_frame_, std::string("world"));
  getParam<std::string>(imp_nh, "tool_link_name", tool_link_name_, std::string("saw"));

  getParam<double>(imp_nh, "tool_frame_roll", tool_frame_roll_, 0.0);
  getParam<double>(imp_nh, "tool_frame_pitch", tool_frame_pitch_, 0.0);
  getParam<double>(imp_nh, "tool_frame_yaw", tool_frame_yaw_, 0.0);

  getParam<double>(imp_nh, "target_override_threshold", target_override_threshold_, 0.05);

  getParam<double>(imp_nh, "axis_enable_x", axis_enable_(0), 1.0);
  getParam<double>(imp_nh, "axis_enable_y", axis_enable_(1), 1.0);
  getParam<double>(imp_nh, "axis_enable_z", axis_enable_(2), 1.0);

  getParam<double>(imp_nh, "mass_x", mass_(0), 5.0);
  getParam<double>(imp_nh, "mass_y", mass_(1), 5.0);
  getParam<double>(imp_nh, "mass_z", mass_(2), 5.0);

  getParam<double>(imp_nh, "damping_x", damping_(0), 20.0);
  getParam<double>(imp_nh, "damping_y", damping_(1), 60.0);
  getParam<double>(imp_nh, "damping_z", damping_(2), 60.0);

  getParam<double>(imp_nh, "stiffness_x", stiffness_(0), 20.0);
  getParam<double>(imp_nh, "stiffness_y", stiffness_(1), 100.0);
  getParam<double>(imp_nh, "stiffness_z", stiffness_(2), 100.0);

  getParam<double>(imp_nh, "force_ref_x", force_ref_tool_(0), 0.0);
  getParam<double>(imp_nh, "force_ref_y", force_ref_tool_(1), 0.0);
  getParam<double>(imp_nh, "force_ref_z", force_ref_tool_(2), 0.0);

  getParam<double>(imp_nh, "force_lpf_alpha", force_lpf_alpha_, 0.2);

  getParam<double>(imp_nh, "force_limit_x", force_limit_tool_(0), 3.0);
  getParam<double>(imp_nh, "force_limit_y", force_limit_tool_(1), 2.0);
  getParam<double>(imp_nh, "force_limit_z", force_limit_tool_(2), 2.0);

  getParam<double>(imp_nh, "displacement_limit_x", displacement_limit_tool_(0), 0.08);
  getParam<double>(imp_nh, "displacement_limit_y", displacement_limit_tool_(1), 0.015);
  getParam<double>(imp_nh, "displacement_limit_z", displacement_limit_tool_(2), 0.015);

  getParam<double>(imp_nh, "velocity_limit_x", velocity_limit_tool_(0), 0.15);
  getParam<double>(imp_nh, "velocity_limit_y", velocity_limit_tool_(1), 0.05);
  getParam<double>(imp_nh, "velocity_limit_z", velocity_limit_tool_(2), 0.05);

  getParam<bool>(imp_nh, "use_pitch_impedance", use_pitch_impedance_, true);

  getParam<double>(imp_nh, "pitch_virtual_inertia", pitch_virtual_inertia_, 0.08);
  getParam<double>(imp_nh, "pitch_damping", pitch_damping_, 0.35);
  getParam<double>(imp_nh, "pitch_stiffness", pitch_stiffness_, 0.8);

  getParam<double>(imp_nh, "pitch_torque_ref", pitch_torque_ref_, 0.0);
  getParam<double>(imp_nh, "pitch_torque_limit", pitch_torque_limit_, 0.8);

  getParam<double>(imp_nh, "pitch_torque_lpf_alpha", pitch_torque_lpf_alpha_, 0.15);
  getParam<double>(imp_nh, "pitch_torque_deadband", pitch_torque_deadband_, 0.02);

  getParam<double>(imp_nh, "pitch_angle_offset_limit", pitch_angle_offset_limit_, 0.20);
  getParam<double>(imp_nh, "pitch_rate_offset_limit", pitch_rate_offset_limit_, 0.60);
  getParam<double>(imp_nh, "pitch_ang_acc_correction_limit", pitch_ang_acc_correction_limit_, 2.0);

  if(pitch_virtual_inertia_ < 0.001)
    pitch_virtual_inertia_ = 0.001;

  if(pitch_damping_ < 0.0)
    pitch_damping_ = 0.0;

  if(pitch_stiffness_ < 0.0)
    pitch_stiffness_ = 0.0;

  if(pitch_torque_limit_ < 0.0)
    pitch_torque_limit_ = 0.0;

  pitch_torque_lpf_alpha_ = clampValue(pitch_torque_lpf_alpha_, 0.0, 1.0);

  if(pitch_torque_deadband_ < 0.0)
    pitch_torque_deadband_ = 0.0;
  
  if(pitch_angle_offset_limit_ < 0.0)
    pitch_angle_offset_limit_ = 0.0;

  if(pitch_rate_offset_limit_ < 0.0)
    pitch_rate_offset_limit_ = 0.0;

  if(pitch_ang_acc_correction_limit_ < 0.0)
    pitch_ang_acc_correction_limit_ = 0.0;
    
  /* Safety cleanup. */
  force_lpf_alpha_ = clampValue(force_lpf_alpha_, 0.0, 1.0);

  if(target_override_threshold_ < 0.0)
    {
      target_override_threshold_ = 0.0;
    }

  for(int i = 0; i < 3; ++i)
    {
      axis_enable_(i) = axis_enable_(i) > 0.5 ? 1.0 : 0.0;

      if(mass_(i) < 0.001)
        {
          mass_(i) = 0.001;
        }

      if(damping_(i) < 0.0)
        {
          damping_(i) = 0.0;
        }

      if(stiffness_(i) < 0.0)
        {
          stiffness_(i) = 0.0;
        }

      if(force_limit_tool_(i) < 0.0)
        {
          force_limit_tool_(i) = 0.0;
        }

      if(displacement_limit_tool_(i) < 0.0)
        {
          displacement_limit_tool_(i) = 0.0;
        }

      if(velocity_limit_tool_(i) < 0.0)
        {
          velocity_limit_tool_(i) = 0.0;
        }
    }

  ROS_INFO_STREAM("[GimbalrotorImpedanceController] use_impedance: " << use_impedance_);
  ROS_INFO_STREAM("[GimbalrotorImpedanceController] is_cutting_topic: " << is_cutting_topic_);
  ROS_INFO_STREAM("[GimbalrotorImpedanceController] target_override_threshold: " << target_override_threshold_);
  ROS_INFO_STREAM("[GimbalrotorImpedanceController] axis_enable: "
                  << axis_enable_(0) << ", "
                  << axis_enable_(1) << ", "
                  << axis_enable_(2));
}

void GimbalrotorImpedanceController::controlCore()
{
  const ros::Time now = ros::Time::now();
  const double dt = (now - prev_time_).toSec();

  if(dt <= 0.0 || dt > 0.1)
    {
      prev_time_ = now;
      GimbalrotorMultilinkController::controlCore();
      return;
    }

  /*
   * New architecture:
   *
   *   interaction_mode_ is the only state that controls impedance behavior.
   *
   * isCutting is kept only for backward compatibility / debug observation.
   * It must NOT reset impedance memory anymore, because that can interfere with PERCH_COMPLIANCE and CUTTING_COMPLIANCE.
   */
  prev_is_cutting_ = is_cutting_;
  
  uint8_t active_mode = interaction_mode_;

  if(active_mode > CUTTING_COMPLIANCE)
    {
      ROS_ERROR_THROTTLE(1.0, "[GimbalrotorImpedanceController] invalid active_mode: %u. Run normal multilink controller.", static_cast<unsigned int>(active_mode));

      std_msgs::Float64 pitch_offset_msg;
      pitch_offset_msg.data = 0.0;
      pitch_joint_compliance_offset_pub_.publish(pitch_offset_msg);

      GimbalrotorMultilinkController::controlCore();
      prev_time_ = now;
      return;
    }

  /* No impedance in NORMAL_FLIGHT or PERCHED. */
  if(!use_impedance_ || active_mode == NORMAL_FLIGHT || active_mode == PERCHED)
    {
      std_msgs::Float64 pitch_offset_msg;
      pitch_offset_msg.data = 0.0;
      pitch_joint_compliance_offset_pub_.publish(pitch_offset_msg);

      GimbalrotorMultilinkController::controlCore();
      prev_time_ = now;
      return;
    }

  /* Read current navigator target.
   *
   * This may be:
   *   1. the previous impedance-modified target
   *   2. or a new external navigation target */
  const Eigen::Vector3d current_nav_world = tfToEigenVector(navigator_->getTargetPos());

  /* Recover the live base target.
   *
   * If the current target is almost equal to the target we wrote last cycle, then current_nav_world still contains our old impedance offset.
   *
   * Therefore: base_target = current_nav_world - prev_dx_world
   *
   * If the current target is very different, assume an external command updated it.
   * Then: base_target = current_nav_world */
  Eigen::Vector3d live_base_target_world = current_nav_world;

  bool external_target_override = false;

  if(prev_modified_target_valid_)
    {
      const double target_difference = (current_nav_world - prev_modified_target_world_).norm();

      if(target_difference < target_override_threshold_)
        {
          live_base_target_world = current_nav_world - prev_dx_world_;
        }
      else
        {
          external_target_override = true;
          live_base_target_world = current_nav_world;
        }
    }

  /* Get current tool frame rotation. */
  Eigen::Matrix3d R_world_tool = Eigen::Matrix3d::Identity();

  if(!getToolRotationWorld(R_world_tool))
    {
      ROS_ERROR_THROTTLE(1.0, "[GimbalrotorImpedanceController] Cannot get tool rotation. Run multilink controller only.");

      GimbalrotorMultilinkController::controlCore();
      prev_time_ = now;
      return;
    }

  /* Convert estimated force to world frame if necessary. */
  Eigen::Vector3d F_world = force_raw_;

  if(external_wrench_frame_ == "cog" || external_wrench_frame_ == "body" || external_wrench_frame_ == "base_link")
    {
      tf::Matrix3x3 R_world_cog_tf = estimator_->getOrientation(Frame::COG, estimate_mode_);

      Eigen::Matrix3d R_world_cog;
      tf::matrixTFToEigen(R_world_cog_tf, R_world_cog);

      F_world = R_world_cog * force_raw_;
    }

  force_world_lpf_ = force_lpf_alpha_ * F_world + (1.0 - force_lpf_alpha_) * force_world_lpf_;

  /* Transform force into tool frame. */
  Eigen::Vector3d F_tool = R_world_tool.transpose() * force_world_lpf_;

  F_tool = clampVectorElementwise(F_tool, force_limit_tool_);

  force_tool_lpf_ = F_tool;

  /*
   * Mode-dependent compliance tuning.
   *
   * PERCH_COMPLIANCE:
   *   hand remains horizontal in navigator.
   *   body target is allowed to move slightly with contact.
   *
   * CUTTING_COMPLIANCE:
   *   use the YAML/default impedance values for cutting.
   */
  Eigen::Vector3d active_mass = mass_;
  Eigen::Vector3d active_damping = damping_;
  Eigen::Vector3d active_stiffness = stiffness_;
  Eigen::Vector3d active_displacement_limit = displacement_limit_tool_;
  Eigen::Vector3d active_velocity_limit = velocity_limit_tool_;
  Eigen::Vector3d active_axis_enable = axis_enable_;

  if(active_mode == PERCH_COMPLIANCE)
    {
      /*
       * Safer perching default:
       *   - smaller displacement than cutting
       *   - enough damping
       *   - still compliant in z because branch height error is common
       */
      active_axis_enable << 1.0, 1.0, 1.0;

      active_mass << 4.0, 4.0, 4.0;
      active_damping << 45.0, 45.0, 35.0;
      active_stiffness << 80.0, 80.0, 45.0;

      active_displacement_limit << 0.025, 0.025, 0.040;
      active_velocity_limit << 0.060, 0.060, 0.060;
    }
  
  /* Selective-axis admittance in tool frame.
   *   M dx_ddot + C dx_dot + K dx = F_ext - F_ref */
  for(int i = 0; i < 3; ++i)
    {
      if(active_axis_enable(i) < 0.5)
        {
          dx_tool_(i) = 0.0;
          dx_dot_tool_(i) = 0.0;
          dx_ddot_tool_(i) = 0.0;
          continue;
        }

      const double force_error = F_tool(i) - force_ref_tool_(i);

      dx_ddot_tool_(i) = (force_error - active_damping(i) * dx_dot_tool_(i) - active_stiffness(i) * dx_tool_(i)) / active_mass(i);
      dx_dot_tool_(i) += dx_ddot_tool_(i) * dt;
      dx_dot_tool_(i) = clampValue(dx_dot_tool_(i), -active_velocity_limit(i), active_velocity_limit(i));
      dx_tool_(i) += dx_dot_tool_(i) * dt;
      dx_tool_(i) = clampValue(dx_tool_(i), -active_displacement_limit(i), active_displacement_limit(i));
    }

  /* Convert tool-frame offset to world-frame offset. */
  const Eigen::Vector3d dx_world = R_world_tool * dx_tool_;

  /* Desired behavior:
   *   target_command = live_normal_target + impedance_offset
   * This lets the robot still move forward/turn/approach while cutting. */
  const Eigen::Vector3d xcmd_world = live_base_target_world + dx_world;

  navigator_->setTargetPos(eigenToTfVector(xcmd_world));

  /* Store memory for next cycle, so we can remove the previous offset and avoid drift. */
  prev_dx_world_ = dx_world;
  prev_modified_target_world_ = xcmd_world;
  prev_modified_target_valid_ = true;

  ROS_WARN_THROTTLE(
      0.5,
      "[Cutting Impedance LiveTarget] active | "
      "F_tool: %.3f %.3f %.3f | "
      "dx_tool: %.4f %.4f %.4f | "
      "base: %.3f %.3f %.3f | "
      "xcmd: %.3f %.3f %.3f | "
      "override: %d",
      F_tool(0), F_tool(1), F_tool(2),
      dx_tool_(0), dx_tool_(1), dx_tool_(2),
      live_base_target_world(0),
      live_base_target_world(1),
      live_base_target_world(2),
      xcmd_world(0), xcmd_world(1), xcmd_world(2),
      external_target_override);

  /* Pitch-joint compliance is only for cutting. During perching, the hand must stay horizontal and stable. */
  if(active_mode != CUTTING_COMPLIANCE)
    {
      std_msgs::Float64 pitch_offset_msg;
      pitch_offset_msg.data = 0.0;
      pitch_joint_compliance_offset_pub_.publish(pitch_offset_msg);
    }
  
  /* Run MULTILINK controller after target modification. */
  GimbalrotorMultilinkController::controlCore();

  prev_time_ = now;
}

void GimbalrotorImpedanceController::modifyTargetRPYForCompliance(tf::Vector3& target_rpy)
{
  /*
   * Requirement:
   *   The hand link has no thrust source.
   *   Therefore, do NOT make body pitch compliance the main cutting behavior.
   *
   * Instead:
   *   - keep target_rpy unchanged here
   *   - compute small pitch compliance offset from estimated torque
   *   - publish it to GimbalrotorMultilinkNavigator
   *   - navigator adds it around hand-horizontal compensation
   *
   * This keeps:
   *   pitch_joint_cmd = hand-horizontal compensation + small cutting compliance offset
   */
  (void)target_rpy;

  uint8_t active_mode = interaction_mode_;

  if(!use_impedance_ || !use_pitch_impedance_ || active_mode != CUTTING_COMPLIANCE)
    {
      std_msgs::Float64 pitch_offset_msg;
      pitch_offset_msg.data = 0.0;
      pitch_joint_compliance_offset_pub_.publish(pitch_offset_msg);
      return;
    }

  const ros::Time now = ros::Time::now();
  const double dt = (now - prev_time_).toSec();

  if(dt <= 0.0 || dt > 0.1)
    {
      return;
    }

  /*
   * est_external_wrench_:
   *   0,1,2 = force
   *   3,4,5 = torque
   */
  Eigen::Vector3d torque_raw(
      est_external_wrench_(3),
      est_external_wrench_(4),
      est_external_wrench_(5));

  Eigen::Vector3d torque_cog = torque_raw;

  if(external_wrench_frame_ == "world")
    {
      tf::Matrix3x3 R_world_cog_tf = estimator_->getOrientation(Frame::COG, estimate_mode_);

      Eigen::Matrix3d R_world_cog;
      tf::matrixTFToEigen(R_world_cog_tf, R_world_cog);

      torque_cog = R_world_cog.transpose() * torque_raw;
    }

  double pitch_torque_raw = torque_cog.y();

  pitch_torque_lpf_ = pitch_torque_lpf_alpha_ * pitch_torque_raw + (1.0 - pitch_torque_lpf_alpha_) * pitch_torque_lpf_;

  double pitch_torque = pitch_torque_lpf_;

  if(std::abs(pitch_torque) < pitch_torque_deadband_)
    {
      pitch_torque = 0.0;
    }

  pitch_torque = clampValue(pitch_torque, -pitch_torque_limit_, pitch_torque_limit_);

  /*
   * Rotational admittance:
   *
   *   Jv * q_offset_ddot
   *   + Dv * q_offset_dot
   *   + Kv * q_offset
   *   =
   *   tau_ext - tau_ref
   *
   * Output:
   *   pitch_angle_offset_
   *
   * This offset is NOT directly body pitch.
   * It is published as pitch-joint compliance offset.
   */
  const double torque_error = pitch_torque - pitch_torque_ref_;

  pitch_ang_acc_offset_ = (torque_error - pitch_damping_ * pitch_rate_offset_ - pitch_stiffness_ * pitch_angle_offset_) / pitch_virtual_inertia_;
  pitch_ang_acc_offset_ = clampValue(pitch_ang_acc_offset_, -pitch_ang_acc_correction_limit_, pitch_ang_acc_correction_limit_);
  pitch_rate_offset_ += pitch_ang_acc_offset_ * dt;
  pitch_rate_offset_ = clampValue(pitch_rate_offset_, -pitch_rate_offset_limit_, pitch_rate_offset_limit_);
  pitch_angle_offset_ += pitch_rate_offset_ * dt;
  pitch_angle_offset_ = clampValue(pitch_angle_offset_, -pitch_angle_offset_limit_, pitch_angle_offset_limit_);

  target_rpy.setY(target_rpy.y() + pitch_angle_offset_);

  std_msgs::Float64 pitch_offset_msg;
  pitch_offset_msg.data = 0.0;
  pitch_joint_compliance_offset_pub_.publish(pitch_offset_msg);

  ROS_WARN_THROTTLE(
      0.5,
      "[Pitch Joint Admittance Offset] tau_raw: %.3f | tau_lpf: %.3f | tau_used: %.3f | joint_offset: %.4f | joint_rate_offset: %.4f | joint_acc_offset: %.4f",
      pitch_torque_raw,
      pitch_torque_lpf_,
      pitch_torque,
      pitch_angle_offset_,
      pitch_rate_offset_,
      pitch_ang_acc_offset_);
}

void GimbalrotorImpedanceController::externalWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  est_external_wrench_(0) = msg->wrench.force.x;
  est_external_wrench_(1) = msg->wrench.force.y;
  est_external_wrench_(2) = msg->wrench.force.z;

  est_external_wrench_(3) = msg->wrench.torque.x;
  est_external_wrench_(4) = msg->wrench.torque.y;
  est_external_wrench_(5) = msg->wrench.torque.z;

  force_raw_(0) = msg->wrench.force.x;
  force_raw_(1) = msg->wrench.force.y;
  force_raw_(2) = msg->wrench.force.z;
}

void GimbalrotorImpedanceController::isCuttingCallback(const std_msgs::Bool::ConstPtr& msg)
{
  is_cutting_ = msg->data;
}

void GimbalrotorImpedanceController::interactionModeCallback(const std_msgs::UInt8::ConstPtr& msg)
{
  if(msg->data > CUTTING_COMPLIANCE)
    {
      ROS_WARN_THROTTLE(1.0, "[GimbalrotorImpedanceController] invalid interaction_mode: %u", static_cast<unsigned int>(msg->data));
      return;
    }

  const uint8_t prev_mode = interaction_mode_;
  const uint8_t new_mode = msg->data;

  if(prev_mode == new_mode)
    {
      return;
    }

  /*
   * Important:
   * If the previous mode used translational admittance, the navigator target may currently be:
   *   current target = normal target + previous impedance offset
   *
   * Before resetting memory, remove that previous offset. */
  if(prev_modified_target_valid_)
    {
      const Eigen::Vector3d current_nav_world = tfToEigenVector(navigator_->getTargetPos());
      const Eigen::Vector3d restored_base_target = current_nav_world - prev_dx_world_;
      navigator_->setTargetPos(eigenToTfVector(restored_base_target));
    }

  interaction_mode_ = new_mode;

  resetImpedanceMemory();

  std_msgs::Float64 pitch_offset_msg;
  pitch_offset_msg.data = 0.0;
  pitch_joint_compliance_offset_pub_.publish(pitch_offset_msg);

  ROS_WARN("[GimbalrotorImpedanceController] interaction mode changed: %u -> %u", static_cast<unsigned int>(prev_mode), static_cast<unsigned int>(interaction_mode_));
}

void GimbalrotorImpedanceController::resetImpedanceMemory()
{
  est_external_wrench_.setZero();

  force_raw_.setZero();
  force_world_lpf_.setZero();
  force_tool_lpf_.setZero();

  dx_tool_.setZero();
  dx_dot_tool_.setZero();
  dx_ddot_tool_.setZero();

  prev_dx_world_.setZero();
  prev_modified_target_world_.setZero();
  prev_modified_target_valid_ = false;

  pitch_torque_lpf_ = 0.0;
  
  pitch_angle_offset_ = 0.0;
  pitch_rate_offset_ = 0.0;
  pitch_ang_acc_offset_ = 0.0;
}

tf::Vector3 GimbalrotorImpedanceController::eigenToTfVector(const Eigen::Vector3d& v) const
{
  return tf::Vector3(v(0), v(1), v(2));
}

Eigen::Vector3d GimbalrotorImpedanceController::tfToEigenVector(const tf::Vector3& v) const
{
  return Eigen::Vector3d(v.x(), v.y(), v.z());
}

bool GimbalrotorImpedanceController::getToolRotationWorld(Eigen::Matrix3d& R_world_tool)
{
  if(!gimbalrotor_multilink_robot_model_for_impedance_)
    {
      ROS_ERROR_THROTTLE(1.0, "[GimbalrotorImpedanceController] Multilink robot model pointer is null.");
      return false;
    }

  /*
   * Current joint positions.
   *
   * This includes pitch_joint / other movable body joints
   * because the loaded robot model is GimbalrotorMultilinkRobotModel.
   */
  const KDL::JntArray joint_positions = gimbalrotor_multilink_robot_model_for_impedance_->getJointPositions();

  KDL::TreeFkSolverPos_recursive fk_solver( gimbalrotor_multilink_robot_model_for_impedance_->getTree());

  /* FK to baselink. */
  KDL::Frame f_baselink;

  const int base_result = fk_solver.JntToCart(joint_positions, f_baselink, gimbalrotor_multilink_robot_model_for_impedance_->getBaselinkName());

  if(base_result < 0)
    {
      ROS_WARN_THROTTLE(1.0, "[GimbalrotorImpedanceController] failed FK for baselink.");
      return false;
    }

  /*
   * FK to tool link.
   *
   * Default:
   *   tool_link_name = saw
   *
   * You can also use:
   *   hand_assem_link
   * if you want the compliance frame attached to hand assembly.
   */
  KDL::Frame f_tool;

  const int tool_result = fk_solver.JntToCart(joint_positions, f_tool, tool_link_name_);

  if(tool_result < 0)
    {
      ROS_WARN_THROTTLE(1.0, "[GimbalrotorImpedanceController] failed FK for tool link [%s].", tool_link_name_.c_str());
      return false;
    }

  /* Construct CoG frame rotation.
   *
   * Same logic as your gimbalrotor/multilink robot model:
   *   R_world_cog = R_world_baselink * R_cog_desired^-1 */
  const KDL::Rotation cog_frame = f_baselink.M * gimbalrotor_multilink_robot_model_for_impedance_ ->getCogDesireOrientation<KDL::Rotation>().Inverse();

  /* Rotation from CoG to tool link.
   *   R_cog_tool = R_world_cog^-1 * R_world_tool_link */
  const KDL::Rotation R_cog_tool_kdl = cog_frame.Inverse() * f_tool.M;

  tf::Quaternion q_cog_tool_tf;
  tf::quaternionKDLToTF(R_cog_tool_kdl, q_cog_tool_tf);

  Eigen::Matrix3d R_cog_tool;
  tf::matrixTFToEigen(tf::Matrix3x3(q_cog_tool_tf), R_cog_tool);

  /*
   * Optional fixed correction:
   *   tool link frame -> desired cutting tool frame
   *
   * Desired convention:
   *   X_tool = cutting feed direction
   *   Y_tool = lateral direction
   *   Z_tool = blade normal direction
   */
  const Eigen::Matrix3d R_toollink_tool = rpyToRot(tool_frame_roll_, tool_frame_pitch_, tool_frame_yaw_);

  /* Current CoG orientation in world frame. */
  tf::Matrix3x3 R_world_cog_tf = estimator_->getOrientation(Frame::COG, estimate_mode_);

  Eigen::Matrix3d R_world_cog;
  tf::matrixTFToEigen(R_world_cog_tf, R_world_cog);

  /* Final tool frame rotation in world:
   *   R_world_tool = R_world_cog * R_cog_tool_link * R_toollink_tool */
  R_world_tool = R_world_cog * R_cog_tool * R_toollink_tool;

  return true;
}

Eigen::Matrix3d GimbalrotorImpedanceController::rpyToRot(double roll, double pitch, double yaw) const
{
  tf::Matrix3x3 tf_rot;
  tf_rot.setRPY(roll, pitch, yaw);

  Eigen::Matrix3d eig_rot;
  tf::matrixTFToEigen(tf_rot, eig_rot);

  return eig_rot;
}

double GimbalrotorImpedanceController::clampValue(double value, double min_value, double max_value) const
{
  return std::max(min_value, std::min(value, max_value));
}

Eigen::Vector3d GimbalrotorImpedanceController::clampVectorElementwise( const Eigen::Vector3d& value, const Eigen::Vector3d& limit) const
{
  Eigen::Vector3d result = value;

  for(int i = 0; i < 3; ++i)
    {
      result(i) = clampValue(result(i), -std::abs(limit(i)), std::abs(limit(i)));
    }

  return result;
}

} // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorImpedanceController,
                       aerial_robot_control::ControlBase)
