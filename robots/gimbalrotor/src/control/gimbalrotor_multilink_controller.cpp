#include <gimbalrotor/control/gimbalrotor_multilink_controller.h>

#include <aerial_robot_estimation/state_estimation.h>

#include <std_msgs/Float32MultiArray.h>
#include <pluginlib/class_list_macros.h>

using namespace std;

namespace aerial_robot_control
{
GimbalrotorMultilinkController::GimbalrotorMultilinkController():
  GimbalrotorController(),
  compensate_multilink_inertia_(true),
  inertia_dot_lpf_rate_(0.2),
  nonlinear_ang_acc_limit_(5.0),
  prev_inertia_initialized_(false),
  prev_inertia_stamp_(0.0)
{
  prev_inertia_.setZero();
  inertia_dot_lpf_.setZero();
}

void GimbalrotorMultilinkController::initialize(
    ros::NodeHandle nh,
    ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate)
{
  GimbalrotorController::initialize(nh,
                                    nhp,
                                    robot_model,
                                    estimator,
                                    navigator,
                                    ctrl_loop_rate);

  /*Extra debug topic for multilink compensation.*/
  multilink_nonlinear_acc_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/multilink_nonlinear_ang_acc", 1);

  ROS_INFO("[GimbalrotorMultilinkController] initialized.");
}

void GimbalrotorMultilinkController::reset()
{
  GimbalrotorController::reset();

  prev_inertia_initialized_ = false;
  prev_inertia_stamp_ = 0.0;
  prev_inertia_.setZero();
  inertia_dot_lpf_.setZero();

  ROS_INFO("[GimbalrotorMultilinkController] reset inertia compensation memory.");
}

void GimbalrotorMultilinkController::rosParamInit()
{
  GimbalrotorController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");

  /*
   * Multilink compensation parameters.
   */
  getParam<bool>(control_nh, "compensate_multilink_inertia", compensate_multilink_inertia_, true);
  getParam<double>(control_nh, "inertia_dot_lpf_rate", inertia_dot_lpf_rate_, 0.2);
  getParam<double>(control_nh, "nonlinear_ang_acc_limit", nonlinear_ang_acc_limit_, 5.0);

  if(inertia_dot_lpf_rate_ < 0.0)
    {
      inertia_dot_lpf_rate_ = 0.0;
    }

  if(inertia_dot_lpf_rate_ > 1.0)
    {
      inertia_dot_lpf_rate_ = 1.0;
    }

  if(nonlinear_ang_acc_limit_ < 0.0)
    {
      nonlinear_ang_acc_limit_ = 0.0;
    }

  ROS_INFO_STREAM("[GimbalrotorMultilinkController] compensate_multilink_inertia: " << compensate_multilink_inertia_);
  ROS_INFO_STREAM("[GimbalrotorMultilinkController] inertia_dot_lpf_rate: " << inertia_dot_lpf_rate_);
  ROS_INFO_STREAM("[GimbalrotorMultilinkController] nonlinear_ang_acc_limit: " << nonlinear_ang_acc_limit_);
}

Eigen::Vector3d GimbalrotorMultilinkController::limitVectorNorm(
    const Eigen::Vector3d& input,
    double max_norm) const
{
  if(max_norm <= 0.0)
    {
      return Eigen::Vector3d::Zero();
    }

  const double norm = input.norm();

  if(norm > max_norm)
    {
      return input / norm * max_norm;
    }

  return input;
}

Eigen::Vector3d GimbalrotorMultilinkController::calculateMultilinkNonlinearAngularAcc(
    const Eigen::Matrix3d& inertia,
    const Eigen::Vector3d& omega)
{
  if(!compensate_multilink_inertia_)
    {
      return Eigen::Vector3d::Zero();
    }

  const double now = ros::Time::now().toSec();

  if(!prev_inertia_initialized_)
    {
      prev_inertia_ = inertia;
      prev_inertia_stamp_ = now;
      inertia_dot_lpf_.setZero();
      prev_inertia_initialized_ = true;

      return Eigen::Vector3d::Zero();
    }

  const double dt = now - prev_inertia_stamp_;

  if(dt > 1e-5 && dt < 0.1)
    {
      /*
       * Jdot is calculated numerically.
       *
       * Why:
       *   Your hand/pitch link changes the total inertia J(q).
       *   If the link moves, J changes with time.
       *
       *   Jdot = dJ / dt
       */
      Eigen::Matrix3d inertia_dot_raw =
          (inertia - prev_inertia_) / dt;

      inertia_dot_lpf_ =
          (1.0 - inertia_dot_lpf_rate_) * inertia_dot_lpf_
          + inertia_dot_lpf_rate_ * inertia_dot_raw;
    }

  prev_inertia_ = inertia;
  prev_inertia_stamp_ = now;

  /*
   * Rigid-body rotational dynamics:
   *
   *   tau = J alpha + omega x J omega + Jdot omega
   *
   * Therefore the extra compensation torque is:
   *
   *   tau_comp = omega x J omega + Jdot omega
   */
  Eigen::Vector3d gyro_torque = omega.cross(inertia * omega);
  Eigen::Vector3d transform_torque = inertia_dot_lpf_ * omega;
  Eigen::Vector3d nonlinear_torque = gyro_torque + transform_torque;
  Eigen::Vector3d nonlinear_ang_acc = inertia.inverse() * nonlinear_torque;
  nonlinear_ang_acc = limitVectorNorm(nonlinear_ang_acc, nonlinear_ang_acc_limit_);
  return nonlinear_ang_acc;
}

void GimbalrotorMultilinkController::controlCore()
{
  /*
   * This function is based on the original GimbalrotorController::controlCore().
   *
   * Difference:
   *   Original:
   *     alpha_cmd = alpha_pid
   *     or alpha_cmd = alpha_pid + omega x J omega
   *
   *   Multilink:
   *     alpha_cmd = alpha_pid + J^-1 (omega x J omega + Jdot omega)
   *
   * This is needed because the pitch_joint / hand changes the total inertia.
   */

  PoseLinearController::controlCore();

  /* Current attitude of CoG frame.*/
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);

  /* Translational PID result in world frame.*/
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());

  /* For underactuated mode, use yaw-only transformed acceleration.*/
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse()* target_acc_w;

  /*For fully-actuated/vectoring mode, use CoG-frame acceleration.*/
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;

  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);

  if(underactuate_)
    {
      target_wrench_acc_cog.head(3) =
          Eigen::Vector3d(target_acc_dash.x(),
                          target_acc_dash.y(),
                          target_acc_dash.z());
    }
  else
    {
      target_wrench_acc_cog.head(3) =
          Eigen::Vector3d(target_acc_cog.x(),
                          target_acc_cog.y(),
                          target_acc_cog.z());
    }

  /* Angular PID result.*/
  const double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  const double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  const double target_ang_acc_z = pid_controllers_.at(YAW).result();

  /*
   * This inertia should come from GimbalrotorMultilinkRobotModel.
   *
   * Because that model updates with pitch_joint/hand position,
   * this inertia should represent the current whole robot body.
   */
  Eigen::Matrix3d inertia = gimbalrotor_robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Matrix3d inertia_inv = inertia.inverse();

  /* Current body angular velocity.*/
  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);

  /*
   * New multilink compensation.
   *
   * This includes:
   *   1. omega x J omega
   *   2. Jdot omega
   */
  Eigen::Vector3d nonlinear_ang_acc =
      calculateMultilinkNonlinearAngularAcc(inertia, omega);

  /*
   * Final angular acceleration command.
   *
   * If gimbal_calc_in_fc_ is true:
   *   The flight controller calculates part of the torque allocation.
   *   However, the flight controller probably does NOT know the moving hand inertia.
   *
   * Therefore, for multilink experiments, recommended:
   *   gimbal_calc_in_fc: false
   */
  target_wrench_acc_cog.tail(3) =
      Eigen::Vector3d(target_ang_acc_x,
                      target_ang_acc_y,
                      target_ang_acc_z)
      + nonlinear_ang_acc;

  setTargetWrenchAccCog(target_wrench_acc_cog);

  /*
   * Publish debug compensation.
   */
  {
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(nonlinear_ang_acc.x());
    msg.data.push_back(nonlinear_ang_acc.y());
    msg.data.push_back(nonlinear_ang_acc.z());
    multilink_nonlinear_acc_pub_.publish(msg);
  }

  /*
   * PID debug message, same as original controller.
   */
  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();

  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  /*
   * Allocation part.
   *
   * Important:
   *   The hand link is NOT added as a thrust source.
   *
   * We only use the main-body rotors.
   * But the rotor positions are taken from the current CoG,
   * so if the hand changes the CoG, the allocation still changes.
   */
  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_);

  const double mass_inv = 1.0 / gimbalrotor_robot_model_->getMass();

  std::vector<Eigen::Vector3d> rotors_origin_from_cog = gimbalrotor_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();

  const auto& rotor_direction = gimbalrotor_robot_model_->getRotorDirection();

  const double m_f_rate = gimbalrotor_robot_model_->getMFRate();

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);

  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

  int last_col = 0;

  for(int i = 0; i < motor_num_; i++)
    {
      /*
       * Force:
       *   F_i
       *
       * Torque:
       *   r_i x F_i + drag torque
       */
      wrench_map.block(3, 0, 3, 3) =
          aerial_robot_model::skew(rotors_origin_from_cog.at(i))
          + rotor_direction.at(i + 1)
          * m_f_rate
          * Eigen::Matrix3d::Identity();

      full_q_mat.middleCols(last_col, 3) =
          wrench_map;

      last_col += 3;
    }

  /* Convert force/torque map to acceleration map.*/
  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  /* Calculate masked thrust-coordinate rotation.*/
  std::vector<KDL::Rotation> thrust_coords_rot = gimbalrotor_robot_model_->getThrustCoordRot<KDL::Rotation>();

  std::vector<Eigen::MatrixXd> masked_rot;

  for(int i = 0; i < motor_num_; i++)
    {
      tf::Quaternion r;
      tf::quaternionKDLToTF(thrust_coords_rot.at(i), r);

      Eigen::Matrix3d conv_cog_from_thrust;
      tf::matrixTFToEigen(tf::Matrix3x3(r),
                          conv_cog_from_thrust);

      if(gimbal_dof_ == 1)
        {
          /*
           * 1-DoF gimbal:
           *
           * Original gimbalrotor virtual force basis.
           */
          Eigen::MatrixXd mask(3, 2);
          mask << 0, 0,
                  1, 0,
                  0, 1;

          masked_rot.push_back(conv_cog_from_thrust * mask);
        }
      else if(gimbal_dof_ == 2)
        {
          Eigen::MatrixXd mask = Eigen::Matrix3d::Identity();
          masked_rot.push_back(conv_cog_from_thrust * mask);
        }
      else
        {
          ROS_ERROR_THROTTLE(1.0, "[GimbalrotorMultilinkController] unsupported gimbal_dof: %d", gimbal_dof_);
          return;
        }
    }

  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, rotor_coef_ * motor_num_);

  Eigen::MatrixXd integrated_map = Eigen::MatrixXd::Zero(6, rotor_coef_ * motor_num_);

  for(int i = 0; i < motor_num_; i++)
    {
      integrated_rot.block(3 * i,
                           rotor_coef_ * i,
                           3,
                           rotor_coef_) =
          masked_rot.at(i);
    }

  integrated_map =
      full_q_mat * integrated_rot;

  /*
   * Extract controlled axes.
   *
   * Underactuated:
   *   z, roll, pitch, yaw
   *
   * Fully actuated:
   *   x, y, z, roll, pitch, yaw
   */
  if(underactuate_)
    {
      target_wrench_acc_cog = target_wrench_acc_cog.tail(4);
      integrated_map = integrated_map.bottomRows(4);
    }

  /* Vectoring force mapping.*/
  Eigen::MatrixXd integrated_map_inv = aerial_robot_model::pseudoinverse(integrated_map);

  integrated_map_inv_trans_ = integrated_map_inv.leftCols(underactuate_ ? 1 : 3);
  integrated_map_inv_rot_ = integrated_map_inv.rightCols(3);

  if(underactuate_)
    {
      target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog(0);
    }
  else
    {
      target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog.topRows(3);
    }

  target_vectoring_f_rot_ = integrated_map_inv_rot_ * target_wrench_acc_cog.bottomRows(3);

  /*
   * Underactuated target roll/pitch.
   */
  if(underactuate_)
    {
      if(hovering_approximate_)
        {
          target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
          target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
        }
      else
        {
          target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
          target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
        }

      navigator_->setTargetRoll(target_roll_);
      navigator_->setTargetPitch(target_pitch_);
    }
  else
    {
      target_roll_ = navigator_->getTargetRPY().x();
      target_pitch_ = navigator_->getTargetRPY().y();
    }

  /*
   * Calculate target base thrust.
   *
   * This part is kept close to original GimbalrotorController.
   */
  double max_yaw_scale = 0.0;

  last_col = 0;

  for(int i = 0; i < motor_num_; i++)
    {
      Eigen::VectorXd f_i = target_vectoring_f_trans_.segment(last_col, rotor_coef_);

      if(gimbal_dof_ == 1)
        {
          target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
          target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
        }
      else if(gimbal_dof_ == 2)
        {
          target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
          target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
          target_base_thrust_.at(rotor_coef_ * i + 2) = f_i[2];
        }

      const int yaw_col = underactuate_ ? YAW - 2 : YAW;

      if(integrated_map_inv(i, yaw_col) > max_yaw_scale)
        {
          max_yaw_scale = integrated_map_inv(i, yaw_col);
        }

      last_col += rotor_coef_;
    }

  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;

  /* Calculate full thrust and gimbal angles.*/
  last_col = 0;

  for(int i = 0; i < motor_num_; i++)
    {
      Eigen::VectorXd f_i_integrated = target_vectoring_f_rot_.segment(last_col, rotor_coef_) + target_vectoring_f_trans_.segment(last_col, rotor_coef_);

      target_full_thrust_.at(i) = f_i_integrated.norm();

      if(gimbal_dof_ == 1)
        {
          target_gimbal_angles_.at(i) = atan2(-f_i_integrated[0], f_i_integrated[1]);
        }
      else if(gimbal_dof_ == 2)
        {
          if(std::abs(f_i_integrated[0]) < 1e-6 || std::abs(f_i_integrated[2]) < 1e-6)
            {
              last_col += rotor_coef_;
              continue;
            }

          double gimbal_roll = atan2(-f_i_integrated[1], f_i_integrated[2]);
          double gimbal_pitch = atan2(f_i_integrated[0], -f_i_integrated[1] * sin(gimbal_roll) + f_i_integrated[2] * cos(gimbal_roll));

          target_gimbal_angles_.at(2 * i) = gimbal_roll;
          target_gimbal_angles_.at(2 * i + 1) = gimbal_pitch;
        }

      last_col += rotor_coef_;
    }

  ROS_DEBUG_STREAM_THROTTLE(0.5, "[GimbalrotorMultilinkController] nonlinear_ang_acc: " << nonlinear_ang_acc.transpose() << ", omega: " << omega.transpose());
}

} // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorMultilinkController,
                       aerial_robot_control::ControlBase);
