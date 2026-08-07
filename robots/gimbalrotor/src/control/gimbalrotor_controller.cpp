#include <algorithm>

#include <gimbalrotor/control/gimbalrotor_controller.h>

using namespace std;

namespace aerial_robot_control
{
  GimbalrotorController::GimbalrotorController():
    PoseLinearController()
  {
    perching_servo_neutral_mode_ = false;
    previous_perching_servo_neutral_mode_ = false;
    previous_perching_takeoff_collective_active_ = false;
    perching_servo_neutral_mode_topic_ = "perching/servo_neutral_mode";
    perching_neutral_collective_acc_ = 9.80665;
    perching_neutral_collective_ramp_time_ = 0.8;
    perching_neutral_enter_time_ = ros::Time(0);
  }

  void GimbalrotorController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                         boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                         boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                         boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                         double ctrl_loop_rate
                                         )
  {
    PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
    gimbalrotor_robot_model_ = boost::dynamic_pointer_cast<GimbalrotorRobotModel>(robot_model);

    GimbalrotorController::rosParamInit();

    rotor_coef_ = gimbal_dof_ + 1; //number of virtual rotors in each rotor arm

    target_base_thrust_.resize(motor_num_ * rotor_coef_);
    target_full_thrust_.resize(motor_num_);
    target_gimbal_angles_.resize(motor_num_ * gimbal_dof_, 0);

    flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
    gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
    gimbal_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
    rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
    torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
    gimbal_dof_pub_ = nh_.advertise<std_msgs::UInt8>("gimbal_dof", 1);
    perching_servo_neutral_mode_sub_ =
      nh_.subscribe(perching_servo_neutral_mode_topic_, 1, &GimbalrotorController::perchingServoNeutralModeCallback, this);
  }

  void GimbalrotorController::reset()
  {
    PoseLinearController::reset();

    previous_perching_takeoff_collective_active_ = false;
    perching_neutral_enter_time_ = ros::Time(0);
    setAttitudeGains();
  }

  void GimbalrotorController::rosParamInit()
  {
    ros::NodeHandle control_nh(nh_, "controller");
    getParam<int>(control_nh, "gimbal_dof", gimbal_dof_, 1);
    getParam<bool>(control_nh, "gimbal_calc_in_fc", gimbal_calc_in_fc_, true);
    getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
    getParam<bool>(control_nh, "underactuate", underactuate_, false);
    getParam<double>(control_nh, "perching_neutral_collective_acc", perching_neutral_collective_acc_, 9.80665);
    getParam<double>(control_nh, "perching_neutral_collective_ramp_time", perching_neutral_collective_ramp_time_, 0.8);

    ros::NodeHandle navi_nh(nh_, "navigation");
    getParam<std::string>(navi_nh, "perching_servo_neutral_mode_topic", perching_servo_neutral_mode_topic_, std::string("perching/servo_neutral_mode"));

    if(!std::isfinite(perching_neutral_collective_acc_) ||
       perching_neutral_collective_acc_ <= 0.0)
      {
        perching_neutral_collective_acc_ = 9.80665;
      }

    if(!std::isfinite(perching_neutral_collective_ramp_time_) ||
       perching_neutral_collective_ramp_time_ < 0.0)
      {
        perching_neutral_collective_ramp_time_ = 0.8;
      }
  }

  bool GimbalrotorController::update()
  {
    sendGimbalCommand();
    if(gimbal_calc_in_fc_){
      std_msgs::UInt8 msg;
      msg.data = gimbal_dof_;
      gimbal_dof_pub_.publish(msg);
    }

    return PoseLinearController::update();
  }

void GimbalrotorController::modifyTargetWrenchAccCog(Eigen::VectorXd& target_wrench_acc_cog)
{
  (void)target_wrench_acc_cog;
}

void GimbalrotorController::controlCore()
{
  PoseLinearController::controlCore();

  /*
   * Servo-neutral mode is used during both perching takeoff and perching landing.
   * The dedicated positive collective ramp must be active only during the actual perching TAKEOFF_STATE.
   */
  const bool perching_takeoff_collective_active =
    perching_servo_neutral_mode_ &&
    navigator_->getNaviState() ==
      aerial_robot_navigation::TAKEOFF_STATE;

  const bool entering_perching_takeoff_collective = perching_takeoff_collective_active && !previous_perching_takeoff_collective_active_;

  if(entering_perching_takeoff_collective)
  {
    perching_neutral_enter_time_ = ros::Time::now();

    ROS_WARN(
      "[GimbalrotorController] "
      "perching takeoff collective ramp started");
  }

  if(!perching_takeoff_collective_active)
  {
    /*
     * Clear the takeoff ramp timer in ARM_ON, HOVER, LAND,
     * STOP and ARM_OFF states.
     */
    perching_neutral_enter_time_ = ros::Time(0);
  }

  previous_perching_takeoff_collective_active_ = perching_takeoff_collective_active;

  const bool leaving_servo_neutral_mode = previous_perching_servo_neutral_mode_ && !perching_servo_neutral_mode_;

    if(perching_servo_neutral_mode_ || leaving_servo_neutral_mode)
      {
        /*
         * X and Y are unavailable in fixed-axis mode.
         * Prevent unavailable-axis integral windup and a vectoring kick when
         * normal gimbal control resumes.
         */
        pid_controllers_.at(X).setErrI(0.0);
        pid_controllers_.at(Y).setErrI(0.0);
      }

    previous_perching_servo_neutral_mode_ =
      perching_servo_neutral_mode_;

    const bool effective_underactuate = underactuate_;
    const bool use_yaw_aligned_translation = underactuate_;
    tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
    tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                             pid_controllers_.at(Y).result(),
                             pid_controllers_.at(Z).result());
    tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;
    tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
    Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);

    if(use_yaw_aligned_translation)
      target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());
    else
      target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

    double target_ang_acc_x = pid_controllers_.at(ROLL).result();
    double target_ang_acc_y = pid_controllers_.at(PITCH).result();
    double target_ang_acc_z = pid_controllers_.at(YAW).result();
    Eigen::Matrix3d inertia = gimbalrotor_robot_model_->getInertia<Eigen::Matrix3d>();
    Eigen::Vector3d omega;
    tf::vectorTFToEigen(omega_, omega);
    Eigen::Vector3d gyro = omega.cross(inertia * omega);

    if(gimbal_calc_in_fc_)
      target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
    else
      target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z) + gyro;

    modifyTargetWrenchAccCog(target_wrench_acc_cog);

    setTargetWrenchAccCog(target_wrench_acc_cog);
    
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

    Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_);

    double mass_inv = 1 / gimbalrotor_robot_model_->getMass();

    Eigen::Matrix3d inertia_inv = inertia.inverse();

    std::vector<Eigen::Vector3d> rotors_origin_from_cog = gimbalrotor_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
    const auto& rotor_direction = gimbalrotor_robot_model_->getRotorDirection();
    const double m_f_rate = gimbalrotor_robot_model_->getMFRate();

    Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
    wrench_map.block(0, 0, 3, 3) =  Eigen::MatrixXd::Identity(3, 3);
    int last_col = 0;

    /* calculate normal allocation */
    for(int i = 0; i < motor_num_; i++){
      wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i)) + rotor_direction.at(i + 1) * m_f_rate * Eigen::Matrix3d::Identity();
      full_q_mat.middleCols(last_col, 3) = wrench_map;
      last_col += 3;
    }

    full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
    full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

    /* calculate masked rotation matrix */
    std::vector<KDL::Rotation> thrust_coords_rot = gimbalrotor_robot_model_->getThrustCoordRot<KDL::Rotation>();
    std::vector<Eigen::MatrixXd> masked_rot;
    for(int i = 0; i < motor_num_; i++){
      tf::Quaternion r;  tf::quaternionKDLToTF(thrust_coords_rot.at(i), r);
      Eigen::Matrix3d conv_cog_from_thrust; tf::matrixTFToEigen(tf::Matrix3x3(r),conv_cog_from_thrust);
      if(gimbal_dof_ == 1)
        {
          Eigen::MatrixXd mask(3, 2);
          mask << 0, 0, 1, 0, 0, 1;
          masked_rot.push_back(conv_cog_from_thrust * mask);
        }
      else if(gimbal_dof_ == 2)
        {
          Eigen::MatrixXd mask = Eigen::Matrix3d::Identity();
          masked_rot.push_back(conv_cog_from_thrust * mask);
        }
    }

    /* mask integrated allocation */
    Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, rotor_coef_ * motor_num_);
    Eigen::MatrixXd integrated_map = Eigen::MatrixXd::Zero(6, (gimbal_dof_ + 1) * motor_num_);
    for(int i = 0; i< motor_num_; i++){
      integrated_rot.block(3*i, rotor_coef_*i, 3, rotor_coef_) = masked_rot[i];
    }
    integrated_map = full_q_mat * integrated_rot;

    if(perching_servo_neutral_mode_)
      {
        const int fixed_axis_column = rotor_coef_ - 1;
        const int virtual_force_num = motor_num_ * rotor_coef_;

        Eigen::MatrixXd fixed_control_map = Eigen::MatrixXd::Zero(4, motor_num_);
        for(int i = 0; i < motor_num_; ++i)
          {
            const int source_column = rotor_coef_ * i + fixed_axis_column;
            fixed_control_map.col(i) = integrated_map.bottomRows(4).col(source_column);
          }

        const Eigen::MatrixXd fixed_control_map_inv =
          aerial_robot_model::pseudoinverse(fixed_control_map);

        if(fixed_control_map_inv.rows() != motor_num_ ||
           fixed_control_map_inv.cols() != 4 ||
           !fixed_control_map_inv.allFinite())
          {
            ROS_FATAL_THROTTLE(
                1.0,
                "[GimbalrotorController] "
                "invalid fixed-axis allocation in servo-neutral mode");

            target_vectoring_f_trans_ = Eigen::VectorXd::Zero(virtual_force_num);
            target_vectoring_f_rot_ = Eigen::VectorXd::Zero(virtual_force_num);
            integrated_map_inv_trans_ = Eigen::MatrixXd::Zero(virtual_force_num, 1);
            integrated_map_inv_rot_ = Eigen::MatrixXd::Zero(virtual_force_num, 3);

            std::fill(target_base_thrust_.begin(), target_base_thrust_.end(), 0.0f);
            std::fill(target_full_thrust_.begin(), target_full_thrust_.end(), 0.0f);

            for(int i = 0; i < motor_num_; ++i)
              {
                if(gimbal_dof_ == 1) target_gimbal_angles_.at(i) = 0.0;
                else if(gimbal_dof_ == 2)
                  {
                    target_gimbal_angles_.at(2 * i) = 0.0;
                    target_gimbal_angles_.at(2 * i + 1) = 0.0;
                  }
              }

            target_roll_ = navigator_->getTargetRPY().x();
            target_pitch_ = navigator_->getTargetRPY().y();
            candidate_yaw_term_ = 0.0;
            return;
          }

        Eigen::MatrixXd expanded_fixed_inv = Eigen::MatrixXd::Zero(virtual_force_num, 4);
        for(int i = 0; i < motor_num_; ++i)
          {
            const int fixed_row = rotor_coef_ * i + fixed_axis_column;
            expanded_fixed_inv.row(fixed_row) = fixed_control_map_inv.row(i);
          }

       /*
        * Default neutral-mode collective:
        *
        * During landing or neutral settling, retain the normal controller's body-Z output. Do not apply the takeoff collective ramp.
        */
        double neutral_collective_acc = std::max(0.0, target_wrench_acc_cog(2));

        if(perching_takeoff_collective_active)
        {
          /*
          * Only perching TAKEOFF_STATE uses the dedicated positive collective ramp.
          */
          double collective_ramp = 1.0;

          if(perching_neutral_collective_ramp_time_ > 1.0e-6 && !perching_neutral_enter_time_.isZero())
          {
            const double elapsed = (ros::Time::now() - perching_neutral_enter_time_).toSec();

            if(std::isfinite(elapsed))
            {
              collective_ramp =
                std::max(
                  0.0,
                  std::min(
                    1.0,
                    elapsed /
                    perching_neutral_collective_ramp_time_));
            }
          }

          neutral_collective_acc = collective_ramp * perching_neutral_collective_acc_;
        }

        Eigen::Vector4d neutral_base_command = Eigen::Vector4d::Zero();
        neutral_base_command(0) = neutral_collective_acc;

        const Eigen::VectorXd neutral_base_force = fixed_control_map_inv * neutral_base_command;

        if(neutral_base_force.size() != motor_num_ || !neutral_base_force.allFinite())
          {
            ROS_FATAL_THROTTLE(
                1.0,
                "[GimbalrotorController] "
                "invalid perching neutral base force");

            std::fill(
                target_base_thrust_.begin(),
                target_base_thrust_.end(),
                0.0f);

            std::fill(
                target_full_thrust_.begin(),
                target_full_thrust_.end(),
                0.0f);

            return;
          }

        target_vectoring_f_trans_ =
          Eigen::VectorXd::Zero(virtual_force_num);
        target_vectoring_f_rot_ =
          Eigen::VectorXd::Zero(virtual_force_num);
        integrated_map_inv_trans_ = expanded_fixed_inv.leftCols(1);
        integrated_map_inv_rot_ = expanded_fixed_inv.rightCols(3);

       /*
        * Preload the normal Z integrator only during perching takeoff.
        * Its only purpose is to prevent collective thrust from dropping when servo-neutral mode ends and fully actuated hover begins.
        * Do not force this preload during landing.
        */
        if(perching_takeoff_collective_active)
        {
          PID& z_pid = pid_controllers_.at(Z);

          const double z_i_gain = z_pid.getIGain();

          if(std::fabs(z_i_gain) > 1.0e-9)
          {
            double required_i_term = neutral_collective_acc - z_pid.getPTerm() - z_pid.getDTerm();

            required_i_term =
              std::max(
                -z_pid.getLimitI(),
                std::min(
                  z_pid.getLimitI(),
                  required_i_term));

            const double required_err_i = required_i_term / z_i_gain;

            z_pid.setErrI(required_err_i);
          }
        }

        std::fill(
            target_base_thrust_.begin(),
            target_base_thrust_.end(),
            0.0f);

        std::fill(
            target_full_thrust_.begin(),
            target_full_thrust_.end(),
            0.0f);

        for(int i = 0; i < motor_num_; ++i)
          {
            const int fixed_row = rotor_coef_ * i + fixed_axis_column;
            double base_force = neutral_base_force(i);

            if(base_force < 0.0 &&
               base_force > -1.0e-6)
              {
                base_force = 0.0;
              }

            if(base_force < 0.0)
              {
                ROS_ERROR_THROTTLE(
                    1.0,
                    "[GimbalrotorController] "
                    "negative perching neutral base "
                    "force for motor %d: %.6f",
                    i,
                    base_force);

                std::fill(
                    target_base_thrust_.begin(),
                    target_base_thrust_.end(),
                    0.0f);

                std::fill(
                    target_full_thrust_.begin(),
                    target_full_thrust_.end(),
                    0.0f);

                return;
              }

            double total_force = base_force;

            if(!gimbal_calc_in_fc_)
              {
                Eigen::Vector3d neutral_attitude_command;
                neutral_attitude_command <<
                  target_ang_acc_x,
                  target_ang_acc_y,
                  target_ang_acc_z;

                const Eigen::VectorXd neutral_attitude_force =
                  fixed_control_map_inv.rightCols(3) *
                  neutral_attitude_command;

                double attitude_scale = 1.0;
                for(int j = 0; j < motor_num_; ++j)
                  {
                    const double base = neutral_base_force(j);
                    const double delta = neutral_attitude_force(j);

                    if(delta < 0.0)
                      {
                        attitude_scale =
                          std::min(
                              attitude_scale,
                              base / (-delta));
                      }
                  }

                attitude_scale =
                  std::max(
                      0.0,
                      std::min(
                          1.0,
                          attitude_scale));

                target_vectoring_f_rot_(fixed_row) =
                  attitude_scale *
                  neutral_attitude_force(i);
                total_force += target_vectoring_f_rot_(fixed_row);
              }

            target_vectoring_f_trans_(fixed_row) = base_force;
            target_base_thrust_.at(fixed_row) = static_cast<float>(base_force);
            target_full_thrust_.at(i) = static_cast<float>(std::max(0.0, total_force));

            if(gimbal_dof_ == 1) target_gimbal_angles_.at(i) = 0.0;
            else if(gimbal_dof_ == 2)
              {
                target_gimbal_angles_.at(2 * i) = 0.0;
                target_gimbal_angles_.at(2 * i + 1) = 0.0;
              }
          }

        candidate_yaw_term_ = 0.0;

        target_roll_ = navigator_->getTargetRPY().x();
        target_pitch_ = navigator_->getTargetRPY().y();
        return;
      }

    /* extract controlled axis  */
    if(effective_underactuate)
      {
        target_wrench_acc_cog = target_wrench_acc_cog.tail(4);  // z, roll, pitch, yaw
        integrated_map = integrated_map.bottomRows(4);          // z, roll, pitch, yaw
      }

    /* vectoring force mapping */
    Eigen::MatrixXd integrated_map_inv = aerial_robot_model::pseudoinverse(integrated_map);
    integrated_map_inv_trans_ = integrated_map_inv.leftCols(effective_underactuate ? 1 : 3);
    integrated_map_inv_rot_ = integrated_map_inv.rightCols(3);
    if(effective_underactuate)
      target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog(0);
    else
      target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog.topRows(3);
    target_vectoring_f_rot_ = integrated_map_inv_rot_ * target_wrench_acc_cog.bottomRows(3); //debug
    last_col = 0;

    /* under actuated axis  */
    if(effective_underactuate)
      {
        if(hovering_approximate_)
          {
            target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
            target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
            navigator_->setTargetRoll(target_roll_);
            navigator_->setTargetPitch(target_pitch_);
          }
        else
          {
            target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
            target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
            navigator_->setTargetRoll(target_roll_);
            navigator_->setTargetPitch(target_pitch_);
          }
      }
    else
      {
	target_roll_ = navigator_->getTargetRPY().x();
	target_pitch_ = navigator_->getTargetRPY().y();
      }

    /*  calculate target base thrust (considering only translational components)*/
    double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
    for(int i = 0; i < motor_num_; i++){
      Eigen::VectorXd f_i = target_vectoring_f_trans_.segment(last_col, rotor_coef_);
      if(gimbal_dof_ == 1)
        {
          target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
          target_base_thrust_.at(rotor_coef_ * i+1) = f_i[1];
        }else if(gimbal_dof_ == 2){
          target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
          target_base_thrust_.at(rotor_coef_ * i+1) = f_i[1];
          target_base_thrust_.at(rotor_coef_ * i+2) = f_i[2];
        }
      if(integrated_map_inv(i, (effective_underactuate ? YAW - 2 : YAW)) > max_yaw_scale) max_yaw_scale = integrated_map_inv(i, (effective_underactuate ? YAW - 2 : YAW));  // underactuated: yaw col is shifted

      last_col += rotor_coef_;
    }
    candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;

    /* calculate target full thrusts and gimbal angles (considering full components)*/
    last_col = 0;
    for(int i = 0; i < motor_num_; i++){
      Eigen::VectorXd f_i_integrated = target_vectoring_f_rot_.segment(last_col, rotor_coef_) + target_vectoring_f_trans_.segment(last_col, rotor_coef_);
      target_full_thrust_.at(i) = f_i_integrated.norm();
      if(perching_servo_neutral_mode_)
        {
          if(gimbal_dof_ == 1) target_gimbal_angles_.at(i) = 0.0;
          else if(gimbal_dof_ == 2)
            {
              target_gimbal_angles_.at(2 * i) = 0.0;
              target_gimbal_angles_.at(2 * i + 1) = 0.0;
            }
        }
      else if(gimbal_dof_ == 1)
        {
          target_gimbal_angles_.at(i) = atan2(-f_i_integrated[0], f_i_integrated[1]);
        }
      else if(gimbal_dof_ == 2)
        {
          if(f_i_integrated[0] == 0 || f_i_integrated[2] == 0) continue;

          double gimbal_roll = atan2(-f_i_integrated[1], f_i_integrated[2]);
          double gimbal_pitch = atan2(f_i_integrated[0], -f_i_integrated[1] * sin(gimbal_roll) + f_i_integrated[2]* cos(gimbal_roll));
          target_gimbal_angles_.at(2 * i) = gimbal_roll;
          target_gimbal_angles_.at(2 * i + 1) = gimbal_pitch;
        }
      last_col += rotor_coef_;
    }
  }

  void GimbalrotorController::sendCmd()
  {
    PoseLinearController::sendCmd();

    if(gimbal_calc_in_fc_){
      sendTorqueAllocationMatrixInv();
      sendFourAxisCommand();
    }
    else
      {
        sendFourAxisCommand();

        sensor_msgs::JointState gimbal_control_msg;
        gimbal_control_msg.header.stamp = ros::Time::now();
        for(int i = 0; i < motor_num_; i++){
          if(gimbal_dof_ == 1)
            {
              gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
            }
          else if(gimbal_dof_ == 2)
            {
              gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2*i));
              gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2*i + 1));
            }
        }
        gimbal_control_pub_.publish(gimbal_control_msg);
        
        std_msgs::Float32MultiArray target_vectoring_force_msg;
        target_vectoring_f_ = target_vectoring_f_trans_ + target_vectoring_f_rot_;
        for(int i = 0; i < target_vectoring_f_.size(); i++){
          target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
        }
        target_vectoring_force_pub_.publish(target_vectoring_force_msg);
        
      }
  }

  void GimbalrotorController::sendFourAxisCommand()
  {
    spinal::FourAxisCommand flight_command_data;

    flight_command_data.angles[0] = target_roll_;
    flight_command_data.angles[1] = target_pitch_;

    if(gimbal_calc_in_fc_){
      flight_command_data.base_thrust = target_base_thrust_;
      flight_command_data.angles[2] = candidate_yaw_term_;
    }
    else
      {
        flight_command_data.base_thrust = target_full_thrust_;
      }

    flight_cmd_pub_.publish(flight_command_data);
  }

  void GimbalrotorController::sendGimbalCommand()
  {
    sensor_msgs::JointState gimbal_state_msg;
    gimbal_state_msg.header.stamp = ros::Time::now();
    for(int i = 0; i < motor_num_; i++){
      if(gimbal_dof_ == 1)
        {
          gimbal_state_msg.position.push_back(target_gimbal_angles_.at(i));
          std::string gimbal_name = "gimbal" + std::to_string(i+1);
          gimbal_state_msg.name.push_back(gimbal_name);
        }
      else if(gimbal_dof_ == 2)
      {
        gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2*i));
        gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2*i+1));
        std::string gimbal_roll_name = "gimbal" + std::to_string(i+1) + "_roll";
        std::string gimbal_pitch_name = "gimbal" + std::to_string(i+1) + "_pitch";
        gimbal_state_msg.name.push_back(gimbal_roll_name);
        gimbal_state_msg.name.push_back(gimbal_pitch_name);
      }
    }
    // gimbal_state_pub_.publish(gimbal_state_msg);

  }

  void GimbalrotorController::sendTorqueAllocationMatrixInv()
  {
    spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
    torque_allocation_matrix_inv_msg.rows.resize(motor_num_ * rotor_coef_);
    Eigen::MatrixXd torque_allocation_matrix_inv = integrated_map_inv_rot_;
    if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
      ROS_ERROR("Torque Allocation Matrix overflow");
    for (unsigned int i = 0; i < motor_num_* rotor_coef_; i++)
      {
        torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
        torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
        torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
      }
    torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
  }

  void GimbalrotorController::setAttitudeGains()
  {
    spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
    /* to flight controller via rosserial scaling by 1000 */
    rpy_gain_msg.motors.resize(1);
    rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
    rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
    rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
    rpy_gain_pub_.publish(rpy_gain_msg);
  }

  void GimbalrotorController::perchingServoNeutralModeCallback(const std_msgs::Bool::ConstPtr& msg)
  {
    /*
    * This flag controls only whether the gimbals are fixed at zero.
    *
    * The takeoff collective ramp is started separately when the navigator actually enters TAKEOFF_STATE.
    */
    perching_servo_neutral_mode_ = msg->data;
  }
} //namespace aerial_robot_controller



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorController, aerial_robot_control::ControlBase);
