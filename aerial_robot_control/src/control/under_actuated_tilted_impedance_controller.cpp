// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <aerial_robot_control/control/under_actuated_tilted_impedance_controller.h>

using namespace aerial_robot_control;

void UnderActuatedTiltedImpedanceController::initialize(ros::NodeHandle nh,
                                           ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  UnderActuatedImpedanceController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  desired_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);

  pid_msg_.z.p_term.resize(1);
  pid_msg_.z.i_term.resize(1);
  pid_msg_.z.d_term.resize(1);
  z_limit_ = pid_controllers_.at(Z).getLimitSum();
  pid_controllers_.at(Z).setLimitSum(1e6); // do not clamp the sum of PID terms for z axis
  target_wrench_cog_ = Eigen::VectorXd::Zero(6);
  est_external_wrench_clamped_ = Eigen::VectorXd::Zero(6);
}

void UnderActuatedTiltedImpedanceController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  spinal::FourAxisCommandImpedance flight_command_impedance_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_command_impedance_data.base_thrust = target_base_thrust_;
  flight_command_impedance_data.roll_thrust = target_roll_thrust_;
  flight_command_impedance_data.pitch_thrust = target_pitch_thrust_;
  flight_command_impedance_data.yaw_thrust = target_yaw_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
  flight_impedance_cmd_pub_.publish(flight_command_impedance_data);
}


void UnderActuatedTiltedImpedanceController::controlCore()
{
  PoseLinearController::controlCore();


  // Inerial params
  double uav_mass = robot_model_->getMass();
  double mx = 0.5 * uav_mass;
  double my = 2.0 * uav_mass;
  double mz = 2.0 * uav_mass;
  Eigen::Matrix3d I = robot_model_->getInertia<Eigen::Matrix3d>();
  //Eigen::Matrix3d Id = 2 * I;
  Eigen::Matrix3d Id = Eigen::Matrix3d::Zero();
  Id(0, 0) = 0.4;
  Id(1, 1) = 0.4;
  Id(2, 2) = 0.8;
  // Control params
  double Kpx = x_y_p_;
  double Kpy = x_y_p_;
  double Kpz = z_p_;
  double Kdx = 2 * x_y_zeta_ * sqrt(x_y_p_);
  double Kdy = 2 * x_y_zeta_ * sqrt(x_y_p_);
  double Kdz = 2 * z_zeta_ * sqrt(z_p_);
  Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd Zeta = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(3, 3);
  Kp.block(0, 0, 2, 2) = roll_pitch_p_ * Eigen::Matrix2d::Identity();
  Kp(2, 2) = yaw_p_;
  Zeta.block(0, 0, 2, 2) = roll_pitch_zeta_ * Eigen::Matrix2d::Identity();
  Zeta(2, 2) = yaw_zeta_;
  Kd = 2 * Zeta * (Kp * Id).sqrt();
 
  tf::Matrix3x3 cog = estimator_->getOrientation(Frame::COG, estimate_mode_);
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      R(i, j) = cog[i][j];
    }
  }

  Eigen::VectorXd delta_p =  Eigen::VectorXd::Zero(6); 
  Eigen::VectorXd delta_v =  Eigen::VectorXd::Zero(6); 
  Eigen::VectorXd acc =  Eigen::VectorXd::Zero(6); 

  // tf::Vector3 target_vel_ = navigator_->getTargetVel();
  // tf::Vector3 target_acc_ = navigator_->getTargetAcc();
  // tf::Vector3 target_omega_ = navigator_->getTargetOmega();
  // tf::Vector3 target_ang_acc_ = navigator_->getTargetAngAcc();


  // pos_ = estimator_->getPos(Frame::COG, estimator_->getEstimateMode());
  // vel_ = estimator_->getVel(Frame::COG, estimator_->getEstimateMode());
  // rpy_ = estimator_->getEuler(Frame::COG, estimator_->getEstimateMode());
  // omega_ = estimator_->getAngularVel(Frame::COG, estimator_->getEstimateMode());
  Eigen::Vector3d omega;
  omega(0) = omega_.x();
  omega(1) = omega_.y();
  omega(2) = omega_.z();
  tf::Vector3 target_omega_cog = cog.inverse() * target_omega_;


  // tf::Vector3 target_rpy = tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z())) * target_rpy_cog;

  delta_p(0) = pos_.x() - target_pos_.x();
  delta_p(1) = pos_.y() - target_pos_.y();
  delta_p(2) = pos_.z() - target_pos_.z();
  delta_v(0) = vel_.x() - target_vel_.x();
  delta_v(1) = vel_.y() - target_vel_.y();
  delta_v(2) = vel_.z() - target_vel_.z();
  // acc(0) = target_acc_.x();
  // acc(1) = target_acc_.y();
  // acc(2) = target_acc_.z();
  clampEstExternalWrench();

  // double target_acc_x = acc(0) + (1 / mx - 1 / uav_mass) * est_external_wrench_clamped_[0] + (-Kdx * delta_v(0) - Kpx * delta_p(0));
  // double target_acc_y = acc(1) + (1 / my - 1 / uav_mass) * est_external_wrench_clamped_[1] + (-Kdy * delta_v(1) - Kpy * delta_p(1));
  // double target_acc_z = acc(2) + (1 / mz - 1 / uav_mass) * est_external_wrench_clamped_[2] + (-Kdz * delta_v(2) - Kpy * delta_p(2)) + aerial_robot_estimation::G;


  double target_acc_x = (-Kdx * delta_v(0) - Kpx * delta_p(0));
  double target_acc_y = (-Kdy * delta_v(1) - Kpy * delta_p(1));
  double target_acc_z = (-Kdz * delta_v(2) - Kpy * delta_p(2)) + aerial_robot_estimation::G;


  if (target_acc_x > 3.0)
    target_acc_x = 3.0;
  if (target_acc_x < -3.0)
    target_acc_x = -3.0;

  if (target_acc_y > 3.0)
    target_acc_y = 3.0;
  if (target_acc_y < -3.0)
    target_acc_y = -3.0;

  if (target_acc_z > 15.0)
    target_acc_z = 15.0;
  if (target_acc_z < -15.0)
    target_acc_z = -15.0;

  // std::cout<<"x"<<target_acc_y<<std::endl;
  // std::cout<<"y"<<(1 / my - 1 / uav_mass) * est_external_wrench_clamped_[1]<<std::endl;
  // std::cout<<"z"<<(-Kdx * delta_v(1) - Kpx * delta_p(1));
  // std::cout<<"w"<<acc(1);
  // std::cout<<"delta_p"<<delta_p<<std::endl;
  // std::cout<<"pos_.x()"<<pos_.x()<<std::endl;
  // std::cout<<"target_pos_.x()"<<target_pos_.x()<<std::endl;
  // std::cout<<"vel_.x()"<<vel_.x()<<std::endl;
  // std::cout<<"target_vel_.x()"<<target_vel_.x()<<std::endl;
  tf::Vector3 target_acc_w(target_acc_x,
                          target_acc_y,
                          target_acc_z);

  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  Eigen::VectorXd f = robot_model_->getStaticThrust();
  Eigen::VectorXd g = robot_model_->getGravity();
  Eigen::VectorXd allocate_scales = f / g.norm();
  Eigen::VectorXd target_thrust_z_term = allocate_scales * target_acc_w.length();
   std::cout<<"f"<<f<<std::endl;
  //Eigen::VectorXd target_thrust_z_term = Eigen::VectorXd::Zero(4);
  target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
  target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));

  // double rate = pid_controllers_.at(Z).result() / (aerial_robot_estimation::G - 0.3);
  // target_thrust_z_term = rate * target_thrust_z_term;
  if(navigator_->getForceLandingFlag())
  {
    target_pitch_ = 0;
    target_roll_ = 0;
  }

  Eigen::Matrix3d target_R = (Eigen::AngleAxisd(navigator_->getTargetRPY().z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(target_pitch_, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(target_roll_, Eigen::Vector3d::UnitX())).toRotationMatrix();
  Eigen::Matrix3d eR = (target_R.transpose() * R - R.transpose() * target_R) / 2;
  delta_p(3) = (eR(2, 1) - eR(1, 2)) / 2;
  delta_p(4) = (eR(0, 2) - eR(2, 0)) / 2;
  delta_p(5) = (eR(1, 0) - eR(0, 1)) / 2;
  delta_v(3) = omega_.x() - target_omega_cog.x();
  delta_v(4) = omega_.y() - target_omega_cog.y();
  delta_v(5) = omega_.z() - target_omega_cog.z();
  // acc(3) = target_ang_acc_.x();
  // acc(4) = target_ang_acc_.y();
  // acc(5) = target_ang_acc_.z();
  Eigen::Vector3d tau_cmd = Eigen::Vector3d::Zero();

  //tau_cmd = I * acc.segment(3, 3) + (I * Id.inverse() - Eigen::Matrix3d::Identity()) * est_external_wrench_clamped_.segment(3, 3) + (-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3)) + aerial_robot_model::skew(omega) * I * omega;
  tau_cmd = (-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3)) + aerial_robot_model::skew(omega) * I * omega;
  target_wrench_cog_(2) = target_acc_w.length() * uav_mass;
  std::cout<<"tar"<<tau_cmd<<std::endl;
  std::cout<<"dtheta"<<Kp * delta_p.segment(3, 3)<<std::endl;
  target_wrench_cog_.segment(3, 3) = tau_cmd;
  // std::cout<<"tau_cmd"<<tau_cmd<<std::endl;
  // std::cout<<"y"<<(I * Id.inverse() - Eigen::Matrix3d::Identity()) * est_external_wrench_clamped_.segment(3, 3)<<std::endl;
  // std::cout<<"z"<<(-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3)) + aerial_robot_model::skew(omega) * I * omega<<std::endl;
  for (int i = 0; i < 3; i++)
  {
    if (target_wrench_cog_(i) > 8.0)
    {
      target_wrench_cog_(i) = 8.0;
    }
    else if (target_wrench_cog_(i) < -8.0)
    {
      target_wrench_cog_(i) = -8.0;
    }
  }
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_rot_inv = aerial_robot_model::pseudoinverse(P.bottomRows(3));

  // Eigen::VectorXd target_total_thrust = P_inv.col(3) * u(0) + P_inv.col(4) * u(1) + P_inv.col(5) * u(2);

  target_thrust_roll_term_ = P_rot_inv.col(0) * tau_cmd(0);
  target_thrust_pitch_term_ = P_rot_inv.col(1) * tau_cmd(1);
  target_thrust_yaw_term_ =  P_rot_inv.col(2) * tau_cmd(2); 
  std::cout<<"roll_ "<<target_thrust_roll_term_ <<std::endl;
  std::cout<<"pitch_ "<<target_thrust_pitch_term_ <<std::endl;
  std::cout<<"yaw_ "<<target_thrust_yaw_term_ <<std::endl;
  std::cout<<"tau"<<tau_cmd<<std::endl;
  std::cout<<"P "<<P <<std::endl;
  std::cout<<"wrench"<<P * (target_thrust_roll_term_ + target_thrust_pitch_term_ + target_thrust_yaw_term_)<<std::endl;

  // constraint z (also  I term)
  int index;
  double max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - z_limit_;

  if(residual > 0)
    {
      pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
      target_thrust_z_term *= (1 - residual / max_term);
    }
  for(int i = 0; i < motor_num_; i++)
    {
      target_base_thrust_.at(i) = target_thrust_z_term(i) + target_thrust_roll_term_(i) + target_thrust_pitch_term_(i) + target_thrust_yaw_term_(i);
      target_roll_thrust_.at(i) = target_thrust_roll_term_(i);
      target_pitch_thrust_.at(i) = target_thrust_pitch_term_(i);
      target_yaw_thrust_.at(i) = target_thrust_yaw_term_(i);
      pid_msg_.z.total.at(i) =  target_thrust_z_term(i);
    }

  Eigen::MatrixXd q_mat_inv = getQInv();
  double ff_ang_yaw = navigator_->getTargetAngAcc().z();
  Eigen::VectorXd ff_ang_yaw_term = q_mat_inv.col(3) * ff_ang_yaw;
  target_thrust_yaw_term_ += ff_ang_yaw_term;
  // constraint yaw (also I term)
  int index_yaw;
  double max_yaw_term = target_thrust_yaw_term_.cwiseAbs().maxCoeff(&index_yaw);
  double yaw_residual = max_yaw_term - pid_controllers_.at(YAW).getLimitSum();
  if(yaw_residual > 0)
    {
      pid_controllers_.at(YAW).setErrI(pid_controllers_.at(YAW).getPrevErrI());
      target_thrust_yaw_term_ *= (1 - yaw_residual / max_yaw_term);
    }
  // special process for yaw since the bandwidth between PC and spinal
  //std::cout<<"P"<<robot_model_->calcWrenchMatrixOnCoG()<<std::endl;
  candidate_yaw_term_ = target_thrust_yaw_term_(0);
  candidate_yaw_term_ = 0.0;
}

void UnderActuatedTiltedImpedanceController::clampEstExternalWrench()
{
  est_external_wrench_clamped_ = est_external_wrench_;
  for (int i = 0; i < 6; i++)
  {
    if (i < 3)
    {
      if (est_external_wrench_clamped_[i] > 20)
      {
        est_external_wrench_clamped_[i] = 20;
      }
      else if (est_external_wrench_clamped_[i] < -20)
      {
        est_external_wrench_clamped_[i] = -20;
      }
    }
    else
    {
      if (est_external_wrench_clamped_[i] > 6)
      {
        est_external_wrench_clamped_[i] = 6;
      }
      else if (est_external_wrench_clamped_[i] < -6)
      {
        est_external_wrench_clamped_[i] = -6;
      }
    }
  }
}

bool UnderActuatedTiltedImpedanceController::optimalGain()
{
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_inv = aerial_robot_model::pseudoinverse(P);


  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i) = Eigen::Vector3d(P_inv(i,3) * roll_pitch_weight_(0),  0, P_inv(i,3) * roll_pitch_weight_(2));
      pitch_gains_.at(i) = Eigen::Vector3d(P_inv(i,4) * roll_pitch_weight_(0), 0, P_inv(i,4) * roll_pitch_weight_(2));
      yaw_gains_.at(i) = Eigen::Vector3d(P_inv(i,5) * yaw_weight_(0), 0, P_inv(i,5) * yaw_weight_(2));

    }

  return true;
}

void UnderActuatedTiltedImpedanceController::publishGain()
{
  UnderActuatedImpedanceController::publishGain();

  double roll,pitch, yaw;
  robot_model_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

  spinal::DesireCoord coord_msg;
  coord_msg.roll = roll;
  coord_msg.pitch = pitch;
  desired_baselink_rot_pub_.publish(coord_msg);
}

void UnderActuatedTiltedImpedanceController::rosParamInit()
{
  UnderActuatedImpedanceController::rosParamInit();

}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedTiltedImpedanceController, aerial_robot_control::ControlBase);
