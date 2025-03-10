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

}

void UnderActuatedTiltedImpedanceController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());

  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
  target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));

  if(navigator_->getForceLandingFlag())
    {
      target_pitch_ = 0;
      target_roll_ = 0;
    }

  Eigen::VectorXd f = robot_model_->getStaticThrust();
  Eigen::VectorXd g = robot_model_->getGravity();
  Eigen::VectorXd allocate_scales = f / g.norm();
  Eigen::VectorXd target_thrust_z_term = allocate_scales * target_acc_w.length();

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
      std::cout<<target_thrust_z_term(i)<<std::endl; 
      target_base_thrust_.at(i) = target_thrust_z_term(i);
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

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");

  getParam<double>(lqi_nh, "trans_constraint_weight", trans_constraint_weight_, 1.0);
  getParam<double>(lqi_nh, "att_control_weight", att_control_weight_, 1.0);
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedTiltedImpedanceController, aerial_robot_control::ControlBase);
