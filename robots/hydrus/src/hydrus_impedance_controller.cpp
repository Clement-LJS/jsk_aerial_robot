#include <hydrus/hydrus_impedance_controller.h>

using namespace aerial_robot_control;

HydrusImpedanceController::HydrusImpedanceController():
  UnderActuatedImpedanceController()
{
}


void HydrusImpedanceController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  UnderActuatedImpedanceController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller1/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller2/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller3/simulation/command", 1));
  pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("pos/x", 1));
  pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("pos/y", 1));
  pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("pos/z", 1));
  target_joint_pos_[0] = 1.57;
  target_joint_pos_[1] = 1.57;
  target_joint_pos_[2] = 1.57;
  pos_cmd_.x = -0.3;
  pos_cmd_.y = 0.3;
  pos_cmd_.z = 0.0;
}

bool HydrusImpedanceController::checkRobotModel()
{
  boost::shared_ptr<HydrusRobotModel> hydrus_robot_model = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model_);
  lqi_mode_ = hydrus_robot_model->getWrenchDof();

  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("LQI gain generator", "LQI gain generator: robot model is not initiliazed");
      return false;
    }

  if(!robot_model_->stabilityCheck(verbose_))
    {
      ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: invalid pose, stability is invalid");
      if(hydrus_robot_model->getWrenchDof() == 4 && hydrus_robot_model->getFeasibleControlRollPitchMin() > hydrus_robot_model->getFeasibleControlRollPitchMinThre())
        {
          ROS_WARN_NAMED("LQI gain generator", "LQI gain generator: change to three axis stable mode");
          lqi_mode_ = 3;
          return true;
        }

      return false;
    }
  return true;
}

void HydrusImpedanceController::controlCore()
{
  Eigen::MatrixXd BE = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Bpr = Eigen::MatrixXd::Zero(3, 6);
  Eigen::MatrixXd CE = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd x =  Eigen::VectorXd::Zero(6); 
  Eigen::VectorXd x_dot =  Eigen::VectorXd::Zero(6); 
  Eigen::VectorXd x_d_dot =  Eigen::VectorXd::Zero(6); 
  Eigen::VectorXd x_d_ddot =  Eigen::VectorXd::Zero(6); 
  Eigen::VectorXd u = Eigen::VectorXd::Zero(6); 
 

  Eigen::Matrix3d J1_p = getPositionJacobian("link1");
  Eigen::Matrix3d J2_p = getPositionJacobian("link2");
  Eigen::Matrix3d J3_p = getPositionJacobian("link3");
  Eigen::Matrix3d J4_p = getPositionJacobian("link4");
  Eigen::Matrix3d Je_p = getPositionJacobian("end_effector");
  Eigen::Matrix3d J1_o = getOrientationJacobian("link1");
  Eigen::Matrix3d J2_o = getOrientationJacobian("link2");
  Eigen::Matrix3d J3_o = getOrientationJacobian("link3");
  Eigen::Matrix3d J4_o = getOrientationJacobian("link4");

  Eigen::Vector3d P1 = robot_model_->getPosition("link1");
  Eigen::Vector3d P2 = robot_model_->getPosition("link2");
  Eigen::Vector3d P3 = robot_model_->getPosition("link3");
  Eigen::Vector3d P4 = robot_model_->getPosition("link4");
  Eigen::Vector3d Pe = robot_model_->getPosition("end_effector");
  Eigen::Matrix3d R1 = robot_model_->getRotation("link1");
  Eigen::Matrix3d R2 = robot_model_->getRotation("link2");
  Eigen::Matrix3d R3 = robot_model_->getRotation("link3");
  Eigen::Matrix3d R4 = robot_model_->getRotation("link4");
  double M1 = robot_model_->getInertiaMap().at("link1").getMass();
  double M2 = robot_model_->getInertiaMap().at("link2").getMass();
  double M3 = robot_model_->getInertiaMap().at("link3").getMass();
  double M4 = robot_model_->getInertiaMap().at("link4").getMass();
  Eigen::Matrix3d I1 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link1").getRotationalInertia());
  Eigen::Matrix3d I2 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link2").getRotationalInertia());
  Eigen::Matrix3d I3 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link3").getRotationalInertia());
  Eigen::Matrix3d I4 = aerial_robot_model::kdlToEigen(robot_model_->getInertiaMap().at("link4").getRotationalInertia());
  Eigen::Matrix3d Rc = aerial_robot_model::kdlToEigen(robot_model_->getCog<KDL::Frame>().M);
  Eigen::Vector3d Pc = aerial_robot_model::kdlToEigen(robot_model_->getCog<KDL::Frame>().p);

  tf::Matrix3x3 cog = estimator_->getOrientation(Frame::COG, estimate_mode_);

  tf::Vector3 rpy = estimator_->getEuler(Frame::COG, estimate_mode_);
 
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0, 0) = cog.getRow(0).x();
  R(0, 1) = cog.getRow(0).y();
  R(0, 2) = cog.getRow(0).z();

  R(1, 0) = cog.getRow(1).x();
  R(1, 1) = cog.getRow(1).y();
  R(1, 2) = cog.getRow(1).z();

  R(2, 0) = cog.getRow(2).x();
  R(2, 1) = cog.getRow(2).y();
  R(2, 2) = cog.getRow(2).z();

  Eigen::Matrix3d T;
  T(0, 0) = 1.0;
  T(0, 1) = 0.0;
  T(0, 2) = -sin(rpy.y());

  T(1, 0) = 0.0;
  T(1, 1) = cos(rpy.x());
  T(1, 2) = sin(rpy.x()) * cos(rpy.y());

  T(2, 0) = 0.0;
  T(2, 1) = -sin(rpy.x());
  T(2, 2) = cos(rpy.x()) * cos(rpy.y());

  Eigen::Matrix3d Q = R.transpose() * T;

 
  if (mode_.data == 1) // position_control
    J.block(3, 3, 3, 3) = Rc.inverse() * (Je_p - (J1_p + J2_p + J3_p + J4_p) / 4);

  double uav_mass = robot_model_->getMass();
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();

  // Cartesian Impedance Control of a UAV with a Robotic Arm, Equation (14)

  Eigen::Matrix3d B12 = - M1*aerial_robot_model::skew(R*(P1+P2)/2)*T - M2*aerial_robot_model::skew(R*(P2+P3)/2)*T - M3*aerial_robot_model::skew(R*(P3+P4)/2)*T - M4*aerial_robot_model::skew(R*(P4+Pe)/2)*T;;
  Eigen::Matrix3d B13 = M1*R*J1_p + M2*R*J2_p + M3*R*J3_p + M4*R*J4_p;
  Eigen::Matrix3d B23 = Q.transpose()*R1.transpose()*I1*R1*J1_o + Q.transpose()*R2.transpose()*I2*R2*J2_o + Q.transpose()*R3.transpose()*I3*R3*J3_o + Q.transpose()*R4.transpose()*I4*R4*J4_o - T.transpose()*M1*aerial_robot_model::skew(R*(P1+P2)/2).transpose()*R*J1_p - T.transpose()*M2*aerial_robot_model::skew(R*(P2+P3)/2).transpose()*R*J2_p - T.transpose()*M3*aerial_robot_model::skew(R*(P3+P4)/2).transpose()*R*J3_p - T.transpose()*M4*aerial_robot_model::skew(R*(P4+Pe)/2).transpose()*R*J4_p;
  Bpr.block(0, 0, 3, 3) = B12;
  Bpr.block(0, 3, 3, 3) = B13;
  BE.block(0, 0, 3, 3) = Q.transpose() * inertia * Q;
  BE.block(3, 3, 3, 3) = M1*J1_p.transpose()*J1_p + M2*J2_p.transpose()*J2_p + M3*J3_p.transpose()*J3_p + M4*J4_p.transpose()*J4_p + J1_o.transpose()*R1.transpose()*I1*R1*J1_o + J2_o.transpose()*R2.transpose()*I2*R2*J2_o + J3_o.transpose()*R3.transpose()*I3*R3*J3_o + J4_o.transpose()*R4.transpose()*I4*R4*J4_o;
  BE.block(0, 3, 3, 3) = B23;
  BE.block(3, 0, 3, 3) = B23.transpose();
  ros::Time time = ros::Time::now();
  BE -= Bpr.transpose() * Bpr / uav_mass;
  CE = - Bpr.transpose() * (Bpr - Pre_Bpr_) / (time-time_).toSec() / (uav_mass * uav_mass);
 
  

  // Parameters for impedance control
  // Setting Kd as uav_mass + (-Cx + 2 * sqrt(Bx * Kp) place the system at critical damping
  // If you set Kd so small(at underdamped), you can see the UAV jumping like inertia-spring system
  // Here Bx = uav_mass and Cx = -Bx
  // See Exploiting Redundancy in Cartesian Impedance Control of UAVs  Equipped with a Robotic Arm, Equation (10)

  Kp.block(0, 0, 2, 2) = roll_pitch_p_ * Eigen::Matrix2d::Identity();
  Kp(2, 2) = yaw_p_;
  if (mode_.data == 1)
    Kp.block(3, 3, 3, 3) = joints_p_ * Eigen::Matrix3d::Identity();
  else
    Kp.block(3, 3, 3, 3) = pos_p_ * Eigen::Matrix3d::Identity();

  
  tf::Vector3 target_vel_ = navigator_->getTargetVel();
  tf::Vector3 target_acc_ = navigator_->getTargetAcc();
  tf::Vector3 target_omega_ = navigator_->getTargetOmega();
  tf::Vector3 target_ang_acc_ = navigator_->getTargetAngAcc();
 

  x(0) = pid_controllers_.at(ROLL).getErrP();
  x(1) = pid_controllers_.at(PITCH).getErrP();
  x(2) = pid_controllers_.at(YAW).getErrP();
  if (mode_.data == 1)
  {
    x(3) = pos_cmd_.x - (Rc.inverse() * (Pe - Pc))[0];
    x(4) = pos_cmd_.y - (Rc.inverse() * (Pe - Pc))[1];
    x(5) = pos_cmd_.z - (Rc.inverse() * (Pe - Pc))[2];
  }
  else
  {  
    x(3) = target_joint_pos_[0] - joint_pos_[0];
    x(4) = target_joint_pos_[1] - joint_pos_[1];
    x(5) = target_joint_pos_[2] - joint_pos_[2];
  }

  x_dot(0) = pid_controllers_.at(ROLL).getErrD();
  x_dot(1) = pid_controllers_.at(PITCH).getErrD();
  x_dot(2) = pid_controllers_.at(YAW).getErrD();
  if (mode_.data == 1)
  {
    x_dot(3) = - ((Rc.inverse() * (Pe - Pc))[0] - Pre_Pe_[0]) / (time-time_).toSec();
    x_dot(4) = - ((Rc.inverse() * (Pe - Pc))[1] - Pre_Pe_[1]) / (time-time_).toSec();
    x_dot(5) = - ((Rc.inverse() * (Pe - Pc))[2] - Pre_Pe_[2]) / (time-time_).toSec();
  }
  else
  {  
    x_dot(3) = target_joint_vel_[0] - joint_vel_[0];
    x_dot(4) = target_joint_vel_[1] - joint_vel_[1];
    x_dot(5) = target_joint_vel_[2] - joint_vel_[2];
  }

  x_d_dot(0) = target_omega_.x();
  x_d_dot(1) = target_omega_.y();
  x_d_dot(2) = target_omega_.z();
  if (mode_.data == 1)
  {

  }
  else
  {
    x_d_dot(3) = target_joint_vel_[0];
    x_d_dot(4) = target_joint_vel_[1];
    x_d_dot(5) = target_joint_vel_[2];

  }

  x_d_ddot(0) = target_ang_acc_.x();
  x_d_ddot(1) = target_ang_acc_.y();
  x_d_ddot(2) = target_ang_acc_.z();
  if (mode_.data == 1)
  {

  }
  else
  {
    x_d_ddot(3) = target_joint_acc_[0];
    x_d_ddot(4) = target_joint_acc_[1];
    x_d_ddot(5) = target_joint_acc_[2];
  }



  // Suppose C = 0, then Cx = -Bx,  see Exploiting Redundancy in Cartesian Impedance Control of UAVs Equipped with a Robotic Arm, Equation (9)
  
  Eigen::MatrixXd Bx = aerial_robot_model::pseudoinverse(J).transpose() * BE * aerial_robot_model::pseudoinverse(J);
  Eigen::MatrixXd Cx = aerial_robot_model::pseudoinverse(J).transpose() * (CE - BE * aerial_robot_model::pseudoinverse(J) * (J - Pre_J_) / (time-time_).toSec()) * aerial_robot_model::pseudoinverse(J);
  
  
  //Kd = -Cx + 2 * (Kp * Bx).sqrt();
  Eigen::MatrixXd Sigma = Eigen::MatrixXd::Zero(6, 6);
  for (int i = 0; i < 6; i++) Sigma(i, i) = abs(Bx(i, i));




  //Kd.block(0, 0, 3, 3) = -Cx.block(0, 0, 3, 3) + 2 * 0.9 * (Kp.block(0, 0, 3, 3) * abs(Bx(2, 2))).sqrt();
  Kd.block(0, 0, 2, 2) = roll_pitch_d_ * Eigen::Matrix2d::Identity();
  Kd(2, 2) = yaw_d_;
  if (mode_.data == 1)
    Kd.block(3, 3, 3, 3) = joints_d_ * Eigen::Matrix3d::Identity();
  else
    Kd.block(3, 3, 3, 3) = pos_d_ * Eigen::Matrix3d::Identity();
  //Kd.block(3, 3, 3, 3) = -Cx.block(3, 3, 3, 3) + 2 * 0.2 * (Kp.block(3, 3, 3, 3) * Bx.block(3, 3, 3, 3)).sqrt();

  //Kd = -Cx + 2 * 1.0 * (Kp * Sigma).sqrt();

 // std::cout<<"x: "<<x<<std::endl;

  // Exploiting Redundancy in Cartesian Impedance Control of UAVs Equipped with a Robotic Arm, Equation (9)
  u = J.transpose() * (Bx * x_d_ddot + Cx * x_d_dot + Kd * x_dot + Kp * x);
  // Gravity compensation

  double Kpz = 10.0;
  double Kdz = 2 * sqrt(uav_mass * Kpz);
  double uz = Kdz * pid_controllers_.at(Z).getErrP() + Kpz * pid_controllers_.at(Z).getErrD() + uav_mass * aerial_robot_estimation::G;
  //


  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_inv_ = aerial_robot_model::pseudoinverse(P);


  target_thrust_z_term_ = P_inv_.col(2) * uz;
  //if (target_joint_pos_[0] > 1.56)
  target_thrust_yaw_term_ =  P_inv_.col(5) * u(2); 
  std_msgs::Float64 j1_term, j2_term, j3_term;

  j1_term.data = u(3);
  j2_term.data = u(4);
  j3_term.data = u(5);

  // std::cout<<target_thrust_z_term_<<std::endl;

  // joint_cmd_pubs_[0].publish(j1_term);
  // joint_cmd_pubs_[1].publish(j2_term);
  // joint_cmd_pubs_[2].publish(j3_term);
  
 
  // std::cout<<"target_thrust_yaw_term_: "<< target_thrust_yaw_term_<<std::endl;
  // std::cout<<"sum: "<< target_thrust_yaw_term_(0) + target_thrust_yaw_term_(1) +target_thrust_yaw_term_(2) +target_thrust_yaw_term_(3)<<std::endl;
  // std::cout<<"j1: "<< u(3)<<std::endl;
  // std::cout<<"j2: "<< u(4)<<std::endl;
  // std::cout<<"j3: "<< u(5)<<std::endl;
  // std::cout<<"x: "<< x <<std::endl;
  // std::cout<<"pe: "<< Rc.inverse() * (Pe - Pc)<<std::endl;
  // std::cout<<"Bx: "<< Bx<<std::endl;
  // std::cout<<"Cx: "<< Cx<<std::endl;
  // std::cout<<"Kd_: "<< Kd_<<std::endl;
  // std::cout<<"Kp_: "<< Kp_<<std::endl;
  // std::cout<<"j: "<<  J_<<std::endl;
  // std::cout<<"j: "<<  Pre_J_<<std::endl;
  // std::cout<<"Jdot: "<<  (J_ - Pre_J_) / (time-time_).toSec()<<std::endl;
  // std::cout<<"u: "<< u<<std::endl;
  Eigen::MatrixXd pe = Rc.inverse() * (Pe - Pc);
  std::cout<<"Je_p"<<Je_p<<std::endl;
  std::cout<<"Rc"<<Rc<<std::endl;

  std::cout<<"J "<<J<<std::endl;
  std::cout<<"joint_pos_:"<<joint_pos_[4]<<" "<<joint_pos_[5]<<" "<<joint_pos_[6]<<std::endl;
  std::cout<<"tar_joint_pos_:"<<target_joint_pos_[0]<<" "<<target_joint_pos_[1]<<" "<<target_joint_pos_[2]<<std::endl;
  std::cout<<"u: "<< u<<std::endl;
  std::cout<<"j1: "<< u(3)<<std::endl;
  std::cout<<"j2: "<< u(4)<<std::endl;
  std::cout<<"j3: "<< u(5)<<std::endl;
  std::cout<<"------------------------"<<std::endl;
//   // std::cout<<"x: "<< x <<std::endl;
  std::cout<<"pe: "<< pe<<std::endl;
  std_msgs::Float64 pe1_term, pe2_term, pe3_term;

  pe1_term.data = pe(0);
  pe2_term.data = pe(1);
  pe3_term.data = pe(2);


  pos_pubs_[0].publish(pe1_term);
  pos_pubs_[1].publish(pe2_term);
  pos_pubs_[2].publish(pe3_term);
  Pre_J_ = J;
  Pre_Pe_ = Rc.inverse() * (Pe - Pc);
  Pre_Bpr_ = Bpr;
  time_ = time;


  UnderActuatedImpedanceController::controlCore();
}


void HydrusImpedanceController::rosParamInit()
{
  UnderActuatedImpedanceController::rosParamInit();

  ros::NodeHandle param_nh(nh_, "controller/impedance");

  getParam<double>(param_nh, "roll_pitch_p", roll_pitch_p_, 30.0);
  getParam<double>(param_nh, "yaw_p", yaw_p_, 30.0);
  getParam<double>(param_nh, "joints_p", joints_p_, 15.0);
  getParam<double>(param_nh, "pos_p", pos_p_, 10.0);
  getParam<double>(param_nh, "roll_pitch_d", roll_pitch_d_, 30.0);
  getParam<double>(param_nh, "yaw_d", yaw_d_, 30.0);
  getParam<double>(param_nh, "joints_d", joints_d_, 5.0);
  getParam<double>(param_nh, "pos_d", pos_d_, 4.0);
}

Eigen::Matrix3d HydrusImpedanceController::getPositionJacobian(std::string name)
{
    Eigen::MatrixXd jacobian = robot_model_->getJacobians(name);
    Eigen::Matrix3d p_jacobian = jacobian.block(0, 0, 3, 3);
    return p_jacobian;

}

Eigen::Matrix3d HydrusImpedanceController::getOrientationJacobian(std::string name)
{
    Eigen::MatrixXd jacobian = robot_model_->getJacobians(name);
    Eigen::Matrix3d o_jacobian = jacobian.block(3, 0, 3, 3);
    return o_jacobian;
}



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusImpedanceController, aerial_robot_control::ControlBase);
