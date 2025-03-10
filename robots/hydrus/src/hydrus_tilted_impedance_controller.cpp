#include <hydrus/hydrus_tilted_impedance_controller.h>

using namespace aerial_robot_control;

HydrusTiltedImpedanceController::HydrusTiltedImpedanceController():
  UnderActuatedTiltedImpedanceController()
{
}

void HydrusTiltedImpedanceController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  UnderActuatedTiltedImpedanceController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
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
  est_external_wrench_ = Eigen::VectorXd::Zero(6);
  init_sum_momentum_ = Eigen::VectorXd::Zero(6);
  integrate_term_ = Eigen::VectorXd::Zero(6);
  prev_est_wrench_timestamp_ = 0;
  estimate_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("estimated_external_wrench", 1);

  wrench_estimate_thread_ = boost::thread([this]()
                                          {
                                            ros::Rate loop_rate(100.0);
                                            while(ros::ok())
                                              {
                                                externalWrenchEstimate();
                                                loop_rate.sleep();
                                              }
                                          });
  
}

bool HydrusTiltedImpedanceController::checkRobotModel()
{
  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("Impedance gain generator", "Impedance gain generator: robot model is not initiliazed");
      return false;
    }

  if(!robot_model_->stabilityCheck(verbose_))
    {
      ROS_ERROR_NAMED("Impedance gain generator", "Impedance gain generator: invalid pose, stability is invalid");

      return false;
    }
  return true;
  
}

void HydrusTiltedImpedanceController::controlCore()
{
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  setTargetWrenchAccCog(target_wrench_acc_cog);


  Eigen::MatrixXd BE = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Bpr = Eigen::MatrixXd::Zero(3, 6);
  Eigen::MatrixXd Bn = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd CE = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd J = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd xi =  Eigen::VectorXd::Zero(9); 
  Eigen::VectorXd xi_dot =  Eigen::VectorXd::Zero(9); 
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
  //J.block(0, 0, 3, 3) = Q;
  if (mode_.data == 1) // position_control
    J.block(3, 3, 3, 3) = Rc.inverse() * (Je_p - (J1_p + J2_p + J3_p + J4_p) / 4);

  // std::cout<<"Q"<<Q<<std::endl;

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
  Bn = BE;
  ros::Time time = ros::Time::now();
  BE -= Bpr.transpose() * Bpr / uav_mass;

  // Parameters for impedance control
  // Setting Kd as uav_mass + (-Cx + 2 * sqrt(Bx * Kp) place the system at critical damping
  // If you set Kd so small(at underdamped), you can see the UAV jumping like inertia-spring system
  // Here Bx = uav_mass and Cx = -Bx
  // See Exploiting Redundancy in Cartesian Impedance Control of UAVs  Equipped with a Robotic Arm, Equation (10)
  Kp.block(0, 0, 2, 2) = roll_pitch_p_ * Eigen::Matrix2d::Identity();
  Kp(2, 2) = yaw_p_;
  if (mode_.data == 1)
    Kp.block(3, 3, 3, 3) = pos_p_ * Eigen::Matrix3d::Identity();
  else
    Kp.block(3, 3, 3, 3) = joints_p_ * Eigen::Matrix3d::Identity();

  
  tf::Vector3 target_vel_ = navigator_->getTargetVel();
  tf::Vector3 target_acc_ = navigator_->getTargetAcc();
  tf::Vector3 target_omega_ = navigator_->getTargetOmega();
  tf::Vector3 target_ang_acc_ = navigator_->getTargetAngAcc();


  pos_ = estimator_->getPos(Frame::COG, estimator_->getEstimateMode());
  vel_ = estimator_->getVel(Frame::COG, estimator_->getEstimateMode());
  rpy_ = estimator_->getEuler(Frame::COG, estimator_->getEstimateMode());
  omega_ = estimator_->getAngularVel(Frame::COG, estimator_->getEstimateMode());
  x(0) = pid_controllers_.at(ROLL).getErrP();
  x(1) = pid_controllers_.at(PITCH).getErrP();
  x(2) = pid_controllers_.at(YAW).getErrP();
  x_dot(0) = pid_controllers_.at(ROLL).getErrD();
  x_dot(1) = pid_controllers_.at(PITCH).getErrD();
  x_dot(2) = pid_controllers_.at(YAW).getErrD();
  x_d_dot(0) = target_omega_.x();
  x_d_dot(1) = target_omega_.y();
  x_d_dot(2) = target_omega_.z();
  x_d_ddot(0) = target_ang_acc_.x();
  x_d_ddot(1) = target_ang_acc_.y();
  x_d_ddot(2) = target_ang_acc_.z();
  xi(0) = pos_.x();
  xi(1) = pos_.y();
  xi(2) = pos_.z();
  xi(3) = rpy_.x();
  xi(4) = rpy_.y();
  xi(5) = rpy_.z();
  xi(6) = joint_pos_[4];
  xi(7) = joint_pos_[5];
  xi(8) = joint_pos_[6];
  xi_dot(0) = vel_.x();
  xi_dot(1) = vel_.y();
  xi_dot(2) = vel_.z();
  xi_dot(3) = omega_.x();
  xi_dot(4) = omega_.y();
  xi_dot(5) = omega_.z();
  xi_dot(6) = joint_vel_[4];
  xi_dot(7) = joint_vel_[5];
  xi_dot(8) = joint_vel_[6];
  if (mode_.data == 1)
  {
    x(3) = pos_cmd_.x - (Rc.inverse() * (Pe - Pc))[0];
    x(4) = pos_cmd_.y - (Rc.inverse() * (Pe - Pc))[1];
    x(5) = pos_cmd_.z - (Rc.inverse() * (Pe - Pc))[2];
    x_dot(3) = - ((Rc.inverse() * (Pe - Pc))[0] - Pre_Pe_[0]) / (time-time_).toSec();
    x_dot(4) = - ((Rc.inverse() * (Pe - Pc))[1] - Pre_Pe_[1]) / (time-time_).toSec();
    x_dot(5) = - ((Rc.inverse() * (Pe - Pc))[2] - Pre_Pe_[2]) / (time-time_).toSec();
  }
  else
  {  
    x(3) = target_joint_pos_[0] - joint_pos_[4];
    x(4) = target_joint_pos_[1] - joint_pos_[5];
    x(5) = target_joint_pos_[2] - joint_pos_[6];
    x_dot(3) = target_joint_vel_[0] - joint_vel_[4];
    x_dot(4) = target_joint_vel_[1] - joint_vel_[5];
    x_dot(5) = target_joint_vel_[2] - joint_vel_[6];
    x_d_dot(3) = target_joint_vel_[0];
    x_d_dot(4) = target_joint_vel_[1];
    x_d_dot(5) = target_joint_vel_[2];
    x_d_ddot(3) = target_joint_acc_[0];
    x_d_ddot(4) = target_joint_acc_[1];
    x_d_ddot(5) = target_joint_acc_[2];
  }

  Eigen::MatrixXd delta_B = Eigen::MatrixXd::Zero(9, 9);
  delta_B.block(0, 3, 3, 6) = Bpr - Pre_Bpr_;
  delta_B.block(3, 0, 6, 3) = delta_B.block(0, 3, 3, 6).transpose();
  delta_B.block(3, 3, 6, 6) = Bn - Pre_Bn_;

  Eigen::MatrixXd C = getCmatrix(delta_B, xi - Pre_xi_, xi_dot);
  CE = -C.block(3, 0, 6, 3) * Bpr / uav_mass + C.block(3, 3, 6, 6);



  // Suppose C = 0, then Cx = -Bx,  see Exploiting Redundancy in Cartesian Impedance Control of UAVs Equipped with a Robotic Arm, Equation (9)
  
  Eigen::MatrixXd Bx = aerial_robot_model::pseudoinverse(J).transpose() * BE * aerial_robot_model::pseudoinverse(J);
  Eigen::MatrixXd Cx = aerial_robot_model::pseudoinverse(J).transpose() * (CE - BE * aerial_robot_model::pseudoinverse(J) * (J - Pre_J_) / (time-time_).toSec()) * aerial_robot_model::pseudoinverse(J);
  
  
  //Kd = -Cx + 2 * (Kp * Bx).sqrt();
  Eigen::MatrixXd Sigma = Eigen::MatrixXd::Zero(6, 6);
  for (int i = 0; i < 6; i++) Sigma(i, i) = abs(Bx(i, i));

  // std::cout<<"CE: "<<CE<<std::endl;
  // std::cout<<"Cx: "<<Cx<<std::endl;

  //Kd.block(0, 0, 3, 3) = -Cx.block(0, 0, 3, 3) + 2 * 0.9 * (Kp.block(0, 0, 3, 3) * abs(Bx(2, 2))).sqrt();
  Kd.block(0, 0, 2, 2) = roll_pitch_d_ * Eigen::Matrix2d::Identity();
  Kd(2, 2) = yaw_d_;
  if (mode_.data == 1)
    Kd.block(3, 3, 3, 3) = pos_d_ * Eigen::Matrix3d::Identity();
  else
    Kd.block(3, 3, 3, 3) = joints_d_ * Eigen::Matrix3d::Identity();
  //Kd.block(3, 3, 3, 3) = -Cx.block(3, 3, 3, 3) + 2 * 0.2 * (Kp.block(3, 3, 3, 3) * Bx.block(3, 3, 3, 3)).sqrt();

  //Kd = -Cx + 2 * 1.0 * (Kp * Sigma).sqrt();


  // Exploiting Redundancy in Cartesian Impedance Control of UAVs Equipped with a Robotic Arm, Equation (9)
  Eigen::VectorXd a = Cx * x_d_dot;
  for (int i = 0; i < a.size(); i++)
  {
    if (a(i) > 0.3)
      a(i) = 0.3;

    if (a(i) < -0.3)
      a(i) = -0.3;
  }
 // u = J.transpose() * (Bx * x_d_ddot + Cx * x_d_dot + Kd * x_dot + Kp * x);
  u = J.transpose() * (Bx * x_d_ddot + a + Kd * x_dot + Kp * x);
  // Gravity compensation

  double Kpz = 10.0;
  double Kdz = 2 * sqrt(uav_mass * Kpz);
  double uz = Kdz * pid_controllers_.at(Z).getErrP() + Kpz * pid_controllers_.at(Z).getErrD() + uav_mass * aerial_robot_estimation::G;
  //


  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_inv = aerial_robot_model::pseudoinverse(P);

  Eigen::VectorXd target_total_thrust = P_inv.col(2) * uz + P_inv.col(3) * u(0) + P_inv.col(4) * u(1) + P_inv.col(5) * u(2);
  target_thrust_z_term_ = P_inv.col(2) * uz;
  //if (target_joint_pos_[0] > 1.56)
  target_thrust_yaw_term_ =  P_inv.col(5) * u(2); 
  std_msgs::Float64 j1_term, j2_term, j3_term;

  //std::cout<<"P"<<P<<std::endl;
  //std::cout<<"P_inv'"<<P_inv<<std::endl;

  Eigen::VectorXd f1 = R1.inverse()*Rc*P.block(0, 0, 3, 1);
  Eigen::VectorXd f2 = R2.inverse()*Rc*P.block(0, 1, 3, 1);
  Eigen::VectorXd f3 = R2.inverse()*Rc*P.block(0, 0, 3, 1);
  Eigen::VectorXd f4 = R4.inverse()*Rc*P.block(0, 3, 3, 1);

  //std::cout<<"J.transpose()"<<J.transpose()<<std::endl;

  // j1_term.data = u(3);
  // j2_term.data = u(4);
  // j3_term.data = u(5);
  j1_term.data = u(3) - f1[1] * target_thrust_z_term_[0] * 0.3;
  j2_term.data = u(4) - f2[1] * target_thrust_z_term_[1] * 0.3 - f3[1] * target_thrust_z_term_[0] * (0.6 + 0.3 * abs(cos(joint_pos_[0])));
  j3_term.data = u(5) - f4[1] * target_thrust_z_term_[3] * 0.3;
  // std::cout<<"u1"<<x_dot<<std::endl;
  // std::cout<<"u2"<<Cx * x_d_dot<<std::endl;
  // std::cout<<"u3"<<Kd * x_dot<<std::endl;
  // std::cout<<"u4"<<Kp * x<<std::endl;

  // std::cout<<"f1"<<f1[1] * target_thrust_z_term_[0] * 0.3<<std::endl;
  // std::cout<<"f2"<<f2[1] * target_thrust_z_term_[1] * 0.3 - f3[1] * target_thrust_z_term_[0] * (0.6 + 0.3 * abs(cos(joint_pos_[0])))<<std::endl;
  // std::cout<<"f3"<<f4[1] * target_thrust_z_term_[3] * 0.3<<std::endl;


  // std::cout<<target_thrust_z_term_<<std::endl;

  // joint_cmd_pubs_[0].publish(j1_term);
  // joint_cmd_pubs_[1].publish(j2_term);
  // joint_cmd_pubs_[2].publish(j3_term);
  Eigen::MatrixXd pe = Rc.inverse() * (Pe - Pc);
  std_msgs::Float64 pe1_term, pe2_term, pe3_term;

  pe1_term.data = a(4);
  pe2_term.data = (Kd * x_dot)(4);
  pe3_term.data = (Kp * x)(4);


  pos_pubs_[0].publish(pe1_term);
  pos_pubs_[1].publish(pe2_term);
  pos_pubs_[2].publish(pe3_term);

 
//   // std::cout<<"target_thrust_yaw_term_: "<< target_thrust_yaw_term_<<std::endl;
//   // std::cout<<"sum: "<< target_thrust_yaw_term_(0) + target_thrust_yaw_term_(1) +target_thrust_yaw_term_(2) +target_thrust_yaw_term_(3)<<std::endl;
  // std::cout<<"joint_pos_:"<<joint_pos_[4]<<" "<<joint_pos_[5]<<" "<<joint_pos_[6]<<std::endl;
  // std::cout<<"tar_joint_pos_:"<<target_joint_pos_[0]<<" "<<target_joint_pos_[1]<<" "<<target_joint_pos_[2]<<std::endl;
  // std::cout<<"uz: "<< uz<<std::endl;
  // std::cout<<"j1: "<< u(3)<<std::endl;
  // std::cout<<"j2: "<< u(4)<<std::endl;
  // std::cout<<"j3: "<< u(5)<<std::endl;
  // std::cout<<"------------------------"<<std::endl;
  // std::cout<<"x: "<< x <<std::endl;
  // std::cout<<"x_dot: "<< x_dot <<std::endl;
  // std::cout<<"pe: "<< Rc.inverse() * (Pe - Pc)<<std::endl;
//   // std::cout<<"Bx: "<< Bx<<std::endl;
//   // std::cout<<"Cx: "<< Cx<<std::endl;
//   // std::cout<<"Kd_: "<< Kd_<<std::endl;
//   // std::cout<<"Kp_: "<< Kp_<<std::endl;
//   // std::cout<<"j: "<<  J_<<std::endl;
//   // std::cout<<"j: "<<  Pre_J_<<std::endl;
//   // std::cout<<"Jdot: "<<  (J_ - Pre_J_) / (time-time_).toSec()<<std::endl;
// std::cout<<"u: "<< u<<std::endl;

  Pre_J_ = J;
  Pre_Pe_ = Rc.inverse() * (Pe - Pc);
  Pre_Bpr_ = Bpr;
  Pre_Bn_ = Bn;
  Pre_xi_ = xi;
  time_ = time;


  UnderActuatedTiltedImpedanceController::controlCore();
}

Eigen::MatrixXd HydrusTiltedImpedanceController::getCmatrix(Eigen::MatrixXd delta_M, Eigen::VectorXd delta_xi,  Eigen::VectorXd xi_dot)
{
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(9, 9);
  for (int i = 0; i < 9; i++)
  {
    for (int j = 0; j < 9; j++)
    {
      double c = 0;
      for (int k = 0; k < 9; k++)
      {
        c +=  (delta_M(i, j)/delta_xi(k) + delta_M(i, k)/delta_xi(j) + delta_M(j, k)/delta_xi(i)) * xi_dot(k) / 2.0;      
      }
      C(i, j) = c;
    }
  }
  return C;

}

void HydrusTiltedImpedanceController::rosParamInit()
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
  momentum_observer_matrix_ = Eigen::MatrixXd::Identity(6,6);
  // double force_weight, torque_weight;
  // getParam<double>(control_nh, "momentum_observer_force_weight", force_weight, 10.0);
  // getParam<double>(control_nh, "momentum_observer_torque_weight", torque_weight, 10.0);
  momentum_observer_matrix_.topRows(3) *= 3;
  momentum_observer_matrix_.bottomRows(3) *= 2.5;
}

void HydrusTiltedImpedanceController::externalWrenchEstimate()
{
  if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
     navigator_->getNaviState() != aerial_robot_navigation::LAND_STATE)
    {
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(6);
      return;
    }

  Eigen::Vector3d vel_w, omega_cog; // workaround: use the filtered value
  auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::HydrusImu>(estimator_->getImuHandler(0));
  if (!imu_handler)
  {
    ROS_ERROR("HydrusImu is null!");
    return;
  }

  tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);
  tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);
  Eigen::Matrix3d cog_rot;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  double mass = robot_model_->getMass();

  Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
  sum_momentum.head(3) = mass * vel_w;
  sum_momentum.tail(3) = inertia * omega_cog;

  Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6,6);
  J_t.topLeftCorner(3,3) = cog_rot;
  Eigen::VectorXd N = mass * robot_model_->getGravity();
  N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

  const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();
  Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
  target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
  target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);
  if(prev_est_wrench_timestamp_ == 0)
    {
      prev_est_wrench_timestamp_ = ros::Time::now().toSec();
      init_sum_momentum_ = sum_momentum; // not good
    }

  double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;
  integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_) * dt;
  est_external_wrench_ = momentum_observer_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);
  Eigen::VectorXd est_external_wrench_cog = est_external_wrench_;
  est_external_wrench_cog.head(3) = cog_rot.inverse() * est_external_wrench_.head(3);
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
  wrench_msg.wrench.force.x = est_external_wrench_(0);
  wrench_msg.wrench.force.y = est_external_wrench_(1);
  wrench_msg.wrench.force.z = est_external_wrench_(2);
  wrench_msg.wrench.torque.x = est_external_wrench_(3);
  wrench_msg.wrench.torque.y = est_external_wrench_(4);
  wrench_msg.wrench.torque.z = est_external_wrench_(5);
  estimate_external_wrench_pub_.publish(wrench_msg);
  prev_est_wrench_timestamp_ = ros::Time::now().toSec();
}


Eigen::Matrix3d HydrusTiltedImpedanceController::getPositionJacobian(std::string name)
{
    Eigen::MatrixXd jacobian = robot_model_->getJacobians(name);
    Eigen::MatrixXd p_jacobian = Eigen::Matrix3d::Zero();
    // std::cout<< p_jacobian<<std::endl;
    p_jacobian.block(0, 0, 3, 1) = jacobian.block(0, 1, 3, 1);
    p_jacobian.block(0, 1, 3, 1) = jacobian.block(0, 3, 3, 1);
    p_jacobian.block(0, 2, 3, 1) = jacobian.block(0, 5, 3, 1);
    return p_jacobian;

}

Eigen::Matrix3d HydrusTiltedImpedanceController::getOrientationJacobian(std::string name)
{
    Eigen::MatrixXd jacobian = robot_model_->getJacobians(name);
    Eigen::MatrixXd o_jacobian = Eigen::Matrix3d::Zero();
    o_jacobian.block(0, 0, 3, 1) = jacobian.block(3, 1, 3, 1);
    o_jacobian.block(0, 1, 3, 1) = jacobian.block(3, 3, 3, 1);
    o_jacobian.block(0, 2, 3, 1) = jacobian.block(3, 5, 3, 1);
    return o_jacobian;
}



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedImpedanceController, aerial_robot_control::ControlBase);
