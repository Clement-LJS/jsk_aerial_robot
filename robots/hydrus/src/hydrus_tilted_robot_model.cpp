#include <hydrus/hydrus_tilted_robot_model.h>

HydrusTiltedRobotModel::HydrusTiltedRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon):
  HydrusRobotModel(init_with_rosparam, verbose, fc_t_min_thre, 0, epsilon, 4)
{
  const int rotor_num = getRotorNum();
  const auto& joint_indices = getJointIndices();
  const auto& link_joint_indices = getLinkJointIndices();
  gimbal_jacobian_ = Eigen::MatrixXd::Zero(6 + getJointNum(), 6 + getLinkJointIndices().size());
  gimbal_jacobian_.topLeftCorner(6,6) = Eigen::MatrixXd::Identity(6,6);
  gimbal_rotation_from_link_.resize(rotor_num);

  int j = 0;
  for(int i = 0; i < joint_indices.size(); i++)
    {
      if(joint_indices.at(i) == link_joint_indices.at(j))
        {
          gimbal_jacobian_(6 + i, 6 + j) = 1;
          j++;
        }
      if(j == link_joint_indices.size()) break;
    }

}


void HydrusTiltedRobotModel::calcStaticThrust()
{
  calcWrenchMatrixOnRoot(); // update Q matrix

  /* calculate the static thrust on CoG frame */
  /* note: can not calculate in root frame, sine the projected f_x, f_y is different in CoG and root */
  Eigen::MatrixXd wrench_mat_on_cog = calcWrenchMatrixOnCoG();

  Eigen::VectorXd static_thrust = aerial_robot_model::pseudoinverse(wrench_mat_on_cog.middleRows(2, 4)) * getGravity().segment(2,4) * getMass();
  setStaticThrust(static_thrust);

 

}

Eigen::MatrixXd HydrusTiltedRobotModel::getJacobian(const KDL::JntArray& joint_positions, std::string segment_name, KDL::Vector offset)
{
  Eigen::MatrixXd jacobian = aerial_robot_model::transformable::RobotModel::getJacobian(joint_positions, segment_name, offset);
  return jacobian * gimbal_jacobian_;
}


void HydrusTiltedRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);

  if(getStaticThrust().minCoeff() < 0)
    {
      setCogDesireOrientation(0, 0, 0);
      return; // invalid robot state
    }
  /* special process to find the hovering axis for tilt model */
  Eigen::MatrixXd cog_rot_inv = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>().Inverse());
  Eigen::MatrixXd wrench_mat_on_cog = calcWrenchMatrixOnCoG();
  Eigen::VectorXd f = cog_rot_inv * wrench_mat_on_cog.topRows(3) * getStaticThrust();

  double f_norm_roll = atan2(f(1), f(2));
  double f_norm_pitch = atan2(-f(0), sqrt(f(1)*f(1) + f(2)*f(2)));
  /* set the hoverable frame as CoG and reupdate model */
  setCogDesireOrientation(f_norm_roll, f_norm_pitch, 0);
  HydrusRobotModel::updateRobotModelImpl(joint_positions);

  if(getVerbose())
  {
    ROS_INFO_STREAM("f_norm_pitch: " << f_norm_pitch << "; f_norm_roll: " << f_norm_roll);
    ROS_INFO_STREAM("rescaled static thrust: " << getStaticThrust().transpose());
  }
}

void HydrusTiltedRobotModel::calcCoGMomentumJacobian()
{
  setCOGJacobian(Eigen::MatrixXd::Zero(3, 6 + getJointNum()));
  setLMomentumJacobian(Eigen::MatrixXd::Zero(3, 6 + getJointNum()));

  aerial_robot_model::transformable::RobotModel::calcCoGMomentumJacobian();
}

void HydrusTiltedRobotModel::updateJacobians(const KDL::JntArray& joint_positions, bool update_model) //TODO Check which functions are necessary
{

  if(update_model) updateRobotModel(joint_positions);

  calcCoGMomentumJacobian(); // should be processed first!

  calcBasicKinematicsJacobian(); // need cog_jacobian_

  calcLambdaJacobian();

  calcJointTorque(false);

  calcJointTorqueJacobian();

  // // external wrench realted jacobians
  // calcExternalWrenchCompThrust();
  // calcCompThrustJacobian();
  // addCompThrustToStaticThrust();
  // addCompThrustToLambdaJacobian();
  // addCompThrustToJointTorque();
  // addCompThrustToJointTorqueJacobian();

  calcFeasibleControlRollPitchDistsJacobian();

  // // convert to jacobian only for link joint
  std::vector<Eigen::MatrixXd> u_jacobians = getUJacobians();
  for(auto& jacobian: u_jacobians)
    {
      jacobian = jacobian * gimbal_jacobian_;
    }
  setUJacobians(u_jacobians);

  std::vector<Eigen::MatrixXd> p_jacobians = getPJacobians();
  for(auto& jacobian: p_jacobians) jacobian = jacobian * gimbal_jacobian_;
  setPJacobians(p_jacobians);

  std::vector<Eigen::MatrixXd> thrust_coord_jacobians = getThrustCoordJacobians();
  for(auto& jacobian: thrust_coord_jacobians) jacobian = jacobian * gimbal_jacobian_;
  setThrustTCoordJacobians(thrust_coord_jacobians);

  std::vector<Eigen::MatrixXd> cog_coord_jacobians = getCOGCoordJacobians();
  for(auto& jacobian: cog_coord_jacobians) jacobian = jacobian * gimbal_jacobian_;
  setCOGCoordJacobians(cog_coord_jacobians);

  Eigen::MatrixXd cog_jacobian = getCOGJacobian();
  setCOGJacobian(cog_jacobian * gimbal_jacobian_);
  Eigen::MatrixXd l_momentum_jacobian = getLMomentumJacobian();
  setLMomentumJacobian(l_momentum_jacobian * gimbal_jacobian_);

  Eigen::MatrixXd lambda_jacobian = getLambdaJacobian();
  setLambdaJacobian(lambda_jacobian * gimbal_jacobian_);
  Eigen::MatrixXd joint_torque_jacobian = getJointTorqueJacobian();
  setJointTorqueJacobian(joint_torque_jacobian * gimbal_jacobian_);

  // //Eigen::MatrixXd comp_thrust_jacobian = getCompThrustJacobian();
  // //setCompThrustJacobian(comp_thrust_jacobian * gimbal_jacobian_);

  Eigen::MatrixXd fc_rp_dists_jacobian = getFeasibleControlRollPitchDistsJacobian();
  setFeasibleControlRollPitchDistsJacobian(fc_rp_dists_jacobian * gimbal_jacobian_);

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(HydrusTiltedRobotModel, aerial_robot_model::RobotModel);
