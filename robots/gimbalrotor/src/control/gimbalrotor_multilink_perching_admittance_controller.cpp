#include <gimbalrotor/control/gimbalrotor_multilink_perching_admittance_controller.h>
#include <pluginlib/class_list_macros.h>
#include <stdexcept>

namespace aerial_robot_control
{
void GimbalrotorMultilinkPerchingAdmittanceController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double rate)
{
  GimbalrotorPerchingAdmittanceController::initialize(nh, nhp, model, estimator, navigator, rate);
  articulated_navigator_ = dynamic_cast<aerial_robot_navigation::GimbalrotorMultilinkPerchingNavigator*>(navigator.get());
  if(!articulated_navigator_) throw std::invalid_argument("articulated controller requires the multilink perching navigator");
  articulated_model_ = boost::dynamic_pointer_cast<GimbalrotorMultilinkPerchingModel>(model);
  if(!articulated_model_) throw std::invalid_argument("articulated controller requires the shared articulated model");
  ROS_INFO("Multilink contact admittance uses world/CoG external wrench projected by articulated virtual work.");
  ROS_WARN("Generalized momentum observer unavailable: no verified effective wrench and calibrated joint torque input.");
}

bool GimbalrotorMultilinkPerchingAdmittanceController::externalLoad(Eigen::VectorXd& load, Eigen::VectorXd& tangent, double& stamp)
{
  ExternalWrenchSnapshot wrench;
  aerial_robot_navigation::GimbalrotorPerchingNavigator::Sample sample;
  if(!synchronizedExternalWrench(wrench, sample)) return false;
  Eigen::VectorXd pitch_tangent;
  reason_ = "body-local pitch tangent unavailable";
  if(!articulated_navigator_->pitchTangent(sample.q, articulated_navigator_->movingCoordinates(), pitch_tangent) ||
      pitch_tangent.size() != sample.q.size() || !pitch_tangent.allFinite()) return false;

  GimbalrotorPerchingModel::State state, plus, minus;
  const double epsilon = 1e-5;
  reason_ = "invalid locked articulated geometry for CoG virtual work";
  if(!articulated_model_->evaluate(sample.q, state) ||
      !articulated_model_->evaluate(sample.q + epsilon * pitch_tangent, plus) ||
      !articulated_model_->evaluate(sample.q - epsilon * pitch_tangent, minus)) return false;
  const Eigen::Vector3d velocity = (plus.cog.translation() - minus.cog.translation()) / (2 * epsilon);
  const Eigen::Vector3d omega = perching_geometry::logSO3(plus.body.linear() * minus.body.linear().transpose()) / (2 * epsilon);
  // World wrench at the moving CoG: translational work already includes the
  // hinge moment arm. Do not also shift this moment to the pivot.
  const double torque = velocity.dot(wrench.raw_wrench.head<3>()) + omega.dot(wrench.raw_wrench.tail<3>());
  if(!std::isfinite(torque)) return false;
  load = Eigen::VectorXd::Constant(1, torque);
  tangent = Eigen::VectorXd::Ones(1);
  stamp = wrench.measurement_stamp.toSec();
  reason_ = "world/CoG wrench projected by articulated virtual work";
  return true;
}

}

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorMultilinkPerchingAdmittanceController,
                       aerial_robot_control::ControlBase)
