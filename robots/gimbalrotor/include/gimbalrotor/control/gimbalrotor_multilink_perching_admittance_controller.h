// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/control/gimbalrotor_perching_admittance_controller.h>
#include <gimbalrotor/gimbalrotor_multilink_perching_navigation.h>
#include <gimbalrotor/model/gimbalrotor_multilink_perching_model.h>
namespace aerial_robot_control
{
class GimbalrotorMultilinkPerchingAdmittanceController : public GimbalrotorPerchingAdmittanceController
{
public:
  void initialize(ros::NodeHandle, ros::NodeHandle, boost::shared_ptr<aerial_robot_model::RobotModel>,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator>, double) override;
protected:
  bool externalLoad(Eigen::VectorXd&, Eigen::VectorXd&, double&) override;
  const char* controllerName() const override { return "GimbalrotorMultilinkPerchingAdmittanceController"; }
private:
  boost::shared_ptr<GimbalrotorMultilinkPerchingModel> articulated_model_;
  aerial_robot_navigation::GimbalrotorMultilinkPerchingNavigator* articulated_navigator_ = nullptr;
};
}
