// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/control/gimbalrotor_perching_admittance_controller.h>
#include <gimbalrotor/gimbalrotor_multilink_perching_navigation.h>
#include <gimbalrotor/model/gimbalrotor_multilink_perching_model.h>
#include <gimbalrotor/control/gimbalrotor_multilink_perching_observer.h>
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
  void commandWrenchCallback(const ros::MessageEvent<geometry_msgs::WrenchStamped const>&);
  struct CommandSample
  {
    double stamp = 0, receipt = 0;
    Eigen::Matrix<double, 6, 1> wrench = Eigen::Matrix<double, 6, 1>::Zero();
  };
  ros::Subscriber command_wrench_sub_;
  std::mutex command_mutex_;
  std::deque<CommandSample> command_history_;
  perching_dynamics::MultilinkModel dynamics_;
  perching_dynamics::ObservationFilter observation_filter_;
  perching_dynamics::MomentumObserver observer_;
  ros::Time observer_lock_stamp_;
  double observation_stamp_ = 0, published_observer_stamp_ = 0;
  double observer_gain_ = 10, feedback_timeout_ = .15;
  boost::shared_ptr<GimbalrotorMultilinkPerchingModel> articulated_model_;
  aerial_robot_navigation::GimbalrotorMultilinkPerchingNavigator* articulated_navigator_ = nullptr;
};
}
