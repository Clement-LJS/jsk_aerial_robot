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
  void reset() override;
protected:
  void controlCore() override;
  bool externalLoad(Eigen::VectorXd&, Eigen::VectorXd&, double&) override;
  bool feedbackWrench(Eigen::Matrix<double, 6, 1>&) override;
  const char* controllerName() const override { return "GimbalrotorMultilinkPerchingAdmittanceController"; }
private:
  void commandWrenchCallback(const geometry_msgs::WrenchStampedConstPtr&);
  boost::shared_ptr<GimbalrotorMultilinkPerchingModel> articulated_model_;
  aerial_robot_navigation::GimbalrotorMultilinkPerchingNavigator* articulated_navigator_ = nullptr;
  ros::Subscriber command_sub_;
  std::vector<PID> joint_pids_;
  perching_dynamics::MultilinkModel dynamics_;
  perching_dynamics::MomentumObserver observer_;
  perching_dynamics::ObservationFilter observation_filter_;
  bool dynamics_valid_ = false;
  double filtered_stamp_ = 0, observation_start_ = 0;
  Eigen::VectorXd observation_tangent_;
  struct Command
  {
    double stamp = 0, receipt = 0;
    Eigen::Matrix<double, 6, 1> wrench;
    Eigen::Vector3d offset;
  };
  std::deque<Command> commands_;
};
}
