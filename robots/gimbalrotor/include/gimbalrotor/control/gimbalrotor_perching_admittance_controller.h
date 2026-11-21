// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/control/gimbalrotor_admittance_controller.h>
#include <gimbalrotor/control/gimbalrotor_multilink_perching_observer.h>
#include <gimbalrotor/gimbalrotor_perching_navigation.h>
#include <gimbalrotor/model/gimbalrotor_perching_model.h>
#include <std_msgs/Empty.h>
#include <deque>
#include <tf/transform_broadcaster.h>

namespace aerial_robot_control
{
class GimbalrotorPerchingAdmittanceController : public GimbalrotorAdmittanceController
{
public:
  void initialize(ros::NodeHandle, ros::NodeHandle, boost::shared_ptr<aerial_robot_model::RobotModel>,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator>, double) override;
  void reset() override;
protected:
  void controlCore() override;
  void preparePerchingReference() override;
  void applyAdmittanceOutputToNavigator(const tf::Vector3&, const tf::Vector3&, const AdmittanceCoreOutput&) override;
  const char* controllerName() const override { return "GimbalrotorPerchingAdmittanceController"; }
  void preparePerchingAdmittanceInput();
  void updateAdmittance();
  virtual bool externalLoad(Eigen::VectorXd&, Eigen::VectorXd&, double&);
  boost::shared_ptr<GimbalrotorPerchingModel> perching_model_;
  void updateContactGate(double residual_pitch_torque, double dt);
  void perchingAdmittanceEnableCallback(const std_msgs::BoolConstPtr&);
  void resetCalibration();
  void publishContactAdmittanceDiagnostics();
  void updatePitchIntegralPolicy(PID&, bool);
  aerial_robot_navigation::GimbalrotorPerchingNavigator* perching_navigator_ = nullptr;
  mutable std::recursive_mutex perching_state_mutex_;
  ros::Subscriber perching_admittance_enable_sub_, tare_sub_, normal_enable_sub_;
  ros::Publisher contact_active_pub_, recovery_active_pub_, pivot_torque_raw_pub_, pivot_torque_filtered_pub_, pitch_offset_pub_;
  ros::Publisher generalized_load_pub_, generalized_valid_pub_, pivot_wrench_pub_;
  perching_geometry::Session session_{perching_geometry::Mode::DISABLED, ros::Time(0)};
  bool perching_active_ = false, normal_admittance_enabled_ = false, perching_admittance_enabled_ = false;
  bool reset_requested_ = true, contact_active_ = false, recovery_active_ = false;
  bool equilibrium_wrench_ready_ = false, input_valid_ = false, output_accepted_ = false;
  bool pitch_i_limit_suppressed_ = false;
  double normal_pitch_i_limit_ = 0;
  int equilibrium_wrench_required_samples_ = 80, equilibrium_wrench_sample_count_ = 0;
  Eigen::VectorXd baseline_, baseline_sum_;
  double pitch_torque_lpf_ = 0;
  double last_load_stamp_ = 0, last_tare_stamp_ = 0;
  double contact_on_timer_ = 0, contact_off_timer_ = 0;
  double contact_torque_filter_alpha_ = .2, contact_on_threshold_ = .04, contact_off_threshold_ = .02;
  double contact_on_duration_ = .1, contact_off_duration_ = .2;
  double recovery_angle_epsilon_ = .00174533, recovery_rate_epsilon_ = .01;
  double residual_pitch_torque_raw_ = 0, residual_pitch_torque_filtered_ = 0, perching_pitch_torque_sign_ = 1;
  double pitch_acceleration_limit_ = .1, observer_gain_ = 10, observer_max_interval_ = .15, synchronization_tolerance_ = .05;
  ros::Time previous_contact_gate_time_;
  Eigen::Matrix<double, 6, 1> prepared_perching_admittance_wrench_world_ = Eigen::Matrix<double, 6, 1>::Zero();
  tf::TransformBroadcaster pivot_tf_;
  std::string pivot_frame_;
  std::string reason_ = "disarmed", world_ = "world";
  std::uint64_t tare_generation_ = 0, tare_request_sequence_ = 0, enable_request_sequence_ = 0;
};
}
