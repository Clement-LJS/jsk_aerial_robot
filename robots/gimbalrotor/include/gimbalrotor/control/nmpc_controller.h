#pragma once

#include <aerial_robot_control/control/base/base.h>
#include <gimbalrotor/control/nmpc_solver.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <spinal/ServoStates.h>
#include <spinal/Imu.h>
#include <spinal/FourAxisCommand.h>
#include <std_srvs/SetBool.h>
#include <atomic>
#include <memory>
#include <mutex>

namespace gimbalrotor_nmpc
{
// BaseNavigator normally runs BEFORE the controller and can arm from START.
// This opt-in navigator gates that path before calling the existing state machine.
// Keep the CoG axes fixed to the airframe; pose targets use FlightNav roll/pitch.
class Navigator : public aerial_robot_navigation::BaseNavigator
{
public:
  void setReady(bool ready) { ready_ = ready; }
  void update() override;
private:
  std::atomic<bool> ready_{false};
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;
};

class Controller : public aerial_robot_control::ControlBase
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                  double loop_duration) override;
  bool update() override;
private:
  boost::shared_ptr<Navigator> navigation_;
  std::unique_ptr<Solver> solver_;
  ros::Publisher thrust_pub_, servo_pub_;
  ros::Subscriber odom_sub_, servo_sub_, mocap_sub_, imu_sub_, ack_sub_;
  ros::ServiceClient mode_client_;
  std::mutex feedback_mutex_;
  nav_msgs::Odometry odom_;
  ros::Time pose_stamp_;
  ros::Time imu_stamp_;
  ros::WallTime imu_receipt_;
  ros::WallTime odom_receipt_, pose_receipt_, ack_receipt_;
  std::array<ros::Time, 4> servo_stamp_{};
  std::array<ros::WallTime, 4> servo_receipt_{};
  std::array<double, 4> alpha_{}, scale_{}, offset_{}, sign_{}, tau_{};
  std::array<int, 4> servo_index_{};
  std::array<double, 4> last_reference_alpha_{};
  std::array<double, 4> previous_quaternion_{{1, 0, 0, 0}};
  std::array<double, 4> reference_angle_rate_{};
  bool have_reference_ = false, mode_ready_ = false, fault_ = false;
  int ack_ = -1;
  ros::WallTime mode_request_, last_update_;
  Bounds bounds_;
  double state_timeout_ = 0, servo_timeout_ = 0, solve_budget_ = 0;
  std::string cog_frame_;
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void servoCallback(const spinal::ServoStatesConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void imuCallback(const spinal::ImuConstPtr& msg);
  void ackCallback(const std_msgs::UInt8ConstPtr& msg);
  bool measuredState(State& x);
  Parameters physicalParameters();
  std::vector<Reference> references(const State& x, const Parameters& p);
  void stop(const std::string& reason);
  void sendStop();
};
}
