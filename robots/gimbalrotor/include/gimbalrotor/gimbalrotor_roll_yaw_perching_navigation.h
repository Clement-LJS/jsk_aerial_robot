#pragma once
#include <gimbalrotor/gimbalrotor_perching_navigation.h>
#include <gimbalrotor/roll_yaw_cutting_trajectory.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>
#include <future>

namespace aerial_robot_navigation
{
class GimbalrotorRollYawPerchingNavigator : public GimbalrotorPerchingNavigator
{
public:
  void initialize(ros::NodeHandle,ros::NodeHandle,boost::shared_ptr<aerial_robot_model::RobotModel>,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>,double) override;
  void update() override;
protected:
  bool supportsPerchingMode(perching_geometry::Mode m) const override { return m==perching_geometry::Mode::NORMAL; }
  bool ownsNavigationTargets() const override;
  std::unique_lock<std::recursive_mutex> lockNavigationTargets() override
  { return std::unique_lock<std::recursive_mutex>(perchingStateMutex()); }
  void navigationTargetConflict(const char*) override;
  bool tryLockPerching(const std::string&) override;
  void resetPerchingLock() override;
  void selectPerchingMode(perching_geometry::Mode,bool) override;
  void relockCallback(const std_msgs::EmptyConstPtr&) override;
  void resetCallback(const std_msgs::EmptyConstPtr&) override;
  void applyActivePerchingTarget() override;
  void applyManualPitchDelta(double) override;
  bool applyPerchingConstraint(aerial_robot_msgs::FlightNav&) override { return false; }
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr&) override;
private:
  bool snapshotModel(roll_yaw::Kinematics&,std::string&);
  roll_yaw::Feedback feedback() const;
  void jointCallback(const sensor_msgs::JointStateConstPtr&);
  void bodyCallback(const nav_msgs::OdometryConstPtr&);
  void request(const std::string&,std::uint64_t);
  void diagnostics();
  roll_yaw::Executor core_;
  roll_yaw::Feedback measured_;
  roll_yaw::Mat neutral_=roll_yaw::Mat::Identity();
  roll_yaw::Pose cp_=roll_yaw::Pose::Identity();
  roll_yaw::Vec axis_=roll_yaw::Vec::Zero();
  std::string roll_,yaw_,contact_,prefix_,world_,body_frame_;
  bool calibration_valid_=false, joint_valid_=false, body_valid_=false;
  bool output_inhibited_=false;
  double synchronization_tolerance_=.05, cog_model_tolerance_=.005;
  std::future<roll_yaw::Plan> pending_;
  ros::Time lock_stamp_;
  std::vector<ros::Subscriber> subscriptions_;
  ros::Publisher joints_pub_,state_pub_,session_pub_,reference_pub_,measured_pub_,cog_pub_,details_pub_;
};
}
