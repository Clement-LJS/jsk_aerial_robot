#pragma once
#include <gimbalrotor/gimbalrotor_perching_navigation.h>
#include <gimbalrotor/roll_yaw_cutting_trajectory.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
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
  void applyActivePerchingTarget() override;
  void applyManualPitchDelta(double) override;
  bool applyPerchingConstraint(aerial_robot_msgs::FlightNav&) override { return false; }
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr&) override;
  // Enable, disable, reset and relock are inherited, exactly as in multilink.
private:
  bool snapshotModel(roll_yaw::Kinematics&,std::string&);
  roll_yaw::Feedback feedback() const;
  std::string contactRejection(const roll_yaw::Feedback&) const;
  void jointCallback(const sensor_msgs::JointStateConstPtr&);
  void bodyCallback(const nav_msgs::OdometryConstPtr&);
  void mechanismTargetCallback(const std_msgs::Float64ConstPtr&,int axis);
  void manualCallback(const geometry_msgs::Vector3StampedConstPtr&);
  bool setNominalTarget(const roll_yaw::Vec&,int changed_axis=-1);
  bool buildFinalMechanismTarget(roll_yaw::Reference&);
  void commandDesiredBaselinkPose(const roll_yaw::Reference&);
  void publishJointTarget();
  void invalidatePlanner();
  void stopMotion(const std::string&);
  void faultHold(const std::string&);
  void request(const std::string&,std::uint64_t);
  void diagnostics();

  // Multilink lifecycle, adapted from pitch+secondary to passive alpha+roll+yaw.
  bool lock_valid_=false, faulted_=false;
  roll_yaw::Kinematics model_;
  roll_yaw::Vec nominal_=roll_yaw::Vec::Zero(); // independent accepted destinations
  roll_yaw::Reference final_target_;         // sole applied FK/controller reference
  roll_yaw::Estimate measured_contact_;
  std::string reason_="free flight", previous_failure_;
  ros::Time lock_stamp_, manual_receipt_stamp_;

  enum class PitchRezeroState { IDLE, ACTIVE, HOLD };
  void pitchRezeroCallback(const std_msgs::EmptyConstPtr&);
  void resetPitchRezeroState();
  void updateBodyPitchFeedback();
  void updatePitchRezeroCommand();
  bool verifyPitchRezeroFrozenJoints();
  void applyPitchRezeroTarget();
  void failPitchRezero(const std::string&);
  PitchRezeroState pitch_rezero_state_=PitchRezeroState::IDLE;
  bool pitch_rezero_ready_=false, pitch_rezero_failed_=false, rezero_configuration_valid_=false;
  bool target_applied_this_update_=false;
  double rezero_pitch_=0, rezero_kp_=1, rezero_sign_=-1, rezero_rate_=.1745329, rezero_travel_=.5235988;
  double rezero_tolerance_=.0349066, rezero_rate_tolerance_=.05;
  double rezero_stable_duration_=.3, rezero_timeout_=5;
  double rezero_roll_tolerance_=.02, rezero_yaw_tolerance_=.02;
  double rezero_started_=0, rezero_stable_since_=0, rezero_alpha_=0;
  double rezero_previous_pitch_=0, rezero_previous_time_=0, rezero_measured_rate_=0, rezero_feedback_dt_=0;
  bool rezero_rate_valid_=false;
  roll_yaw::Vec rezero_frozen_=roll_yaw::Vec::Zero();

  // Explicit QP requests are an optional target producer, never the manual lifecycle.
  bool qp_planning_=false, qp_configuration_valid_=false;
  roll_yaw::PlanningConfig planning_config_;
  roll_yaw::ExecutionConfig feedback_config_;
  std::unique_ptr<roll_yaw::Executor> planner_;
  std::future<roll_yaw::Plan> pending_;
  std::uint64_t producer_generation_=0, pending_generation_=0;
  roll_yaw::Mat neutral_=roll_yaw::Mat::Identity();

  roll_yaw::Feedback measured_;
  roll_yaw::Pose cp_=roll_yaw::Pose::Identity();
  roll_yaw::Vec axis_=roll_yaw::Vec::Zero();
  std::string roll_,yaw_,contact_,prefix_,world_,body_frame_;
  std::string joint_failure_="no joint feedback", body_failure_="no body feedback";
  std::uint64_t joint_ignored_stamps_=0, body_ignored_stamps_=0;
  bool configuration_valid_=false, joint_valid_=false, body_valid_=false;
  double synchronization_tolerance_=.05, cog_model_tolerance_=.005;
  std::vector<ros::Subscriber> subscriptions_;
  ros::Publisher joints_pub_,state_pub_,session_pub_,reference_pub_,measured_pub_,cog_pub_,details_pub_;
  ros::Publisher angles_pub_,nominal_pub_,rezero_active_pub_,rezero_ready_pub_,rezero_failed_pub_,pitch_pub_;
  ros::Publisher position_lower_pub_,position_upper_pub_;
};
}
