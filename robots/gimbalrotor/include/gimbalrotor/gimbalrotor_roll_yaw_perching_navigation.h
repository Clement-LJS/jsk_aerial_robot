#pragma once
#include <gimbalrotor/gimbalrotor_perching_navigation.h>
#include <gimbalrotor/roll_yaw_cutting_trajectory.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <gimbalrotor/roll_yaw_worker.h>
#include <limits>
#include <gimbalrotor/roll_yaw_dynamics.h>
#include <gimbalrotor/roll_yaw_admittance.h>

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
  void bodyAlignmentCallback(const std_msgs::Float64ConstPtr&,int axis);
  roll_yaw::Request alignment_request_;
  bool body_alignment_valid_=false;
  void mechanismTargetCallback(const std_msgs::Float64ConstPtr&,int axis);
  void manualCallback(const geometry_msgs::Vector3StampedConstPtr&);
  bool setNominalTarget(const roll_yaw::Vec&,int changed_axis=-1);
  bool buildFinalMechanismTarget(roll_yaw::Reference&);
  void commandDesiredBaselinkPose(const roll_yaw::Reference&);
  void publishJointTarget();
  void invalidatePlanner();
  void stopMotion(const std::string&);
  void faultHold(const std::string&);
  void request(const std::string&,std::uint64_t,int axis=-1,double value=0);
  void diagnostics();

  // Multilink lifecycle, adapted from pitch+secondary to passive alpha+roll+yaw.
  bool lock_valid_=false, faulted_=false;
  roll_yaw::Kinematics model_;
  std::shared_ptr<const roll_yaw::Kinematics> planning_model_;
  roll_yaw::Vec nominal_=roll_yaw::Vec::Zero(); // independent accepted destinations
  roll_yaw::Path manual_path_;
  double manual_time_=0, manual_phase_=0, manual_speed_=1, manual_wall_=0;
  bool stop_pending_=false;
  roll_yaw::SettlingWindow preparation_window_;
  roll_yaw::SettlingStatistics preparation_stats_;
  bool preparation_ready_=false;
  double preparation_clock_=0;
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

  // Offline orientation paths share the manual FK/controller output.
  bool planning_configuration_valid_=false;
  roll_yaw::PlanningConfig planning_config_;
  roll_yaw::ExecutionConfig feedback_config_;
  std::unique_ptr<roll_yaw::Executor> planner_;
  roll_yaw::Worker<roll_yaw::Plan> prepare_worker_;
  bool prepare_pending_=false;
  std::uint64_t prepare_cancellations_=0;
  std::string prepare_result_="not requested";
  double prepare_started_=0,prepare_deadline_=120,prepare_wall_=0;
  struct PlanningStatus {
    std::atomic<int> interval{0},total{0}; std::atomic<bool> validating{false};
    std::atomic<double> solve_seconds{0},validation_seconds{0};
  };
  std::shared_ptr<PlanningStatus> planning_status_=std::make_shared<PlanningStatus>();
  std::uint64_t producer_generation_=0, pending_generation_=0;
  roll_yaw::Mat neutral_=roll_yaw::Mat::Identity();

  struct DynamicsResult {
    roll_yaw::DynamicsTerms terms; std::string reason;
    std::uint64_t generation=0,session=0; double stamp=0,submitted=0,wall=0;
  };
  roll_yaw::Worker<DynamicsResult> dynamics_worker_;
  std::shared_ptr<roll_yaw::ArticulatedDynamics> dynamics_model_;
  roll_yaw::MomentumObserver observer_;
  roll_yaw::PitchAdmittance admittance_;
  bool dynamics_pending_=false,torque_valid_=false;
  bool last_command_accepted_=false;
  double last_dynamics_stamp_=0,observer_started_=0;
  double dynamics_wall_=0,last_dynamics_=0;
  double torque_stamp_=0,torque_receipt_=0,control_pitch_=0;
  std::uint64_t plan_id_=0;
  std::string load_reason_="no computable input",admittance_reason_="disarmed";
  roll_yaw::Vec servo_effort_=roll_yaw::Vec::Constant(std::numeric_limits<double>::quiet_NaN());
  double effort_stamp_=0,effort_receipt_=0,body_receipt_=0,joint_receipt_=0,wrench_receipt_=0;
  geometry_msgs::WrenchStamped input_wrench_;
  struct ArticulationSample { double position=0,rate=0,stamp=0; };
  std::map<std::string,ArticulationSample> articulation_samples_;
  void admittanceCallback(const std_msgs::BoolConstPtr&);
  void externalWrenchCallback(const geometry_msgs::WrenchStampedConstPtr&);
  void tareCallback(const std_msgs::EmptyConstPtr&);
  void updateInteraction(const roll_yaw::Feedback&);
  bool interaction_configured_=false,tare_requested_=false,interaction_valid_=false;
  std::string interaction_frame_, interaction_point_, interaction_topic_;
  double interaction_sign_=0,interaction_scale_=0,interaction_torque_=0,interaction_stamp_=0,interaction_receipt_=0;
  double compliance_time_=0;
  geometry_msgs::WrenchStamped external_wrench_;
  void wrenchCallback(const geometry_msgs::WrenchStampedConstPtr&);
  void updateDynamics(const roll_yaw::Feedback&);
  void resetCompliance();
  ros::Publisher status_pub_,task_pub_,compliance_pub_,ack_pub_;
  std::string last_status_;
  double last_diagnostics_=0,navigation_wall_=0;

  roll_yaw::Feedback measured_;
  roll_yaw::Pose cp_=roll_yaw::Pose::Identity();
  roll_yaw::Vec axis_=roll_yaw::Vec::Zero();
  std::string roll_,yaw_,contact_,prefix_,world_,body_frame_;
  std::string joint_failure_="no joint feedback", body_failure_="no body feedback";
  std::uint64_t joint_ignored_stamps_=0, body_ignored_stamps_=0,joint_invalid_messages_=0,body_invalid_messages_=0;
  double last_joint_message_stamp_=0,last_body_message_stamp_=0;
  bool configuration_valid_=false, joint_valid_=false, body_valid_=false;
  double synchronization_tolerance_=.05, cog_model_tolerance_=.005;
  std::vector<ros::Subscriber> subscriptions_;
  ros::Publisher joints_pub_,state_pub_,session_pub_,reference_pub_,measured_pub_,cog_pub_,details_pub_;
  ros::Publisher angles_pub_,nominal_pub_,rezero_active_pub_,rezero_ready_pub_,rezero_failed_pub_,pitch_pub_;
  ros::Publisher position_lower_pub_,position_upper_pub_;
};
}
