// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/gimbalrotor_perching_navigation.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>

#include <kdl/frames.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

namespace aerial_robot_navigation
{

class GimbalrotorMultilinkPerchingNavigator
  : public GimbalrotorPerchingNavigator
{
public:
  GimbalrotorMultilinkPerchingNavigator();
  ~GimbalrotorMultilinkPerchingNavigator() override = default;

  void initialize(
      ros::NodeHandle nh,
      ros::NodeHandle nhp,
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
      double loop_du) override;

  void update() override;

  bool multilinkModelValid() const;
  bool multilinkLockValid() const;
  bool secondaryJointSettled() const;
  std::uint64_t mechanismTargetGeneration() const;

protected:
  bool supportsPerchingMode(perching_geometry::Mode mode) const override
  {
    return mode == perching_geometry::Mode::NORMAL;
  }

  bool tryLockPerching(const std::string& reason) override;
  void resetPerchingLock() override;
  void applyActivePerchingTarget() override;
  bool applyPerchingConstraint(
      aerial_robot_msgs::FlightNav& nav_msg) override;
  void applyManualPitchDelta(double delta) override;

private:
  enum class PitchRezeroState { IDLE = 0, ACTIVE, HOLD };

  void pitchRezeroCallback(const std_msgs::EmptyConstPtr& msg);
  void resetPitchRezeroState();
  void failPitchRezero(const char* reason);
  bool verifyPitchRezeroFrozenJoints();
  void updateBodyPitchFeedback();
  void updatePitchRezeroCommand();
  void applyPitchRezeroTarget();
  bool readBaselinkPose(KDL::Frame& pose) const;
  double currentBaselinkPitch() const;
  static double wrapAngle(double angle);
  bool commandDesiredBaselinkPose(const KDL::Frame& T_W_B_des,
                                 const KDL::Vector& p_B_G);

  // This hinge is an external contact DOF, never an additional model joint.
  PitchRezeroState pitch_rezero_state_ = PitchRezeroState::IDLE;
  bool pitch_rezero_ready_ = false;
  bool pitch_rezero_failed_ = false;
  bool pitch_rezero_config_valid_ = false;
  bool rezero_target_applied_this_update_ = false;
  std::string pitch_rezero_topic_;
  KDL::Vector pitch_rezero_pivot_offset_;
  KDL::Vector pitch_rezero_axis_;
  double pitch_rezero_target_pitch_ = 0.0;
  double pitch_rezero_kp_ = 1.0;
  double pitch_rezero_rate_limit_ = 0.1745329;
  double pitch_rezero_max_delta_ = 0.5235988;
  double pitch_rezero_command_sign_ = 1.0;
  double pitch_rezero_pitch_tolerance_ = 0.0349066;
  double pitch_rezero_rate_tolerance_ = 0.05;
  double pitch_rezero_stable_duration_ = 0.30;
  double pitch_rezero_timeout_ = 5.0;
  double pitch_rezero_pitch_joint_hold_tolerance_ = 0.02;
  double pitch_rezero_secondary_joint_hold_tolerance_ = 0.02;

  double rezero_frozen_pitch_joint_ = 0.0;
  double rezero_frozen_secondary_joint_ = 0.0;
  double rezero_pitch_joint_hold_error_ = 0.0;
  double rezero_secondary_joint_hold_error_ = 0.0;
  KDL::Vector rezero_pivot_world_;
  KDL::Vector rezero_axis_world_;
  KDL::Vector rezero_p_B_G_;
  KDL::Frame rezero_reference_body_world_;
  KDL::Frame rezero_last_body_target_;
  double rezero_alpha_cmd_ = 0.0;
  ros::Time rezero_start_stamp_;
  ros::Time rezero_stable_start_stamp_;
  double rezero_previous_body_pitch_ = 0.0;
  ros::Time rezero_previous_body_pitch_stamp_;
  double rezero_measured_pitch_rate_ = 0.0;
  double rezero_feedback_dt_ = 0.0;
  bool rezero_pitch_rate_valid_ = false;

  ros::Subscriber pitch_rezero_sub_;
  ros::Publisher pitch_rezero_active_pub_;
  ros::Publisher pitch_rezero_ready_pub_;
  ros::Publisher pitch_rezero_failed_pub_;
  ros::Publisher pitch_joint_hold_error_pub_;
  ros::Publisher secondary_joint_hold_error_pub_;
  ros::Publisher passive_pitch_delta_pub_;
  ros::Publisher body_pitch_pub_;
  ros::Publisher body_pitch_rate_pub_;

  enum SecondaryAxisType
  {
    SECONDARY_AXIS_INVALID = 0,
    SECONDARY_AXIS_ROLL,
    SECONDARY_AXIS_YAW
  };

  void multilinkRosParamInit();
  bool resolveMechanismModel();
  bool resolveJointLimits(
      const std::string& joint_name,
      bool configured_limits_valid,
      double configured_lower,
      double configured_upper,
      double& lower,
      double& upper) const;

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);
  void secondaryJointTargetCallback(const std_msgs::Float64ConstPtr& msg);

  bool readCurrentMechanismState(
      double& pitch_position,
      double& secondary_position,
      double& pitch_velocity,
      double& secondary_velocity) const;

  bool computeBaselinkToContactTransform(
      double pitch_position,
      double secondary_position,
      KDL::Frame& T_B_C) const;

  bool computeBaselinkToCogVector(
      double pitch_position,
      double secondary_position,
      KDL::Vector& p_B_G) const;

  bool buildFinalJointTarget(
      double& pitch_final,
      double& secondary_final) const;

  void updateSecondarySettledState();
  void publishJointTarget(
      double pitch_final,
      double secondary_final);
  void publishDiagnostics() const;

  static bool frameFinite(const KDL::Frame& frame);
  static double clamp(double value, double lower, double upper);

  ros::Subscriber joint_state_sub_;
  ros::Subscriber secondary_joint_target_sub_;

  ros::Publisher joint_control_pub_;
  ros::Publisher model_valid_pub_;
  ros::Publisher secondary_settled_pub_;
  ros::Publisher pitch_measured_pub_;
  ros::Publisher pitch_nominal_pub_;
  ros::Publisher pitch_final_pub_;
  ros::Publisher secondary_measured_pub_;
  ros::Publisher secondary_target_pub_;
  ros::Publisher target_body_pose_pub_;

  std::string pitch_joint_name_;
  std::string secondary_joint_name_;
  std::string contact_link_name_;
  std::string joint_command_topic_;
  std::string joint_state_topic_;
  std::string secondary_joint_target_topic_;

  double secondary_position_tolerance_;
  double secondary_velocity_tolerance_;
  double secondary_settle_duration_;

  double joint_state_timeout_;
  double joint_state_future_tolerance_;

  double pitch_axis_alignment_threshold_;
  double secondary_axis_alignment_threshold_;

  double pitch_command_sign_;
  double secondary_command_sign_;

  bool configured_pitch_limits_valid_;
  bool configured_secondary_limits_valid_;
  double configured_pitch_lower_;
  double configured_pitch_upper_;
  double configured_secondary_lower_;
  double configured_secondary_upper_;

  std::size_t pitch_joint_index_;
  std::size_t secondary_joint_index_;

  double pitch_joint_lower_;
  double pitch_joint_upper_;
  double secondary_joint_lower_;
  double secondary_joint_upper_;

  Eigen::Vector3d pitch_axis_local_;
  Eigen::Vector3d secondary_axis_local_;
  SecondaryAxisType secondary_axis_type_;

  bool model_resolution_attempted_;
  bool multilink_model_valid_;
  bool multilink_lock_valid_;
  KDL::Frame locked_contact_world_;

  double locked_pitch_joint_;
  double locked_secondary_joint_;
  double pitch_joint_nominal_target_;
  double secondary_joint_nominal_target_;
  double pitch_joint_final_target_;
  double secondary_joint_final_target_;

  bool joint_state_received_;
  ros::Time joint_state_measurement_stamp_;
  ros::Time joint_state_receive_stamp_;
  double measured_pitch_joint_;
  double measured_secondary_joint_;
  double measured_pitch_velocity_;
  double measured_secondary_velocity_;
  double previous_measured_pitch_joint_;
  double previous_measured_secondary_joint_;

  bool secondary_settled_;
  ros::Time secondary_settle_start_;

  std::uint64_t mechanism_target_generation_;
};

}  // namespace aerial_robot_navigation
