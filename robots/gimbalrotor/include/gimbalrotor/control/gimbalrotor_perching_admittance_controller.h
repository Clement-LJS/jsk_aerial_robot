// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/control/gimbalrotor_admittance_controller.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <mutex>
#include <cstdint>

namespace aerial_robot_control
{

class GimbalrotorPerchingAdmittanceController
  : public GimbalrotorAdmittanceController
{
public:
  GimbalrotorPerchingAdmittanceController();
  virtual ~GimbalrotorPerchingAdmittanceController() = default;

  virtual void initialize(
      ros::NodeHandle nh,
      ros::NodeHandle nhp,
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
      boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
      double ctrl_loop_rate) override;

  virtual void reset() override;

protected:
  virtual void controlCore() override;

  virtual const char* controllerName() const override
  {
    return "GimbalrotorPerchingAdmittanceController";
  }

  virtual void applyAdmittanceOutputToNavigator(
      const tf::Vector3& original_target_pos,
      const tf::Vector3& original_target_rpy,
      const AdmittanceCoreOutput& output) override;

  virtual Eigen::Matrix<double, 6, 1>
  getExternalWrenchWorld() const override;

private:
  void perchingRosParamInit();

  void normalAdmittanceEnableCallback(
      const std_msgs::Bool::ConstPtr& msg);

  void perchingAdmittanceEnableCallback(
      const std_msgs::Bool::ConstPtr& msg);

  void perchingEnableCallback(
      const std_msgs::Bool::ConstPtr& msg);

  void perchingPointCallback(
      const geometry_msgs::PointStamped::ConstPtr& msg);

  void branchPoseCallback(
      const geometry_msgs::PoseStamped::ConstPtr& msg);

  void lockedPoseCallback(
      const geometry_msgs::PoseStamped::ConstPtr& msg);

  void lockedPivotCallback(
      const geometry_msgs::PointStamped::ConstPtr& msg);

  void updateLockedConstraintFromLockedPoseAndPivot();

  void resetEquilibriumWrenchUnsafe();

  /*
   * The caller must already hold perching_state_mutex_.
   */
  void resetContactGateUnsafe();

  /*
   * Remove external admittance forcing while preserving a valid
   * nonzero admittance state so stiffness and damping can return
   * the pitch correction smoothly to zero.
   *
   * The caller must already hold perching_state_mutex_.
   */
  void enterZeroInputRecoveryUnsafe();

  void preparePerchingAdmittanceInput();

  void updateContactGate(
      double residual_pitch_torque,
      double dt);

  bool recoveryComplete() const;

  bool perchingPitchAdmittanceConfigValid() const;

  bool perchingPitchOutputFinite(
      const AdmittanceCoreOutput& output) const;

  double safeAdmittanceDt() const;

  void publishContactAdmittanceDiagnostics() const;

  tf::Vector3 computePerchingArcPositionFromPitch(
      double target_pitch,
      const tf::Vector3& original_target_pos) const;

  double clamp(
      double value,
      double min_value,
      double max_value) const;

  double normalizeAngle(
      double angle) const;

  double norm2D(
      double x,
      double z) const;

  void poseMsgToTfPosRpy(
      const geometry_msgs::PoseStamped& msg,
      tf::Vector3& pos,
      tf::Vector3& rpy) const;

  virtual Eigen::Matrix3d getComplianceToWorldRotation() const override;
  void publishPivotWrenchFrame(
      const tf::Vector3& pivot_world,
      const Eigen::Matrix3d& R_world_constraint,
      const ros::Time& stamp);

private:
  ros::Subscriber normal_admittance_enable_sub_;
  ros::Subscriber perching_admittance_enable_sub_;

  ros::Subscriber perching_enable_sub_for_constraint_;
  ros::Subscriber perching_point_sub_;
  ros::Subscriber branch_pose_sub_;
  ros::Subscriber locked_pose_sub_;
  ros::Subscriber locked_pivot_sub_;

  ros::Publisher contact_active_pub_;
  ros::Publisher recovery_active_pub_;
  ros::Publisher pivot_torque_raw_pub_;
  ros::Publisher pivot_torque_filtered_pub_;
  ros::Publisher pitch_offset_pub_;
  ros::Publisher pivot_external_wrench_est_pub_;

  std::string perching_enable_topic_for_constraint_;
  std::string perching_admittance_enable_topic_;
  std::string perching_point_topic_;
  std::string perching_branch_pose_topic_;
  std::string perching_locked_pose_topic_;
  std::string perching_locked_pivot_topic_;
  std::string perching_pivot_wrench_frame_id_;

  bool normal_admittance_enabled_;
  bool perching_admittance_enabled_;
  bool effective_admittance_enabled_;

  bool perching_enabled_for_constraint_;
  bool has_perching_point_;
  bool has_branch_pose_;
  bool has_locked_pose_msg_;
  bool has_locked_pivot_;
  bool has_locked_pose_;

  bool use_branch_pose_if_no_point_;
  bool require_perching_lock_;

  double min_valid_radius_;
  double max_pitch_delta_;
  double arc_pitch_sign_;

  double contact_torque_filter_alpha_;
  double contact_on_threshold_;
  double contact_off_threshold_;
  double contact_on_duration_;
  double contact_off_duration_;
  double recovery_angle_epsilon_;
  double recovery_rate_epsilon_;
  double perching_pitch_torque_sign_;

  tf::Vector3 perching_point_world_;
  tf::Vector3 branch_pos_world_;

  tf::Vector3 locked_robot_pos_world_;
  tf::Vector3 locked_robot_rpy_;
  tf::Vector3 locked_pivot_world_;
  tf::Vector3 locked_radius_vec_world_;

  double locked_radius_;
  double locked_x_side_;

  Eigen::Matrix3d R_world_constraint_;
  Eigen::Vector3d constraint_axis_world_;

  mutable std::mutex perching_state_mutex_;

  ros::Time locked_pose_stamp_;
  ros::Time locked_pivot_stamp_;

  ros::Time accepted_locked_pose_stamp_;
  ros::Time accepted_locked_pivot_stamp_;

  double maximum_lock_stamp_difference_;

  /*
  * Number of control cycles used to calculate the average no-contact wrench while the robot is perched.
  *
  * At a 40 Hz control rate: 80 samples = approximately 2 seconds.
  */
  int equilibrium_wrench_required_samples_;

  /*
  * getExternalWrenchWorld() is const because it overrides the
  * base controller function. These members are mutable because
  * tare collection is performed inside that function.
  */
  mutable Eigen::Matrix<double, 6, 1> equilibrium_wrench_pivot_world_;
  mutable Eigen::Matrix<double, 6, 1> equilibrium_wrench_pivot_sum_;

  mutable int equilibrium_wrench_sample_count_;
  mutable bool equilibrium_wrench_ready_;

  bool admittance_reset_requested_;

  bool contact_active_;
  bool recovery_active_;

  double contact_on_timer_;
  double contact_off_timer_;

  double residual_pitch_torque_raw_;
  double residual_pitch_torque_filtered_;

  ros::Time previous_contact_gate_time_;

  /*
   * These two values are prepared from the same locked-pivot
   * snapshot and are consumed together by the parent admittance
   * controller.
   */
  Eigen::Matrix<double, 6, 1> prepared_perching_admittance_wrench_world_;

  Eigen::Matrix3d prepared_R_world_constraint_;

  /*
   * Normal pitch integral-term output limit.
   * This stores controller/pitch/limit_i from the YAML file.
   * It is restored when perching mode is disabled.
   */
  double normal_pitch_i_limit_;

  /*
   * Indicates that pitch limit_i is currently forced to zero.
   */
  bool pitch_i_limit_suppressed_;
  tf::TransformBroadcaster pivot_wrench_tf_broadcaster_;

  std::uint64_t last_published_pivot_wrench_sequence_ = 0;
};

} // namespace aerial_robot_control
