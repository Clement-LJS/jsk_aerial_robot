// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/control/gimbalrotor_admittance_controller.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include <mutex>

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

  /*
   * Hybrid PID-admittance supervisor states:
   *
   * HYBRID_DISABLED:
   *   Hybrid operation is unavailable because perching or the
   *   locked constraint is not valid.
   *
   * TARE_WAIT:
   *   Collect the no-contact equilibrium pivot wrench.
   *
   * ARMED_PID:
   *   Hybrid operation is armed, but ordinary pose PID remains active
   *   until persistent cutting contact is detected.
   *
   * CONTACT_CANDIDATE:
   *   The contact threshold has been crossed but has not yet persisted
   *   for contact_on_duration.
   *
   * COMPLIANT_CUTTING:
   *   Directional admittance modifies the perching pitch target.
   *
   * RECOVERY:
   *   External admittance forcing is removed and the compliant offset
   *   decays smoothly toward zero.
   *
   * FAULT:
   *   Safety fault is latched and the saw-abort request is published.
   */
  enum HybridState
  {
    HYBRID_DISABLED = 0,
    TARE_WAIT = 1,
    ARMED_PID = 2,
    CONTACT_CANDIDATE = 3,
    COMPLIANT_CUTTING = 4,
    RECOVERY = 5,
    FAULT = 6
  };

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
  /*
   * Fresh no-contact pivot-torque statistics collected with Welford's
   * online algorithm, so mean/variance can be updated one sample at a
   * time without storing the whole sample history.
   */
  struct WelfordStats
  {
    long count = 0;
    double mean = 0.0;
    double m2 = 0.0;

    void reset()
    {
      count = 0;
      mean = 0.0;
      m2 = 0.0;
    }

    void update(double x)
    {
      ++count;
      const double delta = x - mean;
      mean += delta / static_cast<double>(count);
      const double delta2 = x - mean;
      m2 += delta * delta2;
    }

    double variance() const
    {
      if(count < 2)
      {
        return 0.0;
      }

      return m2 / static_cast<double>(count - 1);
    }

    double stddev() const
    {
      return std::sqrt(std::max(0.0, variance()));
    }
  };

  /*
   * Single-pass result of the pivot wrench update, computed once per
   * control cycle by updatePivotWrenchAndTare() and consumed by the
   * hybrid supervisor, the contact filters, and the diagnostics
   * publisher. Keeping this in one struct is what section 12 of the
   * spec calls the "recommended internal refactor": it avoids
   * duplicating the pivot shift/tare bookkeeping across a const getter
   * and separate stateful logic.
   */
  struct PivotWrenchResult
  {
    bool fresh = false;
    bool lock_valid = false;
    bool tare_ready = false;
    bool finite = false;
    bool cutting_gate_valid = false;
    bool dt_valid = false;
    bool control_gap_fault = false;
    bool cutting_signal_fresh = false;
    bool armed = false;
    bool disarm_requested = false;
    bool snapshot_generation_valid = false;

    uint64_t lock_generation = 0;

    Eigen::Matrix<double, 6, 1> wrench_cog_world =
        Eigen::Matrix<double, 6, 1>::Zero();

    Eigen::Matrix<double, 6, 1> wrench_pivot_world =
        Eigen::Matrix<double, 6, 1>::Zero();

    Eigen::Matrix<double, 6, 1> residual_pivot_world =
        Eigen::Matrix<double, 6, 1>::Zero();

    /*
     * Pivot torque projected onto the constraint axis before tare
     * subtraction.
     */
    double raw_pivot_torque_axis = 0.0;

    /*
     * Pivot torque projected onto the constraint axis after tare
     * subtraction.
     */
    double residual_pivot_torque_axis = 0.0;

    double tare_torque_stddev = 0.0;
    double dt = 0.0;
  };

  struct PerchingSnapshot
  {
    bool perching_active = false;
    bool lock_valid = false;
    bool arm_command = false;
    bool disarm_requested = false;
    bool cutting_active = false;
    bool cutting_signal_fresh = false;

    Eigen::Vector3d pivot_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d locked_robot_pos_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d locked_robot_rpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d locked_radius_vec_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d constraint_axis_world = Eigen::Vector3d::UnitY();
    Eigen::Matrix3d R_world_constraint = Eigen::Matrix3d::Identity();

    double locked_radius = 0.0;
    double locked_x_side = 1.0;
    uint64_t lock_generation = 0;
  };

  enum HybridHardResetReason
  {
    HARD_RESET_NONE = 0,
    HARD_RESET_CONTROLLER_RESET = 1,
    HARD_RESET_PERCHING_DISABLED = 2,
    HARD_RESET_NEW_LOCK = 3,
    HARD_RESET_INVALID_LOCK = 4,
    HARD_RESET_INVALID_GEOMETRY = 5
  };

  enum HybridFaultCode
  {
    HYBRID_FAULT_NONE = 0,
    HYBRID_FAULT_STALE_WRENCH = 1,
    HYBRID_FAULT_HARD_TORQUE = 2,
    HYBRID_FAULT_LOCK_LOST = 3,
    HYBRID_FAULT_INVALID_GEOMETRY = 4,
    HYBRID_FAULT_NONFINITE_STATE = 5,
    HYBRID_FAULT_CONTROL_GAP = 6,
    HYBRID_FAULT_CUTTING_SIGNAL_TIMEOUT = 7
  };

  enum class AdmittanceOperatingMode
  {
    NONE = 0,
    NORMAL_FLIGHT,
    LEGACY_PERCHING,
    HYBRID_PERCHING
  };

  void perchingRosParamInit();
  void hybridRosParamInit();

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

  void cuttingActiveCallback(
      const std_msgs::Bool::ConstPtr& msg);

  void updateLockedConstraintFromLockedPoseAndPivot();
  PerchingSnapshot makePerchingSnapshot();
  void consumePendingCommandsUnsafe(PerchingSnapshot& snapshot);
  bool validateHybridParameters();
  void requestHardResetUnsafe(HybridHardResetReason reason);
  void resetHybridTare();
  void enterRecoveryFromDisarm();
  void processPendingGlobalTransitions(const PerchingSnapshot& snapshot);
  void restorePitchPidForNormalFlight();
  void performHybridHardReset(HybridHardResetReason reason);
  void switchAdmittanceMode(AdmittanceOperatingMode requested_mode);
  void runNormalFlightAdmittance(const PerchingSnapshot& snapshot);
  void runLegacyPerchingAdmittance(const PerchingSnapshot& snapshot);
  void runHybridPerchingAdmittance(const PerchingSnapshot& snapshot);
  bool runInnerLoopWithAdmittanceOverride(
      const PerchingSnapshot* snapshot,
      const AdmittanceCoreOutput* output_override);  
  void handleInvalidHybridTiming(
      const PivotWrenchResult& pivot,
      double dt);

  void resetEquilibriumWrenchUnsafe() const;

  tf::Vector3 computePerchingArcPositionFromPitch(
      const PerchingSnapshot& snapshot,
      double target_pitch,
      const tf::Vector3& original_target_pos) const;

  bool applyHybridAdmittanceOutputToNavigator(
      const PerchingSnapshot& snapshot,
      const tf::Vector3& original_target_pos,
      const tf::Vector3& original_target_rpy,
      const AdmittanceCoreOutput& output);

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

  bool isFiniteTfVector(
      const tf::Vector3& vector) const;

  bool isValidRotationMatrix(
      const Eigen::Matrix3d& rotation) const;

  bool validateLockedPoseMessage(
      const geometry_msgs::PoseStamped& msg) const;

  bool validateLockedPivotMessage(
      const geometry_msgs::PointStamped& msg) const;

  bool validateSnapshotGeometry(
      const PerchingSnapshot& snapshot) const;

  bool faultAllowsHeldTarget() const;

  virtual Eigen::Matrix3d getComplianceToWorldRotation() const override;

  /*
   * legacy* keeps the pre-hybrid behavior byte-for-byte reachable when
   * controller/admittance/perching/hybrid/enabled is false, for direct
   * regression comparison (spec Test H).
   */
  void legacyControlCore();
  Eigen::Matrix<double, 6, 1> legacyGetExternalWrenchWorld() const;

  void hybridControlCore(const PerchingSnapshot& snapshot);

  Eigen::Matrix<double, 6, 1> shiftWrenchToPivot(
      const Eigen::Matrix<double, 6, 1>& wrench_cog_world,
      const Eigen::Vector3d& cog_pos_world,
      const Eigen::Vector3d& pivot_pos_world) const;

  double projectTorqueOntoAxis(
      const Eigen::Matrix<double, 6, 1>& wrench_pivot_world,
      const Eigen::Matrix3d& R_world_constraint) const;

  PivotWrenchResult updatePivotWrenchAndTare(
      const PerchingSnapshot& snapshot,
      const ros::Time& now,
      double dt);

  /* Caller must hold perching_state_mutex_. */
  void resetHybridTareUnsafe();

  /* Control-thread-only fields; no lock required. */
  void resetHybridDynamicsUnsafe();

  void updateTorqueFilters(
      double raw_tau_q,
      double dt);

  void updateHybridStateMachine(
      const PivotWrenchResult& pivot,
      double dt);

  void updateAuthority(
      double dt);

  double contactOnThreshold() const;
  double contactOffThreshold() const;
  double directionalDeadband() const;

  double computeDirectionalTorque() const;

  void updatePitchIntegratorMode();

  void enterFault(
      HybridFaultCode code,
      const std::string& reason);

  HybridState decideRestState(
      bool armed,
      bool tare_ready) const;

  void publishHybridDiagnostics(
      const PivotWrenchResult& pivot,
      double tau_directional,
      double tau_input,
      const AdmittanceCoreOutput* applied_output,
      bool fault_target_actually_held);

private:
  ros::Subscriber normal_admittance_enable_sub_;
  ros::Subscriber perching_admittance_enable_sub_;

  ros::Subscriber perching_enable_sub_for_constraint_;
  ros::Subscriber perching_point_sub_;
  ros::Subscriber branch_pose_sub_;
  ros::Subscriber locked_pose_sub_;
  ros::Subscriber locked_pivot_sub_;

  std::string perching_enable_topic_for_constraint_;
  std::string perching_admittance_enable_topic_;
  std::string perching_point_topic_;
  std::string perching_branch_pose_topic_;
  std::string perching_locked_pose_topic_;
  std::string perching_locked_pivot_topic_;

  bool normal_admittance_enabled_;
  bool perching_admittance_enabled_;
  bool effective_admittance_enabled_;
  bool normal_admittance_reset_requested_;
  bool legacy_admittance_reset_requested_;

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
  uint64_t lock_generation_;

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
  *
  * These are only used by the legacy (hybrid.enabled = false) path.
  */
  mutable Eigen::Matrix<double, 6, 1> equilibrium_wrench_pivot_world_;

  mutable Eigen::Matrix<double, 6, 1> equilibrium_wrench_sum_;

  mutable int equilibrium_wrench_sample_count_;
  mutable bool equilibrium_wrench_ready_;

  /*
   * Normal pitch integral-term output limit.
   *This stores controller/pitch/limit_i from the YAML file.
   * It is restored when perching mode is disabled.
   *
   * Only used by the legacy (hybrid.enabled = false) path. The hybrid
   * path uses PID::setIntegratorFrozen() instead (section 11).
   */
  double normal_pitch_i_limit_;

  /*
   * Indicates that pitch limit_i is currently forced to zero.
   */
  bool pitch_i_limit_suppressed_;

  /* ================= Hybrid contact-gated admittance ================= */

  /*
   * controller/admittance/perching/hybrid/enabled.
   *
   * false: byte-for-byte legacy behavior (problems 1-4 unfixed, for
   *        regression comparison only).
   * true:  hybrid state machine described in the spec.
   */
  bool hybrid_enabled_cfg_;
  AdmittanceCoreConfig normal_admittance_config_;
  AdmittanceCoreConfig legacy_perching_admittance_config_;
  AdmittanceCoreConfig hybrid_perching_admittance_config_;
  AdmittanceOperatingMode active_admittance_mode_;

  ros::Subscriber cutting_active_sub_;
  std::string cutting_active_topic_;
  bool require_cutting_active_;
  bool cutting_active_;
  ros::Time last_cutting_active_receive_time_;
  bool has_cutting_active_message_;
  double cutting_active_timeout_;

  double contact_filter_time_constant_;
  double admittance_filter_time_constant_;

  double contact_on_sigma_multiplier_;
  double contact_off_sigma_multiplier_;
  double contact_on_min_torque_;
  double contact_off_min_torque_;
  double contact_on_duration_;
  double contact_off_duration_;

  double authority_ramp_up_time_;

  double relief_pitch_sign_;
  double penetration_torque_ratio_;
  double directional_deadband_sigma_multiplier_;
  double directional_deadband_min_torque_;

  double recovery_angle_epsilon_;
  double recovery_rate_epsilon_;

  double hard_residual_pivot_torque_limit_;
  double max_hybrid_control_dt_;
  double recontact_on_duration_;
  double max_tare_torque_stddev_;
  bool hold_relief_target_on_measurement_fault_;
  std::string abort_request_topic_;

  bool hybrid_arm_command_;
  bool hybrid_disarm_request_;
  bool tare_reset_pending_;
  HybridHardResetReason hard_reset_request_;

  bool pending_normal_enable_valid_;
  bool pending_normal_enable_value_;

  bool pending_perching_arm_valid_;
  bool pending_perching_arm_value_;

  bool pending_cutting_active_valid_;
  bool pending_cutting_active_value_;

  /*
   * Everything below is touched only from the control thread
   * (controlCore() and the functions it calls), so it does not need
   * perching_state_mutex_ protection.
   */
  HybridState hybrid_state_;
  bool hybrid_contact_;
  bool hybrid_fault_;
  std::string hybrid_fault_reason_;
  HybridFaultCode hybrid_fault_code_;

  double tau_fast_;
  double tau_slow_;

  double contact_on_timer_;
  double contact_off_timer_;
  double recontact_on_timer_;

  double authority_alpha_;

  ros::Time prev_hybrid_time_;
  ros::Time last_valid_hybrid_control_time_;

  /*
   * Cached supervised wrench supplied to AdmittanceCore.
   *
   * The rotational part is constructed along the locked constraint
   * axis. It is not assumed to be world Y.
   */
  Eigen::Matrix<double, 6, 1> supervised_wrench_world_;

  WelfordStats hybrid_tare_stats_;
  Eigen::Matrix<double, 6, 1> hybrid_equilibrium_wrench_sum_;
  Eigen::Matrix<double, 6, 1> hybrid_equilibrium_wrench_pivot_world_;
  int hybrid_equilibrium_wrench_sample_count_;
  bool hybrid_tare_ready_;
  bool hybrid_tare_collection_active_;
  uint64_t hybrid_tare_lock_generation_;
  AdmittanceCoreOutput last_safe_admittance_output_;
  bool last_safe_output_valid_;

  ros::Publisher hybrid_state_pub_;
  ros::Publisher hybrid_contact_pub_;
  ros::Publisher hybrid_armed_pub_;
  ros::Publisher hybrid_disarm_requested_pub_;
  ros::Publisher hybrid_authority_pub_;
  ros::Publisher hybrid_torque_residual_pub_;
  ros::Publisher hybrid_torque_filtered_pub_;
  ros::Publisher hybrid_cutting_active_pub_;
  ros::Publisher hybrid_cutting_signal_fresh_pub_;
  ros::Publisher hybrid_cutting_gate_valid_pub_;
  ros::Publisher hybrid_tare_sample_count_pub_;
  ros::Publisher hybrid_tare_torque_stddev_pub_;
  ros::Publisher hybrid_tare_collection_active_pub_;
  ros::Publisher hybrid_pivot_torque_raw_pub_;
  ros::Publisher hybrid_pivot_torque_fast_pub_;
  ros::Publisher hybrid_pivot_torque_slow_pub_;
  ros::Publisher hybrid_contact_on_threshold_pub_;
  ros::Publisher hybrid_contact_off_threshold_pub_;
  ros::Publisher hybrid_directional_deadband_pub_;
  ros::Publisher hybrid_directional_torque_pub_;
  ros::Publisher hybrid_pitch_offset_pub_;
  ros::Publisher hybrid_pitch_offset_rate_pub_;
  ros::Publisher hybrid_pitch_offset_acceleration_pub_;
  ros::Publisher hybrid_nominal_pitch_pub_;
  ros::Publisher hybrid_modified_pitch_pub_;
  ros::Publisher hybrid_dt_pub_;
  ros::Publisher hybrid_dt_valid_pub_;
  ros::Publisher hybrid_wrench_fresh_pub_;
  ros::Publisher hybrid_lock_valid_pub_;
  ros::Publisher hybrid_lock_generation_pub_;
  ros::Publisher hybrid_snapshot_generation_valid_pub_;
  ros::Publisher hybrid_tare_ready_pub_;
  ros::Publisher hybrid_pid_i_frozen_pub_;
  ros::Publisher hybrid_fault_pub_;
  ros::Publisher hybrid_effective_tau_input_pub_;
  ros::Publisher hybrid_fault_code_pub_;
  ros::Publisher hybrid_fault_reason_pub_;
  ros::Publisher hybrid_fault_holds_target_pub_;
  ros::Publisher hybrid_abort_request_pub_;
};

} // namespace aerial_robot_control
