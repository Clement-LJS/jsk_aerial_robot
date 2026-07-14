// -*- mode: c++ -*-
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <string>

namespace aerial_robot_control
{

struct AdmittanceCoreConfig
{
  bool use_admittance = true;

  Eigen::Vector3d trans_enable = Eigen::Vector3d::Ones();
  Eigen::Vector3d rot_enable = Eigen::Vector3d::Ones();

  Eigen::Vector3d trans_virtual_mass = Eigen::Vector3d(5.0, 5.0, 5.0);
  Eigen::Vector3d trans_damping = Eigen::Vector3d(30.0, 30.0, 30.0);
  Eigen::Vector3d trans_stiffness = Eigen::Vector3d(60.0, 60.0, 60.0);

  Eigen::Vector3d rot_virtual_inertia = Eigen::Vector3d(0.08, 0.08, 0.08);
  Eigen::Vector3d rot_damping = Eigen::Vector3d(0.35, 0.35, 0.35);
  Eigen::Vector3d rot_stiffness = Eigen::Vector3d(0.8, 0.8, 0.8);

  Eigen::Vector3d force_ref = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque_ref = Eigen::Vector3d::Zero();

  Eigen::Vector3d force_limit = Eigen::Vector3d(3.0, 3.0, 3.0);
  Eigen::Vector3d torque_limit = Eigen::Vector3d(0.8, 0.8, 0.8);

  Eigen::Vector3d pos_offset_limit = Eigen::Vector3d(0.05, 0.05, 0.05);
  Eigen::Vector3d vel_offset_limit = Eigen::Vector3d(0.10, 0.10, 0.10);

  Eigen::Vector3d angle_offset_limit = Eigen::Vector3d(0.15, 0.15, 0.15);
  Eigen::Vector3d angular_vel_offset_limit = Eigen::Vector3d(0.50, 0.50, 0.50);

  double force_lpf_alpha = 0.2;
  double torque_lpf_alpha = 0.2;
};

struct AdmittanceCoreInput
{
  /*
   * external_wrench_world:
   *   [Fx, Fy, Fz, Tx, Ty, Tz]
   *
   * The wrapper/controller should convert the wrench to world frame before
   * passing it into this core.
   */
  Eigen::Matrix<double, 6, 1> external_wrench_world = Eigen::Matrix<double, 6, 1>::Zero();

  /*
   * R_world_compliance:
   *   Rotation from compliance frame to world frame.
   *
   * If compliance frame is world:
   *   R_world_compliance = Identity
   *
   * If compliance frame is body/CoG:
   *   R_world_compliance = R_world_body
   *
   * If compliance frame is tool:
   *   R_world_compliance = R_world_tool
   */
  Eigen::Matrix3d R_world_compliance = Eigen::Matrix3d::Identity();

  double dt = 0.0;
  bool enabled = false;
};

struct AdmittanceCoreOutput
{
  bool valid = false;

  Eigen::Vector3d force_compliance = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque_compliance = Eigen::Vector3d::Zero();

  Eigen::Vector3d pos_offset_compliance = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_offset_compliance = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_offset_compliance = Eigen::Vector3d::Zero();

  Eigen::Vector3d angle_offset_compliance = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_vel_offset_compliance = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acc_offset_compliance = Eigen::Vector3d::Zero();

  Eigen::Vector3d pos_offset_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel_offset_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_offset_world = Eigen::Vector3d::Zero();

  /*
   * For small attitude compliance.
   * This is still represented as roll/pitch/yaw-like small angle offset.
   * Robot-specific code decides how to use it.
   */
  Eigen::Vector3d rpy_offset_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_vel_offset_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acc_offset_world = Eigen::Vector3d::Zero();
};

class AdmittanceCore
{
public:
  AdmittanceCore()
  {
    reset();
    sanitizeConfig();
  }

  explicit AdmittanceCore(const AdmittanceCoreConfig& config)
    : config_(config)
  {
    reset();
    sanitizeConfig();
  }

  void setConfig(const AdmittanceCoreConfig& config)
  {
    config_ = config;
    sanitizeConfig();
  }

  const AdmittanceCoreConfig& getConfig() const
  {
    return config_;
  }

  void reset()
  {
    force_world_lpf_.setZero();
    torque_world_lpf_.setZero();

    pos_offset_compliance_.setZero();
    vel_offset_compliance_.setZero();
    acc_offset_compliance_.setZero();

    angle_offset_compliance_.setZero();
    angular_vel_offset_compliance_.setZero();
    angular_acc_offset_compliance_.setZero();

    last_output_ = AdmittanceCoreOutput();
  }

  AdmittanceCoreOutput update(const AdmittanceCoreInput& input)
  {
    if(!config_.use_admittance || !input.enabled)
      {
        reset();
        return last_output_;
      }

    if(input.dt <= 0.0 || input.dt > 0.1)
      {
        last_output_.valid = false;
        return last_output_;
      }

    const Eigen::Vector3d force_world_raw = input.external_wrench_world.segment<3>(0);

    const Eigen::Vector3d torque_world_raw = input.external_wrench_world.segment<3>(3);

    force_world_lpf_ =
        config_.force_lpf_alpha * force_world_raw +
        (1.0 - config_.force_lpf_alpha) * force_world_lpf_;

    torque_world_lpf_ =
        config_.torque_lpf_alpha * torque_world_raw +
        (1.0 - config_.torque_lpf_alpha) * torque_world_lpf_;

    const Eigen::Matrix3d R_world_compliance = input.R_world_compliance;
    const Eigen::Matrix3d R_compliance_world = R_world_compliance.transpose();

    Eigen::Vector3d force_compliance = R_compliance_world * force_world_lpf_;
    Eigen::Vector3d torque_compliance = R_compliance_world * torque_world_lpf_;

    force_compliance = clampVectorElementwise(force_compliance, config_.force_limit);
    torque_compliance = clampVectorElementwise(torque_compliance, config_.torque_limit);

    /*
     * Translational admittance:
     *   M x_ddot + D x_dot + K x = F_ext - F_ref
     */
    for(int i = 0; i < 3; ++i)
      {
        if(config_.trans_enable(i) < 0.5)
          {
            pos_offset_compliance_(i) = 0.0;
            vel_offset_compliance_(i) = 0.0;
            acc_offset_compliance_(i) = 0.0;
            continue;
          }

        const double force_error = force_compliance(i) - config_.force_ref(i);

        acc_offset_compliance_(i) = (force_error - config_.trans_damping(i) * vel_offset_compliance_(i) - config_.trans_stiffness(i) * pos_offset_compliance_(i)) / config_.trans_virtual_mass(i);

        vel_offset_compliance_(i) += acc_offset_compliance_(i) * input.dt;
        vel_offset_compliance_(i) = clampValue(vel_offset_compliance_(i), -config_.vel_offset_limit(i), config_.vel_offset_limit(i));

        pos_offset_compliance_(i) += vel_offset_compliance_(i) * input.dt;
        pos_offset_compliance_(i) = clampValue(pos_offset_compliance_(i), -config_.pos_offset_limit(i), config_.pos_offset_limit(i));
      }

    /*
     * Rotational admittance:
     *   J theta_ddot + D theta_dot + K theta = tau_ext - tau_ref
     */
    for(int i = 0; i < 3; ++i)
      {
        if(config_.rot_enable(i) < 0.5)
          {
            angle_offset_compliance_(i) = 0.0;
            angular_vel_offset_compliance_(i) = 0.0;
            angular_acc_offset_compliance_(i) = 0.0;
            continue;
          }

        const double torque_error = torque_compliance(i) - config_.torque_ref(i);

        angular_acc_offset_compliance_(i) = (torque_error - config_.rot_damping(i) * angular_vel_offset_compliance_(i) - config_.rot_stiffness(i) * angle_offset_compliance_(i)) / config_.rot_virtual_inertia(i);
        
        angular_vel_offset_compliance_(i) += angular_acc_offset_compliance_(i) * input.dt;
        angular_vel_offset_compliance_(i) = clampValue(angular_vel_offset_compliance_(i), -config_.angular_vel_offset_limit(i), config_.angular_vel_offset_limit(i));

        angle_offset_compliance_(i) += angular_vel_offset_compliance_(i) * input.dt;
        angle_offset_compliance_(i) = clampValue(angle_offset_compliance_(i), -config_.angle_offset_limit(i), config_.angle_offset_limit(i));
      }

    last_output_.valid = true;

    last_output_.force_compliance = force_compliance;
    last_output_.torque_compliance = torque_compliance;

    last_output_.pos_offset_compliance = pos_offset_compliance_;
    last_output_.vel_offset_compliance = vel_offset_compliance_;
    last_output_.acc_offset_compliance = acc_offset_compliance_;

    last_output_.angle_offset_compliance = angle_offset_compliance_;
    last_output_.angular_vel_offset_compliance = angular_vel_offset_compliance_;
    last_output_.angular_acc_offset_compliance = angular_acc_offset_compliance_;

    last_output_.pos_offset_world = R_world_compliance * pos_offset_compliance_;
    last_output_.vel_offset_world = R_world_compliance * vel_offset_compliance_;
    last_output_.acc_offset_world = R_world_compliance * acc_offset_compliance_;

    /*
     * For small angles, using the same rotation is acceptable as a practical
     * compliance correction. If a robot needs exact attitude geometry, handle
     * it in the robot-specific adapter.
     */
    last_output_.rpy_offset_world = R_world_compliance * angle_offset_compliance_;
    last_output_.angular_vel_offset_world = R_world_compliance * angular_vel_offset_compliance_;
    last_output_.angular_acc_offset_world = R_world_compliance * angular_acc_offset_compliance_;

    return last_output_;
  }

  const AdmittanceCoreOutput& getLastOutput() const
  {
    return last_output_;
  }

private:
  void sanitizeConfig()
  {
    config_.force_lpf_alpha = clampValue(config_.force_lpf_alpha, 0.0, 1.0);
    config_.torque_lpf_alpha = clampValue(config_.torque_lpf_alpha, 0.0, 1.0);

    for(int i = 0; i < 3; ++i)
      {
        config_.trans_enable(i) = config_.trans_enable(i) > 0.5 ? 1.0 : 0.0;

        config_.rot_enable(i) = config_.rot_enable(i) > 0.5 ? 1.0 : 0.0;

        if(config_.trans_virtual_mass(i) < 0.001)
          {
            config_.trans_virtual_mass(i) = 0.001;
          }

        if(config_.rot_virtual_inertia(i) < 0.001)
          {
            config_.rot_virtual_inertia(i) = 0.001;
          }

        if(config_.trans_damping(i) < 0.0)
          {
            config_.trans_damping(i) = 0.0;
          }

        if(config_.rot_damping(i) < 0.0)
          {
            config_.rot_damping(i) = 0.0;
          }

        if(config_.trans_stiffness(i) < 0.0)
          {
            config_.trans_stiffness(i) = 0.0;
          }

        if(config_.rot_stiffness(i) < 0.0)
          {
            config_.rot_stiffness(i) = 0.0;
          }

        if(config_.force_limit(i) < 0.0)
          {
            config_.force_limit(i) = 0.0;
          }

        if(config_.torque_limit(i) < 0.0)
          {
            config_.torque_limit(i) = 0.0;
          }

        if(config_.pos_offset_limit(i) < 0.0)
          {
            config_.pos_offset_limit(i) = 0.0;
          }

        if(config_.vel_offset_limit(i) < 0.0)
          {
            config_.vel_offset_limit(i) = 0.0;
          }

        if(config_.angle_offset_limit(i) < 0.0)
          {
            config_.angle_offset_limit(i) = 0.0;
          }

        if(config_.angular_vel_offset_limit(i) < 0.0)
          {
            config_.angular_vel_offset_limit(i) = 0.0;
          }
      }
  }

  static double clampValue(
      double value,
      double min_value,
      double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }

  static Eigen::Vector3d clampVectorElementwise(
      const Eigen::Vector3d& value,
      const Eigen::Vector3d& limit)
  {
    Eigen::Vector3d result = value;

    for(int i = 0; i < 3; ++i)
      {
        result(i) = clampValue(result(i), -std::abs(limit(i)), std::abs(limit(i)));
      }

    return result;
  }

private:
  AdmittanceCoreConfig config_;

  Eigen::Vector3d force_world_lpf_;
  Eigen::Vector3d torque_world_lpf_;

  Eigen::Vector3d pos_offset_compliance_;
  Eigen::Vector3d vel_offset_compliance_;
  Eigen::Vector3d acc_offset_compliance_;

  Eigen::Vector3d angle_offset_compliance_;
  Eigen::Vector3d angular_vel_offset_compliance_;
  Eigen::Vector3d angular_acc_offset_compliance_;

  AdmittanceCoreOutput last_output_;
};

} // namespace aerial_robot_control
