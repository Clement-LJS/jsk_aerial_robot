// -*- mode: c++ -*-
#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
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

  /*
   * Optional one-sided rotational state bounds.
   *
   * Left as NaN by default, which means "not overridden": sanitizeConfig()
   * derives symmetric bounds from angle_offset_limit, exactly reproducing
   * the previous behavior. A caller (e.g. perching pitch admittance) can
   * set only the axis it needs, such as [0, +limit] or [-limit, 0].
   */
  Eigen::Vector3d rot_angle_min = Eigen::Vector3d(
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN());

  Eigen::Vector3d rot_angle_max = Eigen::Vector3d(
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN());

  /*
   * Per-axis opt-in for boundary velocity cancellation (>0.5 = enabled).
   * Defaults to disabled so normal translational/rotational admittance
   * behavior is unchanged unless a caller explicitly enables it for an
   * axis that uses one-sided bounds.
   */
  Eigen::Vector3d rot_cancel_velocity_at_bound = Eigen::Vector3d::Zero();

  double force_lpf_alpha = 0.2;
  double torque_lpf_alpha = 0.2;
  /*
  * Maximum accepted integration interval.
  *
  * Controllers with slower update rates may configure a larger value.
  * The perching hybrid supervisor assigns its own max_control_dt.
  */
  double max_dt = 0.1;
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

    if(!std::isfinite(input.dt) ||
      input.dt <= 0.0 ||
      input.dt > config_.max_dt)
    {
      last_output_.valid = false;
      return last_output_;
    }

    if(!input.external_wrench_world.allFinite() ||
       !input.R_world_compliance.allFinite())
      {
        last_output_.valid = false;
        return last_output_;
      }

    const double determinant =
        input.R_world_compliance.determinant();

    if(!std::isfinite(determinant) ||
       std::abs(determinant - 1.0) > 1.0e-3)
      {
        last_output_.valid = false;
        return last_output_;
      }

    const Eigen::Matrix3d orthogonality_error =
        input.R_world_compliance.transpose() *
        input.R_world_compliance -
        Eigen::Matrix3d::Identity();

    if(!orthogonality_error.allFinite() ||
       orthogonality_error.norm() > 1.0e-3)
      {
        last_output_.valid = false;
        return last_output_;
      }

    const Eigen::Vector3d force_world_raw = input.external_wrench_world.segment<3>(0);

    const Eigen::Vector3d torque_world_raw = input.external_wrench_world.segment<3>(3);

    Eigen::Vector3d force_world_lpf_candidate =
        config_.force_lpf_alpha * force_world_raw +
        (1.0 - config_.force_lpf_alpha) * force_world_lpf_;

    Eigen::Vector3d torque_world_lpf_candidate =
        config_.torque_lpf_alpha * torque_world_raw +
        (1.0 - config_.torque_lpf_alpha) * torque_world_lpf_;

    if(!force_world_lpf_candidate.allFinite() ||
       !torque_world_lpf_candidate.allFinite())
      {
        last_output_.valid = false;
        return last_output_;
      }

    const Eigen::Matrix3d R_world_compliance = input.R_world_compliance;
    const Eigen::Matrix3d R_compliance_world = R_world_compliance.transpose();

    Eigen::Vector3d force_compliance = R_compliance_world * force_world_lpf_candidate;
    Eigen::Vector3d torque_compliance = R_compliance_world * torque_world_lpf_candidate;


    const Eigen::Vector3d force_error_compliance =
    clampVectorElementwise(
        force_compliance - config_.force_ref,
        config_.force_limit);

    const Eigen::Vector3d torque_error_compliance =
        clampVectorElementwise(
            torque_compliance - config_.torque_ref,
            config_.torque_limit);
            
    force_compliance = clampVectorElementwise(force_compliance, config_.force_limit);
    torque_compliance = clampVectorElementwise(torque_compliance, config_.torque_limit);

    /*
     * Translational admittance:
     *   M x_ddot + D x_dot + K x = F_ext - F_ref
     */
    Eigen::Vector3d pos_offset_candidate = pos_offset_compliance_;
    Eigen::Vector3d vel_offset_candidate = vel_offset_compliance_;
    Eigen::Vector3d acc_offset_candidate = acc_offset_compliance_;

    for(int i = 0; i < 3; ++i)
      {
        if(config_.trans_enable(i) < 0.5)
          {
            pos_offset_candidate(i) = 0.0;
            vel_offset_candidate(i) = 0.0;
            acc_offset_candidate(i) = 0.0;
            continue;
          }

        const double force_error = force_error_compliance(i);

        acc_offset_candidate(i) = (force_error - config_.trans_damping(i) * vel_offset_candidate(i) - config_.trans_stiffness(i) * pos_offset_candidate(i)) / config_.trans_virtual_mass(i);

        vel_offset_candidate(i) += acc_offset_candidate(i) * input.dt;
        vel_offset_candidate(i) = clampValue(vel_offset_candidate(i), -config_.vel_offset_limit(i), config_.vel_offset_limit(i));

        pos_offset_candidate(i) += vel_offset_candidate(i) * input.dt;
        pos_offset_candidate(i) = clampValue(pos_offset_candidate(i), -config_.pos_offset_limit(i), config_.pos_offset_limit(i));
      }

    /*
     * Rotational admittance:
     *   J theta_ddot + D theta_dot + K theta = tau_ext - tau_ref
     */
    Eigen::Vector3d angle_offset_candidate = angle_offset_compliance_;
    Eigen::Vector3d angular_vel_candidate = angular_vel_offset_compliance_;
    Eigen::Vector3d angular_acc_candidate = angular_acc_offset_compliance_;

    for(int i = 0; i < 3; ++i)
      {
        if(config_.rot_enable(i) < 0.5)
          {
            angle_offset_candidate(i) = 0.0;
            angular_vel_candidate(i) = 0.0;
            angular_acc_candidate(i) = 0.0;
            continue;
          }

        const double torque_error = torque_error_compliance(i);
        
        angular_acc_candidate(i) = (torque_error - config_.rot_damping(i) * angular_vel_candidate(i) - config_.rot_stiffness(i) * angle_offset_candidate(i)) / config_.rot_virtual_inertia(i);
        
        angular_vel_candidate(i) += angular_acc_candidate(i) * input.dt;
        angular_vel_candidate(i) = clampValue(angular_vel_candidate(i), -config_.angular_vel_offset_limit(i), config_.angular_vel_offset_limit(i));

        angle_offset_candidate(i) += angular_vel_candidate(i) * input.dt;
        angle_offset_candidate(i) = clampValue(angle_offset_candidate(i), config_.rot_angle_min(i), config_.rot_angle_max(i));

        /*
         * Input gating alone (zero forcing torque outside the allowed
         * direction) is not sufficient to keep a one-sided axis from
         * drifting past its bound under residual velocity. Cancel only
         * the outward-pointing velocity component at the boundary.
         */
        if(config_.rot_cancel_velocity_at_bound(i) > 0.5)
          {
            if(angle_offset_candidate(i) <= config_.rot_angle_min(i) && angular_vel_candidate(i) < 0.0)
              {
                angular_vel_candidate(i) = 0.0;
              }

            if(angle_offset_candidate(i) >= config_.rot_angle_max(i) && angular_vel_candidate(i) > 0.0)
              {
                angular_vel_candidate(i) = 0.0;
              }
          }
      }

    if(!pos_offset_candidate.allFinite() ||
       !vel_offset_candidate.allFinite() ||
       !acc_offset_candidate.allFinite() ||
       !angle_offset_candidate.allFinite() ||
       !angular_vel_candidate.allFinite() ||
       !angular_acc_candidate.allFinite())
      {
        last_output_.valid = false;
        return last_output_;
      }

    force_world_lpf_ = force_world_lpf_candidate;
    torque_world_lpf_ = torque_world_lpf_candidate;
    pos_offset_compliance_ = pos_offset_candidate;
    vel_offset_compliance_ = vel_offset_candidate;
    acc_offset_compliance_ = acc_offset_candidate;
    angle_offset_compliance_ = angle_offset_candidate;
    angular_vel_offset_compliance_ = angular_vel_candidate;
    angular_acc_offset_compliance_ = angular_acc_candidate;

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
    const auto finiteOr = [](double value, double fallback)
        {
          return std::isfinite(value)
              ? value
              : fallback;
        };

    if(!std::isfinite(config_.max_dt) ||config_.max_dt <= 0.0)
    {
      config_.max_dt = 0.1;
    }

    config_.force_lpf_alpha = clampValue(finiteOr(config_.force_lpf_alpha, 0.2), 0.0, 1.0);

    config_.torque_lpf_alpha = clampValue(finiteOr(config_.torque_lpf_alpha, 0.2), 0.0, 1.0);

    for(int i = 0; i < 3; ++i)
    {
      config_.trans_enable(i) =
          std::isfinite(
              config_.trans_enable(i)) &&
          config_.trans_enable(i) > 0.5
              ? 1.0
              : 0.0;

      config_.rot_enable(i) =
          std::isfinite(
              config_.rot_enable(i)) &&
          config_.rot_enable(i) > 0.5
              ? 1.0
              : 0.0;

      if(!std::isfinite(config_.trans_virtual_mass(i)) || config_.trans_virtual_mass(i) < 0.001)
      {
        config_.trans_virtual_mass(i) = 0.001;
      }

      if(!std::isfinite(
             config_.rot_virtual_inertia(i)) ||
         config_.rot_virtual_inertia(i) < 0.001)
      {
        config_.rot_virtual_inertia(i) =
            0.001;
      }

      if(!std::isfinite(
             config_.trans_damping(i)) ||
         config_.trans_damping(i) < 0.0)
      {
        config_.trans_damping(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.rot_damping(i)) ||
         config_.rot_damping(i) < 0.0)
      {
        config_.rot_damping(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.trans_stiffness(i)) ||
         config_.trans_stiffness(i) < 0.0)
      {
        config_.trans_stiffness(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.rot_stiffness(i)) ||
         config_.rot_stiffness(i) < 0.0)
      {
        config_.rot_stiffness(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.force_ref(i)))
      {
        config_.force_ref(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.torque_ref(i)))
      {
        config_.torque_ref(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.force_limit(i)) ||
         config_.force_limit(i) < 0.0)
      {
        config_.force_limit(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.torque_limit(i)) ||
         config_.torque_limit(i) < 0.0)
      {
        config_.torque_limit(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.pos_offset_limit(i)) ||
         config_.pos_offset_limit(i) < 0.0)
      {
        config_.pos_offset_limit(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.vel_offset_limit(i)) ||
         config_.vel_offset_limit(i) < 0.0)
      {
        config_.vel_offset_limit(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.angle_offset_limit(i)) ||
         config_.angle_offset_limit(i) < 0.0)
      {
        config_.angle_offset_limit(i) =
            0.0;
      }

      if(!std::isfinite(
             config_.angular_vel_offset_limit(i)) ||
         config_.angular_vel_offset_limit(i) < 0.0)
      {
        config_.angular_vel_offset_limit(i) =
            0.0;
      }

      const double angle_limit =
          config_.angle_offset_limit(i);

      if(!std::isfinite(
             config_.rot_angle_min(i)))
      {
        config_.rot_angle_min(i) =
            -angle_limit;
      }

      if(!std::isfinite(
             config_.rot_angle_max(i)))
      {
        config_.rot_angle_max(i) =
            angle_limit;
      }

      config_.rot_angle_min(i) =
          clampValue(
              config_.rot_angle_min(i),
              -angle_limit,
              angle_limit);

      config_.rot_angle_max(i) =
          clampValue(
              config_.rot_angle_max(i),
              -angle_limit,
              angle_limit);

      if(config_.rot_angle_min(i) >
         config_.rot_angle_max(i))
      {
        std::swap(
            config_.rot_angle_min(i),
            config_.rot_angle_max(i));
      }

      config_.rot_cancel_velocity_at_bound(i) =
          std::isfinite(
              config_
                  .rot_cancel_velocity_at_bound(i)) &&
          config_
              .rot_cancel_velocity_at_bound(i) >
              0.5
                  ? 1.0
                  : 0.0;
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
