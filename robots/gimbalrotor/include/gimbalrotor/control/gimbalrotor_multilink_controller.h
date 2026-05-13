// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/control/gimbalrotor_controller.h>

#include <Eigen/Dense>

namespace aerial_robot_control
{

class GimbalrotorMultilinkController : public GimbalrotorController
{
public:
  GimbalrotorMultilinkController();

  virtual ~GimbalrotorMultilinkController() = default;

  virtual void initialize(ros::NodeHandle nh,
                          ros::NodeHandle nhp,
                          boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                          boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                          double ctrl_loop_rate) override;

  virtual void reset() override;

protected:
  virtual void rosParamInit() override;

  virtual void controlCore() override;

  Eigen::Vector3d calculateMultilinkNonlinearAngularAcc(const Eigen::Matrix3d& inertia,
                                                        const Eigen::Vector3d& omega);

  Eigen::Vector3d limitVectorNorm(const Eigen::Vector3d& input,
                                  double max_norm) const;

protected:
  /*
   * If true:
   *   alpha_cmd = alpha_pid + J^-1 ( omega x J omega + Jdot omega )
   *
   * If false:
   *   behaves closer to the original gimbalrotor controller.
   */
  bool compensate_multilink_inertia_;

  /*
   * Numerical Jdot low-pass filter.
   * Small value: smoother but slower.
   * Large value: faster but noisier.
   */
  double inertia_dot_lpf_rate_;

  /*
   * Safety limiter for added angular acceleration.
   */
  double nonlinear_ang_acc_limit_;

  /*
   * Numerical differentiation memory.
   */
  bool prev_inertia_initialized_;
  double prev_inertia_stamp_;
  Eigen::Matrix3d prev_inertia_;
  Eigen::Matrix3d inertia_dot_lpf_;

  /*
   * Debug publisher.
   */
  ros::Publisher multilink_nonlinear_acc_pub_;
};

} // namespace aerial_robot_control
