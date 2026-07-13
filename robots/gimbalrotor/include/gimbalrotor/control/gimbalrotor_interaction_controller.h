// -*- mode: c++ -*-
#pragma once

#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <gimbalrotor/gimbalrotor_perching_navigation.h>

#include <aerial_robot_control/control/interaction_controller.h>
#include <aerial_robot_control/control/spatial_constraint.h>

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Bool.h>

#include <Eigen/Core>

#include <string>

namespace aerial_robot_control
{

class GimbalrotorInteractionController : public GimbalrotorController
{
public:
  GimbalrotorInteractionController();
  ~GimbalrotorInteractionController() override = default;

  void initialize(
      ros::NodeHandle nh,
      ros::NodeHandle nhp,
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
      boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
      double ctrl_loop_rate) override;

  void reset() override;

protected:
  void rosParamInit() override;
  void controlCore() override;

  // Called by PoseLinearController after it copies the navigator target into
  // local target_* members and before it runs PID.
  void modifyControlTarget(double dt) override;

private:
  enum class Mode
  {
    ADMITTANCE,
    IMPEDANCE
  };

  void interactionEnableCallback(const std_msgs::BoolConstPtr& msg);

  void updateExternalWrenchEstimate(double dt);
  void publishEstimatedWrench() const;
  void publishConstraintWrench(const Vector6d& wrench_constraint) const;

  void applyFreeFlightAdmittance(double dt);
  void applyPerchingAdmittance(
      aerial_robot_navigation::GimbalrotorPerchingNavigator& navigator,
      double dt);
  void applyFreeFlightImpedance();

  Eigen::Matrix3d getCurrentCogRotationWorld() const;
  Eigen::Vector3d getCurrentCogPositionWorld() const;

  bool loadVector6Param(
      const ros::NodeHandle& nh,
      const std::string& name,
      Vector6d& value);

  static Matrix6d diagonalMatrix(const Vector6d& diagonal);
  static tf::Vector3 eigenToTf(const Eigen::Vector3d& vector);
  static Eigen::Vector3d tfToEigen(const tf::Vector3& vector);

private:
  InteractionController interaction_controller_;
  InteractionControllerConfig interaction_config_;

  Mode mode_ = Mode::ADMITTANCE;
  std::string mode_name_ = "admittance";
  std::string compliance_frame_ = "world";

  bool interaction_enabled_ = false;
  bool observer_enabled_ = true;
  bool observer_only_in_hover_land_ = true;
  bool previous_perching_mode_active_ = false;

  std::string interaction_enable_topic_ = "cutting_enable";

  // This mask is used only when perching is not active. During perching, the
  // mask is supplied by GimbalrotorPerchingNavigator::SpatialConstraint.
  Vector6d free_flight_active_dof_ = Vector6d::Zero();

  Vector6d estimated_wrench_world_cog_ = Vector6d::Zero();
  bool estimated_wrench_valid_ = false;

  ros::Time previous_observer_time_;

  ros::Subscriber interaction_enable_sub_;
  ros::Publisher estimated_wrench_pub_;
  ros::Publisher constraint_wrench_pub_;
};

}  // namespace aerial_robot_control
