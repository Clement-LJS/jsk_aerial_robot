// -*- mode: c++ -*-
#pragma once

#include <aerial_robot_control/control/interaction_controller.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aerial_robot_control
{

struct SpatialConstraintConfig
{
  bool active = false;

  /*
   * Constraint coordinates:
   *   [translation x, y, z, rotation x, y, z]
   *
   * Current implementation accepts any mask for wrench projection, but pose
   * generation currently supports exactly one rotational DOF.
   */
  Vector6d allowed_dof = Vector6d::Zero();

  // +1 or -1. This defines the positive direction of the scalar coordinate q.
  double coordinate_sign = 1.0;

  // Contact/pivot point, fixed in world for the first implementation.
  Eigen::Vector3d pivot_world = Eigen::Vector3d::Zero();

  // Rotation from constraint frame to world frame.
  Eigen::Matrix3d rotation_world_constraint =
      Eigen::Matrix3d::Identity();

  // Robot pose captured when the constraint is locked. This should be the same
  // robot point controlled by the flight controller, normally the COG.
  Eigen::Vector3d locked_position_world = Eigen::Vector3d::Zero();
  Eigen::Matrix3d locked_rotation_world =
      Eigen::Matrix3d::Identity();

  double minimum_coordinate = -1.0e6;
  double maximum_coordinate = 1.0e6;
};

struct SpatialConstraintTarget
{
  bool valid = false;
  double coordinate = 0.0;

  Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation_world = Eigen::Matrix3d::Identity();

  Eigen::Vector3d linear_velocity_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_world = Eigen::Vector3d::Zero();

  Eigen::Vector3d linear_acceleration_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acceleration_world = Eigen::Vector3d::Zero();
};

/*
 * General spatial math used by perching navigation and interaction control.
 *
 * The robot-specific navigator decides:
 *   - where the pivot is,
 *   - how the constraint frame is oriented,
 *   - which DOF is allowed,
 *   - the angle limits.
 *
 * This class only performs the geometry and wrench transformation.
 */
class SpatialConstraint
{
public:
  SpatialConstraint() = default;
  explicit SpatialConstraint(const SpatialConstraintConfig& config);

  bool configure(const SpatialConstraintConfig& config);
  void clear();

  bool isActive() const;
  bool isValid() const;
  bool supportsSingleRotationalDof() const;

  const SpatialConstraintConfig& getConfig() const;
  const Vector6d& getAllowedDof() const;

  // 0 = roll/x, 1 = pitch/y, 2 = yaw/z. Returns -1 if unsupported.
  int getRotationalDofIndex() const;
  int getSpatialDofIndex() const;

  double clampCoordinate(double coordinate) const;

  SpatialConstraintTarget calculateTarget(
      double coordinate,
      double coordinate_velocity = 0.0,
      double coordinate_acceleration = 0.0) const;

  /*
   * Convert a wrench expressed in world and taken about the COG into the
   * constraint frame and shift its torque reference point to the pivot.
   */
  Vector6d wrenchWorldCogToConstraint(
      const Vector6d& wrench_world_cog,
      const Eigen::Vector3d& cog_position_world) const;

  Vector6d projectConstraintVector(
      const Vector6d& vector_constraint) const;

  /*
   * Returns a 6-D generalized effort vector containing only the permitted
   * components. For the current single-rotation implementation, the selected
   * torque also includes coordinate_sign so it is work-conjugate to q.
   */
  Vector6d generalizedEffortFromWorldCogWrench(
      const Vector6d& wrench_world_cog,
      const Eigen::Vector3d& cog_position_world) const;

  static Vector6d rotateWrench(
      const Vector6d& wrench_old,
      const Eigen::Matrix3d& rotation_new_old);

  static Vector6d shiftWrenchReferencePoint(
      const Vector6d& wrench_at_old,
      const Eigen::Vector3d& vector_new_to_old);

  static Eigen::Matrix3d expSO3(
      const Eigen::Vector3d& rotation_vector);

  static Eigen::Vector3d logSO3(
      const Eigen::Matrix3d& rotation);

  static Eigen::Vector3d orientationErrorWorld(
      const Eigen::Matrix3d& rotation_world_desired,
      const Eigen::Matrix3d& rotation_world_actual);

private:
  static Vector6d binaryMask(const Vector6d& value);
  static bool isRotationMatrix(const Eigen::Matrix3d& rotation);

private:
  SpatialConstraintConfig config_;
  bool valid_ = false;
  int rotational_dof_index_ = -1;
};

}  // namespace aerial_robot_control
