#include <aerial_robot_control/control/spatial_constraint.h>

#include <algorithm>
#include <cmath>

namespace aerial_robot_control
{

SpatialConstraint::SpatialConstraint(const SpatialConstraintConfig& config)
{
  configure(config);
}

bool SpatialConstraint::configure(const SpatialConstraintConfig& config)
{
  clear();

  if(!config.allowed_dof.allFinite() ||
     !config.pivot_world.allFinite() ||
     !config.locked_position_world.allFinite() ||
     !config.locked_rotation_world.allFinite() ||
     !config.rotation_world_constraint.allFinite() ||
     !std::isfinite(config.coordinate_sign) ||
     !std::isfinite(config.minimum_coordinate) ||
     !std::isfinite(config.maximum_coordinate) ||
     config.minimum_coordinate > config.maximum_coordinate ||
     !isRotationMatrix(config.locked_rotation_world) ||
     !isRotationMatrix(config.rotation_world_constraint))
  {
    return false;
  }

  config_ = config;
  config_.allowed_dof = binaryMask(config.allowed_dof);
  config_.coordinate_sign = config.coordinate_sign >= 0.0 ? 1.0 : -1.0;

  int enabled_translation_count = 0;
  int enabled_rotation_count = 0;

  for(int i = 0; i < 3; ++i)
  {
    if(config_.allowed_dof(i) > 0.5)
    {
      ++enabled_translation_count;
    }

    if(config_.allowed_dof(3 + i) > 0.5)
    {
      rotational_dof_index_ = i;
      ++enabled_rotation_count;
    }
  }

  // Wrench projection is already 6-D, but target pose generation in this first
  // version intentionally supports one rotational constraint only.
  valid_ = config_.active && enabled_translation_count == 0 && enabled_rotation_count == 1;

  return valid_;
}

void SpatialConstraint::clear()
{
  config_ = SpatialConstraintConfig();
  valid_ = false;
  rotational_dof_index_ = -1;
}

bool SpatialConstraint::isActive() const
{
  return config_.active;
}

bool SpatialConstraint::isValid() const
{
  return valid_;
}

bool SpatialConstraint::supportsSingleRotationalDof() const
{
  return valid_ && rotational_dof_index_ >= 0;
}

const SpatialConstraintConfig&
SpatialConstraint::getConfig() const
{
  return config_;
}

const Vector6d& SpatialConstraint::getAllowedDof() const
{
  return config_.allowed_dof;
}

int SpatialConstraint::getRotationalDofIndex() const
{
  return rotational_dof_index_;
}

int SpatialConstraint::getSpatialDofIndex() const
{
  return rotational_dof_index_ < 0
      ? -1
      : 3 + rotational_dof_index_;
}

double SpatialConstraint::clampCoordinate(double coordinate) const
{
  return std::max(config_.minimum_coordinate, std::min(coordinate, config_.maximum_coordinate));
}

SpatialConstraintTarget SpatialConstraint::calculateTarget(double coordinate, double coordinate_velocity, double coordinate_acceleration) const
{
  SpatialConstraintTarget target;

  if(!supportsSingleRotationalDof() || !std::isfinite(coordinate) || !std::isfinite(coordinate_velocity) || !std::isfinite(coordinate_acceleration))
  {
    return target;
  }

  target.coordinate = clampCoordinate(coordinate);

  Eigen::Vector3d axis_constraint = Eigen::Vector3d::Zero();
  axis_constraint(rotational_dof_index_) = config_.coordinate_sign;

  Eigen::Vector3d axis_world = config_.rotation_world_constraint * axis_constraint;

  const double axis_norm = axis_world.norm();
  if(axis_norm < 1.0e-9)
  {
    return target;
  }
  axis_world /= axis_norm;

  const Eigen::Matrix3d delta_rotation = Eigen::AngleAxisd(target.coordinate, axis_world).toRotationMatrix();

  const Eigen::Vector3d locked_radius_world = config_.locked_position_world - config_.pivot_world;

  const Eigen::Vector3d radius_world = delta_rotation * locked_radius_world;

  target.position_world = config_.pivot_world + radius_world;
  target.rotation_world = delta_rotation * config_.locked_rotation_world;
  target.angular_velocity_world = axis_world * coordinate_velocity;
  target.angular_acceleration_world = axis_world * coordinate_acceleration;
  target.linear_velocity_world = target.angular_velocity_world.cross(radius_world);
  target.linear_acceleration_world = target.angular_acceleration_world.cross(radius_world) + target.angular_velocity_world.cross(target.angular_velocity_world.cross(radius_world));

  target.valid =
      target.position_world.allFinite() &&
      target.rotation_world.allFinite() &&
      target.linear_velocity_world.allFinite() &&
      target.angular_velocity_world.allFinite() &&
      target.linear_acceleration_world.allFinite() &&
      target.angular_acceleration_world.allFinite();

  return target;
}

Vector6d SpatialConstraint::wrenchWorldCogToConstraint(const Vector6d& wrench_world_cog, const Eigen::Vector3d& cog_position_world) const
{
  if(!isValid() || !wrench_world_cog.allFinite() || !cog_position_world.allFinite())
  {
    return Vector6d::Zero();
  }

  // New reference point = pivot, old reference point = COG.
  const Eigen::Vector3d vector_pivot_to_cog = cog_position_world - config_.pivot_world;

  const Vector6d wrench_world_pivot = shiftWrenchReferencePoint(wrench_world_cog, vector_pivot_to_cog);

  const Eigen::Matrix3d rotation_constraint_world = config_.rotation_world_constraint.transpose();

  return rotateWrench(wrench_world_pivot, rotation_constraint_world);
}

Vector6d SpatialConstraint::projectConstraintVector(const Vector6d& vector_constraint) const
{
  if(!isValid() || !vector_constraint.allFinite())
  {
    return Vector6d::Zero();
  }

  return vector_constraint.cwiseProduct(config_.allowed_dof);
}

Vector6d SpatialConstraint::generalizedEffortFromWorldCogWrench(const Vector6d& wrench_world_cog, const Eigen::Vector3d& cog_position_world) const
{
  Vector6d effort = projectConstraintVector(wrenchWorldCogToConstraint(wrench_world_cog, cog_position_world));

  const int dof = getSpatialDofIndex();
  if(dof >= 0)
  {
    effort(dof) *= config_.coordinate_sign;
  }

  return effort;
}

Vector6d SpatialConstraint::rotateWrench(const Vector6d& wrench_old, const Eigen::Matrix3d& rotation_new_old)
{
  Vector6d result;
  result.head<3>() = rotation_new_old * wrench_old.head<3>();
  result.tail<3>() = rotation_new_old * wrench_old.tail<3>();
  return result;
}

Vector6d SpatialConstraint::shiftWrenchReferencePoint(const Vector6d& wrench_at_old, const Eigen::Vector3d& vector_new_to_old)
{
  Vector6d result = wrench_at_old;
  result.tail<3>() = wrench_at_old.tail<3>() + vector_new_to_old.cross(wrench_at_old.head<3>());
  return result;
}

Eigen::Matrix3d SpatialConstraint::expSO3(const Eigen::Vector3d& rotation_vector)
{
  const double angle = rotation_vector.norm();

  if(angle < 1.0e-12)
  {
    Eigen::Matrix3d skew;
    skew << 0.0, -rotation_vector.z(), rotation_vector.y(),
            rotation_vector.z(), 0.0, -rotation_vector.x(),
            -rotation_vector.y(), rotation_vector.x(), 0.0;
    return Eigen::Matrix3d::Identity() + skew;
  }

  return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

Eigen::Vector3d SpatialConstraint::logSO3(const Eigen::Matrix3d& rotation)
{
  Eigen::AngleAxisd angle_axis(rotation);

  if(!std::isfinite(angle_axis.angle()) || angle_axis.angle() < 1.0e-12)
  {
    return Eigen::Vector3d::Zero();
  }

  return angle_axis.angle() * angle_axis.axis();
}

Eigen::Vector3d SpatialConstraint::orientationErrorWorld(const Eigen::Matrix3d& rotation_world_desired, const Eigen::Matrix3d& rotation_world_actual)
{
  const Eigen::Matrix3d rotation_actual_desired = rotation_world_actual.transpose() * rotation_world_desired;

  return rotation_world_actual * logSO3(rotation_actual_desired);
}

Vector6d SpatialConstraint::binaryMask(const Vector6d& value)
{
  Vector6d result = Vector6d::Zero();
  for(int i = 0; i < 6; ++i)
  {
    result(i) = value(i) > 0.5 ? 1.0 : 0.0;
  }
  return result;
}

bool SpatialConstraint::isRotationMatrix(const Eigen::Matrix3d& rotation)
{
  if(!rotation.allFinite())
  {
    return false;
  }

  const Eigen::Matrix3d should_be_identity =rotation.transpose() * rotation;

  return
      (should_be_identity - Eigen::Matrix3d::Identity()).norm()
          < 1.0e-5
      && std::abs(rotation.determinant() - 1.0) < 1.0e-5;
}

}  // namespace aerial_robot_control
