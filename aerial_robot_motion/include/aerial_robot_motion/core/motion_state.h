#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/jntarray.hpp>
#include <ros/time.h>
#include <string>
#include <vector>

namespace aerial_robot_motion
{
using Vector6 = Eigen::Matrix<double, 6, 1>;

struct ModelInfo
{
  std::string root_link;
  std::vector<std::string> joint_names;  // Commanded link joints only, in QP order.
  std::vector<int> joint_indices;       // Corresponding full KDL state indices.
  Eigen::VectorXd lower, upper, velocity;
  int dimension() const { return 6 + joint_names.size(); }
};

struct MotionState
{
  ros::Time stamp;
  Eigen::Isometry3d root = Eigen::Isometry3d::Identity();  // world_T_root
  KDL::JntArray full_joints;  // Preserve measured non-commanded joints for FK.
  Eigen::VectorXd joints;
  Eigen::Isometry3d tool = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d contact = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d cog = Eigen::Isometry3d::Identity();
  // World-frame spatial twists, about the respective frame origins.
  // Columns are [v_root_world, omega_root_world, qdot_link].
  Eigen::MatrixXd tool_jacobian, contact_jacobian;
};

struct MotionContext
{
  MotionState state;
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  Vector6 target_twist = Vector6::Zero();  // World frame, at tool origin.
  double dt = 0.02;
  bool contact_active = false;
  Eigen::Vector3d locked_contact_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d locked_hinge_world = Eigen::Vector3d::UnitY();
  Eigen::Vector3d hinge_contact = Eigen::Vector3d::UnitY();
};

struct WholeBodyReference
{
  MotionState state;
  Eigen::VectorXd velocity;
  bool valid = false;
  std::string status;
};

inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
{
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}

inline Eigen::Matrix3d rotationExp(const Eigen::Vector3d& v)
{
  const double angle = v.norm();
  if (angle < 1e-12) return Eigen::Matrix3d::Identity();
  return Eigen::AngleAxisd(angle, v / angle).toRotationMatrix();
}

inline Eigen::Vector3d rotationLog(const Eigen::Matrix3d& r)
{
  Eigen::AngleAxisd aa(r);
  return aa.angle() * aa.axis();
}

inline Vector6 poseError(const Eigen::Isometry3d& target, const Eigen::Isometry3d& current)
{
  Vector6 error;
  error.head<3>() = target.translation() - current.translation();
  error.tail<3>() = rotationLog(target.linear() * current.linear().transpose());
  return error;
}
}  // namespace aerial_robot_motion
