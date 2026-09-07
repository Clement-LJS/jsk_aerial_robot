#pragma once
#include <aerial_robot_motion/core/motion_state.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <stdexcept>
namespace aerial_robot_motion
{
inline std::string cleanFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') frame.erase(frame.begin());
  return frame;
}
inline Eigen::Isometry3d fromPose(const geometry_msgs::Pose& p)
{
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-9 || !t.translation().allFinite())
    throw std::invalid_argument("pose has invalid quaternion or position");
  t.linear() = q.normalized().toRotationMatrix();
  return t;
}
inline Eigen::Isometry3d fromTransform(const geometry_msgs::Transform& t)
{
  geometry_msgs::Pose p;
  p.position.x = t.translation.x; p.position.y = t.translation.y; p.position.z = t.translation.z;
  p.orientation = t.rotation;
  return fromPose(p);
}
inline geometry_msgs::Pose toPose(const Eigen::Isometry3d& t)
{
  geometry_msgs::Pose p;
  p.position.x = t.translation().x(); p.position.y = t.translation().y(); p.position.z = t.translation().z();
  Eigen::Quaterniond q(t.linear()); q.normalize();
  p.orientation.x = q.x(); p.orientation.y = q.y(); p.orientation.z = q.z(); p.orientation.w = q.w();
  return p;
}
}
