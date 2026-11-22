// -*- mode: c++ -*-
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/segment.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

namespace perching_geometry
{
inline bool forwardKinematics(const KDL::Tree& tree, const KDL::JntArray& joints, std::map<std::string, KDL::Frame>& poses)
{
  if(joints.rows() != tree.getNrOfJoints() || !joints.data.allFinite()) return false;
  poses.clear();
  poses.emplace(tree.getRootSegment()->first, KDL::Frame::Identity());
  auto visit = [&](auto&& self, const auto& element, const KDL::Frame& parent) -> void
  {
    for(const auto& child : GetTreeElementChildren(element))
    {
      // Copy only this segment: Joint::pose has a mutable cache, even on a const tree.
      const KDL::Segment segment = GetTreeElementSegment(child->second);
      const double position = segment.getJoint().getType() == KDL::Joint::None ? 0 : joints(GetTreeElementQNr(child->second));
      const KDL::Frame frame = parent * segment.pose(position);
      poses.emplace(child->first, frame);
      self(self, child->second, frame);
    }
  };
  visit(visit, tree.getRootSegment()->second, KDL::Frame::Identity());
  return true;
}

inline bool finite(const tf::Vector3& v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

struct Pose
{
  tf::Vector3 position;
  tf::Quaternion orientation;
  double delta = 0.0;  // Bounded logical angle, before arc sign.
};

struct Lock
{
  tf::Vector3 position{0, 0, 0}, pivot{0, 0, 0}, radius{0, 0, 0}, axis{0, 0, 0};
  tf::Quaternion orientation{0, 0, 0, 1};
  bool valid = false;

  bool initialize(const tf::Vector3& p, const tf::Vector3& c, const tf::Quaternion& q, double min_radius, const tf::Vector3& hinge_axis)
  {
    valid = false;
    if(!finite(p) || !finite(c) || !std::isfinite(q.x()) || !std::isfinite(q.y()) || !std::isfinite(q.z()) ||
       !std::isfinite(q.w()) || !std::isfinite(q.length2()) || q.length2() < 1.0e-12 || !std::isfinite(min_radius) ||
       min_radius <= 0.0)
      return false;
    position = p;
    pivot = c;
    orientation = q.normalized();
    radius = p - c;
    if(!finite(hinge_axis) || std::abs(hinge_axis.length2() - 1) > 1e-6) return false;
    axis = hinge_axis;
    valid = finite(radius) && finite(axis) && std::isfinite(radial().length()) && radial().length() >= min_radius;
    return valid;
  }

  tf::Vector3 radial() const
  {
    return radius - axis * axis.dot(radius);
  }

  // Legacy absolute-attitude input projected onto the captured local-Y arc.
  bool targetOrientation(const tf::Quaternion& desired, double limit, double sign, Pose& pose) const;

  bool target(double delta, double limit, double sign, Pose& pose) const
  {
    if(!valid || !std::isfinite(delta) || !std::isfinite(limit) || limit <= 0.0 || !std::isfinite(sign) ||
       std::abs(sign) != 1.0 || !finite(position) || !finite(pivot) || !finite(radius) || !finite(axis) ||
       std::abs(axis.length2() - 1.0) > 1.0e-6 || !std::isfinite(orientation.length2()) ||
       std::abs(orientation.length2() - 1.0) > 1.0e-6)
      return false;
    pose.delta = std::max(-limit, std::min(limit, delta));
    // Preserve the lock exactly for zero; avoid subtract/add roundoff.
    pose.position = position;
    pose.orientation = orientation;
    if(pose.delta != 0.0)
    {
      const tf::Quaternion rotation(axis, sign * pose.delta);
      pose.position = pivot + tf::Matrix3x3(rotation) * radius;
      pose.orientation = (rotation * orientation).normalized();
    }
    return finite(pose.position);
  }

  bool project(const tf::Vector3& desired, double limit, double sign, Pose& pose) const
  {
    if(!valid || !finite(desired) || !std::isfinite(sign) || std::abs(sign) != 1.0)
      return false;
    const tf::Vector3 relative = desired - pivot;
    const tf::Vector3 perpendicular = relative - axis * axis.dot(relative);
    if(!finite(perpendicular) || perpendicular.length2() < 1.0e-12)
      return false;
    const tf::Vector3 r = radial().normalized();
    const tf::Vector3 d = perpendicular.normalized();
    const double physical_angle = std::atan2(axis.dot(r.cross(d)), r.dot(d));
    return target(physical_angle / sign, limit, sign, pose);
  }

  tf::Vector3 tangentVelocity(const tf::Vector3& current, const tf::Vector3& velocity) const
  {
    if(!valid || !finite(current) || !finite(velocity))
      return tf::Vector3(0, 0, 0);
    const tf::Vector3 tangent = axis.cross(current - pivot);
    if(!finite(tangent) || tangent.length2() < 1.0e-12)
      return tf::Vector3(0, 0, 0);
    const tf::Vector3 unit_tangent = tangent.normalized();
    return unit_tangent * unit_tangent.dot(velocity);
  }
};

inline bool Lock::targetOrientation(const tf::Quaternion& desired, double limit, double sign, Pose& pose) const
{
  const tf::Quaternion relative = desired * orientation.inverse();
  const double angle = 2 * std::atan2(tf::Vector3(relative.x(), relative.y(), relative.z()).dot(axis), relative.w());
  return target(std::atan2(std::sin(angle), std::cos(angle)) / sign, limit, sign, pose);
}

inline void accumulateLinkInertia(KDL::RigidBodyInertia& total, const KDL::Frame& frame, const std::string& link,
                                  const KDL::RigidBodyInertia& inertia, const std::map<std::string, KDL::Segment>& extra)
{
  total = total + frame * inertia;
  for(const auto& module : extra)
  {
    if(module.second.getName() == link)
      total = total + frame * (module.second.getFrameToTip() * module.second.getInertia());
  }
}

inline Eigen::Vector3d logSO3(const Eigen::Matrix3d& r)
{
  Eigen::AngleAxisd a(r);
  return a.angle() * a.axis();
}
inline Eigen::Matrix3d expSO3(const Eigen::Vector3d& v)
{
  const double a = v.norm();
  return a < 1e-14 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(a, v / a).toRotationMatrix();
}
inline bool rotationValid(const Eigen::Matrix3d& r)
{
  return r.allFinite() && (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-7 &&
         std::abs(r.determinant() - 1) < 1e-7;
}
inline bool poseValid(const Eigen::Isometry3d& p)
{
  return p.matrix().allFinite() && rotationValid(p.linear());
}

inline Eigen::Isometry3d fromKdl(const KDL::Frame& frame)
{
  Eigen::Isometry3d pose;
  tf::transformKDLToEigen(frame, pose);
  return pose;
}

inline KDL::Frame toKdl(const Eigen::Isometry3d& pose)
{
  KDL::Frame out;
  tf::quaternionEigenToKDL(Eigen::Quaterniond(pose.linear()), out.M);
  tf::vectorEigenToKDL(pose.translation(), out.p);
  return out;
}

inline tf::Quaternion tfQuaternion(const Eigen::Matrix3d& r)
{
  tf::Quaternion q;
  tf::quaternionEigenToTF(Eigen::Quaterniond(r), q);
  return q;
}

inline void coordinateMap(const std::vector<std::string>& active_joints, std::map<std::string, int>& coordinates)
{
  coordinates["alpha"] = 0;
  for(std::size_t i = 0; i < active_joints.size(); ++i) coordinates[active_joints[i]] = i + 1;
}

inline bool pitchCoordinates(std::size_t count, const std::map<std::string, int>& coordinates,
                             const std::vector<std::string>& names, const std::string& manual, std::vector<bool>& moving)
{
  moving.assign(count, false);
  for(const auto& name : names)
  {
    if(!coordinates.count(name)) return false;
    moving[coordinates.at(name)] = true;
  }
  return !names.empty() && coordinates.count(manual);
}
}  // namespace perching_geometry

namespace perching_dynamics
{
// Rotate axes only; the wrench remains about the same physical point.
inline Eigen::Matrix<double, 6, 1> rotateWrench(const Eigen::Matrix3d& rotation, const Eigen::Matrix<double, 6, 1>& wrench)
{
  Eigen::Matrix<double, 6, 1> rotated;
  rotated.head<3>() = rotation * wrench.head<3>();
  rotated.tail<3>() = rotation * wrench.tail<3>();
  return rotated;
}

// All vectors use the same axes. source_minus_target points from the new
// reference point to the old one: M_target = M_source + r x F.
inline Eigen::Vector3d shiftMoment(const Eigen::Vector3d& moment, const Eigen::Vector3d& source_minus_target,
                            const Eigen::Vector3d& force)
{
  return moment + source_minus_target.cross(force);
}
}  // namespace perching_dynamics
