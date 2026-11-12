// -*- mode: c++ -*-
#pragma once

#include <algorithm>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <map>
#include <functional>
#include <string>

namespace perching_geometry
{
enum class Mode
{
  DISABLED,
  NORMAL,
  SLANTED
};

struct Session
{
  Mode mode;
  ros::Time lock_stamp;
};

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

  bool initialize(const tf::Vector3& p, const tf::Vector3& c, const tf::Quaternion& q, double min_radius)
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
    axis = tf::Matrix3x3(orientation).getColumn(1).normalized();
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

// Shared interface avoids asynchronous delta topics in the control loop and
// ensures admittance uses the navigator's current lock, limit and arc sign.
class TargetProvider
{
public:
  virtual ~TargetProvider() = default;
  // Read mode and lock identity together across asynchronous callbacks.
  virtual Session perchingSession() const = 0;
  Mode perchingMode() const
  {
    return perchingSession().mode;
  }
  virtual bool perchingAdmittanceTarget(const ros::Time& lock_stamp, double physical_offset,
                                        const tf::Vector3& nominal_position, Pose& pose) const = 0;
};
}  // namespace perching_geometry

// Specialized passive-alpha / active-roll / active-yaw geometry.

namespace perching_geometry
{
// Add an already aggregated link and its attached extra modules in root axes.
// Callers retain their own FK validation and minimum-mass policy.
void accumulateLinkInertia(KDL::RigidBodyInertia& total, const KDL::Frame& frame, const std::string& link,
                           const KDL::RigidBodyInertia& inertia, const std::map<std::string, KDL::Segment>& extra);
}  // namespace perching_geometry

namespace roll_yaw
{
using Vec = Eigen::Vector3d;
using Mat = Eigen::Matrix3d;
using Pose = Eigen::Isometry3d;
Vec logSO3(const Mat& r);
Mat expSO3(const Vec& v);
bool rotationValid(const Mat& r);
bool poseValid(const Pose& p);
Pose fromKdl(const KDL::Frame& f);

// Inertia-weighted CoG reconstruction from a copied articulated model. relative
// is R_GB, exactly RobotModel::getCogDesireOrientation(), not principal axes.
std::function<bool(const Vec&, Pose&)> articulatedCogModel(KDL::Tree tree, KDL::JntArray joints,
                                                           std::map<std::string, KDL::RigidBodyInertia> inertia,
                                                           std::map<std::string, KDL::Segment> extra, std::string base,
                                                           KDL::Rotation relative, unsigned roll_index,
                                                           unsigned yaw_index);

struct Estimate
{
  double alpha = 0, pivot_error = 0, hinge_residual = 0;
};
// B=base_link, C=contact link, P=calibrated physical hinge, G=controller CoG.
// The callback supplies articulated T_BG, including the model's CoG orientation
// convention (not an inertia eigenvector frame). It must own an immutable snapshot.
class Kinematics
{
public:
  bool configure(const KDL::Chain& body_contact, const std::string& roll, const std::string& yaw,
                 const Pose& contact_pivot, const Vec& axis, std::function<bool(const Vec&, Pose&)> body_cog,
                 std::string& error);
  bool bodyPivot(const Vec& eta, Pose& bp) const;
  bool lock(const Pose& measured_body, const Vec& measured_eta);
  bool evaluateBody(const Vec& eta, Pose& body) const;
  bool evaluate(const Vec& eta, Pose& body, Pose& cog) const;
  bool jacobian(const Vec& eta, Mat& j) const;
  bool numericalJacobian(const Vec& eta, Mat& j, double h = 1e-6) const;
  bool estimate(const Pose& body, const Vec& measured_eta, Estimate& result) const;
  const Pose& pivotLock() const
  {
    return lock_;
  }
  bool locked() const
  {
    return locked_;
  }

private:
  KDL::Chain chain_;
  Pose cp_ = Pose::Identity(), lock_ = Pose::Identity();
  Vec axis_ = Vec::UnitY();
  std::function<bool(const Vec&, Pose&)> cog_;
  bool configured_ = false, locked_ = false;
};
}  // namespace roll_yaw
