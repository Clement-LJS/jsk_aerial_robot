// -*- mode: c++ -*-
#pragma once
#include <gimbalrotor/gimbalrotor_navigation.h>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <ros/time.h>
#include <tf/transform_datatypes.h>

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
}  // namespace perching_geometry

namespace perching_dynamics
{
// Rotate axes only; the wrench remains about the same physical point.
Eigen::Matrix<double, 6, 1> rotateWrench(const Eigen::Matrix3d& rotation, const Eigen::Matrix<double, 6, 1>& wrench);

// All vectors use the same axes. source_minus_target points from the new
// reference point to the old one: M_target = M_source + r x F.
Eigen::Vector3d shiftMoment(const Eigen::Vector3d& moment, const Eigen::Vector3d& source_minus_target,
                            const Eigen::Vector3d& force);
}  // namespace perching_dynamics

namespace aerial_robot_navigation
{
class GimbalrotorPerchingNavigator : public GimbalrotorNavigator, public perching_geometry::TargetProvider
{
public:
  using Vector = Eigen::VectorXd;
  using Pose = Eigen::Isometry3d;
  struct Sample
  {
    double stamp = 0, receipt = 0;
    Pose body = Pose::Identity();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    Vector q, rate;
  };
  GimbalrotorPerchingNavigator();
  void initialize(ros::NodeHandle, ros::NodeHandle, boost::shared_ptr<aerial_robot_model::RobotModel>,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>, double) override;
  void update() override;
  perching_geometry::Session perchingSession() const override;
  bool perchingAdmittanceTarget(const ros::Time&, double, const tf::Vector3&, perching_geometry::Pose&) const override;
  std::recursive_mutex& perchingStateMutex() const { return perching_state_mutex_; }
  bool articulated() const { return !active_joints_.empty(); }
  virtual bool motionFeedbackValid() const;
  virtual bool observation(Sample&) const;
  double rigidAngle() const { return arc_pitch_sign_ * rigid_applied_pose_.delta; }
  double rigidRate() const { return (velocity_active_ ? velocity_command_ : 0) + applied_offset_rate_; }
  const std::string& prefix() const { return prefix_; }
  const std::string& worldFrame() const { return world_; }
  virtual bool nominalHeld() const;
  virtual bool trackingHeld() const;
  double appliedPitchOffset() const { return applied_pitch_offset_; }
  virtual bool applyPerchingReference(double pitch_offset, double rate, double acceleration);
  virtual void publishDiagnostics();

protected:
  virtual bool tryLockPerching(const std::string& reason);
  virtual bool supportsPerchingMode(perching_geometry::Mode) const;
  virtual void readProfile();
  virtual void initializeMechanism();
  virtual bool resolveMechanismModel();
  virtual void resetMotion();
  virtual void applyManualPitchDelta(double delta);
  virtual void relockCallback(const std_msgs::EmptyConstPtr& msg);
  virtual void onNonHover() {}
  virtual void onClockRewind() {}
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;
  bool hasPitchCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;
  void commitFixedContactLock(const tf::Vector3& cog_position, const tf::Quaternion& cog_orientation,
                             const tf::Vector3& contact_position);
  bool fresh(double stamp, double receipt) const;
  struct Reference
  {
    Pose body = Pose::Identity(), cog = Pose::Identity();
  };
  bool bodyContact(const Vector&, Pose&, Eigen::Vector3d&) const;
  bool lockGeometry(const Pose& body, const Vector& q, Reference& initial);
  Pose worldCogPose(const Pose& body, const Eigen::Vector3d& cog_body) const;
  bool estimateCoordinates(const Pose&, Vector&, double&, double&) const;

  boost::shared_ptr<GimbalrotorRobotModel> model_;
  Pose pivot_ = Pose::Identity();
  std::vector<std::string> active_joints_;
  // Feedback, nominal commands and controller-visible session state share this lock.
  mutable std::recursive_mutex perching_state_mutex_;
  perching_geometry::Mode perching_mode_;
  bool perching_locked_;
  bool accept_uav_nav_pitch_command_;
  double command_pitch_sign_;
  ros::Time lock_stamp_;
  ros::Publisher status_pub_, state_pub_;
  std::string prefix_ = "perching", world_ = "world", status_name_ = "perching";
  std::string status_ = "free flight";
  bool model_valid_ = false, velocity_active_ = false;
  double applied_pitch_offset_ = 0, applied_offset_rate_ = 0;
  double tracking_tolerance_ = .04, rezero_rate_tolerance_ = .05;
  double rate_limit_ = .03, velocity_command_ = 0, velocity_expires_ = 0, velocity_duration_ = 8;
  double timeout_ = .15, future_tolerance_ = .02, max_step_ = .05, max_gap_ = .15;
  double last_update_ = 0, cycle_dt_ = 0, applied_stamp_ = 0, pivot_error_ = 0, hinge_error_ = 0;

private:
  void rosParamInit() override;
  bool ownsNavigationTargets() const override;
  std::unique_lock<std::recursive_mutex> lockNavigationTargets() override;
  void navigationTargetConflict(const char*) override;
  void selectPerchingMode(perching_geometry::Mode mode, bool enable);
  void resetCallback(const std_msgs::EmptyConstPtr& msg);
  void resetPerchingLock();
  void perchingEnableCallback(const std_msgs::BoolConstPtr& msg);
  void perchingSlantedEnableCallback(const std_msgs::BoolConstPtr& msg);
  void manualPitchDeltaCallback(const std_msgs::Float64ConstPtr& msg);
  void commitPerchingLockIdentity();
  bool applyPerchingConstraint(const aerial_robot_msgs::FlightNav& nav_msg);
  bool hasPositionCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;
  bool hasVelocityCommand(const aerial_robot_msgs::FlightNav& nav_msg) const;
  tf::Vector3 getCurrentRobotPos() const;
  tf::Vector3 getCurrentRobotRPY() const;
  tf::Vector3 getDesiredPosition(const aerial_robot_msgs::FlightNav& nav_msg) const;
  tf::Vector3 getDesiredVelocity(const aerial_robot_msgs::FlightNav& nav_msg) const;
  tf::Vector3 getCurrentBaselinkPos() const;
  tf::Matrix3x3 getCurrentBaselinkRot() const;
  void publishLockedDebugPose();
  void publishLockedPivot();
  void publishCommandedDebugPose(const perching_geometry::Pose& pose);
  bool activePose(perching_geometry::Pose& pose) const;
  void publishApplied();
  bool configureGeometry(std::string& reason);
  bool geometryFrames(const Vector&, std::map<std::string, KDL::Frame>&) const;

  ros::Subscriber perching_enable_sub_;
  ros::Subscriber perching_slanted_enable_sub_;
  ros::Subscriber relock_sub_;
  ros::Subscriber reset_sub_;
  ros::Subscriber manual_pitch_delta_sub_;
  ros::Publisher locked_pose_pub_;
  ros::Publisher locked_pivot_pub_;
  ros::Publisher commanded_pose_pub_;
  ros::Publisher commanded_pitch_delta_pub_;
  bool perching_lock_once_;
  bool command_pitch_as_delta_;
  bool use_pitch_command_for_arc_;
  bool active_perching_hold_enable_;
  double min_valid_radius_;
  double max_pitch_delta_;
  double arc_pitch_sign_;
  std::string perching_enable_topic_;
  std::string perching_slanted_enable_topic_;
  std::string locked_pivot_topic_;
  std::string relock_topic_;
  std::string reset_topic_;
  std::string manual_pitch_delta_topic_;
  double active_pitch_delta_;
  ros::Time last_lock_stamp_;  // Never reuse a lock identity, even with paused ROS time.
  tf::Vector3 locked_robot_pos_world_;
  tf::Vector3 reference_locked_rpy_;  // Reference Euler angles for NORMAL commands and logging.
  tf::Vector3 locked_pivot_world_;
  double locked_radius_;
  KDL::Tree geometry_tree_;
  KDL::JntArray held_joints_;
  Eigen::Matrix3d cog_to_body_ = Eigen::Matrix3d::Identity();
  perching_geometry::Lock hinge_arc_;
  perching_geometry::Pose rigid_applied_pose_;
  bool rigid_applied_valid_ = false;
  bool rigid_command_pending_ = false;
};
}
