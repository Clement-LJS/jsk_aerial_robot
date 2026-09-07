#include <aerial_robot_motion/command/motion_command_manager.h>

#include <cmath>
#include <stdexcept>

namespace aerial_robot_motion::command
{
namespace
{
void validatePose(const Eigen::Isometry3d& pose)
{
  if (!pose.matrix().allFinite() ||
      !(pose.linear().transpose() * pose.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(pose.linear().determinant() - 1.0) > 1e-6)
    throw std::invalid_argument("Cartesian pose is not a finite rigid transform");
}

Eigen::Isometry3d transform(const geometry_msgs::Transform& message)
{
  Eigen::Quaterniond q(message.rotation.w, message.rotation.x, message.rotation.y, message.rotation.z);
  if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-9)
    throw std::invalid_argument("Invalid transform quaternion");
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = q.normalized().toRotationMatrix();
  result.translation() << message.translation.x, message.translation.y, message.translation.z;
  validatePose(result);
  return result;
}
}  // namespace

void MotionCommandManager::initialize(ros::NodeHandle nh, tf2_ros::Buffer& tf,
                                      const std::string& world_frame)
{
  tf_ = &tf;
  world_frame_ = world_frame;
  nh.param("command/frame", default_frame_, std::string("tool"));
  nh.param("command/measured_timeout", measured_timeout_, 0.25);
  nh.param("command/max_translation_increment", max_translation_increment_, 0.1);
  nh.param("command/max_rotation_increment", max_rotation_increment_, 0.35);
  if (world_frame_.empty() || default_frame_.empty() ||
      !std::isfinite(measured_timeout_) || measured_timeout_ <= 0 ||
      !std::isfinite(max_translation_increment_) || max_translation_increment_ <= 0 ||
      !std::isfinite(max_rotation_increment_) || max_rotation_increment_ <= 0)
    throw std::invalid_argument("Invalid Cartesian command frame, timeout, or increment limit");
  pose_subscriber_ = nh.subscribe("command/pose", 1, &MotionCommandManager::poseCallback, this);
  increment_subscriber_ = nh.subscribe("command/increment", 10, &MotionCommandManager::incrementCallback, this);
}

void MotionCommandManager::setMeasuredPose(const Eigen::Isometry3d& pose, const ros::Time& stamp)
{
  validatePose(pose);
  measured_ = pose;
  measured_stamp_ = stamp;
  if (!ready_)
  {
    nominal_ = measured_;
    ready_ = true;
  }
}

void MotionCommandManager::resetTarget(const Eigen::Isometry3d& pose)
{
  validatePose(pose);
  nominal_ = pose;
}

void MotionCommandManager::setAbsolutePose(const Eigen::Isometry3d& world_T_target)
{
  if (!ready_) throw std::runtime_error("Cartesian command requires measured robot state first");
  validatePose(world_T_target);
  nominal_ = world_T_target;
}

void MotionCommandManager::applyIncrement(const Vector6& increment, const Eigen::Matrix3d& world_R_command)
{
  if (!ready_) throw std::runtime_error("Cartesian increment requires measured robot state first");
  if (!increment.allFinite() || !world_R_command.allFinite() ||
      !(world_R_command.transpose() * world_R_command).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(world_R_command.determinant() - 1.0) > 1e-6)
    throw std::invalid_argument("Invalid Cartesian increment or command rotation");
  if (increment.head<3>().norm() > max_translation_increment_ ||
      increment.tail<3>().norm() > max_rotation_increment_)
    throw std::invalid_argument("Cartesian increment exceeds configured per-command limit");
  nominal_.translation() += world_R_command * increment.head<3>();
  nominal_.linear() = rotationExp(world_R_command * increment.tail<3>()) * nominal_.linear();
}

void MotionCommandManager::requireFreshMeasurement() const
{
  const double age = (ros::Time::now() - measured_stamp_).toSec();
  if (!ready_ || measured_stamp_.isZero() || age < -0.01 || age > measured_timeout_)
    throw std::runtime_error("Cartesian command rejected: measured tool pose is unavailable or stale");
}

Eigen::Isometry3d MotionCommandManager::commandFrame(const std::string& frame, const ros::Time& stamp) const
{
  if (frame == "tool") return measured_;  // Virtual configured tool, including its offset.
  if (frame == world_frame_) return Eigen::Isometry3d::Identity();
  if (!tf_) throw std::runtime_error("Command TF buffer is unavailable");
  return transform(tf_->lookupTransform(world_frame_, frame, stamp, ros::Duration(0)).transform);
}

void MotionCommandManager::poseCallback(const geometry_msgs::PoseStampedConstPtr& message)
{
  try
  {
    requireFreshMeasurement();
    const auto& p = message->pose;
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-9)
      throw std::invalid_argument("Invalid Cartesian target quaternion");
    Eigen::Isometry3d frame_T_target = Eigen::Isometry3d::Identity();
    frame_T_target.translation() << p.position.x, p.position.y, p.position.z;
    frame_T_target.linear() = q.normalized().toRotationMatrix();
    const std::string frame = message->header.frame_id.empty() ? world_frame_ : message->header.frame_id;
    setAbsolutePose(commandFrame(frame, message->header.stamp) * frame_T_target);
  }
  catch (const std::exception& error)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Motion pose command ignored: " << error.what());
  }
}

void MotionCommandManager::incrementCallback(const geometry_msgs::TwistStampedConstPtr& message)
{
  try
  {
    requireFreshMeasurement();
    Vector6 increment;
    increment << message->twist.linear.x, message->twist.linear.y, message->twist.linear.z,
                 message->twist.angular.x, message->twist.angular.y, message->twist.angular.z;
    const std::string frame = message->header.frame_id.empty() ? default_frame_ : message->header.frame_id;
    applyIncrement(increment, commandFrame(frame, message->header.stamp).linear());
  }
  catch (const std::exception& error)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Motion increment ignored: " << error.what());
  }
}
}  // namespace aerial_robot_motion::command
