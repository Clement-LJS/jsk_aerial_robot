#include <aerial_robot_motion/compliance/cartesian_admittance.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerial_robot_motion::compliance
{
namespace
{
void readVector(const ros::NodeHandle& nh, const std::string& name, Vector6& vector)
{
  std::vector<double> values;
  if (!nh.getParam(name, values))
  {
    if (nh.hasParam(name)) throw std::invalid_argument(name + " must contain six numbers");
    return;
  }
  if (values.size() != 6) throw std::invalid_argument(name + " must contain six numbers");
  for (int i = 0; i < 6; ++i) vector[i] = values[i];
}

Eigen::Isometry3d transform(const geometry_msgs::Transform& message)
{
  Eigen::Quaterniond q(message.rotation.w, message.rotation.x, message.rotation.y, message.rotation.z);
  if (!q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-9)
    throw std::invalid_argument("Invalid wrench transform quaternion");
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = q.normalized().toRotationMatrix();
  result.translation() << message.translation.x, message.translation.y, message.translation.z;
  if (!result.matrix().allFinite()) throw std::invalid_argument("Non-finite wrench transform");
  return result;
}
}  // namespace

void AdmittanceDynamics::configure(const AdmittanceParameters& p)
{
  if (!p.mass.allFinite() || !p.damping.allFinite() || !p.stiffness.allFinite() ||
      !p.axes.allFinite() || !p.displacement_limit.allFinite() || !p.velocity_limit.allFinite() ||
      !p.disable_rate.allFinite() || !std::isfinite(p.max_dt) || p.max_dt <= 0 ||
      (p.mass.array() <= 0).any() || (p.damping.array() < 0).any() ||
      (p.stiffness.array() < 0).any() || (p.displacement_limit.array() < 0).any() ||
      (p.velocity_limit.array() <= 0).any() || (p.disable_rate.array() <= 0).any() ||
      ((p.axes.array() != 0) && (p.axes.array() != 1)).any())
    throw std::invalid_argument("Invalid admittance M/D/K, axis mask, limits, or maximum dt");
  parameters_ = p;
  reset();
}

void AdmittanceDynamics::setEnabled(bool enabled)
{
  if (enabled == enabled_) return;
  // Preserve any offset still returning after a previous disable to avoid a jump.
  // A first enable starts at the zero state initialized by configure().
  enabled_ = enabled;
  velocity_.setZero();
}

void AdmittanceDynamics::reset()
{
  displacement_.setZero();
  velocity_.setZero();
}

void AdmittanceDynamics::step(const Vector6& wrench, double dt)
{
  if (!wrench.allFinite() || !std::isfinite(dt) || dt <= 0 || dt > parameters_.max_dt)
    throw std::invalid_argument("Admittance step requires finite wrench and a valid bounded dt");
  for (int i = 0; i < 6; ++i)
  {
    if (parameters_.axes[i] == 0)
    {
      displacement_[i] = velocity_[i] = 0;
      continue;
    }
    const double previous = displacement_[i];
    if (!enabled_)
    {
      const double rate = std::min(parameters_.disable_rate[i], parameters_.velocity_limit[i]);
      const double change = std::min(std::abs(previous), rate * dt);
      displacement_[i] -= std::copysign(change, previous);
      velocity_[i] = (displacement_[i] - previous) / dt;
      continue;
    }
    const double m = parameters_.mass[i], d = parameters_.damping[i], k = parameters_.stiffness[i];
    // M v(k+1) = M v(k) + dt * (f - D v(k+1) - K x(k+1)).
    const double denominator = m + dt * d + dt * dt * k;
    const double next_velocity = (m * velocity_[i] + dt * (wrench[i] - k * previous)) / denominator;
    if (!std::isfinite(next_velocity)) throw std::runtime_error("Admittance numerical overflow");
    velocity_[i] = std::clamp(next_velocity, -parameters_.velocity_limit[i], parameters_.velocity_limit[i]);
    displacement_[i] = std::clamp(previous + dt * velocity_[i],
                                  -parameters_.displacement_limit[i], parameters_.displacement_limit[i]);
    // Do not accumulate outward velocity while displacement is saturated.
    if ((displacement_[i] >= parameters_.displacement_limit[i] && velocity_[i] > 0) ||
        (displacement_[i] <= -parameters_.displacement_limit[i] && velocity_[i] < 0))
      velocity_[i] = 0;
  }
}

Vector6 AdmittanceDynamics::transformWrench(const Eigen::Isometry3d& target_T_source,
                                          const Vector6& source_wrench)
{
  if (!target_T_source.matrix().allFinite() || !source_wrench.allFinite() ||
      !(target_T_source.linear().transpose() * target_T_source.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
      std::abs(target_T_source.linear().determinant() - 1.0) > 1e-6)
    throw std::invalid_argument("Wrench transform or wrench is invalid");
  Vector6 result;
  result.head<3>() = target_T_source.linear() * source_wrench.head<3>();
  result.tail<3>() = target_T_source.linear() * source_wrench.tail<3>() +
                    target_T_source.translation().cross(result.head<3>());
  if (!result.allFinite()) throw std::runtime_error("Transformed wrench overflow");
  return result;
}

void CartesianAdmittance::initialize(ros::NodeHandle nh, tf2_ros::Buffer& tf,
                                     const std::string& world_frame)
{
  tf_ = &tf;
  world_frame_ = world_frame;
  nh.param("admittance/frame", frame_, std::string("tool"));
  nh.param("admittance/wrench_timeout", wrench_timeout_, 0.25);
  AdmittanceParameters p;
  readVector(nh, "admittance/mass", p.mass);
  readVector(nh, "admittance/damping", p.damping);
  readVector(nh, "admittance/stiffness", p.stiffness);
  readVector(nh, "admittance/axes", p.axes);
  readVector(nh, "admittance/displacement_limit", p.displacement_limit);
  readVector(nh, "admittance/velocity_limit", p.velocity_limit);
  readVector(nh, "admittance/disable_rate", p.disable_rate);
  readVector(nh, "admittance/wrench_reference", wrench_reference_);
  if (!wrench_reference_.allFinite()) throw std::invalid_argument("admittance wrench_reference must be finite");
  nh.param("admittance/max_dt", p.max_dt, 0.1);
  dynamics_.configure(p);
  if (frame_.empty() || world_frame_.empty() || !std::isfinite(wrench_timeout_) || wrench_timeout_ <= 0)
    throw std::invalid_argument("Invalid admittance frame or wrench timeout");
  bool enabled = false;
  nh.param("admittance/enabled", enabled, false);
  dynamics_.setEnabled(enabled);
  std::string wrench_topic;
  nh.param("admittance/wrench_topic", wrench_topic, std::string("external_wrench"));
  wrench_subscriber_ = nh.subscribe(wrench_topic, 1, &CartesianAdmittance::wrenchCallback, this);
  enable_service_ = nh.advertiseService("admittance/enable", &CartesianAdmittance::enableCallback, this);
  reset_service_ = nh.advertiseService("admittance/reset", &CartesianAdmittance::resetCallback, this);
}

void CartesianAdmittance::reset()
{
  dynamics_.reset();
  have_wrench_ = false;
}

void CartesianAdmittance::wrenchCallback(const geometry_msgs::WrenchStampedConstPtr& message)
{
  const auto& w = message->wrench;
  Vector6 values;
  values << w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z;
  if (!values.allFinite() || message->header.frame_id.empty() || message->header.stamp.isZero())
  {
    have_wrench_ = false;
    ROS_WARN_THROTTLE(1.0, "Admittance wrench ignored: require finite values, frame_id, and timestamp");
    return;
  }
  wrench_ = *message;
  received_stamp_ = ros::Time::now();
  have_wrench_ = true;
}

bool CartesianAdmittance::enableCallback(std_srvs::SetBool::Request& request,
                                        std_srvs::SetBool::Response& response)
{
  dynamics_.setEnabled(request.data);
  // Require a new wrench after a mode transition; do not reuse a pre-enable impulse.
  have_wrench_ = false;
  response.success = true;
  response.message = request.data ? "Admittance enabled; waiting for a fresh wrench" :
                                    "Admittance disabled; compliant offset returns at bounded speed";
  return true;
}

bool CartesianAdmittance::resetCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
{
  // Reset requests use the same bounded return as disable. Enable again after settling.
  dynamics_.setEnabled(false);
  have_wrench_ = false;
  response.success = true;
  response.message = "Admittance disabled and returning to zero at bounded speed; enable again when settled";
  return true;
}

Eigen::Isometry3d CartesianAdmittance::worldFrame(const std::string& frame, const ros::Time& stamp,
                                                const Eigen::Isometry3d& tool) const
{
  if (frame == "tool") return tool;
  if (frame == world_frame_) return Eigen::Isometry3d::Identity();
  if (!tf_) throw std::runtime_error("Wrench TF buffer is unavailable");
  return transform(tf_->lookupTransform(world_frame_, frame, stamp, ros::Duration(0)).transform);
}

Eigen::Isometry3d CartesianAdmittance::update(const Eigen::Isometry3d& nominal,
                                            const Eigen::Isometry3d& measured_tool,
                                            double dt, const ros::Time& now,
                                            const ros::Time& measured_tool_stamp)
{
  if (!nominal.matrix().allFinite() || !measured_tool.matrix().allFinite())
    throw std::invalid_argument("Admittance received invalid Cartesian state");
  if (!dynamics_.enabled() && dynamics_.displacement().isZero()) return nominal;
  Vector6 force = Vector6::Zero();
  const ros::Time tool_stamp = measured_tool_stamp.isZero() ? now : measured_tool_stamp;
  const double tool_age = (now - tool_stamp).toSec();
  if (tool_age < -0.01 || tool_age > wrench_timeout_)
    throw std::runtime_error("Admittance measured tool pose is stale");
  bool frame_valid = false;
  Eigen::Isometry3d world_T_frame = Eigen::Isometry3d::Identity();
  try
  {
    world_T_frame = worldFrame(frame_, tool_stamp, measured_tool);
    last_world_R_frame_ = world_T_frame.linear();
    frame_valid = true;
  }
  catch (const std::exception& error)
  {
    ROS_WARN_STREAM_THROTTLE(1.0, "Admittance frame unavailable; holding correction: " << error.what());
  }
  if (dynamics_.enabled() && have_wrench_ && frame_valid)
  {
    const double age = (now - wrench_.header.stamp).toSec();
    const double received_age = (now - received_stamp_).toSec();
    const double tool_wrench_skew = std::abs((tool_stamp - wrench_.header.stamp).toSec());
    if (age >= -0.01 && age <= wrench_timeout_ && received_age >= -0.01 &&
        received_age <= wrench_timeout_ && tool_wrench_skew <= wrench_timeout_)
    {
      try
      {
        const auto& w = wrench_.wrench;
        Vector6 measured;
        measured << w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z;
        const Eigen::Isometry3d world_T_source = worldFrame(wrench_.header.frame_id, wrench_.header.stamp, measured_tool);
        force = AdmittanceDynamics::transformWrench(world_T_frame.inverse() * world_T_source, measured) - wrench_reference_;
      }
      catch (const std::exception& error)
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "Admittance wrench transform failed; using zero wrench: " << error.what());
      }
    }
    else
      ROS_WARN_THROTTLE(1.0, "Admittance wrench stale; using zero wrench for passive recovery");
  }
  // A missing compliance frame cannot define a new force direction. Freeze until valid.
  // Disabled offsets can still decay in their last valid axes.
  if (frame_valid || !dynamics_.enabled()) dynamics_.step(force, dt);
  Eigen::Isometry3d result = nominal;
  result.translation() += last_world_R_frame_ * dynamics_.displacement().head<3>();
  result.linear() = rotationExp(last_world_R_frame_ * dynamics_.displacement().tail<3>()) * nominal.linear();
  return result;
}
}  // namespace aerial_robot_motion::compliance
