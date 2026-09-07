#include <aerial_robot_motion/constraint/joint_limit.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <pluginlib/class_list_macros.h>
#include <limits>
namespace aerial_robot_motion { namespace constraint {
void JointLimitConstraint::initialize(const ros::NodeHandle& nh, const ModelInfo& info)
{
  Base::initialize(nh, info);
  damping_range_ = nonnegativeParam(nh, "damping_range", 0.15);
  margin_ = nonnegativeParam(nh, "safety_margin", 0.01);
  if (damping_range_ <= 0) throw std::runtime_error("joint damping_range must be positive");
  if (nh.hasParam("velocity_limits"))
    info_.velocity = info_.velocity.cwiseMin(vectorParam(nh, "velocity_limits", info.joint_names.size(), 0.1));
  for (int i = 0; i < info_.lower.size(); ++i)
    if (info_.upper[i] - info_.lower[i] <= 2 * margin_)
      throw std::runtime_error("joint safety margin leaves no usable range");
}
bool JointLimitConstraint::bounds(const ModelInfo& info, const Eigen::VectorXd& q, double dt,
    double damping, double margin, Eigen::VectorXd& lo, Eigen::VectorXd& hi)
{
  const int n = info.joint_names.size();
  if (!validStep(dt) || !std::isfinite(damping) || damping <= 0 || !std::isfinite(margin) || margin < 0 ||
      q.size() != n || !q.allFinite() || info.lower.size() != n || info.upper.size() != n || info.velocity.size() != n)
    return false;
  lo = -info.velocity; hi = info.velocity;
  for (int i = 0; i < n; ++i)
  {
    if (!std::isfinite(info.velocity[i]) || info.velocity[i] < 0 ||
        q[i] < info.lower[i] - 1e-8 || q[i] > info.upper[i] + 1e-8) return false;
    // Mechanical limits are exact one-step bounds; the soft inner margin only
    // damps motion toward a limit and never prohibits retreat from it.
    lo[i] = std::max(lo[i], (info.lower[i] - q[i]) / dt);
    hi[i] = std::min(hi[i], (info.upper[i] - q[i]) / dt);
    const double down = std::max(0.0, (q[i] - info.lower[i] - margin) / damping);
    const double up = std::max(0.0, (info.upper[i] - q[i] - margin) / damping);
    lo[i] = std::max(lo[i], -info.velocity[i] * std::min(1.0, down));
    hi[i] = std::min(hi[i], info.velocity[i] * std::min(1.0, up));
  }
  return true;
}
bool JointLimitConstraint::update(const MotionContext& ctx, QPProblem& p)
{
  if (p.g.size() != info_.dimension()) return false;
  Eigen::VectorXd l, u;
  if (!bounds(info_, ctx.state.joints, ctx.dt, damping_range_, margin_, l, u)) return false;
  Eigen::VectorXd hi = Eigen::VectorXd::Constant(p.g.size(), std::numeric_limits<double>::infinity());
  Eigen::VectorXd lo = -hi;
  lo.tail(l.size()) = l; hi.tail(u.size()) = u;
  p.intersectBounds(lo, hi);
  return true;
}
}}
PLUGINLIB_EXPORT_CLASS(aerial_robot_motion::constraint::JointLimitConstraint, aerial_robot_motion::constraint::Base)
