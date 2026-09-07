#include <aerial_robot_motion/constraint/revolute_contact.h>
#include <aerial_robot_motion/core/plugin_utils.h>
#include <pluginlib/class_list_macros.h>
namespace aerial_robot_motion { namespace constraint {
void RevoluteContactConstraint::initialize(const ros::NodeHandle& nh, const ModelInfo& info)
{
  Base::initialize(nh, info);
  position_gain_ = nonnegativeParam(nh, "position_gain", 3.0);
  orientation_gain_ = nonnegativeParam(nh, "orientation_gain", 3.0);
}
bool RevoluteContactConstraint::equality(const MotionContext& ctx, double kp, double kr,
                                        Eigen::MatrixXd& a, Eigen::VectorXd& b)
{
  const auto& j = ctx.state.contact_jacobian;
  if (j.rows() != 6 || j.cols() < 6 || !j.allFinite() || !ctx.state.contact.matrix().allFinite() ||
      !ctx.locked_contact_position.allFinite() || !ctx.locked_hinge_world.allFinite() ||
      !ctx.hinge_contact.allFinite() || ctx.locked_hinge_world.norm() < 1e-9 || ctx.hinge_contact.norm() < 1e-9)
    return false;
  const Eigen::Vector3d axis = ctx.locked_hinge_world.normalized();
  const Eigen::Vector3d current = ctx.state.contact.linear() * ctx.hinge_contact.normalized();
  const Eigen::Vector3d u = axis.unitOrthogonal();
  Eigen::Matrix<double, 2, 3> basis;
  basis.row(0) = u.transpose(); basis.row(1) = axis.cross(u).transpose();
  a.resize(5, j.cols()); b.resize(5);
  a.topRows<3>() = j.topRows<3>();
  a.bottomRows<2>() = basis * j.bottomRows<3>();
  b.head<3>() = kp * (ctx.locked_contact_position - ctx.state.contact.translation());
  // Only align hinge directions: accumulated free rotation is not an error.
  const Eigen::Vector3d correction = rotationLog(Eigen::Quaterniond::FromTwoVectors(current, axis).toRotationMatrix());
  b.tail<2>() = kr * basis * correction;
  return a.allFinite() && b.allFinite();
}
bool RevoluteContactConstraint::update(const MotionContext& ctx, QPProblem& p)
{
  if (!ctx.contact_active) return true;
  Eigen::MatrixXd a; Eigen::VectorXd b;
  if (!equality(ctx, position_gain_, orientation_gain_, a, b) || a.cols() != p.g.size()) return false;
  p.appendConstraints(a, b, b);
  return true;
}
}}
PLUGINLIB_EXPORT_CLASS(aerial_robot_motion::constraint::RevoluteContactConstraint, aerial_robot_motion::constraint::Base)
