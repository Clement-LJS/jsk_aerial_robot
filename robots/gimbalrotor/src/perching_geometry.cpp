#include <gimbalrotor/perching_geometry.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <cmath>

namespace perching_geometry
{
bool Lock::targetOrientation(const tf::Quaternion& desired, double limit, double sign, Pose& pose) const
{
  const tf::Matrix3x3 relative(orientation.inverse() * desired);
  const double angle = std::atan2(relative[0][2] - relative[2][0], relative[0][0] + relative[2][2]);
  return target(angle / sign, limit, sign, pose);
}

void accumulateLinkInertia(KDL::RigidBodyInertia& total, const KDL::Frame& frame, const std::string& link,
                           const KDL::RigidBodyInertia& inertia, const std::map<std::string, KDL::Segment>& extra)
{
  total = total + frame * inertia;
  for(const auto& module : extra)
  {
    if(module.second.getName() == link)
      total = total + frame * (module.second.getFrameToTip() * module.second.getInertia());
  }
}
}  // namespace perching_geometry

namespace roll_yaw
{
Eigen::Vector3d logSO3(const Eigen::Matrix3d& r)
{
  Eigen::AngleAxisd a(r);
  return a.angle() * a.axis();
}
Eigen::Matrix3d expSO3(const Eigen::Vector3d& v)
{
  const double a = v.norm();
  return a < 1e-14 ? Eigen::Matrix3d::Identity() : Eigen::AngleAxisd(a, v / a).toRotationMatrix();
}
bool rotationValid(const Eigen::Matrix3d& r)
{
  return r.allFinite() && (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-7 &&
         std::abs(r.determinant() - 1) < 1e-7;
}
bool poseValid(const Eigen::Isometry3d& p)
{
  return p.matrix().allFinite() && rotationValid(p.linear());
}
Eigen::Isometry3d fromKdl(const KDL::Frame& f)
{
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  for(int i = 0; i < 3; ++i)
  {
    p.translation()[i] = f.p[i];
    for(int j = 0; j < 3; ++j)
      p.linear()(i, j) = f.M(i, j);
  }
  return p;
}
namespace
{
// Each copy owns both its FK solver and joint buffer. Planning workers never
// evaluate against a mutable live RobotModel or share mutable FK storage.
class ArticulatedCogSnapshot
{
public:
  ArticulatedCogSnapshot(const KDL::Tree& tree, KDL::JntArray joints,
                         std::map<std::string, KDL::RigidBodyInertia> inertia,
                         std::map<std::string, KDL::Segment> extra, std::string base, KDL::Rotation relative,
                         unsigned roll_index, unsigned yaw_index)
      : fk_(tree),
        joints_(std::move(joints)),
        inertia_(std::move(inertia)),
        extra_(std::move(extra)),
        base_(std::move(base)),
        relative_(relative),
        roll_index_(roll_index),
        yaw_index_(yaw_index)
  {
  }

  bool evaluate(const Eigen::Vector3d& q, Eigen::Isometry3d& bg)
  {
    if(!q.allFinite() || roll_index_ >= joints_.rows() || yaw_index_ >= joints_.rows())
      return false;
    joints_(roll_index_) = q[1];
    joints_(yaw_index_) = q[2];
    KDL::Frame body;
    if(fk_.JntToCart(joints_, body, base_) < 0)
      return false;
    KDL::RigidBodyInertia total = KDL::RigidBodyInertia::Zero();
    for(const auto& item : inertia_)
    {
      KDL::Frame frame;
      if(fk_.JntToCart(joints_, frame, item.first) < 0)
        return false;
      perching_geometry::accumulateLinkInertia(total, frame, item.first, item.second, extra_);
    }
    if(!std::isfinite(total.getMass()) || total.getMass() <= 0)
      return false;
    // relative_ is R_GB, the existing model's desired CoG orientation.
    bg = fromKdl(KDL::Frame(relative_.Inverse(), body.Inverse() * total.getCOG()));
    return poseValid(bg);
  }

private:
  KDL::TreeFkSolverPos_recursive fk_;
  KDL::JntArray joints_;
  std::map<std::string, KDL::RigidBodyInertia> inertia_;
  std::map<std::string, KDL::Segment> extra_;
  std::string base_;
  KDL::Rotation relative_;
  unsigned roll_index_, yaw_index_;
};
}  // namespace

std::function<bool(const Eigen::Vector3d&, Eigen::Isometry3d&)> articulatedCogModel(
    KDL::Tree tree, KDL::JntArray joints, std::map<std::string, KDL::RigidBodyInertia> inertia,
    std::map<std::string, KDL::Segment> extra, std::string base, KDL::Rotation relative, unsigned roll_index,
    unsigned yaw_index)
{
  ArticulatedCogSnapshot snapshot(tree, joints, inertia, extra, base, relative, roll_index, yaw_index);
  return [snapshot](const Eigen::Vector3d& q, Eigen::Isometry3d& bg) mutable { return snapshot.evaluate(q, bg); };
}
bool Kinematics::configure(const KDL::Chain& chain, const std::string& roll, const std::string& yaw,
                           const Eigen::Isometry3d& cp, const Eigen::Vector3d& axis,
                           std::function<bool(const Eigen::Vector3d&, Eigen::Isometry3d&)> cog, std::string& error)
{
  configured_ = locked_ = false;
  if(chain.getNrOfJoints() != 2 || roll == yaw || !poseValid(cp) || !axis.allFinite() ||
     std::abs(axis.norm() - 1) > 1e-7 || !cog)
  {
    error = "invalid chain or hinge geometry";
    return false;
  }
  unsigned n = 0;
  for(unsigned i = 0; i < chain.getNrOfSegments(); ++i)
  {
    const auto& j = chain.getSegment(i).getJoint();
    if(j.getType() == KDL::Joint::None)
      continue;
    if(j.getName() != (n == 0 ? yaw : roll) || j.getType() == KDL::Joint::TransAxis ||
       j.getType() == KDL::Joint::TransX || j.getType() == KDL::Joint::TransY || j.getType() == KDL::Joint::TransZ)
    {
      error = "body-to-contact chain must contain yaw then roll revolutes only";
      return false;
    }
    ++n;
  }
  chain_ = chain;
  cp_ = cp;
  axis_ = axis;
  cog_ = std::move(cog);
  configured_ = true;
  return true;
}
bool Kinematics::bodyPivot(const Eigen::Vector3d& eta, Eigen::Isometry3d& bp) const
{
  if(!configured_ || !eta.allFinite())
    return false;
  KDL::JntArray q(2);
  q(0) = eta[2];
  q(1) = eta[1];
  KDL::Frame bc;
  KDL::ChainFkSolverPos_recursive fk(chain_);
  if(fk.JntToCart(q, bc) < 0)
    return false;
  bp = fromKdl(bc) * cp_;
  return poseValid(bp);
}
bool Kinematics::lock(const Eigen::Isometry3d& body, const Eigen::Vector3d& eta)
{
  Eigen::Isometry3d bp;
  if(!poseValid(body) || !bodyPivot(eta, bp) || std::abs(eta[0]) > 1e-12)
    return false;
  lock_ = body * bp;
  locked_ = true;
  return true;
}
bool Kinematics::evaluateBody(const Eigen::Vector3d& eta, Eigen::Isometry3d& body) const
{
  Eigen::Isometry3d bp;
  if(!locked_ || !bodyPivot(eta, bp))
    return false;
  Eigen::Isometry3d hinge = Eigen::Isometry3d::Identity();
  hinge.linear() = expSO3(axis_ * eta[0]);
  body = lock_ * hinge * bp.inverse();
  return poseValid(body) && ((body * bp).translation() - lock_.translation()).norm() < 1e-9;
}
bool Kinematics::evaluate(const Eigen::Vector3d& eta, Eigen::Isometry3d& body, Eigen::Isometry3d& cog) const
{
  Eigen::Isometry3d bg;
  if(!evaluateBody(eta, body) || !cog_(eta, bg) || !poseValid(bg))
    return false;
  cog = body * bg;
  return poseValid(cog);
}
bool Kinematics::jacobian(const Eigen::Vector3d& eta, Eigen::Matrix3d& j) const
{
  Eigen::Isometry3d bp;
  if(!bodyPivot(eta, bp))
    return false;
  KDL::JntArray q(2);
  q(0) = eta[2];
  q(1) = eta[1];
  KDL::Jacobian k(2);
  KDL::ChainJntToJacSolver solver(chain_);
  if(solver.JntToJac(q, k) < 0)
    return false;
  // Inverting T_BP changes the sign of both active-joint angular axes.
  j.col(0) = bp.linear() * axis_;
  j.col(1) = -k.data.block<3, 1>(3, 1);
  j.col(2) = -k.data.block<3, 1>(3, 0);
  return j.allFinite();
}
bool Kinematics::numericalJacobian(const Eigen::Vector3d& eta, Eigen::Matrix3d& j, double h) const
{
  if(!std::isfinite(h) || h <= 0)
    return false;
  Eigen::Isometry3d b;
  if(!evaluateBody(eta, b))
    return false;
  for(int i = 0; i < 3; ++i)
  {
    Eigen::Vector3d ep = eta, em = eta;
    ep[i] += h;
    em[i] -= h;
    Eigen::Isometry3d p, m;
    if(!evaluateBody(ep, p) || !evaluateBody(em, m))
      return false;
    j.col(i) = (logSO3(b.linear().transpose() * p.linear()) - logSO3(b.linear().transpose() * m.linear())) / (2 * h);
  }
  return j.allFinite();
}
bool Kinematics::estimate(const Eigen::Isometry3d& body, const Eigen::Vector3d& eta, Estimate& out) const
{
  Eigen::Isometry3d bp;
  if(!locked_ || !poseValid(body) || !bodyPivot(eta, bp))
    return false;
  const Eigen::Isometry3d measured = body * bp;
  Eigen::Quaterniond q(lock_.linear().transpose() * measured.linear());
  q.normalize();
  // Swing/twist projection extracts the signed hinge rotation, not world Euler pitch.
  const double s = q.vec().dot(axis_);
  if(std::hypot(q.w(), s) < 1e-9)
    return false;
  out.alpha = std::remainder(2 * std::atan2(s, q.w()), 2 * M_PI);
  out.pivot_error = (measured.translation() - lock_.translation()).norm();
  out.hinge_residual = logSO3(expSO3(-axis_ * out.alpha) * lock_.linear().transpose() * measured.linear()).norm();
  return std::isfinite(out.alpha) && std::isfinite(out.pivot_error) && std::isfinite(out.hinge_residual);
}
}  // namespace roll_yaw
