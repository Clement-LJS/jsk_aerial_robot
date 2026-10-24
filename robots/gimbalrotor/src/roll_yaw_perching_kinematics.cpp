#include <gimbalrotor/roll_yaw_perching_kinematics.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <cmath>

namespace roll_yaw
{
Vec logSO3(const Mat& r) { Eigen::AngleAxisd a(r); return a.angle()*a.axis(); }
Mat expSO3(const Vec& v)
{
  const double a=v.norm();
  return a<1e-14 ? Mat::Identity() : Eigen::AngleAxisd(a,v/a).toRotationMatrix();
}
bool rotationValid(const Mat& r)
{ return r.allFinite() && (r.transpose()*r-Mat::Identity()).norm()<1e-7 && std::abs(r.determinant()-1)<1e-7; }
bool poseValid(const Pose& p) { return p.matrix().allFinite() && rotationValid(p.linear()); }
Pose fromKdl(const KDL::Frame& f)
{
  Pose p=Pose::Identity();
  for(int i=0;i<3;++i) { p.translation()[i]=f.p[i]; for(int j=0;j<3;++j) p.linear()(i,j)=f.M(i,j); }
  return p;
}
std::function<bool(const Vec&,Pose&)> articulatedCogModel(
  KDL::Tree tree,KDL::JntArray joints,std::map<std::string,KDL::RigidBodyInertia> inertia,
  std::map<std::string,KDL::Segment> extra,std::string base,KDL::Rotation relative,
  unsigned roll_index,unsigned yaw_index)
{
  return [tree,joints,inertia,extra,base,relative,roll_index,yaw_index](const Vec& q,Pose& bg) mutable {
    if(!q.allFinite()||roll_index>=joints.rows()||yaw_index>=joints.rows()) return false;
    joints(roll_index)=q[1]; joints(yaw_index)=q[2];
    KDL::TreeFkSolverPos_recursive fk(tree); KDL::Frame b;
    if(fk.JntToCart(joints,b,base)<0) return false;
    KDL::RigidBodyInertia total=KDL::RigidBodyInertia::Zero();
    for(const auto& item:inertia) {
      KDL::Frame f; if(fk.JntToCart(joints,f,item.first)<0) return false;
      total=total+f*item.second;
      for(const auto& e:extra) if(e.second.getName()==item.first)
        total=total+f*(e.second.getFrameToTip()*e.second.getInertia());
    }
    if(!std::isfinite(total.getMass())||total.getMass()<=0) return false;
    bg=fromKdl(KDL::Frame(relative.Inverse(),b.Inverse()*total.getCOG())); return poseValid(bg);
  };
}
bool Kinematics::configure(const KDL::Chain& chain,const std::string& roll,const std::string& yaw,
  const Pose& cp,const Vec& axis,std::function<bool(const Vec&,Pose&)> cog,std::string& error)
{
  configured_=locked_=false;
  if(chain.getNrOfJoints()!=2 || roll==yaw || !poseValid(cp) || !axis.allFinite() ||
     std::abs(axis.norm()-1)>1e-7 || !cog) { error="invalid chain or required hinge calibration"; return false; }
  unsigned n=0;
  for(unsigned i=0;i<chain.getNrOfSegments();++i) {
    const auto& j=chain.getSegment(i).getJoint();
    if(j.getType()==KDL::Joint::None) continue;
    if(j.getName()!=(n==0?yaw:roll) || j.getType()==KDL::Joint::TransAxis ||
       j.getType()==KDL::Joint::TransX || j.getType()==KDL::Joint::TransY || j.getType()==KDL::Joint::TransZ)
    { error="body-to-contact chain must contain yaw then roll revolutes only"; return false; }
    ++n;
  }
  chain_=chain; cp_=cp; axis_=axis; cog_=std::move(cog); configured_=true; return true;
}
bool Kinematics::bodyPivot(const Vec& eta,Pose& bp) const
{
  if(!configured_ || !eta.allFinite()) return false;
  KDL::JntArray q(2); q(0)=eta[2]; q(1)=eta[1]; KDL::Frame bc;
  KDL::ChainFkSolverPos_recursive fk(chain_);
  if(fk.JntToCart(q,bc)<0) return false;
  bp=fromKdl(bc)*cp_; return poseValid(bp);
}
bool Kinematics::lock(const Pose& body,const Vec& eta)
{
  Pose bp;
  if(!poseValid(body)||!bodyPivot(eta,bp)||std::abs(eta[0])>1e-12) return false;
  lock_=body*bp; locked_=true; return true;
}
bool Kinematics::evaluate(const Vec& eta,Pose& body,Pose& cog) const
{
  Pose bp,bg;
  if(!locked_||!bodyPivot(eta,bp)||!cog_(eta,bg)||!poseValid(bg)) return false;
  Pose hinge=Pose::Identity(); hinge.linear()=expSO3(axis_*eta[0]);
  body=lock_*hinge*bp.inverse(); cog=body*bg;
  return poseValid(body)&&poseValid(cog)&&((body*bp).translation()-lock_.translation()).norm()<1e-9;
}
bool Kinematics::jacobian(const Vec& eta,Mat& j) const
{
  Pose bp; if(!bodyPivot(eta,bp)) return false;
  KDL::JntArray q(2); q(0)=eta[2]; q(1)=eta[1]; KDL::Jacobian k(2);
  KDL::ChainJntToJacSolver solver(chain_);
  if(solver.JntToJac(q,k)<0) return false;
  // Inverting T_BP changes the sign of both active-joint angular axes.
  j.col(0)=bp.linear()*axis_;
  j.col(1)=-k.data.block<3,1>(3,1); j.col(2)=-k.data.block<3,1>(3,0);
  return j.allFinite();
}
bool Kinematics::numericalJacobian(const Vec& eta,Mat& j,double h) const
{
  if(!std::isfinite(h)||h<=0) return false;
  Pose b,g; if(!evaluate(eta,b,g)) return false;
  for(int i=0;i<3;++i) {
    Vec ep=eta,em=eta; ep[i]+=h; em[i]-=h; Pose p,m;
    if(!evaluate(ep,p,g)||!evaluate(em,m,g)) return false;
    j.col(i)=(logSO3(b.linear().transpose()*p.linear())-logSO3(b.linear().transpose()*m.linear()))/(2*h);
  }
  return j.allFinite();
}
bool Kinematics::estimate(const Pose& body,const Vec& eta,Estimate& out) const
{
  Pose bp; if(!locked_||!poseValid(body)||!bodyPivot(eta,bp)) return false;
  const Pose measured=body*bp;
  Eigen::Quaterniond q(lock_.linear().transpose()*measured.linear()); q.normalize();
  // Swing/twist projection extracts the signed hinge rotation, not world Euler pitch.
  const double s=q.vec().dot(axis_);
  if(std::hypot(q.w(),s)<1e-9) return false;
  out.alpha=std::remainder(2*std::atan2(s,q.w()),2*M_PI);
  out.pivot_error=(measured.translation()-lock_.translation()).norm();
  out.hinge_residual=logSO3(expSO3(-axis_*out.alpha)*lock_.linear().transpose()*measured.linear()).norm();
  return std::isfinite(out.alpha)&&std::isfinite(out.pivot_error)&&std::isfinite(out.hinge_residual);
}
}
