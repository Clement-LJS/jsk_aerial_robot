#include <gimbalrotor/roll_yaw_orientation_qp.h>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <limits>
namespace roll_yaw
{
bool Limits::valid() const
{ return lower.allFinite()&&upper.allFinite()&&velocity.allFinite()&&acceleration.allFinite()&&
    (lower.array()<upper.array()).all()&&(velocity.array()>0).all()&&(acceleration.array()>0).all(); }
bool QpConfig::valid() const
{ return weight.allFinite()&&(weight.array()>0).all()&&std::isfinite(regularization)&&regularization>0&&
    std::isfinite(smoothing)&&smoothing>0&&std::isfinite(gain)&&gain>0&&
    std::isfinite(min_singular_value)&&min_singular_value>0; }
QpResult solveBoxQp(const Mat& h,const Vec& g,const Vec& lo,const Vec& hi)
{
  QpResult out; out.reason="invalid or infeasible box QP";
  if(!h.allFinite()||!g.allFinite()||!lo.allFinite()||!hi.allFinite()||
     (lo.array()>hi.array()).any()||!h.isApprox(h.transpose(),1e-10)) return out;
  Eigen::LLT<Mat> positive(h); if(positive.info()!=Eigen::Success) return out;
  double best=std::numeric_limits<double>::infinity();
  for(int code=0;code<27;++code) {
    int c=code; int state[3]; Vec x=Vec::Zero(); Mat a=h; Vec rhs=-g;
    for(int i=0;i<3;++i) { state[i]=c%3; c/=3; if(state[i]) x[i]=state[i]==1?lo[i]:hi[i]; }
    for(int i=0;i<3;++i) if(state[i]) {
      rhs-=h.col(i)*x[i]; a.col(i).setZero(); a.row(i).setZero(); a(i,i)=1;
    }
    for(int i=0;i<3;++i) if(state[i]) rhs[i]=x[i];
    Eigen::LDLT<Mat> factor(a); if(factor.info()!=Eigen::Success) continue;
    x=factor.solve(rhs);
    if(!x.allFinite()||(x.array()<lo.array()-1e-10).any()||(x.array()>hi.array()+1e-10).any()) continue;
    Vec grad=h*x+g; bool kkt=true;
    for(int i=0;i<3;++i) {
      if((state[i]==0&&std::abs(grad[i])>1e-7)||(state[i]==1&&grad[i]<-1e-7)||
         (state[i]==2&&grad[i]>1e-7)) kkt=false;
    }
    const double cost=.5*x.dot(h*x)+g.dot(x);
    if(kkt&&cost<best) { best=cost; out.u=x; out.ok=true; out.reason="optimal"; }
  }
  return out;
}
QpResult orientationStep(const Mat& j,const Mat& r,const Mat& d,const Vec& omega,
  const Vec& eta,const Vec& previous,double dt,const Limits& l,const QpConfig& c)
{
  QpResult out; out.reason="invalid differential IK input";
  if(!l.valid()||!c.valid()||!j.allFinite()||!rotationValid(r)||!rotationValid(d)||!omega.allFinite()||
     !eta.allFinite()||!previous.allFinite()||!std::isfinite(dt)||dt<=0||
     (eta.array()<l.lower.array()-1e-9).any()||(eta.array()>l.upper.array()+1e-9).any()) return out;
  const double sigma=Eigen::JacobiSVD<Mat>(j).singularValues().minCoeff();
  if(sigma<c.min_singular_value) { out.sigma_min=sigma; out.reason="singular/ill-conditioned mechanism"; return out; }
  const Vec lo=(-l.velocity).cwiseMax((l.lower-eta)/dt).cwiseMax(previous-l.acceleration*dt);
  const Vec hi=l.velocity.cwiseMin((l.upper-eta)/dt).cwiseMin(previous+l.acceleration*dt);
  const Vec cmd=r.transpose()*d*omega+c.gain*logSO3(r.transpose()*d);
  const Mat h=j.transpose()*c.weight.asDiagonal()*j+(c.regularization+c.smoothing)*Mat::Identity();
  const Vec g=-j.transpose()*c.weight.asDiagonal()*cmd-c.smoothing*previous;
  out=solveBoxQp(h,g,lo,hi); out.sigma_min=sigma;
  out.residual=(j*out.u-cmd).norm(); return out;
}
}
