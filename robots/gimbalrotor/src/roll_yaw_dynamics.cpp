#include <gimbalrotor/roll_yaw_dynamics.h>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

namespace roll_yaw
{
ArticulatedDynamics::ArticulatedDynamics(Kinematics model,const KDL::Tree& tree,
  const std::map<std::string,KDL::RigidBodyInertia>& inertia,const std::map<std::string,KDL::Segment>& extra,
  std::string base,unsigned roll,unsigned yaw,Vec gravity):
  model_(std::move(model)),fk_(tree),base_(std::move(base)),roll_(roll),yaw_(yaw),gravity_(gravity)
{
  auto add=[this](const std::string& name,Pose local,KDL::RigidBodyInertia value) {
    if(value.getMass()<=0) return;
    const auto center=value.getCOG();
    const auto tensor=value.RefPoint(center).getRotationalInertia();
    local.translation()+=local.linear()*Vec(center.x(),center.y(),center.z());
    Mat i; for(int r=0;r<3;++r) for(int c=0;c<3;++c) i(r,c)=tensor.data[3*r+c];
    links_.push_back({name,local,value.getMass(),i});
  };
  for(const auto& item:inertia) {
    add(item.first,Pose::Identity(),item.second);
    for(const auto& e:extra) if(e.second.getName()==item.first)
      add(item.first,fromKdl(e.second.getFrameToTip()),e.second.getInertia());
  }
}
bool ArticulatedDynamics::poses(const Vec& eta,const KDL::JntArray& input,std::vector<Pose>& out)
{
  if(roll_>=input.rows()||yaw_>=input.rows()||!input.data.allFinite()) return false;
  KDL::JntArray joints=input; joints(roll_)=eta[1]; joints(yaw_)=eta[2];
  KDL::Frame base; Pose body,cog;
  if(fk_.JntToCart(joints,base,base_)<0||!model_.evaluate(eta,body,cog)) return false;
  const Pose root=body*fromKdl(base).inverse(); out.resize(links_.size());
  for(std::size_t i=0;i<links_.size();++i) {
    KDL::Frame f; if(fk_.JntToCart(joints,f,links_[i].name)<0) return false;
    out[i]=root*fromKdl(f)*links_[i].local;
    if(!poseValid(out[i])) return false;
  }
  return !out.empty();
}
bool ArticulatedDynamics::energy(const Vec& q,const Vec& v,const KDL::JntArray& joints,
                               const KDL::JntArray& rates,DynamicsTerms& out)
{
  constexpr double h=1e-5;
  std::vector<Pose> center,plus,minus;
  if(!poses(q,joints,center)) return false;
  KDL::JntArray jp=joints,jm=joints;
  jp.data+=h*rates.data; jm.data-=h*rates.data;
  if(!poses(q+h*v,jp,plus)||!poses(q-h*v,jm,minus)) return false;
  std::vector<Vec> linear(center.size()),angular(center.size());
  std::vector<Mat> jv(center.size(),Mat::Zero()),jw(center.size(),Mat::Zero());
  out.kinetic=out.potential=0; out.momentum.setZero(); out.mass.setZero();
  for(std::size_t i=0;i<center.size();++i) {
    linear[i]=(plus[i].translation()-minus[i].translation())/(2*h);
    angular[i]=logSO3(plus[i].linear()*minus[i].linear().transpose())/(2*h);
    const Mat iw=center[i].linear()*links_[i].inertia*center[i].linear().transpose();
    out.kinetic+=.5*(links_[i].mass*linear[i].squaredNorm()+angular[i].dot(iw*angular[i]));
    out.potential-=links_[i].mass*gravity_.dot(center[i].translation());
  }
  for(int k=0;k<3;++k) {
    Vec d=Vec::Zero(); d[k]=h;
    if(!poses(q+d,joints,plus)||!poses(q-d,joints,minus)) return false;
    for(std::size_t i=0;i<center.size();++i) {
      jv[i].col(k)=(plus[i].translation()-minus[i].translation())/(2*h);
      jw[i].col(k)=logSO3(plus[i].linear()*minus[i].linear().transpose())/(2*h);
    }
  }
  for(std::size_t i=0;i<center.size();++i) {
    const Mat iw=center[i].linear()*links_[i].inertia*center[i].linear().transpose();
    out.momentum+=links_[i].mass*jv[i].transpose()*linear[i]+jw[i].transpose()*iw*angular[i];
    out.mass+=links_[i].mass*jv[i].transpose()*jv[i]+jw[i].transpose()*iw*jw[i];
  }
  return out.mass.allFinite()&&out.momentum.allFinite()&&std::isfinite(out.kinetic)&&std::isfinite(out.potential);
}
DynamicsTerms ArticulatedDynamics::evaluate(const Vec& q,const Vec& v,const KDL::JntArray& joints,
  const KDL::JntArray& rates,const Vec& force,const Vec& torque,const Vec& servo,double min_singular,
  const std::function<bool()>& cancelled)
{
  DynamicsTerms out; Mat j; Pose body,cog;
  out.reason="invalid dynamics input/kinematics";
  if(!q.allFinite()||!v.allFinite()||!force.allFinite()||!torque.allFinite()||!servo.allFinite()||
     joints.rows()!=rates.rows()||!rates.data.allFinite()||!model_.jacobian(q,j)||!model_.evaluate(q,body,cog)) return out;
  Eigen::JacobiSVD<Mat> svd(j,Eigen::ComputeFullU|Eigen::ComputeFullV);
  if(svd.singularValues().minCoeff()<min_singular) { out.reason="singular task torque map"; return out; }
  out.pitch_map=svd.solve(Vec::UnitY());
  if((j*out.pitch_map-Vec::UnitY()).norm()>1e-8||!energy(q,v,joints,rates,out)) return out;
  if(Eigen::LDLT<Mat>(out.mass).vectorD().minCoeff()<=0) { out.reason="nonpositive generalized inertia"; return out; }
  constexpr double h=1e-4;
  // A resultant applied to the BODY at the instantaneous CoG location. Its
  // translational Jacobian is a body material-point Jacobian, NOT d(CoG)/deta.
  // This is equivalent to summing body-branch rotor forces with frozen gimbals.
  std::vector<Pose> centers;
  if(!poses(q,joints,centers)) return out;
  Vec center=Vec::Zero(); double mass=0;
  for(std::size_t i=0;i<centers.size();++i) { center+=links_[i].mass*centers[i].translation(); mass+=links_[i].mass; }
  if(mass<=0) return out;
  const Vec material=body.inverse()*(center/mass);
  Mat jv;
  for(int k=0;k<3;++k) {
    if(cancelled&&cancelled()) { out.reason="dynamics cancelled/deadline"; return out; }
    Vec d=Vec::Zero(); d[k]=h; DynamicsTerms p,m; Pose bp,bm,g;
    if(!energy(q+d,v,joints,rates,p)||!energy(q-d,v,joints,rates,m)||
       !model_.evaluate(q+d,bp,g)||!model_.evaluate(q-d,bm,g)) return out;
    out.drift[k]=(p.kinetic-m.kinetic-p.potential+m.potential)/(2*h);
    jv.col(k)=((bp*material)-(bm*material))/(2*h);
  }
  out.drift+=jv.transpose()*force+j.transpose()*body.linear().transpose()*torque+Vec(0,servo[1],servo[2]);
  out.ok=out.drift.allFinite(); out.reason=out.ok?"articulated momentum; command-derived body wrench + scaled servo effort":"nonfinite dynamics";
  return out;
}
bool MomentumObserver::update(const DynamicsTerms& terms,double stamp,double max_gap)
{
  if(!terms.ok||!std::isfinite(stamp)||stamp<=0||!std::isfinite(gain)||gain<=0) { reset(); return false; }
  const double dt=stamp-stamp_;
  if(initialized_&&dt==0) return false;
  if(!initialized_||dt<0||dt>max_gap) {
    initial_=terms.momentum; integral_.setZero(); residual.setZero();
    initialized_=true; stamp_=stamp; return false;
  }
  // Backward Euler for integral(b+r): stable for any positive gain and dt.
  residual=gain*(terms.momentum-initial_-integral_-dt*terms.drift)/(1+gain*dt);
  integral_+=dt*(terms.drift+residual); stamp_=stamp;
  torque=terms.pitch_map.dot(residual);
  if(!std::isfinite(torque)) { reset(); return false; }
  return true;
}
}
