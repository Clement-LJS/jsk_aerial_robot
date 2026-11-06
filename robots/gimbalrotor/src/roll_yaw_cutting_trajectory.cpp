#include <gimbalrotor/roll_yaw_cutting_trajectory.h>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <gimbalrotor/roll_yaw_worker.h>

namespace roll_yaw
{
bool Limits::valid() const
{ return lower.allFinite()&&upper.allFinite()&&velocity.allFinite()&&acceleration.allFinite()&&
    (lower.array()<upper.array()).all()&&(velocity.array()>0).all()&&(acceleration.array()>0).all(); }
bool Request::valid() const
{
  for(double x:{yaw,roll,pitch_start,pitch_end,rate,acceleration}) if(!std::isfinite(x)) return false;
  return rate>0 && acceleration>0 && std::abs(yaw)<=M_PI && std::abs(roll)<M_PI/2 &&
    std::abs(pitch_start)<M_PI && std::abs(pitch_end)<M_PI &&
    (align_only || std::abs(pitch_end-pitch_start)>1e-8);
}
bool PlanningConfig::mechanismValid() const
{
  return limits.lower.allFinite()&&limits.upper.allFinite()&&
    (limits.lower.array()<limits.upper.array()).all()&&limits.lower[0]>-M_PI&&limits.upper[0]<M_PI&&
    limits.lower[0]<=0&&limits.upper[0]>=0&&std::isfinite(max_tilt)&&max_tilt>0&&max_tilt<M_PI/2;
}
bool PlanningConfig::valid() const
{
  for(double x:{dt,max_tilt,orientation_tolerance,rate_tolerance,alignment_rate,alignment_acceleration,
                max_duration,cog_speed_limit,cog_acceleration_limit}) if(!std::isfinite(x)||x<=0) return false;
  return mechanismValid()&&limits.valid()&&limits.lower[0]>-M_PI&&limits.upper[0]<M_PI&&
    limits.lower[0]<=0&&limits.upper[0]>=0&&std::isfinite(min_singular_value)&&min_singular_value>0&&dt>=.001&&dt<=.1&&max_tilt<M_PI/2&&
    scaling_attempts>=1&&scaling_attempts<=10&&max_duration/dt<200000;
}
Stroke::Stroke(double distance,double rate,double acc):distance_(std::abs(distance)),sign_(distance>=0?1:-1)
{
  speed_=std::min(rate,std::sqrt(distance_*2*acc/M_PI));
  ramp_=speed_>0?M_PI*speed_/(2*acc):0;
  feed_=speed_>0?std::max(0.0,distance_/speed_-ramp_):0;
}
void Stroke::sample(double t,double& angle,double& rate,double& acc) const
{
  angle=rate=acc=0;
  if(t<=0||duration()==0) return;
  if(t>=duration()) { angle=sign_*distance_; return; }
  if(t<ramp_) {
    angle=speed_*.5*(t-ramp_/M_PI*std::sin(M_PI*t/ramp_));
    rate=speed_*.5*(1-std::cos(M_PI*t/ramp_)); acc=speed_*M_PI/(2*ramp_)*std::sin(M_PI*t/ramp_);
  } else if(t<ramp_+feed_) { angle=speed_*(.5*ramp_+t-ramp_); rate=speed_; }
  else {
    const double d=duration()-t;
    angle=distance_-speed_*.5*(d-ramp_/M_PI*std::sin(M_PI*d/ramp_));
    rate=speed_*.5*(1-std::cos(M_PI*d/ramp_)); acc=-speed_*M_PI/(2*ramp_)*std::sin(M_PI*d/ramp_);
  }
  angle*=sign_; rate*=sign_; acc*=sign_;
}
Desired cuttingOrientation(const Mat& n,const Request& r,double theta,double rate,double acc)
{ return {n*expSO3(Vec::UnitZ()*r.yaw)*expSO3(Vec::UnitX()*r.roll)*expSO3(Vec::UnitY()*theta),
          Vec(0,rate,0),Vec(0,acc,0)}; }
namespace
{
bool fcAttitudeAllowed(const Mat& r)
{
  // Spinal attitude_control.h MAX_TILT_ANGLE: roll/pitch of the CoG frame,
  // matching GimbalrotorController::sendFourAxisCommand. Do not widen it here.
  constexpr double limit=1.0;
  return rotationValid(r)&&std::abs(std::atan2(r(2,1),r(2,2)))<=limit&&
    std::abs(std::atan2(-r(2,0),std::hypot(r(2,1),r(2,2))))<=limit;
}
using Controls=std::array<Vec,6>;
Controls controls(const Knot& a,const Knot& b)
{
  const double dt=b.t-a.t;
  return {{a.eta,a.eta+a.velocity*dt/5,a.eta+2*a.velocity*dt/5+a.acceleration*dt*dt/20,
           b.eta-2*b.velocity*dt/5+b.acceleration*dt*dt/20,b.eta-b.velocity*dt/5,b.eta}};
}
template<std::size_t N> Vec bezier(std::array<Vec,N> p,double s)
{
  for(std::size_t n=N-1;n>0;--n) for(std::size_t i=0;i<n;++i) p[i]=(1-s)*p[i]+s*p[i+1];
  return p[0];
}
bool coordinates(const Path& path,double t,Vec& q,Vec& v,Vec& acc)
{
  if(path.knots.size()<2||!std::isfinite(t)) return false;
  t=std::max(0.0,std::min(t,path.duration()));
  auto it=std::upper_bound(path.knots.begin(),path.knots.end(),t,[](double x,const Knot& k){return x<k.t;});
  if(it==path.knots.end()) --it;
  if(it==path.knots.begin()) ++it;
  const auto& b=*it; const auto& a=*(it-1); const double dt=b.t-a.t,s=(t-a.t)/dt;
  const auto p=controls(a,b); std::array<Vec,5> dv; std::array<Vec,4> da;
  for(int i=0;i<5;++i) dv[i]=5*(p[i+1]-p[i])/dt;
  for(int i=0;i<4;++i) da[i]=4*(dv[i+1]-dv[i])/dt;
  q=bezier(p,s); v=bezier(dv,s); acc=bezier(da,s);
  return q.allFinite()&&v.allFinite()&&acc.allFinite();
}
bool configurationAllowed(const Kinematics& m,const Vec& q,const Desired& des,const PlanningConfig& c,std::string& error)
{
  Pose b,g; Mat j;
  if(!q.allFinite()||!rotationValid(des.rotation)||!m.evaluate(q,b,g)||!m.jacobian(q,j)) { error="invalid FK/reference"; return false; }
  if((q.array()<c.limits.lower.array()-1e-9).any()||(q.array()>c.limits.upper.array()+1e-9).any())
  { error="mechanism position limit [alpha, roll, yaw]: q="+std::to_string(q[0])+","+std::to_string(q[1])+","+std::to_string(q[2]); return false; }
  if(b.linear()(2,2)<std::cos(c.max_tilt)||des.rotation(2,2)<std::cos(c.max_tilt))
  { error="body world tilt exceeds configured operating range"; return false; }
  if(!fcAttitudeAllowed(g.linear()))
  { error="CoG attitude exceeds Spinal roll/pitch command limits"; return false; }
  if(Eigen::JacobiSVD<Mat>(j).singularValues().minCoeff()<c.min_singular_value)
  { error="singular/ill-conditioned mechanism"; return false; }
  if(logSO3(b.linear().transpose()*des.rotation).norm()>c.orientation_tolerance)
  { error="FK orientation tracking tolerance exceeded"; return false; }
  return true;
}
bool validatePath(const Kinematics& m,const Path& path,const PlanningConfig& c,std::string& error)
{
  for(std::size_t k=1;k<path.knots.size();++k) {
    if(c.cancelled&&c.cancelled()) { error="validation cancelled/deadline exceeded"; return false; }
    if(c.report) c.report(k,path.knots.size()-1,true);
    const double dt=path.knots[k].t-path.knots[k-1].t;
    auto failed=[&](const std::string& why,int axis=-1) {
      error="interval="+std::to_string(k)+" t="+std::to_string(path.knots[k-1].t)+" s "+
        (axis<0?"":std::string(axis==0?"alpha":axis==1?"roll":"yaw")+": ")+why; return false;
    };
    const auto p=controls(path.knots[k-1],path.knots[k]); std::array<Vec,5> v;
    for(const auto& q:p) for(int axis=0;axis<3;++axis)
      if(q[axis]<c.limits.lower[axis]-1e-12||q[axis]>c.limits.upper[axis]+1e-12)
        return failed("interpolated position limit",axis);
    for(int i=0;i<5;++i) {
      v[i]=5*(p[i+1]-p[i])/dt;
      for(int axis=0;axis<3;++axis) if(std::abs(v[i][axis])>c.limits.velocity[axis]*.5+1e-9)
        return failed("interpolated velocity reserve exceeded; time scaling required",axis);
    }
    for(int i=0;i<4;++i) if(( (4*(v[i+1]-v[i])/dt).cwiseAbs().array()>c.limits.acceleration.array()*.5+1e-9).any())
      { std::ostringstream why; why<<"interpolated acceleration reserve exceeded; interval="<<k
            <<" coefficient="<<i<<" acceleration="<<(4*(v[i+1]-v[i])/dt).transpose()
            <<" limit="<<(c.limits.acceleration*.5).transpose(); error=why.str(); return false; }
    const int samples=c.min_singular_value==0?24:std::max(4,static_cast<int>(std::ceil(dt/(c.dt/4))));
    for(int i=0;i<=samples;++i) {
      Reference ref; const double t=path.knots[k-1].t+dt*i/samples;
      if(!samplePath(m,path,t,ref)||!configurationAllowed(m,ref.eta,ref.desired,c,error))
        return failed("sample="+std::to_string(i)+": "+error);
      if((ref.body_omega-ref.body.linear().transpose()*ref.desired.rotation*ref.desired.omega).norm()>c.rate_tolerance)
        { error="body angular velocity tracking tolerance exceeded"; return false; }
      if(ref.cog_velocity.norm()>c.cog_speed_limit*.5||ref.cog_acceleration.norm()>c.cog_acceleration_limit*.5)
        { error="CoG speed/acceleration limit"; return false; }
    }
  }
  return true;
}
bool buildPath(const Kinematics& m,const Vec& initial,double duration,const std::function<Desired(double)>& desired,
               const PlanningConfig& c,Path& path,std::string& error)
{
  if(duration>c.max_duration) { error="phase duration exceeds limit"; return false; }
  const int steps=std::max(2,static_cast<int>(std::ceil(duration/c.dt)));
  const double dt=duration/steps;
  path.knots.clear(); path.desired=desired;
  Vec eta=initial;
  for(int i=0;i<=steps;++i) {
    if(c.cancelled&&c.cancelled()) { error="planning cancelled/deadline exceeded"; return false; }
    if(c.report) c.report(i,steps,false);
    const auto d=desired(i*dt); const Vec seed=eta;
    Mat j;
    if(!inverseOrientation(m,d.rotation,seed,c,eta,error)||!configurationAllowed(m,eta,d,c,error)||
       (eta-seed).norm()>.1||(i==0&&(eta-initial).norm()>1e-7)||!m.jacobian(eta,j)) {
      error="sample="+std::to_string(i)+" t="+std::to_string(i*dt)+" s: "+
        (error.empty()?"IK branch/start consistency":error); return false;
    }
    // Differentiate the same orientation constraint: J qdot = omega,
    // J qddot + Jdot qdot = omegadot. Endpoints are zero speed/acceleration.
    Eigen::JacobiSVD<Mat> svd(j,Eigen::ComputeFullU|Eigen::ComputeFullV);
    const Vec velocity=svd.solve(d.omega); Mat jp,jm;
    if(!m.jacobian(eta+1e-4*velocity,jp)||!m.jacobian(eta-1e-4*velocity,jm)) return false;
    const Vec acceleration=svd.solve(d.acceleration-(jp-jm)/(2e-4)*velocity);
    path.knots.push_back({i*dt,eta,velocity,acceleration});
  }
  if(!validatePath(m,path,c,error)) { error="interpolation: "+error; return false; }
  return cachePathTravel(path);
}
}
bool inverseOrientation(const Kinematics& model,const Mat& desired,const Vec& seed,
                        const PlanningConfig& config,Vec& q,std::string& error)
{
  error.clear(); q=seed;
  if(!rotationValid(desired)||!seed.allFinite()) { error="invalid IK input"; return false; }
  for(int iteration=0;iteration<40;++iteration) {
    Pose body,cog; Mat j;
    if(!model.evaluate(q,body,cog)||!model.jacobian(q,j)) { error="IK model failure"; return false; }
    Eigen::JacobiSVD<Mat> svd(j,Eigen::ComputeFullU|Eigen::ComputeFullV);
    if(svd.singularValues().minCoeff()<config.min_singular_value) { error="singular orientation IK"; return false; }
    const Vec residual=logSO3(body.linear().transpose()*desired);
    if(residual.norm()<1e-9) return mechanismAllowed(model,q,config,error);
    // Damped inverse with bounded iteration length and nearest-seed branches.
    const auto singular=svd.singularValues();
    const Vec inverse=singular.array()/(singular.array().square()+1e-10);
    Vec delta=svd.matrixV()*inverse.asDiagonal()*svd.matrixU().transpose()*residual;
    if(delta.norm()>.08) delta*=.08/delta.norm();
    q+=delta;
    for(int axis=0;axis<3;++axis) q[axis]=seed[axis]+std::remainder(q[axis]-seed[axis],2*M_PI);
  }
  error="orientation IK did not converge in 40 iterations"; return false;
}
bool mechanismAllowed(const Kinematics& m,const Vec& q,const PlanningConfig& c,std::string& error)
{
  Pose b,g;
  if(!c.mechanismValid()||!q.allFinite()||!m.evaluate(q,b,g)) { error="invalid mechanism configuration/FK"; return false; }
  if((q.array()<c.limits.lower.array()-1e-9).any()||(q.array()>c.limits.upper.array()+1e-9).any())
    { error="mechanism position limit [alpha, roll, yaw]: q="+std::to_string(q[0])+","+std::to_string(q[1])+","+std::to_string(q[2]); return false; }
  if(b.linear()(2,2)<std::cos(c.max_tilt)||!fcAttitudeAllowed(g.linear()))
    { error="body/CoG attitude limit"; return false; }
  return true;
}
bool mechanismReference(const Kinematics& m,const Vec& q,const Vec& v,const Vec& a,Reference& r)
{
  r=Reference(); r.eta=q; r.velocity=v; r.acceleration=a;
  if(!q.allFinite()||!v.allFinite()||!a.allFinite()||!m.evaluate(q,r.body,r.cog)) return false;
  r.desired.rotation=r.body.linear();
  // Differentiate FK along the interpolated configuration, including articulated
  // CoG translation and any configuration-dependent CoG orientation callback.
  const double h=1e-4;
  Pose bp,gp,bm,gm;
  if(!m.evaluate(r.eta+h*r.velocity+.5*h*h*r.acceleration,bp,gp)||
     !m.evaluate(r.eta-h*r.velocity+.5*h*h*r.acceleration,bm,gm)) return false;
  const Vec wb_p=logSO3(r.body.linear().transpose()*bp.linear());
  const Vec wb_m=logSO3(r.body.linear().transpose()*bm.linear());
  const Vec wg_p=logSO3(r.cog.linear().transpose()*gp.linear());
  const Vec wg_m=logSO3(r.cog.linear().transpose()*gm.linear());
  r.body_omega=(wb_p-wb_m)/(2*h); r.body_acceleration=(wb_p+wb_m)/(h*h);
  r.cog_omega=(wg_p-wg_m)/(2*h); r.cog_angular_acceleration=(wg_p+wg_m)/(h*h);
  r.cog_velocity=(gp.translation()-gm.translation())/(2*h);
  r.cog_acceleration=(gp.translation()+gm.translation()-2*r.cog.translation())/(h*h);
  return r.body_omega.allFinite()&&r.body_acceleration.allFinite()&&r.cog_velocity.allFinite()&&
    r.cog_acceleration.allFinite()&&r.cog_omega.allFinite()&&r.cog_angular_acceleration.allFinite();
}

bool cachePathTravel(Path& path)
{
  path.travel.assign(path.knots.size(),Vec::Zero());
  for(std::size_t k=1;k<path.knots.size();++k) {
    const double dt=path.knots[k].t-path.knots[k-1].t;
    if(!std::isfinite(dt)||dt<=0) return false;
    const auto p=controls(path.knots[k-1],path.knots[k]);
    Vec bound=Vec::Zero();
    for(int i=0;i<5;++i) bound=bound.cwiseMax((5*(p[i+1]-p[i])).cwiseAbs());
    path.travel[k]=path.travel[k-1]+bound;
  }
  return path.knots.size()>=2;
}
Vec pathTravel(const Path& p,double t)
{
  if(p.travel.size()!=p.knots.size()||p.knots.size()<2) return Vec::Constant(std::numeric_limits<double>::infinity());
  t=std::clamp(t,0.0,p.duration());
  auto it=std::upper_bound(p.knots.begin(),p.knots.end(),t,[](double x,const Knot& k){return x<k.t;});
  std::size_t k=std::min<std::size_t>(p.knots.size()-1,std::max<std::size_t>(1,it-p.knots.begin()));
  const double u=(t-p.knots[k-1].t)/(p.knots[k].t-p.knots[k-1].t);
  return p.travel[k-1]+u*(p.travel[k]-p.travel[k-1]);
}
void scaleReference(Reference& r,double v,double a)
{
  r.acceleration=r.acceleration*v*v+r.velocity*a;
  r.body_acceleration=r.body_acceleration*v*v+r.body_omega*a;
  r.cog_acceleration=r.cog_acceleration*v*v+r.cog_velocity*a;
  r.cog_angular_acceleration=r.cog_angular_acceleration*v*v+r.cog_omega*a;
  r.desired.acceleration=r.desired.acceleration*v*v+r.desired.omega*a;
  r.velocity*=v; r.body_omega*=v; r.cog_velocity*=v; r.cog_omega*=v; r.desired.omega*=v;
}
bool samplePath(const Kinematics& m,const Path& p,double t,Reference& r)
{
  Vec q,v,a;
  if(!coordinates(p,t,q,v,a)||!p.desired||!mechanismReference(m,q,v,a,r)) return false;
  r.t=std::max(0.0,std::min(t,p.duration())); r.desired=p.desired(r.t);
  r.orientation_error=logSO3(r.body.linear().transpose()*r.desired.rotation).norm();
  return true;
}
bool mechanismPath(const Kinematics& model,const Reference& from,const Vec& goal,
                   const PlanningConfig& config,Path& path,std::string& error)
{
  if(!config.limits.valid()) { error="invalid manual rate bounds"; return false; }
  const Vec distance=(goal-from.eta).cwiseAbs();
  double duration=std::max(.1,std::max((2*distance.cwiseQuotient(config.limits.velocity)).maxCoeff(),
    (8*distance.cwiseQuotient(config.limits.acceleration)).array().sqrt().maxCoeff()));
  for(int attempt=0;attempt<6;++attempt,duration*=1.5) {
    path.knots={{0,from.eta,from.velocity,from.acceleration},{duration,goal,Vec::Zero(),Vec::Zero()}};
    const auto points=path.knots;
    path.desired=[model,points](double t) {
      Path p; p.knots=points; Vec q,v,a; Reference ref;
      if(!coordinates(p,t,q,v,a)||!mechanismReference(model,q,v,a,ref)) {
        Desired bad; bad.rotation.setConstant(std::numeric_limits<double>::quiet_NaN()); return bad;
      }
      return Desired{ref.body.linear(),ref.body_omega,ref.body_acceleration};
    };
    // Manual motion does not depend on IK, a saw model, or planning availability.
    PlanningConfig manual=config; manual.min_singular_value=0;
    manual.orientation_tolerance=1e-6; manual.rate_tolerance=1e-5;
    manual.dt=.1; // bounded sampling plus exact Bernstein joint bounds
    if(validatePath(model,path,manual,error)) return cachePathTravel(path);
  }
  path=Path(); return false;
}
Plan prepare(const Kinematics& model,const Vec& initial,const Mat& neutral,const Request& request,const PlanningConfig& c)
{
  Plan out; out.model=model; out.goal=request; out.neutral=neutral; out.reason="invalid request/configuration";
  if(!request.valid()||!c.valid()||!rotationValid(neutral)||!initial.allFinite()) return out;
  Pose body,cog; if(!model.evaluate(initial,body,cog)) { out.reason="invalid initial FK"; return out; }
  if(!request.align_only && (request.yaw!=0||request.roll!=0||request.pitch_start!=0||
      logSO3(body.linear().transpose()*neutral).norm()>c.orientation_tolerance)) {
    out.reason="cut must start at measured R0; no implicit alignment allowed"; return out;
  }
  if(!request.align_only&&!c.saw_point_valid) {
    out.reason="downward feed UNVERIFIED: missing rigid saw-link origin in controlled body frame"; return out;
  }
  const Mat end=cuttingOrientation(neutral,request,request.pitch_start,0,0).rotation;
  const Vec delta=logSO3(body.linear().transpose()*end);
  for(int attempt=0;attempt<c.scaling_attempts;++attempt) {
    if(c.cancelled&&c.cancelled()) { out.reason="planning cancelled/deadline exceeded"; return out; }
    const double scale=std::pow(2.0,attempt); out.scale_attempt=attempt;
    const double time=std::max(1.0,std::max(1.875*delta.norm()/c.alignment_rate,
                                         std::sqrt(5.8*delta.norm()/c.alignment_acceleration)))*scale;
    auto align=[start=Mat(body.linear()),delta,time](double t) {
      const double s=std::max(0.0,std::min(1.0,t/time));
      const double a=10*s*s*s-15*std::pow(s,4)+6*std::pow(s,5);
      const double v=30*s*s*(1-s)*(1-s)/time, acc=60*s*(1-s)*(1-2*s)/(time*time);
      return Desired{start*expSO3(delta*a),delta*v,delta*acc};
    };
    if(!buildPath(model,initial,time,align,c,out.align,out.reason)) continue;
    if(request.align_only) {
      out.ok=true; out.reason="body alignment geometry prepared"; return out;
    }
    Stroke stroke(request.pitch_end-request.pitch_start,request.rate/scale,request.acceleration/(scale*scale));
    auto cut=[neutral,request,stroke](double t) {
      double theta,v,a; stroke.sample(t,theta,v,a);
      return cuttingOrientation(neutral,request,request.pitch_start+theta,v,a);
    };
    if(!buildPath(model,out.align.knots.back().eta,stroke.duration(),cut,c,out.cut,out.reason)) continue;
    // Check the actual coordinated proxy trajectory, including interpolation.
    // A curved path is allowed; sustained upward travel is not a downward feed.
    const double start_z=(body*c.saw_point).z(); double previous_z=start_z;
    const int samples=std::max(2,static_cast<int>(std::ceil(out.cut.duration()/(c.dt/4))));
    bool downward=true;
    for(int i=1;i<=samples;++i) {
      if(c.cancelled&&c.cancelled()) { out.reason="proxy validation cancelled"; return out; }
      Reference ref;
      if(!samplePath(model,out.cut,out.cut.duration()*i/samples,ref)) { downward=false; break; }
      const double z=(ref.body*c.saw_point).z();
      if(z>previous_z+1e-8) {
        out.reason="upward saw-origin proxy motion at sample="+std::to_string(i)+
          "; requested sign retained; choose another extent/alignment"; downward=false; break;
      }
      previous_z=z;
    }
    out.proxy_drop=start_z-previous_z;
    if(!downward||out.proxy_drop<=1e-6) {
      if(downward) out.reason="no downward saw-origin proxy displacement";
      out.align.knots.clear(); out.cut.knots.clear(); return out;
    }
    out.ok=true; out.reason="geometry prepared: R0 * Ry(d); saw-origin drop="+
      std::to_string(out.proxy_drop)+" m; physical tooth contact unverified"; return out;
  }
  out.align.knots.clear(); out.cut.knots.clear(); return out;
}
const char* stageName(Stage s)
{
  static const char* names[]={"FREE_FLIGHT","CONTACT_LOCKED","PLANNING","PRE_ALIGN","READY","EXECUTING","PAUSED","COMPLETED_HOLD","FAULT_HOLD"};
  return names[static_cast<int>(s)];
}
bool ExecutionConfig::feedbackConfigValid() const
{
  for(double x:{timeout,pivot_tolerance,hinge_tolerance}) if(!std::isfinite(x)||x<=0) return false;
  return std::isfinite(future_tolerance)&&future_tolerance>=0&&future_tolerance<timeout;
}
bool ExecutionConfig::valid() const
{
  for(double x:{tracking_tolerance,orientation_tolerance,reference_lead,progress_filter_time,goal_tolerance,progress_timeout,require_settled_on_enable?settle_rate:1.0})
    if(!std::isfinite(x)||x<=0) return false;
  return feedbackConfigValid()&&std::isfinite(soft_tracking_fraction)&&soft_tracking_fraction>0&&soft_tracking_fraction<.8&&
    std::isfinite(resume_fraction)&&resume_fraction>0&&resume_fraction<1&&
    reference_lead<soft_tracking_fraction*orientation_tolerance&&
    std::isfinite(preparation_tolerance)&&preparation_tolerance>0&&preparation_tolerance<=hinge_tolerance&&
    std::isfinite(preparation_quiet_time)&&preparation_quiet_time>0;

}
bool ExecutionConfig::governorValid(const PlanningConfig& p) const
{
  if(!valid()||!p.valid()||!std::isfinite(max_execution_step)||max_execution_step<=0||max_execution_step>timeout) return false;
  // One maximum cycle of the path's reserved rates must fit between soft and
  // hard bounds. Braking distance itself is reserved by the cached path horizon.
  const Vec cycle=.5*p.limits.velocity*max_execution_step;
  return cycle.maxCoeff()<(1-soft_tracking_fraction)*tracking_tolerance&&
    cycle.sum()<(1-soft_tracking_fraction)*orientation_tolerance;
}
bool ExecutionConfig::settlingValid() const
{
  for(double x:{settle_time,settle_timeout}) if(!std::isfinite(x)||x<=0) return false;
  if(settle_use_window) {
    for(double x:{settle_body_rate,settle_joint_rate,settle_mean_rate})
      if(!std::isfinite(x)||x<=0) return false;
  } else if(!std::isfinite(settle_rate)||settle_rate<=0) return false;
  return settle_timeout>settle_time;
}
std::string feedbackRejection(const Feedback& s,const ExecutionConfig& config,const PlanningConfig& planning)
{
  if(!config.feedbackConfigValid()||!planning.mechanismValid()) return "invalid feedback/mechanism configuration";
  if(!s.hover) return "contact control requires HOVER without forced landing";
  if(!s.valid) return s.invalid_reason.empty()?"invalid/incoherent feedback":s.invalid_reason;
  if(!std::isfinite(s.now)) return "invalid feedback clock";
  auto fresh=[&](double t){return std::isfinite(t)&&t>0&&s.now-t>=-config.future_tolerance&&s.now-t<=config.timeout;};
  auto age_reason=[&](const char* source,double stamp) {
    std::ostringstream out; out<<"stale/future "<<source<<" feedback: age="<<s.now-stamp
      <<" s; allowed=["<<-config.future_tolerance<<","<<config.timeout<<"] s"; return out.str();
  };
  if(!fresh(s.joint_stamp)) return age_reason("joint",s.joint_stamp);
  if(!fresh(s.body_stamp)) return age_reason("body",s.body_stamp);
  if(!poseValid(s.body)) return "invalid body pose/rotation";
  if(!s.eta.allFinite()) return "nonfinite joint positions";
  if(!s.joint_rate.allFinite()||!s.body_omega.allFinite()) return "invalid angular-rate feedback";
  return "";
}
std::string Executor::feedbackRejection(const Feedback& s) const
{ return roll_yaw::feedbackRejection(s,config,planning); }
bool Executor::attach(const Kinematics& captured,const Reference& held,std::uint64_t identity,double now)
{
  std::string error;
  if(!planning.valid()||!config.governorValid(planning)||!config.settlingValid()||
     !std::isfinite(now)||now<=0||!identity||!mechanismAllowed(captured,held.eta,planning,error))
    { reason="invalid explicit planner attachment: "+error; return false; }
  model=captured; target=held; session=identity; stage=Stage::CONTACT_LOCKED;
  hold("explicit body-orientation planning requested");
  last_time_=now; progress=0; resetSettling(); speed_=1;
  return true;
}
bool Executor::feedbackValid(const Feedback& s) const
{ return feedbackRejection(s).empty(); }
bool Executor::capture(const Feedback& s,const Kinematics& m)
{
  if(owns()) { reason="existing contact retained; disable before capture"; return false; }
  const auto rejection=feedbackRejection(s);
  if(!rejection.empty()) { reason=rejection; return false; }
  if(config.require_settled_on_enable&&
     (s.joint_rate.tail<2>().norm()>config.settle_rate||s.body_omega.norm()>config.settle_rate))
  { reason="optional settled-on-enable rate check failed"; return false; }
  Kinematics next=m; Vec q=s.eta; q[0]=0;
  if(!next.lock(s.body,q)) { reason="invalid fixed-pivot model"; return false; }
  std::string why;
  if(!mechanismAllowed(next,q,planning,why)) { reason=why; return false; }
  Reference held; held.eta=q;
  if(!next.evaluate(q,held.body,held.cog)) { reason="invalid captured body/CoG FK"; return false; }
  held.desired.rotation=held.body.linear();
  model=std::move(next); target=held; ++session; plan=Plan();
  first_fault.clear();
  stage=Stage::CONTACT_LOCKED; reason="measured contact captured; N is locked body orientation";
  last_time_=s.now; progress=0; resetSettling(); stopping_=cancel_=false; speed_=1; return true;
}
bool Executor::submissionMatches(const Plan& p,const Feedback& f,const Reference& held) const
{
  Estimate estimate;
  if(p.session!=session||!p.submission_valid||!feedbackValid(f)||
     !model.estimate(f.body,f.eta,estimate)||estimate.pivot_error>config.pivot_tolerance||
     estimate.hinge_residual>config.hinge_tolerance) return false;
  Vec q=f.eta; q[0]=estimate.alpha;
  return (q-p.submission.eta).cwiseAbs().maxCoeff()<=config.goal_tolerance&&
    logSO3(p.submission.body.linear().transpose()*f.body.linear()).norm()<=config.goal_tolerance&&
    (held.eta-p.applied_start.eta).norm()<1e-9&&
    (held.body.matrix()-p.applied_start.body.matrix()).norm()<1e-9&&
    (held.cog.matrix()-p.applied_start.cog.matrix()).norm()<1e-9&&held.velocity.norm()<1e-6;
}
bool Executor::accept(Plan p)
{
  if(stage!=Stage::PLANNING||p.session!=session) return false;
  resetSettling();
  if(!config.governorValid(planning)) { p.ok=false; p.reason="invalid soft/hard tracking or trajectory limits"; }
  if(!p.ok) { stage=Stage::CONTACT_LOCKED; reason=p.reason; return false; }
  if(p.align.knots.size()<2||(p.align.knots.front().eta-target.eta).norm()>1e-9) { fault("plan start no longer matches held target"); return false; }
  if(!cachePathTravel(p.align)||(!p.goal.align_only&&!cachePathTravel(p.cut))) { fault("invalid path braking envelope"); return false; }
  plan=std::move(p); progress=nominal_progress=0; speed_=0; governor_hold_=false; stopping_=cancel_=false;
  task.reset(plan.goal.pitch_start); planned_pitch=plan.goal.pitch_start;
  path_end_commanded=goal_reached=progress_lag=false; lag_since_=0;
  stage=Stage::CONTACT_LOCKED; // Every connection, including a zero-distance one, settles before READY.
  reason=plan.reason;
  return true;
}
bool Executor::command(const std::string& cmd,std::uint64_t id)
{
  if(id!=session||!owns()) { reason="stale or missing lock session"; return false; }
  if(cmd=="prepare"&&(!planning.valid()||!config.governorValid(planning)||!config.settlingValid()))
  { reason="invalid offline planning configuration"; return false; }
  if(cmd=="prepare"&&(stage==Stage::CONTACT_LOCKED||stage==Stage::COMPLETED_HOLD))
  { plan=Plan(); resetSettling(); stage=Stage::PLANNING; reason="preparing from held configuration"; return true; }
  if(cmd=="align"&&stage==Stage::CONTACT_LOCKED&&plan.ok) {
    stage=Stage::PRE_ALIGN; motion_clock_pending_=true; progress=nominal_progress=0; speed_=0; resetSettling();
    reason="aligning along geometrically validated path"; return true;
  }
  if(cmd=="start"&&stage==Stage::READY) {
    if(plan.cut.knots.empty()) { reason="alignment-only plan; prepare a pitch stroke to start"; return false; }
    stage=Stage::EXECUTING; motion_clock_pending_=true; progress=nominal_progress=0; speed_=0; governor_hold_=false; resetSettling();
    task.reset(plan.goal.pitch_start); lag_since_=0; goal_reached=false;
    reason="executing geometrically validated pitch stroke"; return true;
  }
  if((cmd=="pause"||cmd=="cancel")&&(stage==Stage::PRE_ALIGN||stage==Stage::EXECUTING))
  { stopping_=true; cancel_=cmd=="cancel"; resetSettling(); reason="decelerating along validated path"; return true; }
  if(cmd=="resume"&&stage==Stage::PAUSED) {
    stage=paused_align_?Stage::PRE_ALIGN:Stage::EXECUTING; motion_clock_pending_=true; stopping_=false; lag_since_=0; governor_hold_=false; resetSettling();
    reason=paused_align_?"resuming validated alignment":"resuming validated pitch stroke"; return true;
  }
  if(cmd=="cancel"&&(stage==Stage::PAUSED||stage==Stage::PLANNING||stage==Stage::CONTACT_LOCKED||stage==Stage::READY||stage==Stage::COMPLETED_HOLD))
  { plan=Plan(); resetSettling(); stage=Stage::COMPLETED_HOLD; reason="cancelled; holding contact"; return true; }
  reason="command rejected in current trajectory state"; return false;
}
void Executor::fault(const std::string& why)
{
  const auto last_settling=settling;
  resetSettling();
  settling=last_settling;
  if(owns()) {
    // Preserve the initiating cause for state/UI; diagnostics retain the latest
    // observation in reason even if subsequent tracking checks also fail.
    if(stage!=Stage::FAULT_HOLD) first_fault=why;
    stage=Stage::FAULT_HOLD;
  }
  reason=why; stopping_=cancel_=false; target.velocity.setZero(); target.acceleration.setZero();
  target.body_omega.setZero(); target.body_acceleration.setZero(); target.cog_velocity.setZero(); target.cog_acceleration.setZero();
  target.cog_omega.setZero(); target.cog_angular_acceleration.setZero();
  target.desired={target.body.linear(),Vec::Zero(),Vec::Zero()};
}
void Executor::hold(const std::string& why)
{
  if(!owns()||stage==Stage::FAULT_HOLD) return;
  // Freeze at the applied reference, retaining the contact and lock identity.
  fault(why); first_fault.clear(); stage=Stage::CONTACT_LOCKED;
}
void Executor::reset()
{ stage=Stage::FREE_FLIGHT; ++session; plan=Plan(); first_fault.clear(); reason="perching disabled"; last_time_=0; resetSettling(); }
void Executor::resetSettling()
{ stable_since_=ready_since_=settling_stamp_=0; settling=SettlingStatistics(); settling_window_.reset(); }
void Executor::tick(const Feedback& s)
{
  if(!owns()) return;
  if(!feedbackValid(s)) { fault(feedbackRejection(s)); return; }
  if(!model.estimate(s.body,s.eta,measured)) { fault("passive hinge estimation failed"); return; }
  Vec measured_q=s.eta; measured_q[0]=measured.alpha;
  if((measured_q.array()<planning.limits.lower.array()).any()||
     (measured_q.array()>planning.limits.upper.array()).any()||
     s.body.linear()(2,2)<std::cos(planning.max_tilt))
  { fault("measured mechanism/body outside configured operating range"); return; }
  if(measured.pivot_error>config.pivot_tolerance||measured.hinge_residual>config.hinge_tolerance)
  { std::ostringstream why; why<<"pivot="<<measured.pivot_error<<"/"<<config.pivot_tolerance
      <<" m; hinge="<<measured.hinge_residual<<"/"<<config.hinge_tolerance<<" rad"; fault(why.str()); return; }
  const Reference& tracking=applied_valid?applied:target;
  const double joint_error=(measured_q-tracking.eta).cwiseAbs().maxCoeff();
  const double body_error=logSO3(s.body.linear().transpose()*tracking.body.linear()).norm();
  if(joint_error>config.tracking_tolerance||body_error>config.orientation_tolerance)
  { std::ostringstream why; why<<"effective tracking: mechanism="<<joint_error<<"/"<<config.tracking_tolerance
      <<" rad; body="<<body_error<<"/"<<config.orientation_tolerance<<" rad"; fault(why.str()); return; }
  const double dt=s.now-last_time_; last_time_=s.now;
  const bool moving=stage==Stage::PRE_ALIGN||stage==Stage::EXECUTING;
  if(dt<0) { fault("execution clock moved backwards"); return; }
  if(moving&&motion_clock_pending_) { motion_clock_pending_=false; return; }
  if(moving&&(!std::isfinite(dt)||dt<0||dt>config.max_execution_step)) { fault("invalid execution clock step"); return; }
  if(moving&&dt==0) return;
  if(stage==Stage::FAULT_HOLD) return;
  if(plan.ok&&!plan.goal.align_only) {
    const Mat pre=cuttingOrientation(plan.neutral,plan.goal,0,0,0).rotation;
    if(!task.observe(pre,s.body.linear(),s.body_stamp,plan.goal.pitch_start,plan.goal.pitch_end,
                     config.progress_filter_time,config.orientation_tolerance)) { fault("degenerate measured task twist"); return; }
    goal_error=plan.goal.pitch_end-task.raw;
    goal_orientation_error=logSO3(s.body.linear().transpose()*cuttingOrientation(plan.neutral,plan.goal,plan.goal.pitch_end,0,0).rotation).norm();
  }
  if(plan.ok&&(stage==Stage::READY||stage==Stage::CONTACT_LOCKED)&&!plan.goal.align_only&&
     ((measured_q-(plan.measured_start_valid?plan.measured_start.eta:plan.cut.knots.front().eta)).cwiseAbs().maxCoeff()>config.goal_tolerance||
      logSO3(s.body.linear().transpose()*plan.neutral).norm()>config.preparation_tolerance||
      (plan.measured_start_valid&&logSO3(s.body.linear().transpose()*plan.measured_start.body.linear()).norm()>config.goal_tolerance))) {
    plan=Plan(); hold("prepared start moved; prepare again"); return;
  }
  if(stage!=Stage::PRE_ALIGN&&stage!=Stage::EXECUTING) return;
  const auto& path=stage==Stage::EXECUTING?plan.cut:plan.align;
  if(!plan.ok||plan.session!=session) { fault("trajectory ownership/session mismatch"); return; }
  const double speed_acc=std::min((planning.limits.acceleration.array()/planning.limits.velocity.array()).minCoeff(),
                                planning.cog_acceleration_limit/planning.cog_speed_limit)*.5;
  double phase_limit=path.duration();
  if(stage==Stage::EXECUTING) {
    const double direction=plan.goal.pitch_end>=plan.goal.pitch_start?1:-1;
    const double allowed=std::min(std::abs(plan.goal.pitch_end-plan.goal.pitch_start),task.distance+config.reference_lead);
    // Invert the monotonic nominal pitch map; no new planning in offline mode.
    double lo=progress,hi=path.duration();
    const Mat pre=cuttingOrientation(plan.neutral,plan.goal,0,0,0).rotation;
    for(int i=0;i<32;++i) {
      const double mid=(lo+hi)*.5; double pitch,cross;
      if(!taskPitch(pre,path.desired(mid).rotation,plan.goal.pitch_start,pitch,cross)) { fault("invalid nominal pitch map"); return; }
      if(direction*(pitch-plan.goal.pitch_start)>allowed+1e-12) hi=mid; else lo=mid;
    }
    phase_limit=lo;
  }
  // Joint variation bounds also bound body SO(3) distance by their sum:
  // all three coordinates are revolute, including the configured passive axis.
  // Searching this cached envelope is logarithmic; no FK/search in the callback.
  const Vec origin=pathTravel(path,progress);
  double soft_joint=config.tracking_tolerance*config.soft_tracking_fraction;
  double soft_body=config.orientation_tolerance*config.soft_tracking_fraction;
  // A preparation connection can begin with a valid noisy residual above the
  // nominal soft band. Permit approach toward measured feedback, while retaining
  // one whole maximum cycle below BOTH hard faults. Never lift cut soft bands.
  if(stage==Stage::PRE_ALIGN) {
    const double endpoint_joint=(measured_q-path.knots.back().eta).cwiseAbs().maxCoeff();
    const double endpoint_body=logSO3(s.body.linear().transpose()*path.desired(path.duration()).rotation).norm();
    if(endpoint_joint<joint_error&&joint_error>soft_joint)
      soft_joint=std::max(soft_joint,config.tracking_tolerance-.5*planning.limits.velocity.maxCoeff()*config.max_execution_step-1e-6);
    if(endpoint_body<body_error&&body_error>soft_body)
      soft_body=std::max(soft_body,config.orientation_tolerance-.5*planning.limits.velocity.sum()*config.max_execution_step-1e-6);
  }
  double lo=progress,hi=phase_limit;
  for(int i=0;i<32;++i) {
    const double mid=(lo+hi)*.5;
    const Vec travel=pathTravel(path,mid)-origin;
    if((travel.array()+ (measured_q-tracking.eta).cwiseAbs().array()>soft_joint).any()||
       travel.sum()+body_error>soft_body) hi=mid; else lo=mid;
  }
  phase_limit=lo;
  const double gap=std::max(0.0,phase_limit-progress);
  if(governor_hold_&&joint_error<soft_joint*config.resume_fraction&&body_error<soft_body*config.resume_fraction)
    governor_hold_=false;
  if(speed_<.01&&gap<1e-4&&phase_limit<path.duration()-1e-7) governor_hold_=true;
  // Admittance changes only the scalar destination on the prepared path. The
  // nominal clock is capped by measured lead and by the bounded scalar offset;
  // it cannot accumulate time behind a blockage. Effective progress may stop,
  // then recover gradually. It never advances faster than the original path.
  double effective_limit=phase_limit;
  if(stage==Stage::EXECUTING) {
    const Mat pre=cuttingOrientation(plan.neutral,plan.goal,0,0,0).rotation;
    const double direction=plan.goal.pitch_end>=plan.goal.pitch_start?1:-1;
    auto pitch=[&](double t) { double p=0,c=0; taskPitch(pre,path.desired(t).rotation,plan.goal.pitch_start,p,c); return direction*(p-plan.goal.pitch_start); };
    if(!stopping_&&!interaction_hold&&!governor_hold_) {
      nominal_progress=std::max(nominal_progress,std::min(phase_limit,nominal_progress+dt));
    }
    const double wanted=std::max(0.0,pitch(nominal_progress)+std::min(0.0,relief));
    double a=progress,b=phase_limit;
    for(int i=0;i<32;++i) { double m=(a+b)*.5; if(pitch(m)>wanted) b=m; else a=m; }
    // The cosine endpoint is flat: inverting a sub-nanoradian spring tail
    // otherwise leaves a finite phase gap forever. Only numerical-size relief
    // can use the exact endpoint; measured completion remains mandatory.
    effective_limit=pitch(path.duration())-wanted<=1e-10?phase_limit:a;
  }
  double new_speed=(stopping_||interaction_hold||governor_hold_)?std::max(0.0,speed_-speed_acc*dt):std::min(1.0,speed_+speed_acc*dt);
  // Braking distance includes this trapezoidal step. Never clamp phase after
  // integration: doing so would publish derivatives inconsistent with position.
  if(effective_limit<path.duration()-1e-7) {
    const double gap=std::max(0.0,effective_limit-progress),a=speed_acc*dt;
    // Reserve the final discrete braking step as well as continuous stopping
    // distance, for any allowed future update interval. No phase clamp/jump.
    const double b=speed_acc*(dt+config.max_execution_step);
    const double safe=std::max(0.0,(-b+std::sqrt(std::max(0.0,b*b-4*a*speed_+8*speed_acc*gap)))*.5);
    new_speed=std::max(0.0,std::max(speed_-a,std::min(new_speed,safe)));
  }
  progress_lag=!stopping_&&(governor_hold_||interaction_hold||new_speed<.01)&&progress<path.duration()-1e-7;
  if(progress_lag) {
    if(lag_since_==0) lag_since_=s.now;
    reason="bounded progress hold; original goal pending";
    if(s.now-lag_since_>config.progress_timeout) {
      stopping_=true; cancel_=false; reason="blockage/input timeout; braking to PAUSED, original goal pending";
    }
  } else lag_since_=0;
  const double ds=(new_speed-speed_)/dt;
  progress=std::min(path.duration(),progress+(speed_+new_speed)*.5*dt); speed_=new_speed;
  Reference next;
  if(!samplePath(model,path,progress,next)) { fault("invalid trajectory sample"); return; }
  if(next.body.linear()(2,2)<std::cos(planning.max_tilt)||!fcAttitudeAllowed(next.cog.linear()))
  { fault("trajectory sample exceeds body/FC attitude limits"); return; }
  scaleReference(next,speed_,speed_==0?0:ds);
  target=next;
  if(stage==Stage::EXECUTING) {
    double cross;
    taskPitch(cuttingOrientation(plan.neutral,plan.goal,0,0,0).rotation,next.desired.rotation,planned_pitch,planned_pitch,cross);
    path_end_commanded=progress>=path.duration()-1e-7;
  }
  // Reaching the path's zero-rate endpoint also completes a stop. Otherwise an
  // automatic alignment could enter READY before deceleration finished and leave
  // a pending cancel/disable stranded there.
  if(stopping_&&(speed_==0||progress>=path.duration())) {
    resetSettling();
    paused_align_=stage==Stage::PRE_ALIGN; stage=cancel_?Stage::COMPLETED_HOLD:Stage::PAUSED;
    // A hold must not retain the final deceleration feed-forward indefinitely.
    target.velocity.setZero(); target.acceleration.setZero();
    target.body_omega.setZero(); target.body_acceleration.setZero();
    target.cog_velocity.setZero(); target.cog_acceleration.setZero();
    target.cog_omega.setZero(); target.cog_angular_acceleration.setZero();
    target.desired={target.body.linear(),Vec::Zero(),Vec::Zero()};
    if(cancel_) plan=Plan(); stopping_=cancel_=false; reason="stopped at contact-consistent reference"; return;
  }
  if(progress>=path.duration()-1e-7) {
    if(ready_since_==0) ready_since_=s.now;
    bool qualified=false;
    std::ostringstream detail;
    if(config.settle_use_window) {
      // Do not lose faster-body excursions between distinct joint timestamps.
      if(s.body_omega.norm()>=config.settle_body_rate||
         s.joint_rate.tail<2>().norm()>=config.settle_joint_rate) settling_window_.reset();
      const double sample_stamp=std::min(s.body_stamp,s.joint_stamp);
      const auto stats=settling_window_.observe(sample_stamp,
        {{s.body_omega.x(),s.body_omega.y(),s.body_omega.z()}},
        {{s.joint_rate[1],s.joint_rate[2]}},config.settle_time,config.timeout);
      settling_stamp_=sample_stamp;
      settling=stats;
      qualified=stats.qualifies(config.settle_body_rate,config.settle_joint_rate,config.settle_mean_rate);
      detail<<"window "<<stats.observed<<"/"<<config.settle_time<<" s ("<<stats.status<<")";
      if(stats.complete) {
        detail<<"; body RMS="<<stats.body_rms<<" joint RMS="<<stats.joint_rms
              <<"; body mean="<<stats.body_mean<<" joint mean="<<stats.joint_mean;
        if(stats.body_rms>=config.settle_body_rate) detail<<"; body RMS exceeds bound";
        if(stats.joint_rms>=config.settle_joint_rate) detail<<"; joint RMS exceeds bound";
        if(stats.body_mean>=config.settle_mean_rate) detail<<"; body mean drift exceeds bound";
        if(stats.joint_mean>=config.settle_mean_rate) detail<<"; joint mean drift exceeds bound";
      }
      detail<<"; current body="<<stats.body_current<<" joint="<<stats.joint_current
            <<"; bounds body="<<config.settle_body_rate<<" joint="<<config.settle_joint_rate
            <<" mean="<<config.settle_mean_rate<<" rad/s";
      if(stats.body_current>=config.settle_body_rate) detail<<"; current body rate exceeds bound";
      if(stats.joint_current>=config.settle_joint_rate) detail<<"; current joint rate exceeds bound";
    } else {
      const double body_rate=s.body_omega.norm(),joint_rate=s.joint_rate.tail<2>().norm();
      const bool settled=joint_rate<config.settle_rate&&body_rate<config.settle_rate;
      const double sample_stamp=std::min(s.body_stamp,s.joint_stamp);
      if(sample_stamp<settling_stamp_||sample_stamp-settling_stamp_>config.timeout) stable_since_=0;
      settling_stamp_=sample_stamp;
      if(!settled) stable_since_=0; else if(stable_since_==0) stable_since_=sample_stamp;
      qualified=stable_since_>0&&sample_stamp-stable_since_>=config.settle_time;
      detail<<"strict quiet time "<<(stable_since_>0?sample_stamp-stable_since_:0)<<"/"<<config.settle_time
            <<" s; current body="<<body_rate<<" joint="<<joint_rate<<"; bound="<<config.settle_rate<<" rad/s";
    }
    const bool at_goal=stage==Stage::PRE_ALIGN ?
      (logSO3(s.body.linear().transpose()*path.desired(path.duration()).rotation).norm()<=config.goal_tolerance&&
       (measured_q-path.knots.back().eta).cwiseAbs().maxCoeff()<=config.goal_tolerance) : (task.valid&&std::abs(goal_error)<=config.goal_tolerance&&
      goal_orientation_error<=config.goal_tolerance&&
      (measured_q-plan.cut.knots.back().eta).cwiseAbs().maxCoeff()<=config.tracking_tolerance);
    if(!at_goal) { qualified=false; settling_window_.reset(); settling=SettlingStatistics(); settling_stamp_=0; stable_since_=0; detail<<"; original measured goal pending; pitch error="<<goal_error<<" full SO(3) error="<<goal_orientation_error; }
    settling_elapsed=s.now-ready_since_;
    if(settling_elapsed>config.settle_timeout) { fault("settling timed out: "+detail.str()); return; }
    if(qualified) {
      if(stage==Stage::PRE_ALIGN&&!plan.goal.align_only) {
        plan.measured_start.eta=measured_q; plan.measured_start.body=s.body; plan.measured_start_valid=true;
      }
      stage=stage==Stage::PRE_ALIGN?Stage::READY:Stage::COMPLETED_HOLD;
      goal_reached=stage==Stage::COMPLETED_HOLD;
      reason=stage==Stage::READY?"MOTION_COMPLETE: alignment settled; READY for next command":"GOAL_REACHED: measured original endpoint settled; holding contact";
      resetSettling();
    } else reason="waiting for measured settling: "+detail.str();
  }
}
}
