#include <gimbalrotor/roll_yaw_cutting_trajectory.h>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <gimbalrotor/roll_yaw_worker.h>

namespace roll_yaw
{
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
    limits.lower[0]<=0&&limits.upper[0]>=0&&qp.valid()&&dt>=.001&&dt<=.1&&max_tilt<M_PI/2&&
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
  { error="mechanism position limit"; return false; }
  if(b.linear()(2,2)<std::cos(c.max_tilt)||des.rotation(2,2)<std::cos(c.max_tilt))
  { error="body world tilt exceeds configured operating range"; return false; }
  if(!fcAttitudeAllowed(g.linear()))
  { error="CoG attitude exceeds Spinal roll/pitch command limits"; return false; }
  if(Eigen::JacobiSVD<Mat>(j).singularValues().minCoeff()<c.qp.min_singular_value)
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
    const auto p=controls(path.knots[k-1],path.knots[k]); std::array<Vec,5> v;
    for(const auto& q:p) if((q.array()<c.limits.lower.array()).any()||(q.array()>c.limits.upper.array()).any())
      { error="interpolated position limit"; return false; }
    for(int i=0;i<5;++i) {
      v[i]=5*(p[i+1]-p[i])/dt;
      if((v[i].cwiseAbs().array()>c.limits.velocity.array()*.5+1e-9).any())
        { error="interpolated velocity reserve exceeded; time scaling required"; return false; }
    }
    for(int i=0;i<4;++i) if(( (4*(v[i+1]-v[i])/dt).cwiseAbs().array()>c.limits.acceleration.array()*.5+1e-9).any())
      { std::ostringstream why; why<<"interpolated acceleration reserve exceeded; interval="<<k
            <<" coefficient="<<i<<" acceleration="<<(4*(v[i+1]-v[i])/dt).transpose()
            <<" limit="<<(c.limits.acceleration*.5).transpose(); error=why.str(); return false; }
    const int samples=std::max(4,static_cast<int>(std::ceil(dt/(c.dt/4))));
    for(int i=0;i<=samples;++i) {
      Reference ref; const double t=path.knots[k-1].t+dt*i/samples;
      if(!samplePath(m,path,t,ref)||!configurationAllowed(m,ref.eta,ref.desired,c,error)) return false;
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
  Vec eta=initial,u=Vec::Zero(); path.knots.push_back({0,eta,u});
  for(int i=0;i<steps;++i) {
    if(c.cancelled&&c.cancelled()) { error="planning cancelled/deadline exceeded"; return false; }
    if(c.report) c.report(i,steps,false);
    const double t=i*dt; Pose b,g; Mat j;
    if(!configurationAllowed(m,eta,desired(t),c,error)||!m.evaluate(eta,b,g)||!m.jacobian(eta,j)) return false;
    // Mid-interval feed-forward reduces Euler integration bias; SO(3) feedback
    // is evaluated at the current sample, with exact box intersection.
    auto d=desired(t); auto mid=desired(t+dt*.5);
    const Vec omega=d.rotation.transpose()*mid.rotation*mid.omega;
    auto qp=orientationStep(j,b.linear(),d.rotation,omega,eta,u,dt,c.limits,c.qp);
    if(!qp.ok) { error=qp.reason; return false; }
    u=qp.u; eta+=u*dt; path.knots.push_back({(i+1)*dt,eta,u});
  }
  // C2 quintic Bezier interpolation. All endpoint accelerations are zero;
  // Bernstein convex-hull bounds certify limits for the WHOLE interval.
  for(std::size_t i=1;i+1<path.knots.size();++i)
    path.knots[i].velocity=(path.knots[i+1].eta-path.knots[i-1].eta)/(2*dt);
  path.knots.front().velocity.setZero(); path.knots.back().velocity.setZero();
  return validatePath(m,path,c,error);
}
}
bool mechanismAllowed(const Kinematics& m,const Vec& q,const PlanningConfig& c,std::string& error)
{
  Pose b,g;
  if(!c.mechanismValid()||!q.allFinite()||!m.evaluate(q,b,g)) { error="invalid mechanism configuration/FK"; return false; }
  if((q.array()<c.limits.lower.array()-1e-9).any()||(q.array()>c.limits.upper.array()+1e-9).any())
    { error="mechanism position limit"; return false; }
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

bool samplePath(const Kinematics& m,const Path& p,double t,Reference& r)
{
  Vec q,v,a;
  if(!coordinates(p,t,q,v,a)||!p.desired||!mechanismReference(m,q,v,a,r)) return false;
  r.t=std::max(0.0,std::min(t,p.duration())); r.desired=p.desired(r.t);
  r.orientation_error=logSO3(r.body.linear().transpose()*r.desired.rotation).norm();
  return true;
}
bool continuation(const Kinematics& model,const Path& nominal,double begin,double horizon,
                  const PlanningConfig& config,Path& result,std::string& reason,const Vec& seed_error)
{
  Reference first,middle,last;
  const double duration=std::min(horizon,nominal.duration()-begin);
  if(duration<2*config.dt||!samplePath(model,nominal,begin,first)||
     !samplePath(model,nominal,begin+duration,last)) { reason="continuation interval unavailable"; return false; }
  const double mid=duration*.5;
  if(!samplePath(model,nominal,begin+mid,middle)) return false;
  // Refresh the interior IK at the unchanged offline planning dt. Keep the
  // nominal polynomial intervals (including exact C2 subdivisions at boundaries)
  // and blend a smooth, measured-state-derived correction inside this horizon.
  Vec q=first.eta+seed_error,u=first.velocity;
  const int steps=std::max(1,static_cast<int>(std::ceil(mid/config.dt)));
  const double dt=mid/steps;
  for(int i=0;i<steps;++i) {
    if(config.cancelled&&config.cancelled()) { reason="continuation cancelled/deadline"; return false; }
    Pose body,cog; Mat j;
    if(!model.evaluate(q,body,cog)||!model.jacobian(q,j)) return false;
    const auto d=nominal.desired(begin+i*dt),next=nominal.desired(begin+(i+.5)*dt);
    auto solved=orientationStep(j,body.linear(),d.rotation,d.rotation.transpose()*next.rotation*next.omega,
                                q,u,dt,config.limits,config.qp);
    if(!solved.ok) { reason=solved.reason; return false; }
    u=solved.u; q+=u*dt;
  }
  result.desired=[source=nominal.desired,begin](double t){return source(begin+t);};
  std::vector<Knot> base={{0,first.eta,first.velocity,first.acceleration}};
  for(const auto& knot:nominal.knots) if(knot.t>begin+1e-9&&knot.t<begin+duration-1e-9)
    base.push_back({knot.t-begin,knot.eta,knot.velocity,knot.acceleration});
  base.push_back({duration,last.eta,last.velocity,last.acceleration});
  for(int attempt=0;attempt<8;++attempt) {
    const double weight=std::pow(.5,attempt); const Vec correction=weight*(q-middle.eta);
    result.knots=base;
    for(auto& knot:result.knots) {
      const double s=knot.t/duration;
      const double b=64*(std::pow(s,3)-3*std::pow(s,4)+3*std::pow(s,5)-std::pow(s,6));
      const double v=64*(3*s*s-12*std::pow(s,3)+15*std::pow(s,4)-6*std::pow(s,5))/duration;
      const double a=64*(6*s-36*s*s+60*std::pow(s,3)-30*std::pow(s,4))/(duration*duration);
      knot.eta+=b*correction; knot.velocity+=v*correction; knot.acceleration+=a*correction;
    }
    if(validatePath(model,result,config,reason)) { reason="validated C2 continuation; IK correction weight="+std::to_string(weight); return true; }
    if(config.cancelled&&config.cancelled()) return false;
  }
  return false;
}

bool nominalMeasurementSeed(const Kinematics& model,const Feedback& measured,const Reference& nominal,
                            const Reference& applied,const PlanningConfig& config,Vec& error,std::string& reason)
{
  Estimate estimate;
  if(!model.estimate(measured.body,measured.eta,estimate)) { reason="invalid measured seed"; return false; }
  Vec q=measured.eta; q[0]=estimate.alpha;
  // Remove the actually applied correction from the measured orientation, not
  // from the goal. Perfect effective tracking must yield zero nominal seed error.
  const Mat desired=measured.body.linear()*applied.body.linear().transpose()*nominal.body.linear();
  for(int i=0;i<8;++i) {
    Pose body,cog; Mat j;
    if(!model.evaluate(q,body,cog)||!model.jacobian(q,j)) return false;
    Eigen::JacobiSVD<Mat> svd(j,Eigen::ComputeFullU|Eigen::ComputeFullV);
    if(svd.singularValues().minCoeff()<config.qp.min_singular_value) { reason="singular measured seed"; return false; }
    const Vec residual=logSO3(body.linear().transpose()*desired);
    if(residual.norm()<1e-8) {
      error=q-nominal.eta; return mechanismAllowed(model,q,config,reason);
    }
    const Vec step=svd.solve(residual);
    if(step.norm()>.2) { reason="measured seed requires excessive IK step"; return false; }
    q+=step;
  }
  reason="measured nominal seed did not converge"; return false;
}

Plan prepare(const Kinematics& model,const Vec& initial,const Mat& neutral,const Request& request,const PlanningConfig& c)
{
  Plan out; out.model=model; out.goal=request; out.neutral=neutral; out.reason="invalid request/configuration";
  if(!request.valid()||!c.valid()||!rotationValid(neutral)||!initial.allFinite()) return out;
  Pose body,cog; if(!model.evaluate(initial,body,cog)) { out.reason="invalid initial FK"; return out; }
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
      out.ok=true; out.reason="validated alignment only"; return out;
    }
    Stroke stroke(request.pitch_end-request.pitch_start,request.rate/scale,request.acceleration/(scale*scale));
    auto cut=[neutral,request,stroke](double t) {
      double theta,v,a; stroke.sample(t,theta,v,a);
      return cuttingOrientation(neutral,request,request.pitch_start+theta,v,a);
    };
    if(!buildPath(model,out.align.knots.back().eta,stroke.duration(),cut,c,out.cut,out.reason)) continue;
    out.ok=true; out.reason="validated alignment and local-body-Y stroke"; return out;
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
  for(double x:{tracking_tolerance,orientation_tolerance,reference_lead,reference_tracking_allowance,progress_filter_time,goal_tolerance,progress_timeout,require_settled_on_enable?settle_rate:1.0})
    if(!std::isfinite(x)||x<=0) return false;
  return feedbackConfigValid();
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
  if(!qp_planning||!planning.valid()||!config.valid()||!config.settlingValid()||
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
bool Executor::accept(Plan p)
{
  if(!qp_planning||stage!=Stage::PLANNING||p.session!=session) return false;
  resetSettling();
  if(!p.ok) { stage=Stage::CONTACT_LOCKED; reason=p.reason; return false; }
  if(p.align.knots.size()<2||(p.align.knots.front().eta-target.eta).norm()>1e-9) { fault("plan start no longer matches held target"); return false; }
  online_path=Path(); plan=std::move(p); progress=0; speed_=1; stopping_=cancel_=false;
  task.reset(plan.goal.pitch_start); planned_pitch=plan.goal.pitch_start;
  path_end_commanded=goal_reached=progress_lag=false; lag_since_=0;
  stage=Stage::CONTACT_LOCKED;
  reason="plan ready; align with current session ID";
  return true;
}
bool Executor::command(const std::string& cmd,std::uint64_t id)
{
  if(id!=session||!owns()) { reason="stale or missing lock session"; return false; }
  if(cmd=="prepare"&&(!qp_planning||!planning.valid()||!config.valid()||!config.settlingValid()))
  { reason=qp_planning?"invalid QP planning configuration":"body-orientation request requires qp_planning=true"; return false; }
  if(cmd=="prepare"&&(stage==Stage::CONTACT_LOCKED||stage==Stage::COMPLETED_HOLD))
  { plan=Plan(); resetSettling(); stage=Stage::PLANNING; reason="preparing from held configuration"; return true; }
  if(cmd=="align"&&stage==Stage::CONTACT_LOCKED&&plan.ok) {
    stage=Stage::PRE_ALIGN; progress=0; speed_=1; resetSettling();
    reason="aligning along validated path"; return true;
  }
  if(cmd=="start"&&stage==Stage::READY) {
    if(plan.cut.knots.empty()) { reason="alignment-only plan; prepare a pitch stroke to start"; return false; }
    stage=Stage::EXECUTING; progress=0; speed_=0; resetSettling();
    task.reset(plan.goal.pitch_start); lag_since_=0; goal_reached=false;
    reason="executing validated pitch stroke"; return true;
  }
  if((cmd=="pause"||cmd=="cancel")&&(stage==Stage::PRE_ALIGN||stage==Stage::EXECUTING))
  { stopping_=true; cancel_=cmd=="cancel"; resetSettling(); reason="decelerating along validated path"; return true; }
  if(cmd=="resume"&&stage==Stage::PAUSED) {
    stage=paused_align_?Stage::PRE_ALIGN:Stage::EXECUTING; stopping_=false; resetSettling();
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
  if(!std::isfinite(dt)||dt<=0||dt>config.timeout) { fault("invalid execution clock step"); return; }
  if(stage==Stage::FAULT_HOLD) return;
  if(plan.ok&&!plan.goal.align_only) {
    const Mat pre=cuttingOrientation(plan.neutral,plan.goal,0,0,0).rotation;
    if(!task.observe(pre,s.body.linear(),s.body_stamp,plan.goal.pitch_start,plan.goal.pitch_end,
                     config.progress_filter_time,config.orientation_tolerance)) { fault("degenerate measured task twist"); return; }
    goal_error=plan.goal.pitch_end-task.raw;
    goal_orientation_error=logSO3(s.body.linear().transpose()*cuttingOrientation(plan.neutral,plan.goal,plan.goal.pitch_end,0,0).rotation).norm();
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
      if(direction*(pitch-plan.goal.pitch_start)>allowed) hi=mid; else lo=mid;
    }
    phase_limit=lo;
  }
  double new_speed=stopping_?std::max(0.0,speed_-speed_acc*dt):std::min(1.0,speed_+speed_acc*dt);
  // Reserve enough phase distance to brake at the bounded phase acceleration.
  // Includes the current trapezoidal integration step (prevents lead overshoot).
  if(stage==Stage::EXECUTING&&phase_limit<path.duration()-1e-7) {
    const double gap=std::max(0.0,phase_limit-progress);
    const double a=speed_acc*dt;
    const double safe=std::max(0.0,(-a+std::sqrt(std::max(0.0,a*a-4*a*speed_+8*speed_acc*gap)))*.5);
    new_speed=std::max(0.0,std::max(speed_-speed_acc*dt,std::min(new_speed,safe)));
  }
  progress_lag=stage==Stage::EXECUTING&&!stopping_&&new_speed<.01&&phase_limit<path.duration()-1e-7;
  if(progress_lag) {
    if(lag_since_==0) lag_since_=s.now;
    reason="measured-progress lag; nominal reference lead consumed";
    if(s.now-lag_since_>config.progress_timeout) { fault("measured-progress lag timeout; original goal pending"); return; }
  } else lag_since_=0;
  const double ds=(new_speed-speed_)/dt;
  progress=std::min(phase_limit,progress+(speed_+new_speed)*.5*dt); speed_=new_speed;
  if(!online_path.knots.empty()&&!online_started) {
    if(s.now-online_stamp>online_activation_timeout) { online_path=Path(); ++online_expired; }
    else if(progress>=online_begin) online_started=true;
  }
  Reference next;
  const bool use_online=stage==Stage::EXECUTING&&!online_path.knots.empty()&&progress>=online_begin&&progress<=online_begin+online_path.duration();
  if(!(use_online?samplePath(model,online_path,progress-online_begin,next):samplePath(model,path,progress,next))) { fault("invalid trajectory sample"); return; }
  if(next.body.linear()(2,2)<std::cos(planning.max_tilt)||!fcAttitudeAllowed(next.cog.linear()))
  { fault("trajectory sample exceeds body/FC attitude limits"); return; }
  next.acceleration=next.acceleration*speed_*speed_+next.velocity*ds;
  next.body_acceleration=next.body_acceleration*speed_*speed_+next.body_omega*ds;
  next.cog_acceleration=next.cog_acceleration*speed_*speed_+next.cog_velocity*ds;
  next.cog_angular_acceleration=next.cog_angular_acceleration*speed_*speed_+next.cog_omega*ds;
  next.desired.acceleration=next.desired.acceleration*speed_*speed_+next.desired.omega*ds;
  next.velocity*=speed_; next.body_omega*=speed_; next.cog_velocity*=speed_; next.cog_omega*=speed_; next.desired.omega*=speed_;
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
      const double sample_stamp=std::min(s.body_stamp,s.joint_stamp);
      const auto stats=sample_stamp>settling_stamp_?settling_window_.observe(sample_stamp,
        {{s.body_omega.x(),s.body_omega.y(),s.body_omega.z()}},
        {{s.joint_rate[1],s.joint_rate[2]}},config.settle_time,config.timeout):settling;
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
      if(!settled) stable_since_=0; else if(stable_since_==0) stable_since_=s.now;
      qualified=stable_since_>0&&s.now-stable_since_>=config.settle_time;
      detail<<"strict quiet time "<<(stable_since_>0?s.now-stable_since_:0)<<"/"<<config.settle_time
            <<" s; current body="<<body_rate<<" joint="<<joint_rate<<"; bound="<<config.settle_rate<<" rad/s";
    }
    const bool at_goal=stage==Stage::PRE_ALIGN || (task.valid&&std::abs(goal_error)<=config.goal_tolerance&&
      goal_orientation_error<=config.goal_tolerance&&
      (measured_q-plan.cut.knots.back().eta).cwiseAbs().maxCoeff()<=config.tracking_tolerance);
    if(!at_goal) { qualified=false; settling_window_.reset(); settling=SettlingStatistics(); settling_stamp_=0; stable_since_=0; detail<<"; original measured goal pending; pitch error="<<goal_error<<" full SO(3) error="<<goal_orientation_error; }
    settling_elapsed=s.now-ready_since_;
    if(settling_elapsed>config.settle_timeout) { fault("settling timed out: "+detail.str()); return; }
    if(qualified) {
      stage=stage==Stage::PRE_ALIGN?Stage::READY:Stage::COMPLETED_HOLD;
      goal_reached=stage==Stage::COMPLETED_HOLD;
      reason=stage==Stage::READY?"alignment settled; start with current session ID":"GOAL_REACHED: measured original endpoint settled; holding contact";
      resetSettling();
    } else reason="waiting for measured settling: "+detail.str();
  }
}
}
