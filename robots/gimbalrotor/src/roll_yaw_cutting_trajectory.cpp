#include <gimbalrotor/roll_yaw_cutting_trajectory.h>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>

namespace roll_yaw
{
bool Request::valid() const
{
  for(double x:{yaw,roll,pitch_start,pitch_end,rate,acceleration}) if(!std::isfinite(x)) return false;
  return rate>0 && acceleration>0 && std::abs(yaw)<=M_PI && std::abs(roll)<M_PI/2 &&
    std::abs(pitch_start)<M_PI && std::abs(pitch_end)<M_PI && std::abs(pitch_end-pitch_start)>1e-8;
}
bool PlanningConfig::valid() const
{
  for(double x:{dt,max_tilt,orientation_tolerance,rate_tolerance,alignment_rate,alignment_acceleration,
                max_duration,cog_speed_limit,cog_acceleration_limit}) if(!std::isfinite(x)||x<=0) return false;
  return limits.valid()&&limits.lower[0]>-M_PI&&limits.upper[0]<M_PI&&
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
using Controls=std::array<Vec,6>;
Controls controls(const Knot& a,const Knot& b)
{
  const double dt=b.t-a.t;
  return {{a.eta,a.eta+a.velocity*dt/5,a.eta+2*a.velocity*dt/5,
           b.eta-2*b.velocity*dt/5,b.eta-b.velocity*dt/5,b.eta}};
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
  { error="body world tilt exceeds validated operating range"; return false; }
  if(Eigen::JacobiSVD<Mat>(j).singularValues().minCoeff()<c.qp.min_singular_value)
  { error="singular/ill-conditioned mechanism"; return false; }
  if(logSO3(b.linear().transpose()*des.rotation).norm()>c.orientation_tolerance)
  { error="FK orientation tracking tolerance exceeded"; return false; }
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
  for(std::size_t k=1;k<path.knots.size();++k) {
    const auto p=controls(path.knots[k-1],path.knots[k]); std::array<Vec,5> v;
    for(const auto& q:p) if((q.array()<c.limits.lower.array()).any()||(q.array()>c.limits.upper.array()).any())
      { error="interpolated position limit"; return false; }
    for(int i=0;i<5;++i) {
      v[i]=5*(p[i+1]-p[i])/dt;
      if((v[i].cwiseAbs().array()>c.limits.velocity.array()*.5+1e-9).any())
        { error="interpolated velocity reserve exceeded; time scaling required"; return false; }
    }
    for(int i=0;i<4;++i) if(( (4*(v[i+1]-v[i])/dt).cwiseAbs().array()>c.limits.acceleration.array()*.5+1e-9).any())
      { error="interpolated acceleration reserve exceeded; time scaling required"; return false; }
    for(int i=0;i<=4;++i) {
      Reference ref; const double t=path.knots[k-1].t+dt*i/4;
      if(!samplePath(m,path,t,ref)||!configurationAllowed(m,ref.eta,ref.desired,c,error)) return false;
      if((ref.body_omega-ref.body.linear().transpose()*ref.desired.rotation*ref.desired.omega).norm()>c.rate_tolerance)
        { error="body angular velocity tracking tolerance exceeded"; return false; }
      if(ref.cog_velocity.norm()>c.cog_speed_limit*.5||ref.cog_acceleration.norm()>c.cog_acceleration_limit*.5)
        { error="CoG speed/acceleration limit"; return false; }
    }
  }
  return true;
}
}
bool samplePath(const Kinematics& m,const Path& p,double t,Reference& r)
{
  if(!coordinates(p,t,r.eta,r.velocity,r.acceleration)||!p.desired||!m.evaluate(r.eta,r.body,r.cog)) return false;
  r.t=std::max(0.0,std::min(t,p.duration())); r.desired=p.desired(r.t);
  r.orientation_error=logSO3(r.body.linear().transpose()*r.desired.rotation).norm();
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
Plan prepare(const Kinematics& model,const Vec& initial,const Mat& neutral,const Request& request,const PlanningConfig& c)
{
  Plan out; out.model=model; out.reason="invalid request/configuration";
  if(!request.valid()||!c.valid()||!rotationValid(neutral)||!initial.allFinite()) return out;
  Pose body,cog; if(!model.evaluate(initial,body,cog)) { out.reason="invalid initial FK"; return out; }
  const Mat end=cuttingOrientation(neutral,request,request.pitch_start,0,0).rotation;
  const Vec delta=logSO3(body.linear().transpose()*end);
  for(int attempt=0;attempt<c.scaling_attempts;++attempt) {
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
  static const char* names[]={"FREE_FLIGHT","CONTACT_LOCKED","REZERO","PRE_ALIGN","READY","EXECUTING","PAUSED","COMPLETED_HOLD","FAULT_HOLD"};
  return names[static_cast<int>(s)];
}
bool ExecutionConfig::valid() const
{
  for(double x:{timeout,pivot_tolerance,hinge_tolerance,tracking_tolerance,orientation_tolerance,
                settle_rate,settle_time,settle_timeout}) if(!std::isfinite(x)||x<=0) return false;
  return std::isfinite(future_tolerance)&&future_tolerance>=0&&future_tolerance<timeout&&settle_timeout>settle_time;
}
bool Executor::feedbackValid(const Feedback& s) const
{
  auto fresh=[&](double t){return std::isfinite(t)&&t>0&&s.now-t>=-config.future_tolerance&&s.now-t<=config.timeout;};
  return config.valid()&&planning.valid()&&s.valid&&s.hover&&std::isfinite(s.now)&&fresh(s.joint_stamp)&&fresh(s.body_stamp)&&
    poseValid(s.body)&&s.eta.allFinite()&&s.joint_rate.allFinite()&&s.body_omega.allFinite();
}
bool Executor::capture(const Feedback& s,const Kinematics& m)
{
  if(!feedbackValid(s)||s.joint_rate.tail<2>().norm()>config.settle_rate||s.body_omega.norm()>config.settle_rate)
  { reason="lock requires fresh, valid, settled HOVER feedback"; return false; }
  Kinematics next=m; Vec q=s.eta; q[0]=0;
  if(!next.lock(s.body,q)) { reason="invalid calibrated model"; return false; }
  std::string why;
  if(!configurationAllowed(next,q,Desired{s.body.linear(),Vec::Zero(),Vec::Zero()},planning,why)) { reason=why; return false; }
  model=next; target=Reference(); target.eta=q; model.evaluate(q,target.body,target.cog);
  target.desired.rotation=target.body.linear(); ++session; plan=Plan();
  stage=Stage::CONTACT_LOCKED; reason="measured contact captured; N is locked body orientation";
  last_time_=s.now; progress=stable_since_=ready_since_=0; stopping_=cancel_=false; speed_=1; return true;
}
bool Executor::accept(Plan p)
{
  if(stage!=Stage::REZERO||p.session!=session) return false;
  if(!p.ok) { stage=Stage::CONTACT_LOCKED; reason=p.reason; return false; }
  if((p.align.knots.front().eta-target.eta).norm()>1e-9) { fault("plan start no longer matches held target"); return false; }
  plan=std::move(p); stage=Stage::CONTACT_LOCKED; reason="plan ready; align with current session ID"; return true;
}
bool Executor::command(const std::string& cmd,std::uint64_t id)
{
  if(id!=session||!owns()) { reason="stale or missing lock session"; return false; }
  if(cmd=="prepare"&&(stage==Stage::CONTACT_LOCKED||stage==Stage::COMPLETED_HOLD))
  { plan=Plan(); stage=Stage::REZERO; reason="preparing from held configuration"; return true; }
  if(cmd=="align"&&stage==Stage::CONTACT_LOCKED&&plan.ok) {
    stage=Stage::PRE_ALIGN; progress=0; speed_=1; stable_since_=ready_since_=0; return true;
  }
  if(cmd=="start"&&stage==Stage::READY) {
    stage=Stage::EXECUTING; progress=0; speed_=1; stable_since_=ready_since_=0; return true;
  }
  if((cmd=="pause"||cmd=="cancel")&&(stage==Stage::PRE_ALIGN||stage==Stage::EXECUTING))
  { stopping_=true; cancel_=cmd=="cancel"; reason="decelerating along validated path"; return true; }
  if(cmd=="resume"&&stage==Stage::PAUSED) {
    stage=paused_align_?Stage::PRE_ALIGN:Stage::EXECUTING; stopping_=false; return true;
  }
  if(cmd=="cancel"&&(stage==Stage::PAUSED||stage==Stage::REZERO||stage==Stage::CONTACT_LOCKED||stage==Stage::READY))
  { plan=Plan(); stage=Stage::COMPLETED_HOLD; reason="cancelled; holding contact"; return true; }
  reason="command rejected in current trajectory state"; return false;
}
void Executor::fault(const std::string& why)
{
  if(owns()) stage=Stage::FAULT_HOLD;
  reason=why; target.velocity.setZero(); target.acceleration.setZero();
  target.body_omega.setZero(); target.body_acceleration.setZero(); target.cog_velocity.setZero(); target.cog_acceleration.setZero();
  target.cog_omega.setZero(); target.cog_angular_acceleration.setZero();
  target.desired={target.body.linear(),Vec::Zero(),Vec::Zero()};
}
void Executor::reset()
{ stage=Stage::FREE_FLIGHT; ++session; plan=Plan(); reason="perching disabled"; last_time_=0; }
void Executor::tick(const Feedback& s)
{
  if(!owns()) return;
  if(!feedbackValid(s)) { fault("invalid/stale joint or body feedback, or loss of HOVER"); return; }
  if(!model.estimate(s.body,s.eta,measured)) { fault("passive hinge estimation failed"); return; }
  Vec measured_q=s.eta; measured_q[0]=measured.alpha;
  if((measured_q.array()<planning.limits.lower.array()).any()||
     (measured_q.array()>planning.limits.upper.array()).any()||
     s.body.linear()(2,2)<std::cos(planning.max_tilt))
  { fault("measured mechanism/body outside configured operating range"); return; }
  if(measured.pivot_error>config.pivot_tolerance||measured.hinge_residual>config.hinge_tolerance)
  { fault("physical pivot moved or single-axis hinge residual exceeded"); return; }
  if((measured_q-target.eta).cwiseAbs().maxCoeff()>config.tracking_tolerance||
     logSO3(s.body.linear().transpose()*target.body.linear()).norm()>config.orientation_tolerance)
  { fault("mechanism/body tracking error exceeded"); return; }
  const double dt=s.now-last_time_; last_time_=s.now;
  if(!std::isfinite(dt)||dt<=0||dt>config.timeout) { fault("invalid execution clock step"); return; }
  if(stage==Stage::FAULT_HOLD) return;
  if(stage!=Stage::PRE_ALIGN&&stage!=Stage::EXECUTING) return;
  const auto& path=stage==Stage::PRE_ALIGN?plan.align:plan.cut;
  if(!plan.ok||plan.session!=session) { fault("trajectory ownership/session mismatch"); return; }
  const double speed_acc=std::min((planning.limits.acceleration.array()/planning.limits.velocity.array()).minCoeff(),
                                planning.cog_acceleration_limit/planning.cog_speed_limit)*.5;
  const double new_speed=stopping_?std::max(0.0,speed_-speed_acc*dt):std::min(1.0,speed_+speed_acc*dt);
  const double ds=(new_speed-speed_)/dt;
  progress=std::min(path.duration(),progress+(speed_+new_speed)*.5*dt); speed_=new_speed;
  Reference next;
  if(!samplePath(model,path,progress,next)) { fault("invalid trajectory sample"); return; }
  next.acceleration=next.acceleration*speed_*speed_+next.velocity*ds;
  next.body_acceleration=next.body_acceleration*speed_*speed_+next.body_omega*ds;
  next.cog_acceleration=next.cog_acceleration*speed_*speed_+next.cog_velocity*ds;
  next.cog_angular_acceleration=next.cog_angular_acceleration*speed_*speed_+next.cog_omega*ds;
  next.desired.acceleration=next.desired.acceleration*speed_*speed_+next.desired.omega*ds;
  next.velocity*=speed_; next.body_omega*=speed_; next.cog_velocity*=speed_; next.cog_omega*=speed_; next.desired.omega*=speed_;
  target=next;
  if(stopping_&&speed_==0) {
    paused_align_=stage==Stage::PRE_ALIGN; stage=cancel_?Stage::COMPLETED_HOLD:Stage::PAUSED;
    if(cancel_) plan=Plan(); stopping_=cancel_=false; reason="stopped at contact-consistent reference"; return;
  }
  if(progress>=path.duration()) {
    if(ready_since_==0) ready_since_=s.now;
    const bool settled=s.joint_rate.tail<2>().norm()<config.settle_rate&&s.body_omega.norm()<config.settle_rate;
    if(!settled) stable_since_=0; else if(stable_since_==0) stable_since_=s.now;
    if(s.now-ready_since_>config.settle_timeout) { fault("settling timed out"); return; }
    if(stable_since_>0&&s.now-stable_since_>=config.settle_time) {
      stage=stage==Stage::PRE_ALIGN?Stage::READY:Stage::COMPLETED_HOLD;
      reason=stage==Stage::READY?"alignment settled; start with current session ID":"stroke complete; holding contact";
    }
  }
}
}
