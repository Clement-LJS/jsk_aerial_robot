#include <gimbalrotor/roll_yaw_admittance.h>
#include <Eigen/SVD>

namespace roll_yaw
{
bool AdmittanceConfig::valid() const
{
  for(double x:{inertia,damping,stiffness,torque_limit,angle_limit,rate_limit,acceleration_limit,
                filter_time,on_threshold,on_dwell,off_dwell}) if(!std::isfinite(x)||x<=0) return false;
  return std::isfinite(reference_torque)&&std::isfinite(off_threshold)&&off_threshold>=0&&
    off_threshold<on_threshold&&angle_limit<.5&&baseline_samples>=2&&baseline_samples<=10000;
}
void PitchAdmittance::reset()
{ const auto c=config; *this=PitchAdmittance(); config=c; }
void PitchAdmittance::observe(double torque,double stamp,bool no_cut)
{
  if(!std::isfinite(torque)||!std::isfinite(stamp)||stamp<=stamp_) return;
  const double dt=stamp_>0?stamp-stamp_:0; stamp_=stamp;
  if(!baseline_ready&&dt>.15) { sum_=0; samples=0; }
  if(!baseline_ready&&!enabled&&no_cut) {
    sum_+=torque; ++samples;
    if(samples>=config.baseline_samples) { bias=sum_/samples; baseline_ready=true; }
  }
  if(!baseline_ready||dt<=0||dt>.15) { contact=false; on_=off_=0; return; }
  filtered+=(1-std::exp(-dt/config.filter_time))*(torque-bias-config.reference_torque-filtered);
  if(!enabled) { contact=false; on_=off_=0; return; }
  if(!contact) {
    off_=0; on_=std::abs(filtered)>config.on_threshold?on_+dt:0;
    if(on_>=config.on_dwell) { contact=true; on_=0; }
  } else {
    on_=0; off_=std::abs(filtered)<config.off_threshold?off_+dt:0;
    if(off_>=config.off_dwell) { contact=false; off_=0; }
  }
}
bool PitchAdmittance::step(double dt,bool fresh,bool relief_only)
{
  if(!config.valid()||!std::isfinite(dt)||dt<=0||dt>.15) return false;
  if(!fresh) { contact=false; on_=off_=0; }
  const double torque=enabled&&fresh&&contact?std::clamp(filtered,-config.torque_limit,relief_only?0.0:config.torque_limit):0;
  const double proposed=relief_only&&enabled&&!fresh?0:(config.inertia*rate+dt*(torque-config.stiffness*offset))/
    (config.inertia+dt*config.damping+dt*dt*config.stiffness);
  double v=std::clamp(proposed,rate-config.acceleration_limit*dt,rate+config.acceleration_limit*dt);
  v=std::clamp(v,-config.rate_limit,config.rate_limit);
  // Braking distance prevents hitting the angle clamp at nonzero velocity.
  const double a=config.acceleration_limit;
  auto safe=[&](double gap,double old) {
    // Include the final partial-speed discrete step, which a continuous
    // v^2/(2a) envelope omits. Future scalar steps are bounded by 0.15 s.
    const double b=a*(dt+.15);
    return std::max(0.0,(-b+std::sqrt(std::max(0.0,b*b-4*a*dt*old+8*a*gap)))*.5);
  };
  v=std::clamp(v,-safe(config.angle_limit+offset,-rate),safe((relief_only?0.0:config.angle_limit)-offset,rate));
  acceleration=(v-rate)/dt; offset+=.5*(rate+v)*dt; rate=v;
  return std::isfinite(offset)&&std::abs(offset)<=config.angle_limit+1e-9&&
    std::abs(acceleration)<=config.acceleration_limit+1e-7;
}
bool compliantReference(const Kinematics& m,const Reference& nominal,const Reference& previous,
  double delta,double rate,double acceleration,double dt,const PlanningConfig& config,Reference& out,std::string& reason)
{
  const Mat correction=expSO3(Vec::UnitY()*delta);
  const Mat desired=nominal.body.linear()*correction;
  Vec q=previous.eta; Mat j; Pose body,cog;
  for(int i=0;i<8;++i) {
    if(!m.evaluate(q,body,cog)||!m.jacobian(q,j)) { reason="compliant FK/Jacobian unavailable"; return false; }
    Eigen::JacobiSVD<Mat> svd(j,Eigen::ComputeFullU|Eigen::ComputeFullV);
    if(svd.singularValues().minCoeff()<config.min_singular_value) { reason="compliant singularity"; return false; }
    const Vec error=logSO3(body.linear().transpose()*desired);
    if(error.norm()<1e-9) break;
    Vec step=svd.solve(error); if(step.norm()>.1) step*=.1/step.norm(); q+=step;
  }
  if(!m.evaluate(q,body,cog)||logSO3(body.linear().transpose()*desired).norm()>1e-7||
     !mechanismAllowed(m,q,config,reason)||!m.jacobian(q,j)) { reason="compliant pose/limits: "+reason; return false; }
  const Vec omega=correction.transpose()*nominal.body_omega+Vec::UnitY()*rate;
  const Vec angular_acc=correction.transpose()*nominal.body_acceleration-
    (Vec::UnitY()*rate).cross(correction.transpose()*nominal.body_omega)+Vec::UnitY()*acceleration;
  Eigen::JacobiSVD<Mat> svd(j,Eigen::ComputeFullU|Eigen::ComputeFullV);
  const Vec velocity=svd.solve(omega); Mat jp,jm;
  if(!m.jacobian(q+1e-4*velocity,jp)||!m.jacobian(q-1e-4*velocity,jm)) return false;
  const Vec acc=svd.solve(angular_acc-(jp-jm)/(2e-4)*velocity);
  if(!std::isfinite(dt)||dt<=0||(velocity.cwiseAbs().array()>config.limits.velocity.array()).any()||
     (acc.cwiseAbs().array()>config.limits.acceleration.array()).any()||
     ((q-previous.eta).cwiseAbs().array()>config.limits.velocity.array()*dt+1e-6).any()||
     ((velocity-previous.velocity).cwiseAbs().array()>config.limits.acceleration.array()*dt+1e-5).any())
  { reason="compliant joint rate/acceleration bound"; return false; }
  if(!mechanismReference(m,q,velocity,acc,out)||out.cog_velocity.norm()>config.cog_speed_limit||
     out.cog_acceleration.norm()>config.cog_acceleration_limit) { reason="compliant CoG derivatives"; return false; }
  out.desired={desired,omega,angular_acc}; return true;
}
bool advanceCompliance(const Kinematics& model,const Reference& nominal,const Reference& previous,
  double dt,bool fresh,const PlanningConfig& config,PitchAdmittance& state,Reference& out,std::string& reason)
{
  const auto before=state; auto proposal=state;
  if(!proposal.step(dt,fresh)) { reason="invalid admittance integration step"; return false; }
  // Bound local work and apply anti-windup in the scalar state itself. Reduced
  // acceleration and a final braking candidate are tried without moving the goal.
  for(int attempt=0;attempt<7;++attempt) {
    auto candidate=proposal;
    const double a=attempt==6?std::clamp(-before.rate/dt,-state.config.acceleration_limit,state.config.acceleration_limit):
      proposal.acceleration*std::pow(.5,attempt);
    candidate.acceleration=a; candidate.rate=before.rate+a*dt;
    candidate.offset=before.offset+.5*(before.rate+candidate.rate)*dt;
    if(std::abs(candidate.offset)>state.config.angle_limit||std::abs(candidate.rate)>state.config.rate_limit) continue;
    if(compliantReference(model,nominal,previous,candidate.offset,candidate.rate,a,dt,config,out,reason)) {
      state=candidate; reason=attempt==0?"compliant reference accepted":"compliant acceleration constrained; anti-windup"; return true;
    }
  }
  return false;
}
}
