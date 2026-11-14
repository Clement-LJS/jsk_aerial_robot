#include <gtest/gtest.h>
#include <gimbalrotor/gimbalrotor_perching.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <Eigen/SVD>
#include <cstdlib>

using namespace roll_yaw;
int main(int argc,char** argv)
{ testing::InitGoogleTest(&argc,argv); return RUN_ALL_TESTS(); }
namespace
{
struct Fixture
{
  KDL::Tree tree{"body"}; KDL::JntArray joints{2};
  std::map<std::string,KDL::RigidBodyInertia> inertia;
  Kinematics model; PlanningConfig config;
  Fixture()
  {
    tree.addSegment(KDL::Segment("mount",KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Rotation::RotZ(M_PI),KDL::Vector(-.06,0,.017))),"body");
    tree.addSegment(KDL::Segment("yaw",KDL::Joint("joint_yaw",KDL::Joint::RotZ),KDL::Frame(KDL::Vector(.24,0,0))),"mount");
    tree.addSegment(KDL::Segment("hand",KDL::Joint("joint_roll",KDL::Joint::RotX),KDL::Frame(KDL::Vector(.008,0,-.068))),"yaw");
    inertia["body"]=KDL::RigidBodyInertia(2,KDL::Vector(.03,0,-.05),KDL::RotationalInertia(.1,.12,.08));
    inertia["yaw"]=KDL::RigidBodyInertia(.3,KDL::Vector(.02,.01,0),KDL::RotationalInertia(.01,.01,.01));
    inertia["hand"]=KDL::RigidBodyInertia(.2,KDL::Vector(.1,0,0),KDL::RotationalInertia(.005,.005,.005));
    KDL::Chain chain; tree.getChain("body","hand",chain);
    Pose cp=Pose::Identity(); cp.translation()=Vec(.21049,0,.0205);
    std::string error;
    EXPECT_TRUE(model.configure(chain,"joint_roll","joint_yaw",cp,Vec::UnitY(),
      articulatedCogModel(tree,joints,inertia,{},"body",KDL::Rotation::Identity(),1,0),error))<<error;
    EXPECT_TRUE(model.lock(Pose::Identity(),Vec::Zero()));
    config.max_tilt=.9; config.saw_point=Vec(.05918,.00025,.017); config.saw_point_valid=true;
  }
  Plan plan(double start=0,double end=.1745329252) const
  {
    Request request; request.pitch_end=end-start;
    auto result=prepare(model,Vec::Zero(),Mat::Identity(),request,config);
    EXPECT_TRUE(result.ok)<<result.reason; result.session=1; return result;
  }
};
Feedback feedback(const Reference& reference,double now)
{
  Feedback f; f.now=f.body_stamp=f.joint_stamp=now; f.hover=f.valid=true;
  f.body=reference.body; f.eta=reference.eta; f.joint_rate=reference.velocity; f.body_omega=reference.body_omega; return f;
}
Executor executor(const Fixture& fixture,Plan plan,double& time,bool start=true)
{
  Executor e; e.planning=fixture.config; e.config.settle_use_window=true;
  Reference first; EXPECT_TRUE(mechanismReference(fixture.model,Vec::Zero(),Vec::Zero(),Vec::Zero(),first));
  EXPECT_TRUE(e.attach(fixture.model,first,1,1)); EXPECT_TRUE(e.command("prepare",1)); EXPECT_TRUE(e.accept(std::move(plan)));
  EXPECT_EQ(e.stage,Stage::CONTACT_LOCKED);
  EXPECT_FALSE(e.command("start",1));
  EXPECT_TRUE(e.command("align",1));
  EXPECT_EQ(e.stage,Stage::PRE_ALIGN);
  time=1;
  for(int i=0;i<30000&&e.stage==Stage::PRE_ALIGN;++i) { time+=.01; e.tick(feedback(e.target,time)); }
  EXPECT_EQ(e.stage,Stage::READY)<<e.reason;
  EXPECT_TRUE(e.plan.measured_start_valid);
  if(start) EXPECT_TRUE(e.command("start",1));
  return e;
}
}

TEST(TaskPitch,TaskFrameBothDirectionsNonzeroStartAndWrap)
{
  const Mat pre=expSO3(Vec(.1,.2,-.3));
  for(double direction:{-1.,1.}) {
    TaskProgress p; const double start=direction*.04,end=direction*.2; p.reset(start);
    double previous=0;
    for(int i=0;i<100;++i) {
      const double theta=start+(end-start)*i/100+direction*.0001*std::sin(i);
      ASSERT_TRUE(p.observe(pre,pre*expSO3(Vec::UnitY()*theta),1+i*.02,start,end,.1,.03));
      EXPECT_NEAR(p.raw,theta,1e-12); EXPECT_GE(p.distance,previous); previous=p.distance;
    }
    const double distance=p.distance;
    p.observe(pre,pre*expSO3(Vec::UnitY()*start),4,start,end,.1,.03);
    EXPECT_EQ(p.distance,distance);
  }
  double pitch,cross;
  ASSERT_TRUE(taskPitch(pre,pre*expSO3(Vec::UnitY()*(-M_PI+.01)),M_PI-.01,pitch,cross));
  EXPECT_NEAR(pitch,M_PI+.01,1e-12);
  EXPECT_FALSE(taskPitch(pre,pre*expSO3(Vec::UnitX()*M_PI),0,pitch,cross));
  ASSERT_TRUE(taskPitch(pre,pre*expSO3(Vec::UnitX()*.2)*expSO3(Vec::UnitY()*.1),0,pitch,cross));
  EXPECT_GT(cross,.19);
}
TEST(Kinematics,FixedPivotJacobianSignAndCopyIsolation)
{
  Fixture f;
  for(Vec q:{Vec::Zero().eval(),Vec(.1,.3,-.4),Vec(-.2,-.2,.5)}) {
    Mat analytic,numeric; ASSERT_TRUE(f.model.jacobian(q,analytic)); ASSERT_TRUE(f.model.numericalJacobian(q,numeric));
    EXPECT_LT((analytic-numeric).norm(),1e-8);
    Pose body,cog,bp; ASSERT_TRUE(f.model.evaluate(q,body,cog)); ASSERT_TRUE(f.model.bodyPivot(q,bp));
    EXPECT_LT(((body*bp).translation()-f.model.pivotLock().translation()).norm(),1e-12);
    auto n=analytic.fullPivLu().solve(Vec::UnitY()).eval(); const Vec residual(.2,-.3,.1);
    EXPECT_NEAR(residual.dot(n*.07),n.dot(residual)*.07,1e-14);
    Estimate estimate; ASSERT_TRUE(f.model.estimate(body,q,estimate)); EXPECT_NEAR(estimate.alpha,q[0],1e-12);
  }
  Mat j; f.model.jacobian(Vec::Zero(),j); EXPECT_LT(j(1,0),-.99);
  auto copy=f.model; Pose a,b,c,d;
  f.model.evaluate(Vec(.1,.3,.2),a,b); copy.evaluate(Vec(-.1,-.4,.1),c,d);
  f.model.evaluate(Vec(.1,.3,.2),c,d); EXPECT_LT((b.matrix()-d.matrix()).norm(),1e-12);
}
TEST(Execution,MeasuredPositiveGoal)
{
  Fixture f;
  for(double direction:{1.}) {
    double time; auto e=executor(f,f.plan(0,direction*.1745329252),time);
    for(int i=0;i<30000&&e.stage==Stage::EXECUTING;++i) { time+=.01; e.tick(feedback(e.target,time)); }
    EXPECT_EQ(e.stage,Stage::COMPLETED_HOLD)<<e.reason;
    EXPECT_TRUE(e.goal_reached); EXPECT_TRUE(e.path_end_commanded);
    EXPECT_NEAR(e.plan.goal.pitch_end,direction*.1745329252,1e-12);
    EXPECT_LT(std::abs(e.goal_error),e.config.goal_tolerance);
  }
}
TEST(Execution,BlockedRobotConsumesFiniteLeadWithoutSuccess)
{
  Fixture f; double time; auto e=executor(f,f.plan(),time);
  // Exercise the progress governor with a wider tracking allowance.
  e.config.tracking_tolerance=e.config.orientation_tolerance=.2; e.config.progress_timeout=100;
  const Reference stationary=e.target;
  double last=0;
  for(int i=0;i<2500&&e.stage==Stage::EXECUTING;++i) {
    time+=.01; e.tick(feedback(stationary,time));
    EXPECT_GE(e.progress,last); last=e.progress;
    EXPECT_LE(e.planned_pitch,e.config.reference_lead+1e-7);
  }
  EXPECT_EQ(e.stage,Stage::EXECUTING)<<e.reason; EXPECT_TRUE(e.progress_lag);
  EXPECT_GT(e.planned_pitch,.001); EXPECT_FALSE(e.goal_reached); EXPECT_FALSE(e.path_end_commanded);
}
TEST(Execution,NominalTenControlNineMeasuredNineIsPending)
{
  Fixture f; double time; auto e=executor(f,f.plan(),time); const double goal=e.plan.goal.pitch_end;
  e.progress=e.plan.cut.duration(); ASSERT_TRUE(samplePath(e.model,e.plan.cut,e.progress,e.target));
  Reference effective;
  std::string reason;
  ASSERT_TRUE(compliantReference(e.model,e.target,e.target,-M_PI/180,0,0,1,e.planning,effective,reason))<<reason;
  e.applied=effective; e.applied_valid=true;
  e.config.tracking_tolerance=e.config.orientation_tolerance=.005; // nominal/effective gap is intentionally larger
  for(int i=0;i<100;++i) { time+=.01; e.tick(feedback(effective,time)); }
  EXPECT_EQ(e.stage,Stage::EXECUTING)<<e.reason; EXPECT_FALSE(e.goal_reached);
  EXPECT_TRUE(e.path_end_commanded); EXPECT_EQ(e.plan.goal.pitch_end,goal);
  EXPECT_NEAR(e.goal_error,M_PI/180,.003);
  for(int i=0;i<1100&&e.stage==Stage::EXECUTING;++i) { time+=.01; e.tick(feedback(effective,time)); }
  EXPECT_EQ(e.stage,Stage::EXECUTING); EXPECT_TRUE(e.first_fault.empty());
  EXPECT_FALSE(e.goal_reached); EXPECT_EQ(e.plan.goal.pitch_end,goal);
  e.fault("invalid generated reference");
  const auto first=e.first_fault; e.fault("later fault"); EXPECT_EQ(e.first_fault,first);
}
TEST(Planning,Cancellation)
{
  Fixture f; auto config=f.config; config.cancelled=[]{return true;}; Request request; request.pitch_end=.17;
  EXPECT_FALSE(prepare(f.model,Vec::Zero(),Mat::Identity(),request,config).ok);
}
TEST(Execution,CoordinatedComplianceReleaseAndMeasuredCompletion)
{
  Fixture f; double time; auto e=executor(f,f.plan(),time); Reference applied=e.target;
  PitchAdmittance a; a.baseline_ready=true; a.enabled=true;
  double end_at=0;
  const double original=e.plan.goal.pitch_end;
  for(int i=0;i<5000&&e.stage==Stage::EXECUTING;++i) {
    time+=.01;
    if(a.offset<-.02&&end_at==0) end_at=time;
    if(end_at>0&&time-end_at<1) EXPECT_FALSE(e.goal_reached);
    if(end_at>0&&time-end_at>=1) a.enabled=false;
    a.observe(-.1,time,false);
    ASSERT_TRUE(a.step(.01,true,true));
    e.relief=std::min(0.0,a.offset);
    e.applied=applied; e.applied_valid=true;
    const double progress=e.progress;
    e.tick(feedback(applied,time));
    ASSERT_NE(e.stage,Stage::FAULT_HOLD)<<e.reason;
    EXPECT_GE(e.progress,progress); // Relief slows/holds the same path, never retracts.
    const auto& next=e.target;
    Pose bp; ASSERT_TRUE(e.model.bodyPivot(next.eta,bp));
    EXPECT_LT(((next.body*bp).translation()-e.model.pivotLock().translation()).norm(),1e-9);
    EXPECT_EQ(e.plan.goal.pitch_end,original); applied=next;
  }
  EXPECT_GT(end_at,0); EXPECT_FALSE(a.enabled);
  EXPECT_TRUE(e.goal_reached)<<e.reason;
  EXPECT_EQ(e.stage,Stage::COMPLETED_HOLD);
}
TEST(Execution,StaleFeedbackWrongSessionAndDuplicateSettlingSamples)
{
  Fixture f; double time; auto e=executor(f,f.plan(),time);
  EXPECT_FALSE(e.command("pause",2));
  for(int i=0;i<50;++i) { time+=.01; e.tick(feedback(e.target,time)); }
  ASSERT_GT(e.target.velocity.norm(),0);
  const auto held=e.target; const double phase=e.progress;
  auto stale=feedback(e.target,time+.3); stale.body_stamp=time;
  e.tick(stale); EXPECT_EQ(e.stage,Stage::PAUSED); EXPECT_TRUE(e.first_fault.empty());
  EXPECT_TRUE(e.plan.ok); EXPECT_FALSE(e.goal_reached); EXPECT_EQ(e.session,1u);
  EXPECT_DOUBLE_EQ(e.progress,phase); EXPECT_EQ(e.target.velocity.norm(),0);
  EXPECT_EQ(e.target.cog_acceleration.norm(),0); EXPECT_EQ(e.target.desired.omega.norm(),0);
  EXPECT_FALSE(e.command("resume",1));
  e.tick(feedback(e.target,time+3)); EXPECT_EQ(e.stage,Stage::EXECUTING);
  EXPECT_EQ((e.target.eta-held.eta).norm(),0); EXPECT_DOUBLE_EQ(e.progress,phase);
  e.tick(feedback(e.target,time+3.01)); EXPECT_GT(e.progress,phase);
  e.tick(feedback(e.target,time+3.02)); EXPECT_GT(e.progress,phase);
  EXPECT_NE(e.stage,Stage::FAULT_HOLD)<<e.reason;
  // Source duplicates should not represent elapsed measured quiet time.
  SettlingWindow window;
  auto first=window.observe(1,{{0,0,0}},{{0,0}},.5,.15);
  EXPECT_FALSE(first.complete);
  auto repeated=window.observe(1,{{0,0,0}},{{0,0}},.5,.15);
  EXPECT_FALSE(repeated.complete);
}
TEST(Execution,ObservationWaitCancelAlignmentAndRealFaults)
{
  Fixture f; double time; auto e=executor(f,f.plan(),time);
  auto gap=feedback(e.target,time+.2); gap.valid=false; gap.invalid_reason="rate bootstrap";
  e.tick(gap); ASSERT_EQ(e.stage,Stage::PAUSED);
  ASSERT_TRUE(e.command("cancel",1));
  e.tick(feedback(e.target,time+2)); EXPECT_EQ(e.stage,Stage::COMPLETED_HOLD);
  EXPECT_FALSE(e.plan.ok); EXPECT_FALSE(e.command("resume",1));
  for(int cause=0;cause<3;++cause) {
    auto x=executor(f,f.plan(),time); auto bad=feedback(x.target,time+.01);
    if(cause==0) bad.body.translation().x()+=.1;
    if(cause==1) bad.eta[1]=2;
    if(cause==2) bad.now=time-1;
    x.tick(bad); EXPECT_EQ(x.stage,cause==1?Stage::FAULT_HOLD:Stage::PAUSED)<<cause;
    x.tick(feedback(x.target,time+.02));
    EXPECT_EQ(x.stage,cause==1?Stage::FAULT_HOLD:Stage::EXECUTING)<<cause;
  }
  auto x=executor(f,f.plan(),time,false);
  x.stage=Stage::PRE_ALIGN; const auto goal=x.plan.goal.pitch_end;
  auto skew=feedback(x.target,time+.02); skew.joint_stamp-=.06;
  x.tick(skew); EXPECT_EQ(x.stage,Stage::PAUSED);
  x.tick(feedback(x.target,time+1));
  EXPECT_EQ(x.stage,Stage::PRE_ALIGN); EXPECT_EQ(x.plan.goal.pitch_end,goal);
}
TEST(Planning,ManualRetargetAppliedDerivativesAndBoundedStop)
{
  Fixture f; Reference applied; Path path; std::string reason;
  ASSERT_TRUE(mechanismReference(f.model,Vec(0,0,-.00076699),Vec::Zero(),Vec::Zero(),applied));
  Vec goal=applied.eta; goal[2]=-.004257648503988658;
  ASSERT_TRUE(mechanismPath(f.model,applied,goal,f.config,path,reason))<<reason;
  EXPECT_FALSE(path.desired); EXPECT_GT(path.duration(),0);
  ASSERT_TRUE(samplePath(f.model,path,.025,applied)); EXPECT_LT(applied.eta[2],-.00076699);
  for(int key=0;key<30;++key) {
    const int axis=key%3; goal[axis]+=(key%2?-.003:.003);
    const auto before=applied;
    ASSERT_TRUE(mechanismPath(f.model,applied,goal,f.config,path,reason))<<key<<": "<<reason;
    EXPECT_EQ((path.knots.back().eta-goal).norm(),0);
    ASSERT_TRUE(samplePath(f.model,path,0,applied));
    EXPECT_LT((applied.velocity-before.velocity).norm(),1e-10);
    EXPECT_LT((applied.acceleration-before.acceleration).norm(),1e-10);
    for(int i=1;i<=5;++i) {
      ASSERT_TRUE(samplePath(f.model,path,i*.025,applied));
      ASSERT_TRUE(referenceAllowed(applied,f.config,reason))<<reason;
      EXPECT_LT((applied.desired.omega-applied.body_omega).norm(),1e-12);
      Pose b,g; ASSERT_TRUE(f.model.evaluate(applied.eta,b,g));
      EXPECT_LT((b.matrix()-applied.body.matrix()).norm(),1e-12);
      EXPECT_LT((g.matrix()-applied.cog.matrix()).norm(),1e-12);
    }
  }
  double phase=.125,speed=1;
  const double braking=.5*std::min((f.config.limits.acceleration.array()/f.config.limits.velocity.array()).minCoeff(),
    f.config.cog_acceleration_limit/f.config.cog_speed_limit);
  while(speed>0&&phase<path.duration()) {
    const double next=std::max(0.,speed-braking*.025);
    phase+=.5*(speed+next)*.025;
    ASSERT_TRUE(samplePath(f.model,path,phase,applied)); scaleReference(applied,next,(next-speed)/.025);
    ASSERT_TRUE(referenceAllowed(applied,f.config,reason))<<reason; speed=next;
  }
}
TEST(Planning,BodyAlignmentThenMeasuredLocalStroke)
{
  Fixture f;
  for(double yaw:{-.4,.4}) for(double roll:{0.,M_PI/6}) {
    Request alignment; alignment.align_only=true; alignment.yaw=yaw; alignment.roll=roll;
    auto aligned=prepare(f.model,Vec::Zero(),Mat::Identity(),alignment,f.config);
    ASSERT_TRUE(aligned.ok)<<aligned.reason;
    const Vec start=aligned.align.knots.back().eta; Pose body,cog;
    ASSERT_TRUE(f.model.evaluate(start,body,cog));
    EXPECT_LT(logSO3(body.linear().transpose()*expSO3(Vec::UnitZ()*yaw)*expSO3(Vec::UnitX()*roll)).norm(),1e-8);
    Request cut; cut.pitch_end=.12;
    const auto plan=prepare(f.model,start,body.linear(),cut,f.config);
    ASSERT_TRUE(plan.ok)<<plan.reason; EXPECT_GT(plan.proxy_drop,0);
    EXPECT_LT((plan.cut.knots.front().eta-start).norm(),1e-8);
    for(int i=0;i<=100;++i) {
      Reference r; ASSERT_TRUE(samplePath(f.model,plan.cut,plan.cut.duration()*i/100,r));
      EXPECT_LT(r.orientation_error,1e-7);
      Pose bp; ASSERT_TRUE(f.model.bodyPivot(r.eta,bp));
      EXPECT_LT(((r.body*bp).translation()-f.model.pivotLock().translation()).norm(),1e-9);
      EXPECT_LE((r.velocity.cwiseQuotient(f.config.limits.velocity)).cwiseAbs().maxCoeff(),1);
      EXPECT_LE((r.acceleration.cwiseQuotient(f.config.limits.acceleration)).cwiseAbs().maxCoeff(),1);
    }
    cut.pitch_end=-.12;
    const auto upward=prepare(f.model,start,body.linear(),cut,f.config);
    EXPECT_FALSE(upward.ok); EXPECT_NE(upward.reason.find("upward"),std::string::npos)<<upward.reason;
  }
}
TEST(Planning,MovingRetargetDurationIntervals)
{
  Fixture f; Reference from; Path path; std::string reason;
  auto check=[&](const Vec& goal) {
    ASSERT_TRUE(mechanismPath(f.model,from,goal,f.config,path,reason))<<reason;
    EXPECT_EQ((path.knots.front().velocity-from.velocity).norm(),0);
    EXPECT_EQ((path.knots.front().acceleration-from.acceleration).norm(),0);
    EXPECT_EQ((path.knots.back().eta-goal).norm(),0);
    const double t=path.duration();
    std::array<Vec,6> p{{from.eta,from.eta+from.velocity*t/5,
      from.eta+2*from.velocity*t/5+from.acceleration*t*t/20,goal,goal,goal}};
    std::array<Vec,5> v;
    for(const auto& q:p) {
      EXPECT_TRUE((q.array()>=f.config.limits.lower.array()-1e-10).all());
      EXPECT_TRUE((q.array()<=f.config.limits.upper.array()+1e-10).all());
    }
    for(int i=0;i<5;++i) {
      v[i]=5*(p[i+1]-p[i])/t;
      EXPECT_TRUE((v[i].cwiseAbs().array()<=.5*f.config.limits.velocity.array()+1e-9).all());
    }
    for(int i=0;i<4;++i)
      EXPECT_TRUE(((4*(v[i+1]-v[i])/t).cwiseAbs().array()<=.5*f.config.limits.acceleration.array()+1e-9).all());
  };
  ASSERT_TRUE(mechanismReference(f.model,Vec(0,0,.27356269315021703),Vec(0,0,.02309625231437117),
    Vec(0,0,-.01577556826737270),from));
  check(Vec(0,0,.27227136331111570));
  EXPECT_GT(path.duration(),2.4407308023);
  for(double sign:{-1.,1.}) {
    ASSERT_TRUE(mechanismReference(f.model,Vec(0,0,sign*1.54),Vec(0,0,sign*.01),Vec::Zero(),from));
    check(from.eta); // zero distance with nonzero speed
    check(from.eta-Vec(0,0,sign*.0034906585)); // moving reversal near either limit
  }
  ASSERT_TRUE(mechanismReference(f.model,Vec::Zero(),Vec::Zero(),Vec::Zero(),from));
  Vec goal=from.eta;
  for(int key=0;key<60;++key) {
    goal[2]+=(key<30?1:-1)*.2*M_PI/180;
    check(goal);
    ASSERT_FALSE(path.knots.empty());
    ASSERT_TRUE(samplePath(f.model,path,std::min(.15,path.duration()),from));
  }
  ASSERT_TRUE(mechanismReference(f.model,Vec(0,0,f.config.limits.upper[2]),Vec(0,0,.01),Vec::Zero(),from));
  EXPECT_FALSE(mechanismPath(f.model,from,from.eta-Vec(0,0,.01),f.config,path,reason));
}

TEST(Execution,CancelThenObservationLossHoldsAppliedWithoutRevival)
{
  Fixture f; double time; auto e=executor(f,f.plan(),time);
  for(int i=0;i<100;++i) { time+=.01; e.tick(feedback(e.target,time)); }
  const auto held=e.target; e.applied=held; e.applied_valid=true;
  ASSERT_TRUE(e.command("cancel",1)); ASSERT_TRUE(e.stopping());
  e.waitForFeedback(time+.2,"stale body");
  EXPECT_EQ(e.stage,Stage::COMPLETED_HOLD); EXPECT_FALSE(e.plan.ok);
  EXPECT_EQ(e.session,1u); EXPECT_EQ((e.target.eta-held.eta).norm(),0);
  EXPECT_EQ((e.target.body.matrix()-held.body.matrix()).norm(),0);
  EXPECT_EQ((e.target.cog.matrix()-held.cog.matrix()).norm(),0);
  EXPECT_EQ(e.target.velocity.norm()+e.target.acceleration.norm()+e.target.cog_velocity.norm()+
    e.target.cog_acceleration.norm()+e.target.body_omega.norm()+e.target.body_acceleration.norm(),0);
  EXPECT_TRUE(e.command("cancel",1));
  e.tick(feedback(e.target,time+1)); EXPECT_EQ(e.stage,Stage::COMPLETED_HOLD);
  EXPECT_FALSE(e.command("resume",1)); EXPECT_FALSE(e.command("start",1));
}

namespace
{
void checkPitchJogs(const Kinematics& model,const PlanningConfig& c,const Vec& start)
{
  Pose body,cog; ASSERT_TRUE(model.evaluate(start,body,cog));
  for(double delta:{-.0872664626,.0872664626,-.0034906585,.0034906585}) {
    Request jog; jog.align_only=jog.body_pitch_jog=true; jog.pitch_start=delta;
    auto plan=prepare(model,start,body.linear(),jog,c);
    ASSERT_TRUE(plan.ok)<<plan.reason; EXPECT_TRUE(plan.cut.knots.empty());
    EXPECT_GT(delta*plan.proxy_drop,0);
    double previous=(body*c.saw_point).z();
    for(int i=0;i<=100;++i) {
      const double s=i/100.,theta=delta*(10*s*s*s-15*std::pow(s,4)+6*std::pow(s,5));
      Reference r; ASSERT_TRUE(samplePath(model,plan.align,plan.align.duration()*s,r));
      EXPECT_LT(logSO3(r.body.linear().transpose()*body.linear()*expSO3(Vec::UnitY()*theta)).norm(),1e-7);
      Pose bp; ASSERT_TRUE(model.bodyPivot(r.eta,bp));
      EXPECT_LT(((r.body*bp).translation()-model.pivotLock().translation()).norm(),1e-9);
      Pose b,g; ASSERT_TRUE(model.evaluate(r.eta,b,g));
      EXPECT_LT((r.cog.matrix()-g.matrix()).norm(),1e-12);
      std::string why; EXPECT_TRUE(referenceAllowed(r,c,why))<<why;
      EXPECT_TRUE((r.velocity.cwiseAbs().array()<=.5*c.limits.velocity.array()+1e-9).all());
      EXPECT_TRUE((r.acceleration.cwiseAbs().array()<=.5*c.limits.acceleration.array()+1e-9).all());
      const double z=(r.body*c.saw_point).z(); EXPECT_LE((delta>0?1:-1)*(z-previous),1e-8); previous=z;
    }
    if(std::abs(start[1])>.1&&std::abs(start[2])>.1)
      EXPECT_GT((plan.align.knots.back().eta-start).tail<2>().norm(),1e-5);
  }
}
}
TEST(Planning,LocalPitchJogDirectionAndPurpose)
{
  Fixture f; checkPitchJogs(f.model,f.config,Vec::Zero());
  for(double yaw:{-.4,.4}) for(double roll:{0.,M_PI/6}) {
    Request align; align.align_only=true; align.yaw=yaw; align.roll=roll;
    const auto plan=prepare(f.model,Vec::Zero(),Mat::Identity(),align,f.config);
    ASSERT_TRUE(plan.ok)<<plan.reason; checkPitchJogs(f.model,f.config,plan.align.knots.back().eta);
  }
  Request jog; jog.align_only=jog.body_pitch_jog=true; jog.pitch_start=.1;
  f.config.saw_point_valid=false;
  EXPECT_FALSE(prepare(f.model,Vec::Zero(),Mat::Identity(),jog,f.config).ok);
  jog.body_pitch_jog=false; // Existing body alignment still does not need a saw.
  EXPECT_TRUE(prepare(f.model,Vec::Zero(),Mat::Identity(),jog,f.config).ok);
}
TEST(Execution,LocalPitchJogPauseResumeAndMeasuredCompletion)
{
  Fixture f; Request jog; jog.align_only=jog.body_pitch_jog=true; jog.pitch_start=.0872664626;
  auto plan=prepare(f.model,Vec::Zero(),Mat::Identity(),jog,f.config);
  ASSERT_TRUE(plan.ok)<<plan.reason; plan.session=17;
  Executor e; e.config.settle_use_window=true; e.planning=f.config;
  Reference held; ASSERT_TRUE(mechanismReference(f.model,Vec::Zero(),Vec::Zero(),Vec::Zero(),held));
  ASSERT_TRUE(e.attach(f.model,held,17,1)); ASSERT_TRUE(e.command("prepare",17));
  ASSERT_TRUE(e.accept(std::move(plan))); ASSERT_TRUE(e.command("align",17));
  double time=1;
  for(int i=0;i<100;++i) { time+=.01; e.tick(feedback(e.target,time)); }
  ASSERT_EQ(e.stage,Stage::PRE_ALIGN); const double progress=e.progress;
  e.waitForFeedback(time+.2,"missing observation"); ASSERT_EQ(e.stage,Stage::PAUSED);
  EXPECT_FALSE(e.command("resume",17));
  time+=2; e.tick(feedback(e.target,time));
  EXPECT_EQ(e.stage,Stage::PRE_ALIGN); EXPECT_EQ(e.progress,progress); // no time catch-up
  EXPECT_DOUBLE_EQ(e.plan.goal.pitch_start,jog.pitch_start);
  for(int i=0;i<30000&&e.stage==Stage::PRE_ALIGN;++i) { time+=.01; e.tick(feedback(e.target,time)); }
  ASSERT_EQ(e.stage,Stage::READY)<<e.reason;
  EXPECT_LT(logSO3(e.target.body.linear().transpose()*expSO3(Vec::UnitY()*jog.pitch_start)).norm(),1e-8);
  EXPECT_FALSE(e.command("start",17)); EXPECT_TRUE(e.plan.cut.knots.empty());
}
TEST(Planning,MechanismAlignmentAndLimitReversalWithoutSawOrIK)
{
  Fixture f; f.config.saw_point_valid=false; f.config.min_singular_value=100;
  Reference held; ASSERT_TRUE(mechanismReference(f.model,Vec::Zero(),Vec::Zero(),Vec::Zero(),held));
  Vec goal(0,.2,.3); Path path; std::string reason;
  ASSERT_TRUE(mechanismPath(f.model,held,goal,f.config,path,reason))<<reason;
  Reference end; ASSERT_TRUE(samplePath(f.model,path,path.duration(),end));
  EXPECT_LT((end.eta-goal).norm(),1e-10);
  f.config.limits.upper[2]=.3; goal[2]-=.01;
  ASSERT_TRUE(mechanismPath(f.model,end,goal,f.config,path,reason))<<reason;
  Reference reversed; ASSERT_TRUE(samplePath(f.model,path,path.duration(),reversed));
  EXPECT_LT(reversed.eta[2],end.eta[2]); EXPECT_NEAR(reversed.eta[1],goal[1],1e-12);
  f.config.min_singular_value=.05; f.config.saw_point_valid=true;
  Request cut; cut.pitch_end=.1;
  EXPECT_TRUE(prepare(f.model,reversed.eta,reversed.body.linear(),cut,f.config).ok);
}
TEST(Execution,GeometricReadinessAndPreparedMovement)
{
  Fixture f; double time; auto e=executor(f,f.plan(),time,false);
  Reference moved; ASSERT_TRUE(mechanismReference(f.model,Vec(.015,0,0),Vec::Zero(),Vec::Zero(),moved));
  e.tick(feedback(moved,time+.01)); EXPECT_FALSE(e.plan.ok); EXPECT_EQ(e.stage,Stage::CONTACT_LOCKED);
  EXPECT_NE(e.reason.find("prepared start moved"),std::string::npos);
}
TEST(Admittance,UnconstrainedStepSatisfiesDiscreteForceBalance)
{
  for(double dt : {.005, .025, .1}) for(double initial_rate : {-.01, .01}) {
    PitchAdmittance a;
    a.enabled = a.baseline_ready = a.contact = true;
    a.offset = -.02;
    a.rate = initial_rate;
    a.filtered = -.06;
    a.config.angle_limit = .4;
    a.config.rate_limit = 1;
    a.config.acceleration_limit = 10;
    const double old_offset = a.offset, old_rate = a.rate;
    ASSERT_TRUE(a.step(dt, true, true));
    // Check the governing equations, independently of the solved velocity formula.
    EXPECT_NEAR(a.offset - old_offset, .5 * dt * (old_rate + a.rate), 1e-14);
    EXPECT_NEAR(a.config.inertia * (a.rate - old_rate) / dt + a.config.damping * a.rate +
                a.config.stiffness * a.offset - a.filtered, 0, 1e-12);
    EXPECT_LT(std::abs(a.rate), a.config.rate_limit);
    EXPECT_LT(std::abs(a.acceleration), a.config.acceleration_limit);
  }
}
TEST(Admittance,ReliefOnlyResistanceAndPositiveLoad)
{
  for(double load : {-.15, .15}) {
    PitchAdmittance a;
    a.enabled = a.baseline_ready = true;
    for(int i = 0; i < 1500; ++i) {
      a.observe(load, 1 + .01 * i, false);
      ASSERT_TRUE(a.step(.01, true, true, .025));
      EXPECT_LE(a.offset, 0);
      EXPECT_GE(a.offset, -.025 - 1e-9);
      EXPECT_LE(std::abs(a.rate), a.config.rate_limit);
      EXPECT_LE(std::abs(a.acceleration), a.config.acceleration_limit + 1e-7);
    }
    EXPECT_EQ(a.contact, load < 0);
    if(load < 0) EXPECT_LT(a.offset, -.02);
    else EXPECT_DOUBLE_EQ(a.offset, 0);
  }
  // A lingering contact latch must not admit positive forward assistance either.
  PitchAdmittance a;
  a.enabled = a.baseline_ready = a.contact = true;
  a.filtered = .15;
  ASSERT_TRUE(a.step(.025, true, true));
  EXPECT_DOUBLE_EQ(a.offset, 0);
  EXPECT_DOUBLE_EQ(a.rate, 0);
}
TEST(Admittance,BaselineDistinctContactSignsLimitsAndSmoothRelease)
{
  for(double direction:{-1.,1.}) {
    PitchAdmittance a; a.config.baseline_samples=3;
    a.observe(1,1,true); a.observe(1,1,true); EXPECT_EQ(a.samples,1);
    a.observe(5,1.05,false); EXPECT_EQ(a.samples,1);
    a.observe(1,1.1,true); a.observe(1,1.15,true); ASSERT_TRUE(a.baseline_ready); EXPECT_DOUBLE_EQ(a.bias,1);
    a.enabled=true; double stamp=1.15;
    for(int i=0;i<1500;++i) {
      stamp+=.01; a.observe(1+direction*.15,stamp,false); ASSERT_TRUE(a.step(.01,true));
      EXPECT_LE(std::abs(a.offset),a.config.angle_limit+1e-8);
      EXPECT_LE(std::abs(a.rate),a.config.rate_limit+1e-8);
      EXPECT_LE(std::abs(a.acceleration),a.config.acceleration_limit+1e-7);
    }
    EXPECT_EQ(a.contact,direction<0); EXPECT_DOUBLE_EQ(a.bias,1);
    if(direction<0) EXPECT_LT(a.offset,-.02);
    else EXPECT_DOUBLE_EQ(a.offset,0); // Positive load does not trigger resisting contact.
    const double before=a.offset; a.enabled=false; ASSERT_TRUE(a.step(.01,true));
    EXPECT_LT(std::abs(a.offset-before),.001);
    for(int i=0;i<2000;++i) ASSERT_TRUE(a.step(.01,false));
    EXPECT_LT(std::abs(a.offset),1e-7); EXPECT_LT(std::abs(a.rate),1e-7);
  }
}
TEST(Worker,BoundedQueueCancellationAndNonblockingTake)
{
  Worker<Plan> worker; std::atomic<bool> started{false};
  ASSERT_TRUE(worker.submit([&](const std::function<bool()>& cancel){started=true; while(!cancel()) std::this_thread::yield(); return Plan();}));
  const double before=steadySeconds(); EXPECT_FALSE(worker.take()); EXPECT_LT(steadySeconds()-before,.02);
  EXPECT_FALSE(worker.submit([](const std::function<bool()>&){return Plan();}));
  while(!started) std::this_thread::yield(); worker.cancel();
}
TEST(ActualUrdf,JacobianCogAndOfflinePlan)
{
  const char* file=std::getenv("ROLL_YAW_TEST_URDF"); if(!file) GTEST_SKIP()<<"Set ROLL_YAW_TEST_URDF to expanded current beetle Xacro";
  KDL::Tree tree; ASSERT_TRUE(kdl_parser::treeFromFile(file,tree)); KDL::Chain chain;
  ASSERT_TRUE(tree.getChain("fc","roll_hand_link",chain));
  unsigned roll=0,yaw=0; std::map<std::string,KDL::RigidBodyInertia> inertia;
  for(const auto& item:tree.getSegments()) {
    inertia[item.first]=item.second.segment.getInertia();
    if(item.second.segment.getJoint().getName()=="joint_roll") roll=item.second.q_nr;
    if(item.second.segment.getJoint().getName()=="joint_yaw") yaw=item.second.q_nr;
  }
  KDL::JntArray joints(tree.getNrOfJoints()); Pose cp=Pose::Identity(); cp.translation()=Vec(.21049,0,.0205);
  Kinematics model; std::string reason;
  ASSERT_TRUE(model.configure(chain,"joint_roll","joint_yaw",cp,Vec::UnitY(),
    articulatedCogModel(tree,joints,inertia,{},"fc",KDL::Rotation::Identity(),roll,yaw),reason))<<reason;
  ASSERT_TRUE(model.lock(Pose::Identity(),Vec::Zero()));
  for(Vec q:{Vec::Zero().eval(),Vec(.1,.2,.3),Vec(-.1,-.3,-.2)}) {
    Mat a,b; ASSERT_TRUE(model.jacobian(q,a)); ASSERT_TRUE(model.numericalJacobian(q,b)); EXPECT_LT((a-b).norm(),1e-8);
  }
  PlanningConfig c; c.max_tilt=.9;
  KDL::TreeFkSolverPos_recursive fk(tree); KDL::Frame fc,saw;
  ASSERT_GE(fk.JntToCart(joints,fc,"fc"),0); ASSERT_GE(fk.JntToCart(joints,saw,"saw"),0);
  c.saw_point=fromKdl(fc.Inverse()*saw).translation(); c.saw_point_valid=true;
  c.limits.lower[0]=-1.2; c.limits.upper[0]=1.2; // Current RollYawPerching.yaml.
  checkPitchJogs(model,c,Vec::Zero());
  Reference manual; ASSERT_TRUE(mechanismReference(model,Vec(0,0,-.00076699),Vec::Zero(),Vec::Zero(),manual));
  Path transition; Vec destination=manual.eta; destination[2]=-.004257648503988658;
  double callback_max=0,sample_max=0;
  for(int i=0;i<100;++i) {
    const double began=steadySeconds();
    ASSERT_TRUE(mechanismPath(model,manual,destination,c,transition,reason))<<reason;
    callback_max=std::max(callback_max,steadySeconds()-began);
    Reference next; const double tick=steadySeconds();
    ASSERT_TRUE(samplePath(model,transition,.025,next)); ASSERT_TRUE(referenceAllowed(next,c,reason))<<reason;
    sample_max=std::max(sample_max,steadySeconds()-tick);
  }
  std::cout<<"Current URDF manual construction max seconds: "<<callback_max<<"; sample/check max: "<<sample_max<<"\n";
  // Offline connection must still invoke the full validator and cancellation.
  c.cancelled=[] {return true;};
  EXPECT_TRUE(mechanismPath(model,manual,destination,c,transition,reason));
  EXPECT_FALSE(validatedMechanismPath(model,manual,destination,c,transition,reason));
  c.cancelled={};
  ASSERT_TRUE(validatedMechanismPath(model,manual,destination,c,transition,reason))<<reason;
  Request r; r.pitch_end=M_PI/18;
  const double start=steadySeconds(); auto plan=prepare(model,Vec::Zero(),Mat::Identity(),r,c);
  ASSERT_TRUE(plan.ok)<<plan.reason; std::cout<<"Current URDF offline prepare wall seconds: "<<steadySeconds()-start<<"\n";
  for(double reach:{-.4,.4}) for(double lean:{0.,M_PI/6}) {
    Request alignment; alignment.align_only=true; alignment.yaw=reach; alignment.roll=lean;
    auto a=prepare(model,Vec::Zero(),Mat::Identity(),alignment,c); ASSERT_TRUE(a.ok)<<a.reason;
    Pose b,g; ASSERT_TRUE(model.evaluate(a.align.knots.back().eta,b,g));
    checkPitchJogs(model,c,a.align.knots.back().eta);
    auto stroke=prepare(model,a.align.knots.back().eta,b.linear(),r,c);
    ASSERT_TRUE(stroke.ok)<<stroke.reason; EXPECT_GT(stroke.proxy_drop,0);
  }
}
