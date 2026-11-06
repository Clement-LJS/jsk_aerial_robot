#include <gtest/gtest.h>
#include <gimbalrotor/roll_yaw_admittance.h>
#include <gimbalrotor/roll_yaw_dynamics.h>
#include <gimbalrotor/roll_yaw_worker.h>
#include <kdl_parser/kdl_parser.hpp>
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
  // Isolate progress governor from the independent physical tracking fault.
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
  EXPECT_EQ(e.stage,Stage::FAULT_HOLD); EXPECT_NE(e.first_fault.find("timed out"),std::string::npos);
  EXPECT_EQ(e.plan.goal.pitch_end,goal);
  const auto first=e.first_fault; e.fault("later feedback gap"); EXPECT_EQ(e.first_fault,first);
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
  auto stale=feedback(e.target,20); stale.body_stamp=19;
  e.tick(stale); EXPECT_EQ(e.stage,Stage::FAULT_HOLD); EXPECT_NE(e.first_fault.find("stale"),std::string::npos);
  // Source duplicates should not represent elapsed measured quiet time.
  SettlingWindow window;
  auto first=window.observe(1,{{0,0,0}},{{0,0}},.5,.15);
  EXPECT_FALSE(first.complete);
  auto repeated=window.observe(1,{{0,0,0}},{{0,0}},.5,.15);
  EXPECT_FALSE(repeated.complete);
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
    EXPECT_TRUE(a.contact); EXPECT_GT(direction*a.offset,.02); EXPECT_DOUBLE_EQ(a.bias,1);
    const double before=a.offset; a.enabled=false; ASSERT_TRUE(a.step(.01,true));
    EXPECT_LT(std::abs(a.offset-before),.001);
    for(int i=0;i<2000;++i) ASSERT_TRUE(a.step(.01,false));
    EXPECT_LT(std::abs(a.offset),1e-7); EXPECT_LT(std::abs(a.rate),1e-7);
  }
}
TEST(Dynamics,EnergyMassGravityMomentumAndTaskPower)
{
  Fixture f; ArticulatedDynamics d(f.model,f.tree,f.inertia,{},"body",1,0,Vec(0,0,-9.80665));
  const Vec q(.1,.2,-.3),v(.04,-.03,.02); KDL::JntArray rates(2);
  auto terms=d.evaluate(q,v,f.joints,rates,Vec::Zero(),Vec::Zero(),Vec::Zero(),.05,{});
  ASSERT_TRUE(terms.ok)<<terms.reason;
  EXPECT_LT((terms.mass-terms.mass.transpose()).norm(),1e-10);
  EXPECT_LT((terms.momentum-terms.mass*v).norm(),1e-7);
  EXPECT_NEAR(terms.kinetic,.5*v.dot(terms.mass*v),1e-8);
  auto stationary=d.evaluate(q,Vec::Zero(),f.joints,rates,Vec::Zero(),Vec::Zero(),Vec::Zero(),.05,{});
  ASSERT_TRUE(stationary.ok);
  Mat j; Pose b,g; f.model.jacobian(q,j); f.model.evaluate(q,b,g);
  EXPECT_NEAR(terms.potential,2.5*9.80665*g.translation().z(),1e-10);
  const Vec gravity_torque=(g.translation()-f.model.pivotLock().translation()).cross(Vec(0,0,-2.5*9.80665));
  EXPECT_NEAR(stationary.drift[0],(f.model.pivotLock().linear()*Vec::UnitY()).dot(gravity_torque),1e-6);
  const Vec compensating_torque=b.linear()*j.transpose().fullPivLu().solve(-stationary.drift);
  auto balanced=d.evaluate(q,Vec::Zero(),f.joints,rates,Vec::Zero(),compensating_torque,Vec::Zero(),.05,{});
  ASSERT_TRUE(balanced.ok); EXPECT_LT(balanced.drift.norm(),1e-7);
  const Vec effort(0,.3,-.2);
  auto actuated=d.evaluate(q,Vec::Zero(),f.joints,rates,Vec::Zero(),compensating_torque,effort,.05,{});
  EXPECT_LT((actuated.drift-effort).norm(),1e-7); // no second scaling or sign change
  const Vec external(.3,-.2,.1); const double pitch_rate=-.05;
  EXPECT_NEAR(external.dot(terms.pitch_map*pitch_rate),terms.pitch_map.dot(external)*pitch_rate,1e-14);
  MomentumObserver observer; DynamicsTerms sample; sample.ok=true; sample.pitch_map=terms.pitch_map;
  EXPECT_FALSE(observer.update(sample,1,.15));
  for(int i=1;i<=300;++i) { sample.momentum=external*(i*.01); ASSERT_TRUE(observer.update(sample,1+i*.01,.15)); }
  EXPECT_LT((observer.residual-external).norm(),1e-8);
  EXPECT_NEAR(observer.torque,terms.pitch_map.dot(external),1e-8);
  EXPECT_FALSE(observer.update(sample,10,.15)); // reinitialize across source gap
}
TEST(Dynamics,ExtraModuleCountedExactlyOnce)
{
  Fixture f; const KDL::Vector center(.1,.02,.03);
  const KDL::Segment payload("body",KDL::Joint(KDL::Joint::None),KDL::Frame(center),
    KDL::RigidBodyInertia(.4,KDL::Vector::Zero(),KDL::RotationalInertia(.002,.002,.002)));
  const std::map<std::string,KDL::Segment> extra={{"tool",payload}};
  ArticulatedDynamics plain(f.model,f.tree,f.inertia,{},"body",1,0,Vec(0,0,-9.80665));
  ArticulatedDynamics loaded(f.model,f.tree,f.inertia,extra,"body",1,0,Vec(0,0,-9.80665));
  KDL::JntArray rates(2); const Vec q(.1,.2,.3);
  auto a=plain.evaluate(q,Vec::Zero(),f.joints,rates,Vec::Zero(),Vec::Zero(),Vec::Zero(),.05,{});
  auto b=loaded.evaluate(q,Vec::Zero(),f.joints,rates,Vec::Zero(),Vec::Zero(),Vec::Zero(),.05,{});
  ASSERT_TRUE(a.ok); ASSERT_TRUE(b.ok); Pose body,cog; f.model.evaluate(q,body,cog);
  EXPECT_NEAR(b.potential-a.potential,.4*9.80665*(body*Vec(center.x(),center.y(),center.z())).z(),1e-10);
  EXPECT_GT((b.mass-a.mass).trace(),0);
}
TEST(Dynamics,MeasuredGimbalMotionContributesToGeneralizedMomentum)
{
  Fixture f;
  ASSERT_TRUE(f.tree.addSegment(KDL::Segment("gimbal",KDL::Joint("gimbal_joint",KDL::Joint::RotY),
    KDL::Frame(KDL::Vector(.2,.1,0))),"body"));
  f.inertia["gimbal"]=KDL::RigidBodyInertia(.3,KDL::Vector(.08,0,.02),KDL::RotationalInertia(.01,.01,.01));
  KDL::JntArray joints(3),rates(3); joints(2)=.4; rates(2)=.2;
  ArticulatedDynamics d(f.model,f.tree,f.inertia,{},"body",1,0,Vec(0,0,-9.80665));
  auto moving=d.evaluate(Vec::Zero(),Vec::Zero(),joints,rates,Vec::Zero(),Vec::Zero(),Vec::Zero(),.05,{});
  rates.data.setZero();
  auto frozen=d.evaluate(Vec::Zero(),Vec::Zero(),joints,rates,Vec::Zero(),Vec::Zero(),Vec::Zero(),.05,{});
  ASSERT_TRUE(moving.ok); ASSERT_TRUE(frozen.ok);
  EXPECT_GT(moving.momentum.norm(),1e-4); EXPECT_GT(moving.kinetic,0);
  EXPECT_LT(frozen.momentum.norm(),1e-12); EXPECT_DOUBLE_EQ(frozen.kinetic,0);
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
  PlanningConfig c; c.max_tilt=.9; c.saw_point=Vec(.05918,.00025,.017-.06927); c.saw_point_valid=true;
  Request r; r.pitch_end=M_PI/18;
  const double start=steadySeconds(); auto plan=prepare(model,Vec::Zero(),Mat::Identity(),r,c);
  ASSERT_TRUE(plan.ok)<<plan.reason; std::cout<<"Current URDF offline prepare wall seconds: "<<steadySeconds()-start<<"\n";
  for(double reach:{-.4,.4}) for(double lean:{0.,M_PI/6}) {
    Request alignment; alignment.align_only=true; alignment.yaw=reach; alignment.roll=lean;
    auto a=prepare(model,Vec::Zero(),Mat::Identity(),alignment,c); ASSERT_TRUE(a.ok)<<a.reason;
    Pose b,g; ASSERT_TRUE(model.evaluate(a.align.knots.back().eta,b,g));
    auto stroke=prepare(model,a.align.knots.back().eta,b.linear(),r,c);
    ASSERT_TRUE(stroke.ok)<<stroke.reason; EXPECT_GT(stroke.proxy_drop,0);
  }
  ArticulatedDynamics dynamics(model,tree,inertia,{},"fc",roll,yaw,Vec(0,0,-9.80665));
  KDL::JntArray rates(joints.rows());
  const double began=steadySeconds(); auto t=dynamics.evaluate(Vec(.1,.2,.3),Vec(.01,.02,-.01),joints,rates,Vec::Zero(),Vec::Zero(),Vec::Zero(),.05,{});
  ASSERT_TRUE(t.ok)<<t.reason; std::cout<<"Current URDF dynamics sample wall seconds: "<<steadySeconds()-began<<"\n";
}
