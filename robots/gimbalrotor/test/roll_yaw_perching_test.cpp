#include <gimbalrotor/roll_yaw_cutting_trajectory.h>
#include <gtest/gtest.h>
#include <random>
#include <kdl_parser/kdl_parser.hpp>
#include <cstdlib>
#include <limits>
using namespace roll_yaw;
namespace
{
// Test geometry reproduces the URDF's actual fixed transforms and joint order.
// Pivot calibration here is a TEST fixture, never a hardware default.
Kinematics mechanism(bool moving_cog_frame=false)
{
  KDL::Chain chain;
  chain.addSegment(KDL::Segment("pipe",KDL::Joint(KDL::Joint::None),
    KDL::Frame(KDL::Rotation::RotZ(3.142),KDL::Vector(-.05918,0,.017))));
  chain.addSegment(KDL::Segment("yaw_origin",KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(.2405,0,0))));
  chain.addSegment(KDL::Segment("assembly",KDL::Joint("joint_yaw",KDL::Joint::RotZ)));
  chain.addSegment(KDL::Segment("roll_origin",KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(.008,0,-.068))));
  chain.addSegment(KDL::Segment("hand",KDL::Joint("joint_roll",KDL::Joint::RotX)));
  Pose cp=Pose::Identity(); cp.translation()=Vec(.13,-.01,.02);
  Kinematics m; std::string error;
  EXPECT_TRUE(m.configure(chain,"joint_roll","joint_yaw",cp,Vec::UnitY(),
    [moving_cog_frame](const Vec& q,Pose& bg) {
      bg=Pose::Identity(); bg.translation()=Vec(.01*std::sin(q[1]),.005*std::sin(q[2]),-.02);
      bg.linear()=expSO3(Vec::UnitZ()*(.2+(moving_cog_frame?.3*q[1]:0))); return true;
    },error))<<error;
  EXPECT_TRUE(m.lock(Pose::Identity(),Vec::Zero())); return m;
}
PlanningConfig testConfig()
{
  PlanningConfig c; c.max_tilt=1.2; c.dt=.04; c.max_duration=300; c.scaling_attempts=6;
  return c;
}
Request stroke(double roll=0,double end=20*M_PI/180)
{ Request r; r.yaw=M_PI/6; r.roll=roll; r.pitch_end=end; return r; }
Feedback feedbackAt(Executor& e,double now)
{
  Feedback f; f.now=f.joint_stamp=f.body_stamp=now; f.valid=f.hover=true;
  f.body=e.target.body; f.eta=e.target.eta; f.joint_rate=e.target.velocity; f.body_omega=e.target.body_omega;
  return f;
}
}
TEST(RollYawKinematics, ExpandedRepositoryUrdf)
{
  const char* path=std::getenv("ROLL_YAW_TEST_URDF");
  if(!path) GTEST_SKIP()<<"Set ROLL_YAW_TEST_URDF to the expanded beetle roll-yaw URDF";
  KDL::Tree tree; ASSERT_TRUE(kdl_parser::treeFromFile(path,tree));
  KDL::Chain chain; ASSERT_TRUE(tree.getChain("base_link","roll_hand_link",chain));
  Pose cp=Pose::Identity(); cp.translation()=Vec(.13,-.01,.02);
  Kinematics actual; std::string error;
  ASSERT_TRUE(actual.configure(chain,"joint_roll","joint_yaw",cp,Vec::UnitY(),
    [](const Vec&,Pose& bg){bg=Pose::Identity(); return true;},error))<<error;
  ASSERT_TRUE(actual.lock(Pose::Identity(),Vec::Zero()));
  std::map<std::string,KDL::RigidBodyInertia> inertias;
  unsigned ri=0,yi=0;
  for(const auto& segment:tree.getSegments()) {
    inertias[segment.first]=segment.second.segment.getInertia();
    if(segment.second.segment.getJoint().getName()=="joint_roll") ri=segment.second.q_nr;
    if(segment.second.segment.getJoint().getName()=="joint_yaw") yi=segment.second.q_nr;
  }
  KDL::JntArray joints(tree.getNrOfJoints()); joints.data.setZero();
  auto cog=articulatedCogModel(tree,joints,inertias,{},"base_link",KDL::Rotation::RotZ(.2),ri,yi);
  Pose bg0,bg1; ASSERT_TRUE(cog(Vec::Zero(),bg0)); ASSERT_TRUE(cog(Vec(0,.5,.4),bg1));
  EXPECT_GT((bg0.translation()-bg1.translation()).norm(),1e-5);
  EXPECT_LT(logSO3(bg0.linear().transpose()*expSO3(Vec(0,0,-.2))).norm(),1e-12);
  EXPECT_FALSE(cog(Vec(0,NAN,0),bg1));
  auto fixture=mechanism();
  for(Vec q:{Vec::Zero().eval(),Vec(.3,.4,-.2),Vec(-.5,-.6,.7)}) {
    Pose p,f; ASSERT_TRUE(actual.bodyPivot(q,p)); ASSERT_TRUE(fixture.bodyPivot(q,f));
    EXPECT_LT((p.matrix()-f.matrix()).norm(),1e-10);
    Mat j,n; ASSERT_TRUE(actual.jacobian(q,j)); ASSERT_TRUE(actual.numericalJacobian(q,n));
    EXPECT_LT((j-n).norm(),1e-8);
  }
}
TEST(RollYawKinematics, FixedPivotAndJacobian)
{
  auto m=mechanism();
  for(double alpha:{-.6,.0,.7}) for(double roll:{-.5,.3,.8}) for(double yaw:{-.7,.2,.5}) {
    Vec q(alpha,roll,yaw); Pose b,g,bp; Mat j,n;
    ASSERT_TRUE(m.evaluate(q,b,g)); ASSERT_TRUE(m.bodyPivot(q,bp));
    EXPECT_LT(((b*bp).translation()-m.pivotLock().translation()).norm(),1e-12);
    ASSERT_TRUE(m.jacobian(q,j)); ASSERT_TRUE(m.numericalJacobian(q,n)); EXPECT_LT((j-n).norm(),1e-8);
    Estimate est; ASSERT_TRUE(m.estimate(b,q,est)); EXPECT_NEAR(alpha,est.alpha,1e-10);
    EXPECT_LT(est.hinge_residual,1e-10); EXPECT_LT(est.pivot_error,1e-12);
  }
}
TEST(RollYawKinematics, InvalidAndHingeResidual)
{
  auto m=mechanism(); Pose b,g; Vec q(.3,.2,.1); ASSERT_TRUE(m.evaluate(q,b,g));
  b.translation().x()+=.1; b.linear()=b.linear()*expSO3(Vec(.1,0,0));
  Estimate e; ASSERT_TRUE(m.estimate(b,q,e)); EXPECT_GT(e.pivot_error,.05); EXPECT_GT(e.hinge_residual,.05);
  q[0]=std::numeric_limits<double>::quiet_NaN(); EXPECT_FALSE(m.evaluate(q,b,g));
  EXPECT_FALSE(rotationValid(Mat::Zero()));
}
TEST(RollYawQp, ExactBoxesAndKkt)
{
  std::mt19937 rng(7); std::normal_distribution<double> normal;
  for(int k=0;k<150;++k) {
    Mat a; Vec g; for(int i=0;i<3;++i) { g[i]=normal(rng); for(int j=0;j<3;++j) a(i,j)=normal(rng); }
    Mat h=a.transpose()*a+.1*Mat::Identity(); Vec lo(-.2,-.3,-.1),hi(.1,.4,.2);
    auto r=solveBoxQp(h,g,lo,hi); ASSERT_TRUE(r.ok); Vec gradient=h*r.u+g;
    for(int i=0;i<3;++i) {
      EXPECT_GE(r.u[i],lo[i]-1e-9); EXPECT_LE(r.u[i],hi[i]+1e-9);
      if(r.u[i]<hi[i]-1e-8) EXPECT_GE(gradient[i],-1e-7);
      if(r.u[i]>lo[i]+1e-8) EXPECT_LE(gradient[i],1e-7);
    }
  }
  EXPECT_FALSE(solveBoxQp(-Mat::Identity(),Vec::Zero(),-Vec::Ones(),Vec::Ones()).ok);
  EXPECT_FALSE(solveBoxQp(Mat::Identity(),Vec::Zero(),Vec::Ones(),-Vec::Ones()).ok);
}
TEST(RollYawQp, LimitsSingularityAndFrames)
{
  Limits l; QpConfig c; Mat j=Mat::Identity();
  Vec q=l.upper-Vec::Constant(.00001),previous=Vec::Zero();
  auto r=orientationStep(j,Mat::Identity(),Mat::Identity(),Vec::Ones(),q,previous,.01,l,c);
  ASSERT_TRUE(r.ok); EXPECT_LE((q+.01*r.u-l.upper).maxCoeff(),1e-9); EXPECT_LE(r.u.maxCoeff(),.001+1e-9);
  EXPECT_FALSE(orientationStep(Mat::Zero(),Mat::Identity(),Mat::Identity(),Vec::Zero(),Vec::Zero(),previous,.01,l,c).ok);
  auto invalid=Mat::Identity().eval(); invalid(0,0)=NAN;
  EXPECT_FALSE(orientationStep(j,invalid,Mat::Identity(),Vec::Zero(),q,previous,.01,l,c).ok);
  EXPECT_FALSE(orientationStep(j,Mat::Identity(),Mat::Identity(),Vec::Zero(),q,Vec::Ones(),.01,l,c).ok);
}
TEST(RollYawTrajectory, IdealLocalPitchNormalAndRate)
{
  for(double roll:{0.0,M_PI/6,M_PI/4}) for(double end:{-.3490658504,.3490658504}) {
    auto r=stroke(roll,end); Stroke s(end,r.rate,r.acceleration);
    const Vec normal=cuttingOrientation(Mat::Identity(),r,0,0,0).rotation*Vec::UnitY();
    double last=0;
    for(int i=1;i<100;++i) {
      double theta,v,a; double t=s.duration()*i/100; s.sample(t,theta,v,a);
      const auto d=cuttingOrientation(Mat::Identity(),r,theta,v,a);
      EXPECT_LT((d.rotation*Vec::UnitY()-normal).norm(),1e-12); EXPECT_EQ(d.omega.x(),0); EXPECT_EQ(d.omega.z(),0);
      EXPECT_LE(std::abs(v),r.rate+1e-12); EXPECT_LE(std::abs(a),r.acceleration+1e-12);
      EXPECT_GE((theta-last)*end,0); last=theta;
      double tp,vp,ap,tm,vm,am; const double h=1e-5;
      s.sample(t+h,tp,vp,ap); s.sample(t-h,tm,vm,am);
      const Mat rp=cuttingOrientation(Mat::Identity(),r,tp,vp,ap).rotation,rm=cuttingOrientation(Mat::Identity(),r,tm,vm,am).rotation;
      EXPECT_LT((logSO3(rp*rm.transpose())/(2*h)-d.rotation*d.omega).norm(),1e-8);
    }
    double theta,v,a; s.sample(s.duration(),theta,v,a); EXPECT_NEAR(theta,end,1e-12); EXPECT_EQ(v,0); EXPECT_EQ(a,0);
  }
}
TEST(RollYawTrajectory, CasesABCAndReverse)
{
  auto m=mechanism(); auto c=testConfig();
  for(double roll:{0.0,M_PI/6,M_PI/4}) for(double end:{.3490658504,-.3490658504}) {
    auto r=stroke(roll,end); auto plan=prepare(m,Vec::Zero(),Mat::Identity(),r,c);
    ASSERT_TRUE(plan.ok)<<"roll="<<roll<<" "<<plan.reason;
    Reference ref,first,last;
    ASSERT_TRUE(samplePath(m,plan.align,plan.align.duration(),last));
    ASSERT_TRUE(samplePath(m,plan.cut,0,first));
    EXPECT_LT((first.eta-last.eta).norm(),1e-12); EXPECT_LT(first.body_omega.norm(),1e-8); EXPECT_LT(last.body_omega.norm(),1e-8);
    for(int i=0;i<=100;++i) {
      ASSERT_TRUE(samplePath(m,plan.cut,plan.cut.duration()*i/100,ref));
      EXPECT_LT(ref.orientation_error,c.orientation_tolerance);
      EXPECT_LE(ref.velocity.cwiseAbs().maxCoeff(),.1);
      EXPECT_LT((ref.cog_omega-(ref.cog.linear().transpose()*ref.body.linear()*ref.body_omega)).norm(),1e-7);
    }
    EXPECT_LT(logSO3(ref.body.linear().transpose()*cuttingOrientation(Mat::Identity(),r,r.pitch_end,0,0).rotation).norm(),c.orientation_tolerance);
    EXPECT_LT(ref.body_omega.norm(),1e-8); EXPECT_LT(ref.cog_velocity.norm(),1e-8);
    Vec travel=Vec::Zero();
    for(const auto& k:plan.cut.knots) travel=travel.cwiseMax((k.eta-plan.cut.knots.front().eta).cwiseAbs());
    EXPECT_GT(travel.minCoeff(),1e-4); // all three coordinates move in the sweep
    // Requested body roll/yaw are demonstrably not held servo positions.
    EXPECT_GT(std::abs(plan.cut.knots.back().eta[2]-r.yaw),.1);
  }
}
TEST(RollYawTrajectory, ArticulatedCogFrameDerivative)
{
  auto m=mechanism(true); auto plan=prepare(m,Vec::Zero(),Mat::Identity(),stroke(M_PI/6),testConfig());
  ASSERT_TRUE(plan.ok)<<plan.reason;
  const double t=plan.cut.duration()*.4,h=1e-4; Reference r,p,n;
  ASSERT_TRUE(samplePath(m,plan.cut,t,r)); ASSERT_TRUE(samplePath(m,plan.cut,t+h,p)); ASSERT_TRUE(samplePath(m,plan.cut,t-h,n));
  EXPECT_LT((logSO3(n.cog.linear().transpose()*p.cog.linear())/(2*h)-r.cog_omega).norm(),1e-6);
  EXPECT_LT(((p.cog.translation()-n.cog.translation())/(2*h)-r.cog_velocity).norm(),1e-6);
  EXPECT_LT(((p.cog_omega-n.cog_omega)/(2*h)-r.cog_angular_acceleration).norm(),1e-4);
}
TEST(RollYawTrajectory, UnsafeInvalidAndScaling)
{
  auto m=mechanism(); auto c=testConfig(); c.max_tilt=.2; c.scaling_attempts=1;
  for(double roll:{0.0,M_PI/6,M_PI/4}) EXPECT_FALSE(prepare(m,Vec::Zero(),Mat::Identity(),stroke(roll),c).ok);
  auto bad=stroke(); bad.yaw=NAN; EXPECT_FALSE(prepare(m,Vec::Zero(),Mat::Identity(),bad,testConfig()).ok);
  c=testConfig(); c.limits.upper[2]=.1; c.limits.lower[2]=-.1; c.scaling_attempts=1;
  EXPECT_FALSE(prepare(m,Vec::Zero(),Mat::Identity(),stroke(),c).ok);
  c=testConfig(); auto fast=stroke(); fast.rate=.5; fast.acceleration=.5;
  auto plan=prepare(m,Vec::Zero(),Mat::Identity(),fast,c); ASSERT_TRUE(plan.ok)<<plan.reason; EXPECT_GT(plan.scale_attempt,0);
}
TEST(RollYawExecutor, LifecyclePauseCancelStaleSessionAndFaults)
{
  Executor e; e.planning=testConfig(); double now=10;
  auto f=feedbackAt(e,now); ASSERT_TRUE(e.capture(f,mechanism())); auto id=e.session;
  EXPECT_FALSE(e.command("prepare",id-1)); ASSERT_TRUE(e.command("prepare",id));
  auto p=prepare(e.model,e.target.eta,Mat::Identity(),stroke(),e.planning); p.session=id;
  ASSERT_TRUE(p.ok)<<p.reason; ASSERT_TRUE(e.accept(p)); ASSERT_TRUE(e.command("align",id));
  auto tick=[&](){ now+=.025; e.tick(feedbackAt(e,now)); };
  for(int i=0;i<20000&&e.stage==Stage::PRE_ALIGN;++i) tick();
  ASSERT_EQ(e.stage,Stage::READY)<<e.reason; ASSERT_TRUE(e.command("start",id));
  for(int i=0;i<100;++i) tick();
  EXPECT_GT(e.target.body_omega.norm(),1e-5);
  ASSERT_TRUE(e.command("pause",id)); for(int i=0;i<1000&&e.stage==Stage::EXECUTING;++i) tick();
  ASSERT_EQ(e.stage,Stage::PAUSED)<<e.reason; const Vec held=e.target.eta; for(int i=0;i<20;++i) tick();
  EXPECT_LT((held-e.target.eta).norm(),1e-12); ASSERT_TRUE(e.command("resume",id));
  for(int i=0;i<20000&&e.stage==Stage::EXECUTING;++i) tick();
  ASSERT_EQ(e.stage,Stage::COMPLETED_HOLD)<<e.reason;
  f=feedbackAt(e,now+.025); f.joint_stamp-=1; e.tick(f); EXPECT_EQ(e.stage,Stage::FAULT_HOLD);
  EXPECT_EQ(e.target.body_omega.norm(),0); e.reset(); EXPECT_FALSE(e.command("start",id));
  f=feedbackAt(e,now+1); ASSERT_TRUE(e.capture(f,mechanism())); EXPECT_NE(e.session,id);
  e.fault("conflicting legacy trajectory"); EXPECT_EQ(e.stage,Stage::FAULT_HOLD);
}
TEST(RollYawExecutor, HoverPivotAndInvalidState)
{
  for(int fault=0;fault<4;++fault) {
    Executor e; e.planning=testConfig(); auto f=feedbackAt(e,10); ASSERT_TRUE(e.capture(f,mechanism()));
    f=feedbackAt(e,10.025);
    if(fault==0) f.hover=false;
    if(fault==1) f.body.translation().x()+=.1;
    if(fault==2) f.body.linear()(0,0)=NAN;
    if(fault==3) f.eta[1]+=.2;
    e.tick(f); EXPECT_EQ(e.stage,Stage::FAULT_HOLD)<<e.reason;
  }
}

TEST(RollYawExecutor, CancelAndDelayedPlanCannotRestart)
{
  Executor e; e.planning=testConfig(); double now=20;
  ASSERT_TRUE(e.capture(feedbackAt(e,now),mechanism())); const auto id=e.session;
  ASSERT_TRUE(e.command("prepare",id));
  auto plan=prepare(e.model,e.target.eta,Mat::Identity(),stroke(),e.planning); plan.session=id;
  ASSERT_TRUE(plan.ok); ASSERT_TRUE(e.command("cancel",id));
  EXPECT_FALSE(e.accept(plan)); EXPECT_EQ(e.stage,Stage::COMPLETED_HOLD);
  ASSERT_TRUE(e.command("prepare",id)); ASSERT_TRUE(e.accept(plan)); ASSERT_TRUE(e.command("align",id));
  for(int i=0;i<150;++i) { now+=.025; e.tick(feedbackAt(e,now)); }
  ASSERT_TRUE(e.command("cancel",id));
  for(int i=0;i<500&&e.stage==Stage::PRE_ALIGN;++i) {
    now+=.025; e.tick(feedbackAt(e,now));
    EXPECT_TRUE((e.target.velocity.cwiseAbs().array()<=e.planning.limits.velocity.array()+1e-8).all());
    EXPECT_TRUE((e.target.acceleration.cwiseAbs().array()<=e.planning.limits.acceleration.array()+1e-8).all());
  }
  EXPECT_EQ(e.stage,Stage::COMPLETED_HOLD); EXPECT_FALSE(e.plan.ok);
  EXPECT_FALSE(e.command("start",id)); EXPECT_LT(e.target.velocity.norm(),1e-10);
  const auto q=e.target.eta; now+=.025; e.tick(feedbackAt(e,now)); EXPECT_LT((q-e.target.eta).norm(),1e-12);
  e.reset(); EXPECT_FALSE(e.accept(plan));
}
TEST(RollYawConfiguration, InvalidParametersFailClosed)
{
  auto c=testConfig(); c.dt=NAN; EXPECT_FALSE(c.valid());
  c=testConfig(); c.limits.acceleration[1]=0; EXPECT_FALSE(c.valid());
  c=testConfig(); c.qp.weight[2]=-1; EXPECT_FALSE(c.valid());
  c=testConfig(); c.max_tilt=M_PI; EXPECT_FALSE(c.valid());
  ExecutionConfig e; e.timeout=0; EXPECT_FALSE(e.valid());
  Request r=stroke(); r.rate=-.1; EXPECT_FALSE(r.valid());
}

int main(int argc,char** argv)
{
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
