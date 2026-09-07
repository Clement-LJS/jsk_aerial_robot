#include <aerial_robot_motion/compliance/cartesian_admittance.h>
#include <gtest/gtest.h>
#include <limits>

using namespace aerial_robot_motion;
using namespace aerial_robot_motion::compliance;

TEST(Admittance, ZeroWrenchStaysAtRest)
{
  AdmittanceParameters p;
  p.axes.setOnes();
  AdmittanceDynamics dynamics;
  dynamics.configure(p);
  dynamics.setEnabled(true);
  for (int i = 0; i < 500; ++i) dynamics.step(Vector6::Zero(), 0.02);
  EXPECT_DOUBLE_EQ(dynamics.displacement().norm(), 0);
  EXPECT_DOUBLE_EQ(dynamics.velocity().norm(), 0);
}

TEST(Admittance, StepWrenchReachesPhysicalEquilibrium)
{
  AdmittanceParameters p;
  p.axes.setOnes();
  p.displacement_limit.setConstant(1.0);
  p.velocity_limit.setConstant(2.0);
  AdmittanceDynamics dynamics;
  dynamics.configure(p);
  dynamics.setEnabled(true);
  for (int i = 0; i < 1000; ++i) dynamics.step(Vector6::Ones(), 0.02);
  EXPECT_NEAR(dynamics.displacement()[0], 1.0 / p.stiffness[0], 1e-7);
  EXPECT_LT(dynamics.velocity().norm(), 1e-7);
}

TEST(Admittance, PitchOnlyComplianceIsBounded)
{
  AdmittanceParameters p;
  p.axes[4] = 1;
  AdmittanceDynamics dynamics;
  dynamics.configure(p);
  dynamics.setEnabled(true);
  for (int i = 0; i < 400; ++i)
  {
    dynamics.step(Vector6::Constant(100), 0.02);
    EXPECT_LE(dynamics.displacement()[4], p.displacement_limit[4]);
    EXPECT_LE(std::abs(dynamics.velocity()[4]), p.velocity_limit[4]);
    for (int axis : {0, 1, 2, 3, 5})
    {
      EXPECT_DOUBLE_EQ(dynamics.displacement()[axis], 0);
      EXPECT_DOUBLE_EQ(dynamics.velocity()[axis], 0);
    }
  }
  EXPECT_GT(dynamics.displacement()[4], 0.0);
}

TEST(Admittance, DisableReturnsContinuouslyAndReenableDoesNotJump)
{
  AdmittanceParameters p;
  p.axes[4] = 1;
  AdmittanceDynamics dynamics;
  dynamics.configure(p);
  dynamics.setEnabled(true);
  for (int i = 0; i < 100; ++i) dynamics.step(Vector6::Constant(100), 0.02);
  const double before = dynamics.displacement()[4];
  dynamics.setEnabled(false);
  EXPECT_DOUBLE_EQ(dynamics.displacement()[4], before);
  dynamics.step(Vector6::Zero(), 0.02);
  EXPECT_LT(dynamics.displacement()[4], before);
  EXPECT_LE(before - dynamics.displacement()[4], p.disable_rate[4] * 0.02 + 1e-12);
  const double returning = dynamics.displacement()[4];
  dynamics.setEnabled(true);
  EXPECT_DOUBLE_EQ(dynamics.displacement()[4], returning);
  dynamics.setEnabled(false);
  for (int i = 0; i < 100; ++i) dynamics.step(Vector6::Zero(), 0.02);
  EXPECT_NEAR(dynamics.displacement().norm(), 0, 1e-12);
}

TEST(Admittance, WrenchTransformIncludesRotationAndMomentArm)
{
  Eigen::Isometry3d target_T_source = Eigen::Isometry3d::Identity();
  target_T_source.linear() = rotationExp(Eigen::Vector3d(0, 0, 1.5707963267948966));
  target_T_source.translation() = Eigen::Vector3d(1, 0, 0);
  Vector6 source = Vector6::Zero();
  source[0] = 2;
  source[3] = 3;
  const Vector6 transformed = AdmittanceDynamics::transformWrench(target_T_source, source);
  EXPECT_NEAR(transformed[0], 0, 1e-12);
  EXPECT_NEAR(transformed[1], 2, 1e-12);
  EXPECT_NEAR(transformed[4], 3, 1e-12);
  EXPECT_NEAR(transformed[5], 2, 1e-12);
  const Vector6 recovered = AdmittanceDynamics::transformWrench(target_T_source.inverse(), transformed);
  EXPECT_TRUE(recovered.isApprox(source, 1e-12));
}

TEST(Admittance, BackwardEulerHandlesStiffStableSystem)
{
  AdmittanceParameters p;
  p.axes.setOnes();
  p.mass.setConstant(0.01);
  p.damping.setConstant(0.01);
  p.stiffness.setConstant(10000);
  AdmittanceDynamics dynamics;
  dynamics.configure(p);
  dynamics.setEnabled(true);
  for (int i = 0; i < 200; ++i) dynamics.step(Vector6::Constant(1), 0.1);
  EXPECT_NEAR(dynamics.displacement()[0], 0.0001, 1e-10);
}

TEST(Admittance, RejectsInvalidConfigurationAndTimeWithoutMoving)
{
  AdmittanceDynamics dynamics;
  AdmittanceParameters p;
  p.mass[0] = 0;
  EXPECT_THROW(dynamics.configure(p), std::invalid_argument);
  p.mass[0] = 1;
  p.damping[0] = -1;
  EXPECT_THROW(dynamics.configure(p), std::invalid_argument);
  EXPECT_THROW(dynamics.step(Vector6::Zero(), 0), std::invalid_argument);
  EXPECT_THROW(dynamics.step(Vector6::Zero(), 100), std::invalid_argument);
  Vector6 invalid = Vector6::Zero();
  invalid[4] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(dynamics.step(invalid, 0.02), std::invalid_argument);
  EXPECT_DOUBLE_EQ(dynamics.displacement().norm(), 0);
}
