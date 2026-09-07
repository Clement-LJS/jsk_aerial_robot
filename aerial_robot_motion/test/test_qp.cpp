#include <aerial_robot_motion/core/qp_problem.h>
#include <aerial_robot_motion/core/ros_conversions.h>
#include <aerial_robot_motion/solver/qpoases_solver.h>
#include <aerial_robot_motion/constraint/joint_limit.h>
#include <aerial_robot_motion/constraint/revolute_contact.h>
#include <aerial_robot_motion/cost/cartesian_pose.h>
#include <Eigen/SVD>
#include <gtest/gtest.h>
using namespace aerial_robot_motion;

TEST(StateInput, RejectsInvalidQuaternionAndNormalizesValidPose)
{
  geometry_msgs::Pose pose;
  EXPECT_THROW(fromPose(pose), std::invalid_argument);
  pose.orientation.w = std::numeric_limits<double>::max();
  EXPECT_THROW(fromPose(pose), std::invalid_argument);
  pose.orientation.w = 2;
  EXPECT_TRUE(fromPose(pose).linear().isApprox(Eigen::Matrix3d::Identity()));
  pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(fromPose(pose), std::invalid_argument);
}

TEST(QP, AddsCostsIntersectsBoundsAndStacksRows)
{
  QPProblem p; p.reset(2, 0.1);
  p.addCost(Eigen::Matrix2d::Identity(), Eigen::Vector2d(1, 2));
  p.addCost(2 * Eigen::Matrix2d::Identity(), Eigen::Vector2d(3, 4));
  EXPECT_TRUE(p.H.isApprox(3.1 * Eigen::Matrix2d::Identity()));
  EXPECT_TRUE(p.g.isApprox(Eigen::Vector2d(4, 6)));
  p.intersectBounds(Eigen::Vector2d(-2, -3), Eigen::Vector2d(3, 4));
  p.intersectBounds(Eigen::Vector2d(-1, -4), Eigen::Vector2d(4, 2));
  EXPECT_TRUE(p.lb.isApprox(Eigen::Vector2d(-1, -3)));
  EXPECT_TRUE(p.ub.isApprox(Eigen::Vector2d(3, 2)));
  Eigen::Matrix<double, 1, 2> row; row << 1, 1;
  p.appendConstraints(row, Eigen::VectorXd::Constant(1, 0), Eigen::VectorXd::Constant(1, 0));
  p.appendConstraints(2 * row, Eigen::VectorXd::Constant(1, -1), Eigen::VectorXd::Constant(1, 2));
  EXPECT_EQ(p.A.rows(), 2); EXPECT_EQ(p.lbA[0], p.ubA[0]); EXPECT_DOUBLE_EQ(p.ubA[1], 2);
  std::string status; EXPECT_TRUE(p.validate(status)) << status;
}
TEST(QP, HotstartChangesHessianAndConstraintMatrixAndResetsDimensions)
{
  QpOasesSolver solver; QPProblem p; p.reset(2, 1.0); p.g << -1, -2;
  Eigen::VectorXd x; std::string status;
  ASSERT_TRUE(solver.solve(p, x, status)); EXPECT_TRUE(x.isApprox(Eigen::Vector2d(1, 2), 1e-6));
  p.H *= 2;
  ASSERT_TRUE(solver.solve(p, x, status)); EXPECT_NE(status.find("hotstart"), std::string::npos);
  EXPECT_TRUE(x.isApprox(Eigen::Vector2d(0.5, 1), 1e-6));
  Eigen::Matrix<double, 1, 2> a; a << 1, 1;
  p.appendConstraints(a, Eigen::VectorXd::Constant(1, 1), Eigen::VectorXd::Constant(1, 1));
  ASSERT_TRUE(solver.solve(p, x, status)); EXPECT_NE(status.find("initialized"), std::string::npos);
  p.A << 1, -1; p.lbA[0] = p.ubA[0] = 0;
  ASSERT_TRUE(solver.solve(p, x, status)); EXPECT_NEAR(x[0], x[1], 1e-6);
  p.reset(6, 1); ASSERT_TRUE(solver.solve(p, x, status)); EXPECT_EQ(x.size(), 6);
}
TEST(QP, InvalidAndInfeasibleProblemsDoNotReturnReferences)
{
  QpOasesSolver solver; QPProblem p; p.reset(2, 1);
  p.lb[0] = 1; p.ub[0] = -1; Eigen::VectorXd x; std::string status;
  EXPECT_FALSE(solver.solve(p, x, status)); EXPECT_EQ(x.size(), 0);
  p.reset(2, 1); p.H(0, 0) = -1;
  EXPECT_FALSE(solver.solve(p, x, status));
  p.reset(2, 1); p.g[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(solver.solve(p, x, status));
  p.reset(2, 1); p.lb.setZero(); p.ub.setOnes();
  p.appendConstraints(Eigen::Matrix<double, 1, 2>::Ones(), Eigen::VectorXd::Constant(1, 3), Eigen::VectorXd::Constant(1, 3));
  EXPECT_FALSE(solver.solve(p, x, status)); EXPECT_EQ(x.size(), 0);
  p.reset(2, 1); EXPECT_TRUE(solver.solve(p, x, status));
}
TEST(Cartesian, RequestsCorrectDirectionAndReportsResidual)
{
  MotionContext ctx; ctx.state.tool_jacobian = Eigen::MatrixXd::Identity(6, 6);
  ctx.target.translation().x() = 0.05;
  QPProblem p; p.reset(6, 1e-6); cost::CartesianPoseCost cost;
  ASSERT_TRUE(cost.update(ctx, p));
  QpOasesSolver solver; Eigen::VectorXd x; std::string status;
  ASSERT_TRUE(solver.solve(p, x, status));
  EXPECT_NEAR(x[0], 0.1, 1e-6); EXPECT_LT(x.tail<5>().norm(), 1e-8);
  EXPECT_LT(cost.residual(ctx, x).norm(), 1e-6);
}
TEST(JointLimits, DamperBlocksTowardLimitButAllowsRetreat)
{
  ModelInfo info; info.joint_names = {"elbow"};
  info.lower = Eigen::VectorXd::Constant(1, -1); info.upper = -info.lower;
  info.velocity = Eigen::VectorXd::Constant(1, 0.5);
  Eigen::VectorXd q = Eigen::VectorXd::Constant(1, 0.95), lo, hi;
  ASSERT_TRUE(constraint::JointLimitConstraint::bounds(info, q, 0.02, 0.2, 0.01, lo, hi));
  EXPECT_NEAR(hi[0], 0.1, 1e-9); EXPECT_NEAR(lo[0], -0.5, 1e-9);
  q[0] = 1; ASSERT_TRUE(constraint::JointLimitConstraint::bounds(info, q, 0.02, 0.2, 0.01, lo, hi));
  EXPECT_DOUBLE_EQ(hi[0], 0); EXPECT_LT(lo[0], 0);
  q[0] = 1.1; EXPECT_FALSE(constraint::JointLimitConstraint::bounds(info, q, 0.02, 0.2, 0.01, lo, hi));
  ModelInfo empty;
  EXPECT_TRUE(constraint::JointLimitConstraint::bounds(empty, Eigen::VectorXd{}, 0.02, 0.2, 0.01, lo, hi));
}
TEST(Contact, ExactlyFiveRowsAllowOnlyHingeRotationAndCorrectDrift)
{
  MotionContext ctx; ctx.contact_active = true; ctx.state.contact_jacobian = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd a; Eigen::VectorXd b;
  ASSERT_TRUE(constraint::RevoluteContactConstraint::equality(ctx, 3, 3, a, b));
  EXPECT_EQ(a.rows(), 5); EXPECT_EQ(a.fullPivLu().rank(), 5);
  Vector6 free = Vector6::Zero(); free[4] = 1; EXPECT_NEAR((a * free).norm(), 0, 1e-12);
  ctx.state.contact.translation().x() = 0.01;
  ctx.state.contact.linear() = rotationExp(Eigen::Vector3d(0, 2.5, 0));
  ASSERT_TRUE(constraint::RevoluteContactConstraint::equality(ctx, 3, 3, a, b));
  EXPECT_NEAR(b[0], -0.03, 1e-12); EXPECT_LT(b.tail<2>().norm(), 1e-10);
  ctx.state.contact.linear() = rotationExp(Eigen::Vector3d(0.02, 0, 0)) * ctx.state.contact.linear();
  ASSERT_TRUE(constraint::RevoluteContactConstraint::equality(ctx, 3, 3, a, b));
  QPProblem p; p.reset(6, 1); p.appendConstraints(a, b, b);
  QpOasesSolver solver; Eigen::VectorXd x; std::string status;
  ASSERT_TRUE(solver.solve(p, x, status)); EXPECT_LT(x[3], 0);
  ctx.locked_hinge_world.setZero(); EXPECT_FALSE(constraint::RevoluteContactConstraint::equality(ctx, 3, 3, a, b));
}
