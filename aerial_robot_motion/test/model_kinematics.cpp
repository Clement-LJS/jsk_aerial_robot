#include <aerial_robot_motion/core/model_adapter.h>
#include <aerial_robot_motion/cost/cartesian_pose.h>
#include <aerial_robot_motion/cost/state_velocity.h>
#include <aerial_robot_motion/constraint/revolute_contact.h>
#include <aerial_robot_motion/constraint/joint_limit.h>
#include <aerial_robot_motion/constraint/state_limit.h>
#include <aerial_robot_motion/solver/qpoases_solver.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <boost/make_shared.hpp>
#include <gtest/gtest.h>
#include <sstream>

using namespace aerial_robot_motion;
namespace
{
std::string fixture(bool articulated)
{
  const std::string inertia = "<inertial><mass value='0.1'/><inertia ixx='0.01' iyy='0.01' izz='0.01' ixy='0' ixz='0' iyz='0'/></inertial>";
  std::ostringstream s;
  s << "<robot name='motion_fixture'><baselink name='body'/><thrust_link name='thrust'/><m_f_rate value='0.02'/>"
    << "<link name='test_root'/><link name='body'>" << inertia << "</link>"
    << "<joint name='base_mount' type='fixed'><parent link='test_root'/><child link='body'/>"
    << "<origin xyz='0.1 -0.2 0.03' rpy='0.1 0.2 -0.3'/></joint>";
  if (articulated)
    s << "<link name='aux'>" << inertia << "</link><joint name='internal_aux' type='revolute'>"
      << "<parent link='body'/><child link='aux'/><axis xyz='0 0 1'/><limit lower='-2' upper='2' velocity='1' effort='2'/></joint>";
  s << "<link name='link2'>" << inertia << "</link><joint name='joint_elbow' type='"
    << (articulated ? "revolute" : "fixed") << "'><parent link='body'/><child link='link2'/>"
    << "<origin xyz='0.2 0 0'/><axis xyz='0 1 0'/><limit lower='-1.5' upper='1.5' velocity='0.5' effort='2'/></joint>"
    << "<link name='link3'>" << inertia << "</link><joint name='wrist_mount' type='fixed'>"
    << "<parent link='link2'/><child link='link3'/><origin xyz='0.4 0 0' rpy='0.2 0 0'/></joint>"
    << "<link name='tool_tip'/><joint name='tool_mount' type='fixed'><parent link='link3'/><child link='tool_tip'/>"
    << "<origin xyz='0.1 0 0'/></joint>";
  const double xy[4][2] = {{0.2,0.2},{-0.2,0.2},{-0.2,-0.2},{0.2,-0.2}};
  for (int i = 1; i <= 4; ++i)
    s << "<link name='thrust" << i << "'/><joint name='thrust_mount" << i << "' type='fixed'>"
      << "<parent link='body'/><child link='thrust" << i << "'/><origin xyz='" << xy[i-1][0] << " " << xy[i-1][1] << " 0'/></joint>"
      << "<link name='prop" << i << "'/><joint name='rotor" << i << "' type='continuous'><parent link='thrust" << i << "'/>"
      << "<child link='prop" << i << "'/><axis xyz='0 0 " << (i%2 ? 1 : -1) << "'/>"
      << "<limit lower='0' upper='20' velocity='100' effort='20'/></joint>";
  return s.str() + "</robot>";
}
struct MotionFixture
{
  ros::NodeHandle pnh{"~model"};
  boost::shared_ptr<aerial_robot_model::RobotModel> raw;
  ModelAdapter adapter;
  MotionState state;
  explicit MotionFixture(bool articulated)
  {
    ros::param::set("robot_description", fixture(articulated));
    pnh.setParam("tool_frame", "tool_tip"); pnh.setParam("contact_frame", "body");
    pnh.setParam("contact_offset_xyz", std::vector<double>{-0.3,0,0});
    pnh.setParam("tool_offset_xyz", std::vector<double>{0.07,0.03,0.02});
    if (articulated) raw = boost::make_shared<aerial_robot_model::transformable::RobotModel>();
    else raw = boost::make_shared<aerial_robot_model::RobotModel>();
    adapter.initializeModel(raw, pnh);
    sensor_msgs::JointState msg;
    if (articulated) { msg.name = {"joint_elbow", "internal_aux"}; msg.position = {0.2, 0.3}; }
    KDL::JntArray full; std::string error;
    if (!adapter.readJoints(msg, full, error)) throw std::runtime_error(error);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(0.2, -0.3, 1);
    pose.linear() = rotationExp(Eigen::Vector3d(0.2,-0.1,0.3));
    state = adapter.measured(pose, full, ros::Time(1));
  }
};
}

TEST(ModelKinematics, NonArticulatedAndSelectedJointColumnsMatchFiniteDifferences)
{
  for (bool articulated : {false, true})
  {
    MotionFixture f(articulated);
    ASSERT_EQ(f.adapter.info().dimension(), articulated ? 7 : 6);
    if (articulated)
    {
      ASSERT_EQ(f.adapter.info().joint_names[0], "joint_elbow");
      ASSERT_NE(f.adapter.info().joint_indices[0], f.raw->getJointIndexMap().at("internal_aux"));
    }
    const double dt = 1e-6;
    for (int column = 0; column < f.adapter.info().dimension(); ++column)
    {
      Eigen::VectorXd unit = Eigen::VectorXd::Zero(f.adapter.info().dimension()); unit[column] = 1;
      const auto next = f.adapter.integrate(f.state, unit, dt, ros::Time(2));
      const Vector6 numerical = poseError(next.tool, f.state.tool) / dt;
      EXPECT_LT((numerical - f.state.tool_jacobian.col(column)).norm(), 2e-6) << column;
      const Vector6 contact = poseError(next.contact, f.state.contact) / dt;
      EXPECT_LT((contact - f.state.contact_jacobian.col(column)).norm(), 2e-6) << column;
      if (articulated) EXPECT_DOUBLE_EQ(next.full_joints(f.raw->getJointIndexMap().at("internal_aux")), 0.3);
    }
  }
}
TEST(ModelKinematics, InvalidJointFeedbackIsRejected)
{
  MotionFixture f(true); sensor_msgs::JointState msg; KDL::JntArray q; std::string error;
  msg.name = {"joint_elbow"}; msg.position = {0};
  EXPECT_FALSE(f.adapter.readJoints(msg, q, error));
  msg.name = {"joint_elbow", "internal_aux"};
  EXPECT_FALSE(f.adapter.readJoints(msg, q, error));
  msg.position = {0, std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(f.adapter.readJoints(msg, q, error));
}
TEST(ModelKinematics, OnlineArticulatedPerchingTracksPitchWithBoundedContactError)
{
  MotionFixture f(true);
  ros::NodeHandle pnh("~perching_test");
  pnh.setParam("weights", std::vector<double>{0,0,0,1,1,1});
  cost::CartesianPoseCost cartesian; cartesian.initialize(pnh, f.adapter.info());
  cost::StateVelocityCost regularizer; regularizer.initialize(pnh, f.adapter.info());
  constraint::StateLimitConstraint state_limits; state_limits.initialize(pnh, f.adapter.info());
  constraint::JointLimitConstraint joint_limits; joint_limits.initialize(pnh, f.adapter.info());
  constraint::RevoluteContactConstraint contact; contact.initialize(pnh, f.adapter.info());
  MotionContext ctx; ctx.state = f.state; ctx.contact_active = true; ctx.dt = 0.02;
  ctx.locked_contact_position = ctx.state.contact.translation(); ctx.hinge_contact = Eigen::Vector3d::UnitY();
  ctx.locked_hinge_world = ctx.state.contact.linear() * ctx.hinge_contact;
  ctx.target = ctx.state.tool;
  ctx.target.linear() = rotationExp(-0.05235987756 * ctx.locked_hinge_world) * ctx.state.tool.linear();
  const auto initial = ctx.state;
  QpOasesSolver solver; double maximum_contact_error = 0, maximum_angle_error = 0;
  for (int cycle = 0; cycle < 400; ++cycle)
  {
    QPProblem p; p.reset(f.adapter.info().dimension(), 1e-6);
    ASSERT_TRUE(cartesian.update(ctx, p)); ASSERT_TRUE(regularizer.update(ctx, p));
    ASSERT_TRUE(state_limits.update(ctx, p)); ASSERT_TRUE(joint_limits.update(ctx, p)); ASSERT_TRUE(contact.update(ctx, p));
    Eigen::VectorXd velocity; std::string status;
    ASSERT_TRUE(solver.solve(p, velocity, status)) << cycle << ": " << status;
    const auto next = f.adapter.integrate(ctx.state, velocity, ctx.dt, ctx.state.stamp + ros::Duration(ctx.dt));
    // Perfect tracking plant supplies fresh feedback at the next update; the
    // reference generator itself has no path sequence or internal IK loop.
    ctx.state = f.adapter.measured(f.adapter.framePose(next, f.adapter.baselink(), Eigen::Isometry3d::Identity()),
                                  next.full_joints, next.stamp);
    maximum_contact_error = std::max(maximum_contact_error, (ctx.state.contact.translation() - ctx.locked_contact_position).norm());
    maximum_angle_error = std::max(maximum_angle_error,
      (ctx.state.contact.linear() * ctx.hinge_contact).cross(ctx.locked_hinge_world).norm());
  }
  EXPECT_LT(poseError(ctx.target, ctx.state.tool).tail<3>().norm(), 1e-5);
  EXPECT_GT(rotationLog(ctx.state.root.linear() * initial.root.linear().transpose()).norm(), 0.005);
  EXPECT_GT((ctx.state.joints - initial.joints).norm(), 0.005);
  EXPECT_LT(maximum_contact_error, 2e-5); EXPECT_LT(maximum_angle_error, 1e-7);
  std::cout << "Contact maximum position error: " << maximum_contact_error << " m; hinge-axis error: "
            << maximum_angle_error << "; final orientation error: "
            << poseError(ctx.target, ctx.state.tool).tail<3>().norm() << " rad\n";
  ctx.contact_active = false; QPProblem free; free.reset(f.adapter.info().dimension());
  ASSERT_TRUE(contact.update(ctx, free)); EXPECT_EQ(free.A.rows(), 0);
}
