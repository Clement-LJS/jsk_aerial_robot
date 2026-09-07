#include <aerial_robot_motion/command/motion_command_manager.h>
#include <gtest/gtest.h>
#include <limits>

using namespace aerial_robot_motion;
using namespace aerial_robot_motion::command;

TEST(MotionCommand, SixNegativePitchIncrementsAccumulateThreeDegrees)
{
  MotionCommandManager command;
  command.setMeasuredPose(Eigen::Isometry3d::Identity(), ros::Time(1));
  Vector6 increment = Vector6::Zero();
  increment[4] = -0.5 * 3.14159265358979323846 / 180.0;
  for (int i = 0; i < 6; ++i) command.applyIncrement(increment, Eigen::Matrix3d::Identity());
  EXPECT_NEAR(rotationLog(command.target().linear()).y(), -3.0 * 3.14159265358979323846 / 180.0, 1e-12);
}

TEST(MotionCommand, UsesMeasuredAxesWhileKeepingAccumulatedNominalTarget)
{
  MotionCommandManager command;
  Eigen::Isometry3d measured = Eigen::Isometry3d::Identity();
  measured.linear() = rotationExp(Eigen::Vector3d(0, 0, 1.5707963267948966));
  command.setMeasuredPose(measured, ros::Time(1));
  Vector6 increment = Vector6::Zero();
  increment[0] = 0.01;
  command.applyIncrement(increment, measured.linear());
  EXPECT_NEAR(command.target().translation().x(), 0, 1e-12);
  EXPECT_NEAR(command.target().translation().y(), 0.01, 1e-12);
  measured.translation().z() = 0.5;
  command.setMeasuredPose(measured, ros::Time(2));
  EXPECT_NEAR(command.target().translation().z(), 0, 1e-12);
  command.applyIncrement(increment, measured.linear());
  EXPECT_NEAR(command.target().translation().y(), 0.02, 1e-12);
}

TEST(MotionCommand, RejectsInvalidInputWithoutChangingTarget)
{
  MotionCommandManager command;
  EXPECT_THROW(command.applyIncrement(Vector6::Zero(), Eigen::Matrix3d::Identity()), std::runtime_error);
  command.setMeasuredPose(Eigen::Isometry3d::Identity(), ros::Time(1));
  Vector6 increment = Vector6::Zero();
  increment[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(command.applyIncrement(increment, Eigen::Matrix3d::Identity()), std::invalid_argument);
  increment[0] = 2;
  EXPECT_THROW(command.applyIncrement(increment, Eigen::Matrix3d::Identity()), std::invalid_argument);
  EXPECT_TRUE(command.target().matrix().isApprox(Eigen::Matrix4d::Identity()));
}
