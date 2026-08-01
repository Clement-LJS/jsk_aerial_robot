#include <aerial_robot_simulation/mujoco/mujoco_cutting_force_model.h>

#include <gtest/gtest.h>

namespace
{
aerial_robot_simulation::MujocoCuttingForceModel::Profile makeProfile()
{
  aerial_robot_simulation::MujocoCuttingForceModel::Profile profile;
  profile.preload_force = 0.2;
  profile.maximum_force = 10.0;
  profile.ripple_amplitude = 0.0;
  profile.ripple_wavelength = 0.01;
  profile.layers.push_back({0.01, 10.0, 1.0});
  profile.layers.push_back({0.01, 20.0, 2.0});
  profile.layers.push_back({0.01, 30.0, 3.0});
  return profile;
}

aerial_robot_simulation::MujocoCuttingForceModel::Config makeConfig()
{
  aerial_robot_simulation::MujocoCuttingForceModel::Config config;
  config.free_travel = 0.005;
  config.force_ramp_time = 0.0;
  config.force_filter_time_constant = 0.0;
  config.maximum_control_dt = 0.1;
  return config;
}

aerial_robot_simulation::MujocoCuttingForceModel::Input makeInput(double travel, double velocity, double dt = 0.01)
{
  aerial_robot_simulation::MujocoCuttingForceModel::Input input;
  input.enabled = true;
  input.cutting_active = true;
  input.valid_geometry = true;
  input.dt = dt;
  input.signed_travel = travel;
  input.tangential_velocity = velocity;
  return input;
}
}  // namespace

TEST(MujocoCuttingForceModelTest, InactiveForceIsZero)
{
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(makeProfile(), makeConfig()));
  auto input = makeInput(0.1, 0.0);
  input.enabled = false;
  const auto output = model.update(input);
  EXPECT_DOUBLE_EQ(output.applied_force, 0.0);
}

TEST(MujocoCuttingForceModelTest, NoForceBeforeFreeTravel)
{
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(makeProfile(), makeConfig()));
  const auto output = model.update(makeInput(0.004, 0.0));
  EXPECT_FALSE(output.contact);
  EXPECT_DOUBLE_EQ(output.applied_force, 0.0);
}

TEST(MujocoCuttingForceModelTest, ContactAfterFreeTravel)
{
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(makeProfile(), makeConfig()));
  const auto output = model.update(makeInput(0.006, 0.0));
  EXPECT_TRUE(output.contact);
  EXPECT_GT(output.applied_force, 0.0);
}

TEST(MujocoCuttingForceModelTest, ForceContinuityAtLayerBoundaries)
{
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(makeProfile(), makeConfig()));
  const auto a = model.update(makeInput(0.015 - 1e-6, 0.0));
  model.reset();
  const auto b = model.update(makeInput(0.015 + 1e-6, 0.0));
  EXPECT_NEAR(a.applied_force, b.applied_force, 1e-3);
}

TEST(MujocoCuttingForceModelTest, DampingUsesOnlyForwardVelocity)
{
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(makeProfile(), makeConfig()));
  const auto forward = model.update(makeInput(0.02, 0.5));
  model.reset();
  const auto reverse = model.update(makeInput(0.02, -0.5));
  EXPECT_GT(forward.applied_force, reverse.applied_force);
}

TEST(MujocoCuttingForceModelTest, SaturationWorks)
{
  auto profile = makeProfile();
  profile.maximum_force = 0.3;
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(profile, makeConfig()));
  const auto output = model.update(makeInput(1.0, 100.0));
  EXPECT_LE(output.applied_force, 0.3);
}

TEST(MujocoCuttingForceModelTest, CompletionWorks)
{
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(makeProfile(), makeConfig()));
  const auto output = model.update(makeInput(0.05, 0.0));
  EXPECT_TRUE(output.completed);
}

TEST(MujocoCuttingForceModelTest, DisableClearsForceImmediately)
{
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(makeProfile(), makeConfig()));
  EXPECT_GT(model.update(makeInput(0.03, 0.0)).applied_force, 0.0);
  auto input = makeInput(0.03, 0.0);
  input.cutting_active = false;
  const auto output = model.update(input);
  EXPECT_DOUBLE_EQ(output.applied_force, 0.0);
}

TEST(MujocoCuttingForceModelTest, ResetClearsFilterRampAndReference)
{
  auto config = makeConfig();
  config.force_ramp_time = 0.1;
  config.force_filter_time_constant = 0.05;
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(makeProfile(), config));
  EXPECT_GT(model.update(makeInput(0.03, 0.0)).applied_force, 0.0);
  model.reset();
  const auto output = model.update(makeInput(0.004, 0.0));
  EXPECT_DOUBLE_EQ(output.applied_force, 0.0);
}

TEST(MujocoCuttingForceModelTest, InvalidProfileRejected)
{
  auto profile = makeProfile();
  profile.layers[0].thickness = -1.0;
  aerial_robot_simulation::MujocoCuttingForceModel model;
  EXPECT_FALSE(model.configure(profile, makeConfig()));
}

TEST(MujocoCuttingForceModelTest, InvalidStateProducesSafeZero)
{
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(makeProfile(), makeConfig()));
  auto input = makeInput(0.03, 0.0);
  input.valid_geometry = false;
  const auto output = model.update(input);
  EXPECT_DOUBLE_EQ(output.applied_force, 0.0);
}

TEST(MujocoCuttingForceModelTest, OversizedDtProducesSafeZero)
{
  aerial_robot_simulation::MujocoCuttingForceModel model;
  ASSERT_TRUE(model.configure(makeProfile(), makeConfig()));
  const auto output = model.update(makeInput(0.03, 0.0, 0.5));
  EXPECT_DOUBLE_EQ(output.applied_force, 0.0);
  EXPECT_FALSE(output.dt_valid);
}
