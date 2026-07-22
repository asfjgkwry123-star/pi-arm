#include "pi_arm_hardware/backend.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace
{

pi_arm_hardware::BackendConfig make_config()
{
  pi_arm_hardware::BackendConfig config;
  const std::vector<std::uint8_t> motor_ids{1U, 2U, 3U, 4U, 5U, 6U};
  for (std::size_t index = 0; index < motor_ids.size(); ++index) {
    config.transmissions.push_back(
      pi_arm_can::TransmissionConfig{
        static_cast<int>(index + 1U), motor_ids[index],
        index < 3U ? 36.0 : 6.0, 1, 1.0, index == 3U || index == 4U});
  }
  // In production these limits are filled from the URDF by the hardware plugin.
  config.joint_lower_limits_rad.assign(motor_ids.size(), -3.14);
  config.joint_upper_limits_rad.assign(motor_ids.size(), 3.14);
  config.joint_velocity_limits_rad_s.assign(motor_ids.size(), 0.17453292519943295);
  return config;
}

TEST(MockBackend, FollowsPositionAndVelocityCommands)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  const std::vector<double> positions{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  const std::vector<double> velocities{0.0, -0.02, 0.03, 0.0, 0.05, 0.06};
  ASSERT_TRUE(backend.write(positions, velocities));

  ASSERT_TRUE(backend.read(sample));
  EXPECT_EQ(sample.positions_rad, positions);
  EXPECT_EQ(sample.velocities_rad_s, velocities);
  EXPECT_FALSE(sample.in_motion[0]);
  EXPECT_TRUE(sample.in_motion[1]);
}

TEST(MockBackend, ClampsVelocityAboveLimit)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  const std::vector<double> positions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> velocities{0.2, 0.0, 0.0, 0.0, 0.0, 0.0};
  ASSERT_TRUE(backend.write(positions, velocities));

  ASSERT_TRUE(backend.read(sample));
  EXPECT_NEAR(sample.velocities_rad_s[0], 0.17453292519943295, 1e-9);
  EXPECT_TRUE(sample.in_motion[0]);
}

TEST(MockBackend, DeactivateClearsMotionAndDisconnects)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  backend.activate();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  const std::vector<double> positions{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  const std::vector<double> velocities{0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
  ASSERT_TRUE(backend.write(positions, velocities));

  backend.deactivate();
  ASSERT_TRUE(backend.read(sample));
  EXPECT_FALSE(sample.connected);
  for (const double velocity : sample.velocities_rad_s) {
    EXPECT_DOUBLE_EQ(velocity, 0.0);
  }
  for (const bool moving : sample.in_motion) {
    EXPECT_FALSE(moving);
  }
  for (const bool fresh : sample.fresh) {
    EXPECT_FALSE(fresh);
  }
}

TEST(MockBackend, AppliesManagementToOneMotorOrAll)
{
  pi_arm_hardware::MockBackend backend(make_config());
  backend.configure();
  pi_arm_hardware::HardwareSample sample;
  for (int index = 0; index < 3; ++index) {
    ASSERT_TRUE(backend.read(sample));
  }
  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 4).success);
  ASSERT_TRUE(backend.read(sample));
  EXPECT_TRUE(sample.enabled[3]);
  EXPECT_FALSE(sample.enabled[0]);

  ASSERT_TRUE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_RUN, 0).success);
  ASSERT_TRUE(backend.read(sample));
  for (const bool enabled : sample.enabled) {
    EXPECT_TRUE(enabled);
  }
  EXPECT_FALSE(
    backend.manage(pi_arm_can::ManagementOperation::MOTOR_OFF, 7).success);
}

}  // namespace
