#include "pi_arm_can/can_driver.hpp"
#include "pi_arm_can/protocol.hpp"
#include "pi_arm_can/transmission.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{

TEST(Protocol, EncodesPositionControlPayloadLittleEndian)
{
  const auto frame = pi_arm_can::make_position_command(5U, -123.45, 321.0);
  EXPECT_EQ(frame.id, 0x145U);
  const std::array<std::uint8_t, 8> expected{
    0xA4, 0x00, 0x41, 0x01, 0xC7, 0xCF, 0xFF, 0xFF};
  EXPECT_EQ(frame.data, expected);
}

TEST(Protocol, RejectsSpeedThatRoundsToZero)
{
  EXPECT_THROW(
    pi_arm_can::make_position_command(1U, 0.0, 0.4), std::out_of_range);
}

TEST(Protocol, DecodesSigned56BitLimits)
{
  const std::array<std::uint8_t, 7> minus_one{
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  const std::array<std::uint8_t, 7> minimum{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80};
  EXPECT_EQ(pi_arm_can::decode_signed_56_le(minus_one.data(), minus_one.size()), -1);
  EXPECT_EQ(
    pi_arm_can::decode_signed_56_le(minimum.data(), minimum.size()),
    -(std::int64_t{1} << 55));
}

TEST(Transmission, LinearRoundTripAndDirection)
{
  const pi_arm_can::Transmission transmission({1, 1U, 36.0, -1, 2.0, false});
  EXPECT_DOUBLE_EQ(transmission.joint_position_to_motor(20.0), -360.0);
  EXPECT_DOUBLE_EQ(transmission.motor_position_to_joint(-360.0), 20.0);
  EXPECT_DOUBLE_EQ(transmission.joint_velocity_to_motor(4.0), -72.0);
}

TEST(Transmission, NonlinearMatchesPythonReference)
{
  const pi_arm_can::Transmission transmission({4, 5U, 6.0, 1, 1.0, true});
  const double motor = transmission.joint_position_to_motor(30.0);
  EXPECT_NEAR(motor, 343.7746770784939, 1e-9);
  EXPECT_NEAR(transmission.motor_position_to_joint(motor), 30.0, 1e-12);
  EXPECT_DOUBLE_EQ(transmission.joint_velocity_to_motor(10.0), 120.0);
  EXPECT_DOUBLE_EQ(transmission.motor_velocity_to_joint(120.0), 10.0);
}

TEST(DriverCache, ParsesStatusAndAngleFrames)
{
  pi_arm_can::DriverConfig config;
  config.motor_ids = {1U};
  pi_arm_can::CanDriver driver(config);

  pi_arm_can::CanFrame status;
  status.id = 0x141U;
  status.data = {0x9A, 0xFB, 0xE8, 0x03, 0x0C, 0xFE, 0x30, 0x02};
  driver.process_received_frame(status);
  auto state = driver.motor_state(1U);
  EXPECT_TRUE(state.status1_valid);
  EXPECT_EQ(state.temperature_c, -5);
  EXPECT_DOUBLE_EQ(state.voltage_v, 10.0);
  EXPECT_DOUBLE_EQ(state.current_a, -5.0);
  EXPECT_EQ(state.motor_state, 0x30);
  EXPECT_EQ(state.error_state, 0x02);

  pi_arm_can::CanFrame speed;
  speed.id = 0x141U;
  speed.data = {0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  driver.process_received_frame(speed);
  driver.process_received_frame(speed);
  driver.process_received_frame(speed);
  state = driver.motor_state(1U);
  EXPECT_EQ(state.stationary_samples, 3U);

  pi_arm_can::CanFrame angle;
  angle.id = 0x141U;
  angle.data = {0x92, 0xC7, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  driver.process_received_frame(angle);
  state = driver.motor_state(1U);
  EXPECT_TRUE(state.angle_valid);
  EXPECT_DOUBLE_EQ(state.angle_deg, -123.45);
}

}  // namespace
