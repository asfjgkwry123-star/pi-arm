#include "pi_arm_can/protocol.hpp"

#include <cmath>
#include <limits>
#include <string>

namespace pi_arm_can
{
namespace
{

void validate_motor_id(const std::uint8_t motor_id)
{
  if (motor_id == 0U || kCanIdBase + motor_id > 0x7FFU) {
    throw std::invalid_argument("motor_id must produce a valid standard CAN identifier");
  }
}

}  // namespace

CanFrame make_request(const std::uint8_t motor_id, const Opcode opcode)
{
  validate_motor_id(motor_id);
  CanFrame frame;
  frame.id = kCanIdBase + motor_id;
  frame.data[0] = static_cast<std::uint8_t>(opcode);
  return frame;
}

CanFrame make_position_command(
  const std::uint8_t motor_id, const double motor_angle_deg,
  const double max_motor_speed_dps)
{
  if (!std::isfinite(motor_angle_deg) || !std::isfinite(max_motor_speed_dps)) {
    throw std::invalid_argument("position command values must be finite");
  }

  const auto speed = std::llround(std::abs(max_motor_speed_dps));
  if (speed < 1 || speed > std::numeric_limits<std::uint16_t>::max()) {
    throw std::out_of_range("motor speed must round to [1, 65535] dps");
  }

  const auto angle = std::llround(motor_angle_deg * 100.0);
  if (angle < std::numeric_limits<std::int32_t>::min() ||
    angle > std::numeric_limits<std::int32_t>::max())
  {
    throw std::out_of_range("motor angle exceeds the 0.01 degree int32 range");
  }

  auto frame = make_request(motor_id, Opcode::POSITION_CONTROL_2);
  frame.data[1] = 0U;
  write_le<std::uint16_t>(&frame.data[2], static_cast<std::uint16_t>(speed));
  write_le<std::int32_t>(&frame.data[4], static_cast<std::int32_t>(angle));
  return frame;
}

std::int64_t decode_signed_56_le(const std::uint8_t * const data, const std::size_t size)
{
  if (data == nullptr || size != 7U) {
    throw std::invalid_argument("signed 56-bit value requires exactly seven bytes");
  }
  std::uint64_t value = 0U;
  for (std::size_t i = 0; i < size; ++i) {
    value |= static_cast<std::uint64_t>(data[i]) << (8U * i);
  }
  if ((value & (std::uint64_t{1} << 55U)) != 0U) {
    value |= 0xFF00000000000000ULL;
  }
  return static_cast<std::int64_t>(value);
}

double decode_multi_turn_angle_deg(const CanFrame & frame)
{
  if (frame.data[0] != static_cast<std::uint8_t>(Opcode::READ_MULTI_TURN_ANGLE)) {
    throw std::invalid_argument("frame is not a multi-turn angle response");
  }
  return static_cast<double>(decode_signed_56_le(&frame.data[1], 7U)) / 100.0;
}

}  // namespace pi_arm_can
