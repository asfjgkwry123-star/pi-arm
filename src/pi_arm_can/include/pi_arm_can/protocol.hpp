#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <type_traits>

namespace pi_arm_can
{

constexpr std::uint32_t kCanIdBase = 0x140U;
constexpr std::size_t kCanPayloadSize = 8U;

enum class Opcode : std::uint8_t
{
  MOTOR_OFF = 0x80,
  MOTOR_RUN = 0x88,
  SET_CURRENT_POSITION_ZERO = 0x19,
  READ_MULTI_TURN_ANGLE = 0x92,
  READ_STATUS_1 = 0x9A,
  CLEAR_ERROR_FLAGS = 0x9B,
  READ_STATUS_2 = 0x9C,
  POSITION_CONTROL_2 = 0xA4,
};

struct CanFrame
{
  std::uint32_t id{0U};
  std::array<std::uint8_t, kCanPayloadSize> data{};
};

CanFrame make_request(std::uint8_t motor_id, Opcode opcode);
CanFrame make_position_command(
  std::uint8_t motor_id, double motor_angle_deg, double max_motor_speed_dps);

std::int64_t decode_signed_56_le(const std::uint8_t * data, std::size_t size);
double decode_multi_turn_angle_deg(const CanFrame & frame);

template<typename T>
T read_le(const std::uint8_t * data)
{
  using Unsigned = std::make_unsigned_t<T>;
  Unsigned value = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    value |= static_cast<Unsigned>(data[i]) << (8U * i);
  }
  return static_cast<T>(value);
}

template<typename T>
void write_le(std::uint8_t * data, T value)
{
  using Unsigned = std::make_unsigned_t<T>;
  const auto bits = static_cast<Unsigned>(value);
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    data[i] = static_cast<std::uint8_t>((bits >> (8U * i)) & 0xFFU);
  }
}

}  // namespace pi_arm_can
