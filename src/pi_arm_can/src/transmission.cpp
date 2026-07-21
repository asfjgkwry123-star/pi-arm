#include "pi_arm_can/transmission.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace pi_arm_can
{
namespace
{

constexpr double kPi = 3.1415926535897932384626433832795;
double radians(const double degrees) {return degrees * kPi / 180.0;}
double degrees(const double radians_value) {return radians_value * 180.0 / kPi;}

}  // namespace

Transmission::Transmission(TransmissionConfig config)
: config_(std::move(config))
{
  if (config_.joint_id <= 0 || config_.motor_id == 0U) {
    throw std::invalid_argument("joint and motor identifiers must be positive");
  }
  if (!std::isfinite(config_.reduction_ratio) || config_.reduction_ratio <= 0.0 ||
    !std::isfinite(config_.cable_ratio) || config_.cable_ratio <= 0.0)
  {
    throw std::invalid_argument("transmission ratios must be finite and positive");
  }
  if (config_.motor_direction != -1 && config_.motor_direction != 1) {
    throw std::invalid_argument("motor direction must be -1 or 1");
  }
}

double Transmission::output_to_motor(const double output_value) const noexcept
{
  return config_.motor_direction * config_.reduction_ratio * output_value;
}

double Transmission::motor_to_output(const double motor_value) const noexcept
{
  return motor_value / (config_.motor_direction * config_.reduction_ratio);
}

double Transmission::joint_position_to_motor(const double joint_angle_deg) const
{
  if (!std::isfinite(joint_angle_deg)) {
    throw std::invalid_argument("joint angle must be finite");
  }
  if (!config_.nonlinear) {
    return output_to_motor(joint_angle_deg / config_.cable_ratio);
  }
  const double output_angle_rad = 2.0 * std::sin(radians(joint_angle_deg)) /
    config_.cable_ratio;
  return output_to_motor(degrees(output_angle_rad));
}

double Transmission::motor_position_to_joint(const double motor_angle_deg) const
{
  if (!std::isfinite(motor_angle_deg)) {
    throw std::invalid_argument("motor angle must be finite");
  }
  const double output_angle_deg = motor_to_output(motor_angle_deg);
  if (!config_.nonlinear) {
    return output_angle_deg * config_.cable_ratio;
  }
  const double argument = radians(output_angle_deg) * config_.cable_ratio / 2.0;
  if (argument < -1.0 || argument > 1.0) {
    throw std::domain_error("motor angle is outside the nonlinear cable inverse range");
  }
  return degrees(std::asin(argument));
}

double Transmission::joint_velocity_to_motor(const double joint_speed_dps) const
{
  if (!std::isfinite(joint_speed_dps)) {
    throw std::invalid_argument("joint speed must be finite");
  }
  const double gain = config_.nonlinear ? 2.0 : 1.0;
  return output_to_motor(gain * joint_speed_dps / config_.cable_ratio);
}

double Transmission::motor_velocity_to_joint(const double motor_speed_dps) const
{
  if (!std::isfinite(motor_speed_dps)) {
    throw std::invalid_argument("motor speed must be finite");
  }
  const double gain = config_.nonlinear ? 0.5 : 1.0;
  return motor_to_output(motor_speed_dps) * config_.cable_ratio * gain;
}

TransmissionRegistry::TransmissionRegistry(std::vector<TransmissionConfig> configs)
{
  if (configs.empty()) {
    throw std::invalid_argument("at least one transmission is required");
  }
  transmissions_.reserve(configs.size());
  for (auto & config : configs) {
    const std::size_t index = transmissions_.size();
    if (by_joint_.count(config.joint_id) != 0U || by_motor_.count(config.motor_id) != 0U) {
      throw std::invalid_argument("joint and motor identifiers must be unique");
    }
    by_joint_.emplace(config.joint_id, index);
    by_motor_.emplace(config.motor_id, index);
    transmissions_.emplace_back(std::move(config));
  }
}

const Transmission & TransmissionRegistry::by_joint_id(const int joint_id) const
{
  return transmissions_.at(by_joint_.at(joint_id));
}

const Transmission & TransmissionRegistry::by_motor_id(const std::uint8_t motor_id) const
{
  return transmissions_.at(by_motor_.at(motor_id));
}

std::vector<std::uint8_t> TransmissionRegistry::motor_ids() const
{
  std::vector<std::uint8_t> ids;
  ids.reserve(transmissions_.size());
  for (const auto & transmission : transmissions_) {
    ids.push_back(transmission.config().motor_id);
  }
  return ids;
}

}  // namespace pi_arm_can
