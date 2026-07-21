#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace pi_arm_can
{

struct TransmissionConfig
{
  int joint_id;
  std::uint8_t motor_id;
  double reduction_ratio;
  int motor_direction;
  double cable_ratio;
  bool nonlinear;
};

class Transmission
{
public:
  explicit Transmission(TransmissionConfig config);

  const TransmissionConfig & config() const noexcept {return config_;}
  double joint_position_to_motor(double joint_angle_deg) const;
  double motor_position_to_joint(double motor_angle_deg) const;
  double joint_velocity_to_motor(double joint_speed_dps) const;
  double motor_velocity_to_joint(double motor_speed_dps) const;

private:
  double output_to_motor(double output_value) const noexcept;
  double motor_to_output(double motor_value) const noexcept;

  TransmissionConfig config_;
};

class TransmissionRegistry
{
public:
  explicit TransmissionRegistry(std::vector<TransmissionConfig> configs);

  const Transmission & by_joint_id(int joint_id) const;
  const Transmission & by_motor_id(std::uint8_t motor_id) const;
  std::vector<std::uint8_t> motor_ids() const;

private:
  std::vector<Transmission> transmissions_;
  std::unordered_map<int, std::size_t> by_joint_;
  std::unordered_map<std::uint8_t, std::size_t> by_motor_;
};

}  // namespace pi_arm_can
