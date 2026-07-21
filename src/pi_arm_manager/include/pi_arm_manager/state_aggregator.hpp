#pragma once

#include <algorithm>
#include <cmath>

#include "pi_arm_interfaces/msg/hardware_state.hpp"
#include "pi_arm_interfaces/msg/robot_state.hpp"
#include "pi_arm_interfaces/msg/task_state.hpp"

namespace pi_arm_manager
{

inline uint8_t aggregate_state(
  const pi_arm_interfaces::msg::HardwareState & hardware,
  const pi_arm_interfaces::msg::TaskState & task)
{
  const auto joint_count = hardware.joint_names.size();
  const bool complete = joint_count > 0 &&
    hardware.positions.size() == joint_count &&
    hardware.velocities.size() == joint_count &&
    hardware.enabled.size() == joint_count &&
    hardware.errors.size() == joint_count &&
    hardware.moving.size() == joint_count &&
    hardware.fresh.size() == joint_count &&
    hardware.ages_sec.size() == joint_count &&
    hardware.error_codes.size() == joint_count &&
    std::all_of(
    hardware.positions.begin(), hardware.positions.end(),
    [](double value) {return std::isfinite(value);}) &&
    std::all_of(
    hardware.velocities.begin(), hardware.velocities.end(),
    [](double value) {return std::isfinite(value);});
  if (!hardware.connected || !complete ||
    !std::all_of(hardware.fresh.begin(), hardware.fresh.end(), [](bool value) {return value;}))
  {
    return pi_arm_interfaces::msg::RobotState::DISCONNECTED;
  }
  if (std::any_of(hardware.errors.begin(), hardware.errors.end(), [](bool value) {return value;}) ||
    std::any_of(
      hardware.error_codes.begin(), hardware.error_codes.end(),
      [](int32_t value) {return value != 0;}))
  {
    return pi_arm_interfaces::msg::RobotState::FAULT;
  }
  if (hardware.enabled.empty() ||
    !std::all_of(hardware.enabled.begin(), hardware.enabled.end(), [](bool value) {return value;}))
  {
    return pi_arm_interfaces::msg::RobotState::DISABLED;
  }
  if (task.status == pi_arm_interfaces::msg::TaskState::RUNNING ||
    std::any_of(hardware.moving.begin(), hardware.moving.end(), [](bool value) {return value;}))
  {
    return pi_arm_interfaces::msg::RobotState::RUNNING;
  }
  return pi_arm_interfaces::msg::RobotState::READY;
}

inline const char * state_name(uint8_t state)
{
  switch (state) {
    case pi_arm_interfaces::msg::RobotState::DISABLED: return "DISABLED";
    case pi_arm_interfaces::msg::RobotState::READY: return "READY";
    case pi_arm_interfaces::msg::RobotState::RUNNING: return "RUNNING";
    case pi_arm_interfaces::msg::RobotState::FAULT: return "FAULT";
    default: return "DISCONNECTED";
  }
}

}  // namespace pi_arm_manager
