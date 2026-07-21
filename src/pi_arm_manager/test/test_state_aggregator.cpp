#include <gtest/gtest.h>

#include "pi_arm_manager/state_aggregator.hpp"

using pi_arm_interfaces::msg::HardwareState;
using pi_arm_interfaces::msg::RobotState;
using pi_arm_interfaces::msg::TaskState;

static HardwareState healthy_hardware()
{
  HardwareState state;
  state.connected = true;
  state.joint_names = {"joint1", "joint2"};
  state.positions = {0.0, 0.0};
  state.velocities = {0.0, 0.0};
  state.fresh = {true, true};
  state.enabled = {true, true};
  state.errors = {false, false};
  state.moving = {false, false};
  state.error_codes = {0, 0};
  state.ages_sec = {0.01, 0.01};
  return state;
}

TEST(StateAggregator, PhysicalMotionIsRunningWithoutTask)
{
  auto hardware = healthy_hardware();
  TaskState task;
  task.status = TaskState::IDLE;
  hardware.moving[1] = true;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::RUNNING);
}

TEST(StateAggregator, AppliesStrictPriority)
{
  auto hardware = healthy_hardware();
  TaskState task;
  task.status = TaskState::RUNNING;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::RUNNING);

  hardware.enabled[0] = false;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISABLED);
  hardware.errors[0] = true;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::FAULT);
  hardware.connected = false;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISCONNECTED);
}

TEST(StateAggregator, ReadyRequiresHealthyEnabledHardware)
{
  auto hardware = healthy_hardware();
  TaskState task;
  task.status = TaskState::IDLE;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::READY);
  hardware.fresh[1] = false;
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISCONNECTED);
}

TEST(StateAggregator, RejectsIncompleteHardwareFrames)
{
  auto hardware = healthy_hardware();
  TaskState task;
  task.status = TaskState::IDLE;
  hardware.velocities.pop_back();
  EXPECT_EQ(pi_arm_manager::aggregate_state(hardware, task), RobotState::DISCONNECTED);
}
