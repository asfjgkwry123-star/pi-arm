#pragma once

#include "pi_arm_can/can_driver.hpp"
#include "pi_arm_can/transmission.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace pi_arm_hardware
{

struct BackendConfig
{
  std::string can_interface{"can0"};
  std::vector<pi_arm_can::TransmissionConfig> transmissions;
  // Per-joint limits, filled by the hardware plugin from the URDF <limit> tags.
  // The single source of truth for these values is the robot description.
  std::vector<double> joint_lower_limits_rad;
  std::vector<double> joint_upper_limits_rad;
  std::vector<double> joint_velocity_limits_rad_s;
  std::chrono::milliseconds poll_interval{20};
  std::size_t status1_poll_divider{5U};
  double moving_speed_threshold_rad_s{0.00017453292519943296};
  double state_stale_timeout_sec{1.0};
};

struct HardwareSample
{
  std::vector<double> positions_rad;
  std::vector<double> velocities_rad_s;
  std::vector<bool> enabled;
  std::vector<bool> has_error;
  std::vector<bool> in_motion;
  std::vector<bool> fresh;
  std::vector<std::int32_t> error_codes;
  std::vector<double> ages_sec;
  bool connected{false};
  std::string driver_mode{"OFFLINE"};
  std::string management_operation{"NONE"};
  std::string error_message;
};

struct ManagementResult
{
  bool success{false};
  int code{-1};
  std::string message;
};

// Thread-safety contract: read() and write() are called from the ros2_control
// loop while manage() is called concurrently from a service thread. Every
// implementation must make read()/write() safe against a concurrent manage()
// without blocking the control loop for the duration of a management command.
// Lifecycle methods (configure/activate/deactivate/cleanup) are never called
// concurrently with each other or with read()/write()/manage(); the plugin
// guarantees this ordering.
class Backend
{
public:
  virtual ~Backend() = default;
  virtual void configure() = 0;
  virtual void activate() = 0;
  virtual void deactivate() noexcept = 0;
  virtual void cleanup() noexcept = 0;
  virtual bool read(HardwareSample & sample) noexcept = 0;
  virtual bool write(
    const std::vector<double> & positions_rad,
    const std::vector<double> & velocities_rad_s) noexcept = 0;
  virtual ManagementResult manage(
    pi_arm_can::ManagementOperation operation, int motor_id) noexcept = 0;
};

class RealBackend final : public Backend
{
public:
  explicit RealBackend(BackendConfig config);
  ~RealBackend() override;

  void configure() override;
  void activate() override;
  void deactivate() noexcept override;
  void cleanup() noexcept override;
  bool read(HardwareSample & sample) noexcept override;
  bool write(
    const std::vector<double> & positions_rad,
    const std::vector<double> & velocities_rad_s) noexcept override;
  ManagementResult manage(
    pi_arm_can::ManagementOperation operation, int motor_id) noexcept override;

private:
  BackendConfig config_;
  pi_arm_can::TransmissionRegistry transmissions_;
  std::unique_ptr<pi_arm_can::CanDriver> driver_;
};

class MockBackend final : public Backend
{
public:
  explicit MockBackend(BackendConfig config);

  void configure() override;
  void activate() override;
  void deactivate() noexcept override;
  void cleanup() noexcept override;
  bool read(HardwareSample & sample) noexcept override;
  bool write(
    const std::vector<double> & positions_rad,
    const std::vector<double> & velocities_rad_s) noexcept override;
  ManagementResult manage(
    pi_arm_can::ManagementOperation operation, int motor_id) noexcept override;

private:
  bool valid_motor_id(int motor_id) const noexcept;

  BackendConfig config_;
  // Guards sample_/stationary_samples_/flags: read()/write() run on the control
  // loop while manage() runs on a service thread. Mock operations are cheap, so
  // a single mutex is sufficient and never blocks for long.
  mutable std::mutex mutex_;
  HardwareSample sample_;
  std::vector<std::size_t> stationary_samples_;
  bool configured_{false};
  bool active_{false};
};

}  // namespace pi_arm_hardware
