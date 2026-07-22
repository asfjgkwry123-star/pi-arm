#pragma once

#include "pi_arm_hardware/backend.hpp"

#include <hardware_interface/system_interface.hpp>
#include <pi_arm_interfaces/msg/hardware_state.hpp>
#include <pi_arm_interfaces/srv/manage_motor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

namespace pi_arm_hardware
{

class PiArmSystem final : public hardware_interface::SystemInterface
{
public:
  ~PiArmSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using ManageMotor = pi_arm_interfaces::srv::ManageMotor;

  bool parse_configuration();
  void start_auxiliary_node();
  void stop_auxiliary_node() noexcept;
  void publish_hardware_state();
  void create_management_service(
    const std::string & name, pi_arm_can::ManagementOperation operation);

  BackendConfig backend_config_;
  std::unique_ptr<Backend> backend_;
  std::string backend_name_;
  std::vector<std::string> joint_names_;
  std::vector<double> state_positions_;
  std::vector<double> state_velocities_;
  std::vector<double> command_positions_;
  std::vector<double> command_velocities_;
  HardwareSample latest_sample_;
  std::mutex sample_mutex_;
  std::atomic_bool hardware_active_{false};
  // One-shot hold: after activate, copy state→command once when feedback is
  // first all_fresh. Controllers then own command; do not re-hold every cycle.
  bool command_hold_synced_{false};

  double state_publish_rate_hz_{10.0};
  std::string node_name_{"pi_arm_hardware_services"};
  std::unique_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Publisher<pi_arm_interfaces::msg::HardwareState>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  std::vector<rclcpp::Service<ManageMotor>::SharedPtr> management_services_;
};

}  // namespace pi_arm_hardware
