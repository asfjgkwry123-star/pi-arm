#include "pi_arm_hardware/pi_arm_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace pi_arm_hardware
{
namespace
{

template<typename T>
T required_joint_parameter(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & joint_name, const std::string & name)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    throw std::invalid_argument(
            "joint '" + joint_name + "' is missing required ros2_control parameter '" + name +
            "'");
  }
  std::istringstream stream(found->second);
  T value{};
  stream >> value;
  if (!stream || !(stream >> std::ws).eof()) {
    throw std::invalid_argument(
            "joint '" + joint_name + "' has an invalid ros2_control parameter '" + name + "'");
  }
  return value;
}

bool required_joint_flag(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & joint_name, const std::string & name)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    throw std::invalid_argument(
            "joint '" + joint_name + "' is missing required ros2_control parameter '" + name +
            "'");
  }
  if (found->second == "true" || found->second == "1") {
    return true;
  }
  if (found->second == "false" || found->second == "0") {
    return false;
  }
  throw std::invalid_argument(
          "joint '" + joint_name + "' parameter '" + name + "' must be true or false");
}

template<typename T>
T parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const T & fallback)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    return fallback;
  }
  std::istringstream stream(found->second);
  T value{};
  stream >> value;
  if (!stream || !(stream >> std::ws).eof()) {
    throw std::invalid_argument("invalid hardware parameter '" + name + "'");
  }
  return value;
}

std::string string_parameter_or(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name, const std::string & fallback)
{
  const auto found = parameters.find(name);
  return found == parameters.end() ? fallback : found->second;
}

#define PI_ARM_DEFINE_HAS_FIELD(field) \
  template<typename T, typename = void> struct has_##field : std::false_type {}; \
  template<typename T> struct has_##field<T, std::void_t<decltype(std::declval<T &>().field)>> \
  : std::true_type {}

PI_ARM_DEFINE_HAS_FIELD(header);
PI_ARM_DEFINE_HAS_FIELD(stamp);
PI_ARM_DEFINE_HAS_FIELD(backend);
PI_ARM_DEFINE_HAS_FIELD(driver_mode);
PI_ARM_DEFINE_HAS_FIELD(management_operation);
PI_ARM_DEFINE_HAS_FIELD(connected);
PI_ARM_DEFINE_HAS_FIELD(joint_names);
PI_ARM_DEFINE_HAS_FIELD(motor_ids);
PI_ARM_DEFINE_HAS_FIELD(positions);
PI_ARM_DEFINE_HAS_FIELD(current_joint_angles);
PI_ARM_DEFINE_HAS_FIELD(velocities);
PI_ARM_DEFINE_HAS_FIELD(current_speeds);
PI_ARM_DEFINE_HAS_FIELD(enabled);
PI_ARM_DEFINE_HAS_FIELD(has_error);
PI_ARM_DEFINE_HAS_FIELD(errors);
PI_ARM_DEFINE_HAS_FIELD(in_motion);
PI_ARM_DEFINE_HAS_FIELD(moving);
PI_ARM_DEFINE_HAS_FIELD(fresh);
PI_ARM_DEFINE_HAS_FIELD(error_codes);
PI_ARM_DEFINE_HAS_FIELD(ages_sec);
PI_ARM_DEFINE_HAS_FIELD(last_update_ages_sec);
PI_ARM_DEFINE_HAS_FIELD(all_enabled);
PI_ARM_DEFINE_HAS_FIELD(any_error);
PI_ARM_DEFINE_HAS_FIELD(any_in_motion);
PI_ARM_DEFINE_HAS_FIELD(error_message);
PI_ARM_DEFINE_HAS_FIELD(message);
PI_ARM_DEFINE_HAS_FIELD(diagnostic);

#undef PI_ARM_DEFINE_HAS_FIELD

template<typename MessageT>
void fill_hardware_state_message(
  MessageT & message, const HardwareSample & sample,
  const std::vector<std::string> & joint_names,
  const std::vector<pi_arm_can::TransmissionConfig> & transmissions,
  const std::string & backend, const rclcpp::Time & now)
{
  if constexpr (has_header<MessageT>::value) {
    message.header.stamp = now;
  }
  if constexpr (has_stamp<MessageT>::value) {
    message.stamp = now;
  }
  if constexpr (has_backend<MessageT>::value) {
    message.backend = backend;
  }
  if constexpr (has_driver_mode<MessageT>::value) {
    message.driver_mode = sample.driver_mode;
  }
  if constexpr (has_management_operation<MessageT>::value) {
    message.management_operation = sample.management_operation;
  }
  if constexpr (has_connected<MessageT>::value) {
    message.connected = sample.connected;
  }
  if constexpr (has_joint_names<MessageT>::value) {
    message.joint_names = joint_names;
  }
  if constexpr (has_motor_ids<MessageT>::value) {
    message.motor_ids.clear();
    for (const auto & transmission : transmissions) {
      message.motor_ids.push_back(transmission.motor_id);
    }
  }
  if constexpr (has_positions<MessageT>::value) {
    message.positions = sample.positions_rad;
  }
  if constexpr (has_current_joint_angles<MessageT>::value) {
    message.current_joint_angles = sample.positions_rad;
  }
  if constexpr (has_velocities<MessageT>::value) {
    message.velocities = sample.velocities_rad_s;
  }
  if constexpr (has_current_speeds<MessageT>::value) {
    message.current_speeds = sample.velocities_rad_s;
  }
  if constexpr (has_enabled<MessageT>::value) {
    message.enabled = sample.enabled;
  }
  if constexpr (has_has_error<MessageT>::value) {
    message.has_error = sample.has_error;
  }
  if constexpr (has_errors<MessageT>::value) {
    message.errors = sample.has_error;
  }
  if constexpr (has_in_motion<MessageT>::value) {
    message.in_motion = sample.in_motion;
  }
  if constexpr (has_moving<MessageT>::value) {
    message.moving = sample.in_motion;
  }
  if constexpr (has_fresh<MessageT>::value) {
    message.fresh = sample.fresh;
  }
  if constexpr (has_error_codes<MessageT>::value) {
    message.error_codes = sample.error_codes;
  }
  if constexpr (has_ages_sec<MessageT>::value) {
    message.ages_sec = sample.ages_sec;
  }
  if constexpr (has_last_update_ages_sec<MessageT>::value) {
    message.last_update_ages_sec = sample.ages_sec;
  }
  if constexpr (has_all_enabled<MessageT>::value) {
    message.all_enabled =
      !sample.enabled.empty() && std::all_of(sample.enabled.begin(), sample.enabled.end(), [](bool v) {
        return v;
      });
  }
  if constexpr (has_any_error<MessageT>::value) {
    message.any_error = std::any_of(sample.has_error.begin(), sample.has_error.end(), [](bool v) {
      return v;
    });
  }
  if constexpr (has_any_in_motion<MessageT>::value) {
    message.any_in_motion =
      std::any_of(sample.in_motion.begin(), sample.in_motion.end(), [](bool v) {return v;});
  }
  if constexpr (has_error_message<MessageT>::value) {
    message.error_message = sample.error_message;
  }
  if constexpr (has_message<MessageT>::value) {
    message.message = sample.error_message;
  }
  if constexpr (has_diagnostic<MessageT>::value) {
    message.diagnostic = sample.error_message;
  }
}

const char * operation_name(const pi_arm_can::ManagementOperation operation)
{
  switch (operation) {
    case pi_arm_can::ManagementOperation::NONE: return "NONE";
    case pi_arm_can::ManagementOperation::MOTOR_OFF: return "MOTOR_OFF";
    case pi_arm_can::ManagementOperation::MOTOR_RUN: return "MOTOR_RUN";
    case pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS: return "CLEAR_ERROR_FLAGS";
    case pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO:
      return "SET_CURRENT_POSITION_ZERO";
  }
  return "UNKNOWN";
}

bool has_exact_interfaces(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces)
{
  if (interfaces.size() != 2U) {
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto & interface : interfaces) {
    names.insert(interface.name);
  }
  return names == std::unordered_set<std::string>{
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY};
}

}  // namespace

PiArmSystem::~PiArmSystem()
{
  // Stopping the auxiliary node first joins the executor thread, so no service
  // callback can reach the backend once cleanup starts.
  stop_auxiliary_node();
  if (backend_) {
    backend_->cleanup();
  }
}

hardware_interface::CallbackReturn PiArmSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    if (info_.joints.empty()) {
      throw std::invalid_argument("pi_arm_hardware requires at least one active joint");
    }
    joint_names_.clear();
    for (const auto & joint : info_.joints) {
      if (!has_exact_interfaces(joint.command_interfaces) ||
        !has_exact_interfaces(joint.state_interfaces))
      {
        throw std::invalid_argument(
                "every active joint requires position and velocity command/state interfaces");
      }
      joint_names_.push_back(joint.name);
    }
    if (!parse_configuration()) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    const std::size_t joint_count = joint_names_.size();
    state_positions_.assign(joint_count, 0.0);
    state_velocities_.assign(joint_count, 0.0);
    command_positions_.assign(joint_count, 0.0);
    command_velocities_ = backend_config_.joint_velocity_limits_rad_s;
    latest_sample_.positions_rad = state_positions_;
    latest_sample_.velocities_rad_s = state_velocities_;
    latest_sample_.enabled.assign(joint_count, false);
    latest_sample_.has_error.assign(joint_count, false);
    latest_sample_.in_motion.assign(joint_count, false);
    latest_sample_.fresh.assign(joint_count, false);
    latest_sample_.error_codes.assign(joint_count, 0);
    latest_sample_.ages_sec.assign(joint_count, std::numeric_limits<double>::infinity());

    if (backend_name_ == "real") {
      backend_ = std::make_unique<RealBackend>(backend_config_);
    } else if (backend_name_ == "mock") {
      backend_ = std::make_unique<MockBackend>(backend_config_);
    } else {
      throw std::invalid_argument("hardware parameter backend must be 'mock' or 'real'");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("PiArmSystem"), "Initialization failed: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

bool PiArmSystem::parse_configuration()
{
  const auto & parameters = info_.hardware_parameters;
  backend_name_ = string_parameter_or(parameters, "backend", "real");
  backend_config_.can_interface = string_parameter_or(parameters, "can_interface", "can0");
  backend_config_.poll_interval = std::chrono::milliseconds{
    parameter_or<int>(parameters, "state_poll_interval_ms", 20)};
  backend_config_.status1_poll_divider =
    parameter_or<std::size_t>(parameters, "status1_poll_divider", 5U);
  backend_config_.moving_speed_threshold_rad_s =
    parameter_or<double>(parameters, "moving_speed_threshold_rad_s", 0.00017453292519943296);
  backend_config_.state_stale_timeout_sec =
    parameter_or<double>(parameters, "state_stale_timeout_sec", 1.0);
  state_publish_rate_hz_ = parameter_or<double>(parameters, "state_publish_rate_hz", 10.0);
  node_name_ = string_parameter_or(parameters, "service_node_name", node_name_);

  if (backend_config_.poll_interval.count() <= 0 || backend_config_.status1_poll_divider == 0U ||
    backend_config_.moving_speed_threshold_rad_s < 0.0 ||
    backend_config_.state_stale_timeout_sec <= 0.0 || state_publish_rate_hz_ <= 0.0)
  {
    throw std::invalid_argument("timing and speed hardware parameters are outside valid ranges");
  }

  // Joint limits come from the URDF <limit> tags (parsed into info_.limits by
  // ros2_control) and the motor mapping from per-joint ros2_control <param>
  // entries, so the robot description remains the single source of truth.
  backend_config_.transmissions.clear();
  backend_config_.joint_lower_limits_rad.clear();
  backend_config_.joint_upper_limits_rad.clear();
  backend_config_.joint_velocity_limits_rad_s.clear();
  std::unordered_set<int> motor_ids;
  for (std::size_t index = 0; index < info_.joints.size(); ++index) {
    const auto & joint = info_.joints[index];
    const auto limits = info_.limits.find(joint.name);
    if (limits == info_.limits.end() || !limits->second.has_position_limits ||
      !limits->second.has_velocity_limits || limits->second.max_velocity <= 0.0)
    {
      throw std::invalid_argument(
              "joint '" + joint.name + "' requires position and velocity limits in the URDF");
    }
    backend_config_.joint_lower_limits_rad.push_back(limits->second.min_position);
    backend_config_.joint_upper_limits_rad.push_back(limits->second.max_position);
    backend_config_.joint_velocity_limits_rad_s.push_back(limits->second.max_velocity);

    const int motor_id = required_joint_parameter<int>(joint.parameters, joint.name, "motor_id");
    if (motor_id <= 0 || motor_id > 0xFF || !motor_ids.insert(motor_id).second) {
      throw std::invalid_argument(
              "joint '" + joint.name + "' needs a unique motor_id between 1 and 255");
    }
    backend_config_.transmissions.push_back(
      pi_arm_can::TransmissionConfig{
        static_cast<int>(index + 1U), static_cast<std::uint8_t>(motor_id),
        required_joint_parameter<double>(joint.parameters, joint.name, "reduction_ratio"),
        required_joint_parameter<int>(joint.parameters, joint.name, "motor_direction"),
        required_joint_parameter<double>(joint.parameters, joint.name, "cable_ratio"),
        required_joint_flag(joint.parameters, joint.name, "nonlinear")});
  }
  return true;
}

std::vector<hardware_interface::StateInterface> PiArmSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 2U);
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION, &state_positions_[index]);
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_VELOCITY, &state_velocities_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> PiArmSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_names_.size() * 2U);
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION, &command_positions_[index]);
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_VELOCITY, &command_velocities_[index]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn PiArmSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  try {
    backend_->configure();
    start_auxiliary_node();
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("PiArmSystem"), "Configuration failed: %s", error.what());
    stop_auxiliary_node();
    if (backend_) {
      backend_->cleanup();
    }
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn PiArmSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Join the service executor thread before touching the backend so no manage()
  // call can run concurrently with cleanup().
  stop_auxiliary_node();
  backend_->cleanup();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PiArmSystem::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    backend_->activate();
    // Do not trust state_positions_ here: the control loop has not read yet, so
    // values are still the init zeros. One-shot hold happens on first all_fresh
    // read (see read()).
    command_hold_synced_ = false;
    command_velocities_ = backend_config_.joint_velocity_limits_rad_s;
    {
      std::lock_guard<std::mutex> sample_lock(sample_mutex_);
      latest_sample_.connected = true;
    }
    hardware_active_.store(true);
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("PiArmSystem"), "Activation failed: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn PiArmSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  backend_->deactivate();
  hardware_active_.store(false);
  command_hold_synced_ = false;
  {
    std::lock_guard<std::mutex> sample_lock(sample_mutex_);
    // Control loop stops updating samples; clear stale motion so manager does
    // not keep seeing RUNNING from frozen in_motion flags.
    std::fill(
      latest_sample_.velocities_rad_s.begin(), latest_sample_.velocities_rad_s.end(), 0.0);
    std::fill(latest_sample_.in_motion.begin(), latest_sample_.in_motion.end(), false);
    std::fill(latest_sample_.fresh.begin(), latest_sample_.fresh.end(), false);
    latest_sample_.connected = false;
  }
  std::fill(state_velocities_.begin(), state_velocities_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PiArmSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // The backend implementations are thread-safe against concurrent manage()
  // calls, so the control loop never contends with slow management commands.
  HardwareSample sample;
  if (!backend_->read(sample) || sample.positions_rad.size() != joint_names_.size() ||
    sample.velocities_rad_s.size() != joint_names_.size())
  {
    return hardware_interface::return_type::ERROR;
  }
  state_positions_ = sample.positions_rad;
  state_velocities_ = sample.velocities_rad_s;
  // One-shot hold (ros2_control convention): seed command from measured state
  // once when feedback first becomes all_fresh after activate. Do NOT re-hold
  // every cycle — during trajectories command belongs to the controller (JTC).
  if (!command_hold_synced_) {
    const bool all_fresh =
      !sample.fresh.empty() && sample.fresh.size() == joint_names_.size() &&
      std::all_of(sample.fresh.begin(), sample.fresh.end(), [](const bool value) {
        return value;
      });
    if (all_fresh) {
      command_positions_ = state_positions_;
      command_hold_synced_ = true;
    }
  }
  {
    std::lock_guard<std::mutex> sample_lock(sample_mutex_);
    latest_sample_ = std::move(sample);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PiArmSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  return backend_->write(command_positions_, command_velocities_) ?
         hardware_interface::return_type::OK : hardware_interface::return_type::ERROR;
}

void PiArmSystem::start_auxiliary_node()
{
  if (node_) {
    return;
  }
  node_ = std::make_unique<rclcpp::Node>(node_name_);
  state_publisher_ = node_->create_publisher<pi_arm_interfaces::msg::HardwareState>(
    "/pi_arm/hardware_state", rclcpp::SensorDataQoS());
  create_management_service(
    "/pi_arm/hardware/disable_motor", pi_arm_can::ManagementOperation::MOTOR_OFF);
  create_management_service(
    "/pi_arm/hardware/enable_motor", pi_arm_can::ManagementOperation::MOTOR_RUN);
  create_management_service(
    "/pi_arm/hardware/reset_motor", pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS);
  create_management_service(
    "/pi_arm/hardware/set_zero",
    pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO);
  state_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / state_publish_rate_hz_),
    [this]() {publish_hardware_state();});
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_->get_node_base_interface());
  executor_thread_ = std::thread([this]() {executor_->spin();});
}

void PiArmSystem::stop_auxiliary_node() noexcept
{
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  management_services_.clear();
  state_timer_.reset();
  state_publisher_.reset();
  if (executor_ && node_) {
    executor_->remove_node(node_->get_node_base_interface());
  }
  executor_.reset();
  node_.reset();
}

void PiArmSystem::create_management_service(
  const std::string & name, const pi_arm_can::ManagementOperation operation)
{
  management_services_.push_back(
    node_->create_service<ManageMotor>(
      name,
      [this, operation](
        const std::shared_ptr<ManageMotor::Request> request,
        std::shared_ptr<ManageMotor::Response> response)
      {
        {
          std::lock_guard<std::mutex> sample_lock(sample_mutex_);
          latest_sample_.management_operation = operation_name(operation);
        }
        const auto result = backend_->manage(operation, request->motor_id);
        {
          std::lock_guard<std::mutex> sample_lock(sample_mutex_);
          latest_sample_.management_operation = "NONE";
          if (!result.success) {
            latest_sample_.error_message = result.message;
          }
        }
        response->success = result.success;
        response->code = result.code;
        response->message = result.message;
      }));
}

void PiArmSystem::publish_hardware_state()
{
  HardwareSample sample;
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample = latest_sample_;
  }
  if (!hardware_active_.load()) {
    // Belt-and-suspenders: never advertise frozen motion while inactive.
    std::fill(sample.velocities_rad_s.begin(), sample.velocities_rad_s.end(), 0.0);
    std::fill(sample.in_motion.begin(), sample.in_motion.end(), false);
    std::fill(sample.fresh.begin(), sample.fresh.end(), false);
    sample.connected = false;
  }
  pi_arm_interfaces::msg::HardwareState message;
  fill_hardware_state_message(
    message, sample, joint_names_, backend_config_.transmissions, backend_name_, node_->now());
  state_publisher_->publish(std::move(message));
}

}  // namespace pi_arm_hardware

PLUGINLIB_EXPORT_CLASS(pi_arm_hardware::PiArmSystem, hardware_interface::SystemInterface)
