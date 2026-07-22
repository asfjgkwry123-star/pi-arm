#include "pi_arm_hardware/backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"

namespace pi_arm_hardware
{
namespace
{

constexpr double kDegreesPerRadian = 57.2957795130823208768;
constexpr int64_t kWriteRejectLogPeriodMs = 1000;

rclcpp::Logger backend_logger()
{
  return rclcpp::get_logger("PiArmBackend");
}

rclcpp::Clock & steady_clock()
{
  static rclcpp::Clock clock(RCL_STEADY_TIME);
  return clock;
}

template<typename ... Args>
void log_write_reject(const char * format, Args && ... args) noexcept
{
  try {
    RCLCPP_ERROR_THROTTLE(
      backend_logger(), steady_clock(), kWriteRejectLogPeriodMs,
      format, std::forward<Args>(args)...);
  } catch (...) {
  }
}

template<typename ... Args>
void log_write_warn(const char * format, Args && ... args) noexcept
{
  try {
    RCLCPP_WARN_THROTTLE(
      backend_logger(), steady_clock(), kWriteRejectLogPeriodMs,
      format, std::forward<Args>(args)...);
  } catch (...) {
  }
}

double clamp_joint_velocity(double velocity, double velocity_limit, std::size_t index) noexcept
{
  if (!(std::abs(velocity) > velocity_limit + 1e-6)) {
    return velocity;
  }
  log_write_warn(
    "write velocity clamped at joint[%zu]: vel=%.6g -> lim=%.6g",
    index, velocity, velocity_limit);
  return std::copysign(velocity_limit, velocity);
}

double age_seconds(
  const std::chrono::steady_clock::time_point timestamp,
  const std::chrono::steady_clock::time_point now)
{
  if (timestamp == std::chrono::steady_clock::time_point{}) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(now - timestamp).count();
}

std::vector<std::uint8_t> selected_motor_ids(
  const BackendConfig & config, const int requested_motor_id)
{
  std::vector<std::uint8_t> result;
  for (const auto & transmission : config.transmissions) {
    if (requested_motor_id == 0 || requested_motor_id == transmission.motor_id) {
      result.push_back(transmission.motor_id);
    }
  }
  return result;
}

}  // namespace

RealBackend::RealBackend(BackendConfig config)
: config_(std::move(config)), transmissions_(config_.transmissions)
{
}

RealBackend::~RealBackend()
{
  cleanup();
}

void RealBackend::configure()
{
  pi_arm_can::DriverConfig driver_config;
  driver_config.interface_name = config_.can_interface;
  driver_config.motor_ids = transmissions_.motor_ids();
  driver_config.state_poll_interval = config_.poll_interval;
  driver_config.status1_poll_divider = config_.status1_poll_divider;
  driver_ = std::make_unique<pi_arm_can::CanDriver>(std::move(driver_config));
  driver_->start();
}

void RealBackend::activate()
{
  if (!driver_ || !driver_->running()) {
    throw std::runtime_error("CAN driver is not running");
  }
}

void RealBackend::deactivate() noexcept
{
  if (driver_) {
    driver_->set_mode(pi_arm_can::DriverMode::MONITORING);
  }
}

void RealBackend::cleanup() noexcept
{
  if (driver_) {
    driver_->stop();
    driver_.reset();
  }
}

bool RealBackend::read(HardwareSample & sample) noexcept
{
  if (!driver_) {
    return false;
  }
  try {
    const auto snapshot = driver_->snapshot();
    const auto now = std::chrono::steady_clock::now();
    const std::size_t count = config_.transmissions.size();
    sample.positions_rad.assign(count, 0.0);
    sample.velocities_rad_s.assign(count, 0.0);
    sample.enabled.assign(count, false);
    sample.has_error.assign(count, false);
    sample.in_motion.assign(count, false);
    sample.fresh.assign(count, false);
    sample.error_codes.assign(count, 0);
    sample.ages_sec.assign(count, std::numeric_limits<double>::infinity());
    sample.connected = driver_->running() && driver_->mode() != pi_arm_can::DriverMode::FAULT;
    sample.driver_mode = pi_arm_can::to_string(driver_->mode());
    sample.management_operation = pi_arm_can::to_string(driver_->management_operation());
    sample.error_message = driver_->last_error();

    for (std::size_t index = 0; index < count; ++index) {
      const auto & transmission = transmissions_.by_joint_id(
        config_.transmissions[index].joint_id);
      const auto & state = snapshot.at(config_.transmissions[index].motor_id);
      if (state.angle_valid) {
        sample.positions_rad[index] =
          transmission.motor_position_to_joint(state.angle_deg) / kDegreesPerRadian;
      }
      if (state.status2_valid) {
        sample.velocities_rad_s[index] =
          transmission.motor_velocity_to_joint(state.speed_dps) / kDegreesPerRadian;
        sample.in_motion[index] =
          std::abs(sample.velocities_rad_s[index]) > config_.moving_speed_threshold_rad_s;
      }
      if (state.status1_valid) {
        sample.enabled[index] = state.motor_state == 0x00U || state.motor_state == 0x30U;
        sample.has_error[index] = state.error_state != 0U;
        sample.error_codes[index] = state.error_state;
      }
      const double angle_age = age_seconds(state.angle_update, now);
      const double status1_age = age_seconds(state.status1_update, now);
      const double status2_age = age_seconds(state.status2_update, now);
      sample.ages_sec[index] = std::max({angle_age, status1_age, status2_age});
      sample.fresh[index] =
        state.angle_valid && state.status1_valid && state.status2_valid &&
        sample.ages_sec[index] <= config_.state_stale_timeout_sec;
    }
    const bool all_enabled =
      !sample.enabled.empty() &&
      std::all_of(sample.enabled.begin(), sample.enabled.end(), [](const bool value) {
        return value;
      });
    if (driver_->mode() == pi_arm_can::DriverMode::MONITORING && all_enabled) {
      driver_->set_mode(pi_arm_can::DriverMode::CONTROL);
      sample.driver_mode = pi_arm_can::to_string(driver_->mode());
    } else if (driver_->mode() == pi_arm_can::DriverMode::CONTROL && !all_enabled) {
      driver_->set_mode(pi_arm_can::DriverMode::MONITORING);
      sample.driver_mode = pi_arm_can::to_string(driver_->mode());
    }
    return true;
  } catch (const std::exception & error) {
    sample.error_message = error.what();
    return false;
  }
}

bool RealBackend::write(
  const std::vector<double> & positions_rad,
  const std::vector<double> & velocities_rad_s) noexcept
{
  if (!driver_ || positions_rad.size() != config_.transmissions.size() ||
    velocities_rad_s.size() != positions_rad.size() ||
    config_.joint_lower_limits_rad.size() != positions_rad.size() ||
    config_.joint_upper_limits_rad.size() != positions_rad.size() ||
    config_.joint_velocity_limits_rad_s.size() != positions_rad.size())
  {
    log_write_reject(
      "write rejected: size/config mismatch (driver=%d pos=%zu vel=%zu joints=%zu)",
      driver_ ? 1 : 0, positions_rad.size(), velocities_rad_s.size(),
      config_.transmissions.size());
    return false;
  }
  if (driver_->mode() == pi_arm_can::DriverMode::MONITORING ||
    driver_->mode() == pi_arm_can::DriverMode::EXCLUSIVE_MANAGEMENT)
  {
    return true;
  }
  if (driver_->mode() != pi_arm_can::DriverMode::CONTROL) {
    log_write_reject(
      "write rejected: driver mode is not CONTROL (mode=%d)",
      static_cast<int>(driver_->mode()));
    return false;
  }
  try {
    std::vector<pi_arm_can::PositionCommand> commands;
    commands.reserve(positions_rad.size());
    for (std::size_t index = 0; index < positions_rad.size(); ++index) {
      if (!std::isfinite(positions_rad[index]) || !std::isfinite(velocities_rad_s[index])) {
        log_write_reject(
          "write rejected: non-finite command at joint[%zu] pos=%.6g vel=%.6g",
          index, positions_rad[index], velocities_rad_s[index]);
        return false;
      }
      if (positions_rad[index] < config_.joint_lower_limits_rad[index] ||
        positions_rad[index] > config_.joint_upper_limits_rad[index])
      {
        log_write_reject(
          "write rejected: position limit exceeded at joint[%zu] pos=%.6g lim=[%.6g,%.6g]",
          index, positions_rad[index],
          config_.joint_lower_limits_rad[index], config_.joint_upper_limits_rad[index]);
        return false;
      }
      const double commanded_velocity = clamp_joint_velocity(
        velocities_rad_s[index], config_.joint_velocity_limits_rad_s[index], index);
      const auto & transmission = transmissions_.by_joint_id(
        config_.transmissions[index].joint_id);
      double requested_speed = std::abs(commanded_velocity);
      if (requested_speed < std::numeric_limits<double>::epsilon()) {
        requested_speed = config_.joint_velocity_limits_rad_s[index];
      }
      commands.push_back(
        pi_arm_can::PositionCommand{
          transmission.config().motor_id,
          transmission.joint_position_to_motor(positions_rad[index] * kDegreesPerRadian),
          std::abs(transmission.joint_velocity_to_motor(requested_speed * kDegreesPerRadian))});
    }
    if (!driver_->enqueue_position_batch(commands)) {
      const auto detail = driver_->last_error();
      log_write_reject(
        "write rejected: enqueue_position_batch failed (%s)",
        detail.empty() ? "no detail" : detail.c_str());
      return false;
    }
    return true;
  } catch (...) {
    log_write_reject("write rejected: unexpected exception");
    return false;
  }
}

ManagementResult RealBackend::manage(
  const pi_arm_can::ManagementOperation operation, const int motor_id) noexcept
{
  if (!driver_) {
    return {false, 2, "CAN driver is not configured"};
  }
  const auto ids = selected_motor_ids(config_, motor_id);
  if (ids.empty()) {
    return {false, 1, "motor_id is not configured (use 0 for all motors)"};
  }
  if (operation != pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS) {
    const auto snapshot = driver_->snapshot();
    const auto now = std::chrono::steady_clock::now();
    for (const auto & transmission : config_.transmissions) {
      const auto id = transmission.motor_id;
      const auto & state = snapshot.at(id);
      if (!state.status2_valid ||
        age_seconds(state.status2_update, now) > config_.state_stale_timeout_sec)
      {
        return {false, 4, "motor speed feedback is stale"};
      }
      if (state.speed_dps != 0.0 || state.stationary_samples < 3U) {
        return {false, 5, "motor must report zero speed for three consecutive samples"};
      }
    }
  }
  if (!driver_->execute_management(operation, ids)) {
    const auto detail = driver_->last_error();
    return {false, 3, detail.empty() ? "management command was rejected" : detail};
  }
  if (operation == pi_arm_can::ManagementOperation::MOTOR_OFF) {
    driver_->set_mode(pi_arm_can::DriverMode::MONITORING);
  }
  return {true, 0, "management command transmitted"};
}

MockBackend::MockBackend(BackendConfig config)
: config_(std::move(config))
{
  const std::size_t count = config_.transmissions.size();
  sample_.positions_rad.assign(count, 0.0);
  sample_.velocities_rad_s.assign(count, 0.0);
  sample_.enabled.assign(count, false);
  sample_.has_error.assign(count, false);
  sample_.in_motion.assign(count, false);
  sample_.fresh.assign(count, true);
  sample_.error_codes.assign(count, 0);
  sample_.ages_sec.assign(count, 0.0);
  stationary_samples_.assign(count, 0U);
  sample_.driver_mode = "OFFLINE";
}

void MockBackend::configure()
{
  std::lock_guard<std::mutex> lock(mutex_);
  configured_ = true;
  sample_.connected = true;
  sample_.driver_mode = "MONITORING";
}

void MockBackend::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    throw std::runtime_error("mock backend is not configured");
  }
  active_ = true;
  sample_.connected = true;
  sample_.driver_mode = "CONTROL";
}

void MockBackend::deactivate() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  sample_.connected = false;
  sample_.driver_mode = configured_ ? "MONITORING" : "OFFLINE";
  std::fill(sample_.velocities_rad_s.begin(), sample_.velocities_rad_s.end(), 0.0);
  std::fill(sample_.in_motion.begin(), sample_.in_motion.end(), false);
  std::fill(sample_.fresh.begin(), sample_.fresh.end(), false);
}

void MockBackend::cleanup() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  configured_ = false;
  sample_.connected = false;
  sample_.driver_mode = "OFFLINE";
  std::fill(sample_.enabled.begin(), sample_.enabled.end(), false);
}

bool MockBackend::read(HardwareSample & sample) noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (std::size_t index = 0; index < sample_.velocities_rad_s.size(); ++index) {
    stationary_samples_[index] =
      std::abs(sample_.velocities_rad_s[index]) <= config_.moving_speed_threshold_rad_s ?
      std::min<std::size_t>(3U, stationary_samples_[index] + 1U) : 0U;
  }
  sample = sample_;
  return configured_;
}

bool MockBackend::write(
  const std::vector<double> & positions_rad,
  const std::vector<double> & velocities_rad_s) noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || positions_rad.size() != sample_.positions_rad.size() ||
    velocities_rad_s.size() != positions_rad.size() ||
    config_.joint_lower_limits_rad.size() != positions_rad.size() ||
    config_.joint_upper_limits_rad.size() != positions_rad.size() ||
    config_.joint_velocity_limits_rad_s.size() != positions_rad.size())
  {
    log_write_reject(
      "mock write rejected: inactive or size/config mismatch (active=%d pos=%zu expected=%zu)",
      active_ ? 1 : 0, positions_rad.size(), sample_.positions_rad.size());
    return false;
  }
  for (std::size_t index = 0; index < positions_rad.size(); ++index) {
    if (!std::isfinite(positions_rad[index]) || !std::isfinite(velocities_rad_s[index])) {
      log_write_reject(
        "mock write rejected: non-finite command at joint[%zu] pos=%.6g vel=%.6g",
        index, positions_rad[index], velocities_rad_s[index]);
      return false;
    }
    if (positions_rad[index] < config_.joint_lower_limits_rad[index] ||
      positions_rad[index] > config_.joint_upper_limits_rad[index])
    {
      log_write_reject(
        "mock write rejected: position limit exceeded at joint[%zu] pos=%.6g lim=[%.6g,%.6g]",
        index, positions_rad[index],
        config_.joint_lower_limits_rad[index], config_.joint_upper_limits_rad[index]);
      return false;
    }
  }
  std::vector<double> clamped_velocities = velocities_rad_s;
  for (std::size_t index = 0; index < clamped_velocities.size(); ++index) {
    clamped_velocities[index] = clamp_joint_velocity(
      clamped_velocities[index], config_.joint_velocity_limits_rad_s[index], index);
  }
  const bool all_enabled =
    !sample_.enabled.empty() &&
    std::all_of(sample_.enabled.begin(), sample_.enabled.end(), [](const bool value) {
      return value;
    });
  if (!all_enabled) {
    std::fill(sample_.velocities_rad_s.begin(), sample_.velocities_rad_s.end(), 0.0);
    std::fill(sample_.in_motion.begin(), sample_.in_motion.end(), false);
    return true;
  }
  sample_.positions_rad = positions_rad;
  sample_.velocities_rad_s = clamped_velocities;
  for (std::size_t index = 0; index < clamped_velocities.size(); ++index) {
    sample_.in_motion[index] =
      std::abs(clamped_velocities[index]) > config_.moving_speed_threshold_rad_s;
    sample_.ages_sec[index] = 0.0;
  }
  return true;
}

bool MockBackend::valid_motor_id(const int motor_id) const noexcept
{
  return std::any_of(
    config_.transmissions.begin(), config_.transmissions.end(),
    [motor_id](const auto & transmission) {return transmission.motor_id == motor_id;});
}

ManagementResult MockBackend::manage(
  const pi_arm_can::ManagementOperation operation, const int motor_id) noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    return {false, 2, "mock backend is not configured"};
  }
  if (motor_id != 0 && !valid_motor_id(motor_id)) {
    return {false, 1, "motor_id is not configured (use 0 for all motors)"};
  }
  if (operation != pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS) {
    for (std::size_t index = 0; index < config_.transmissions.size(); ++index) {
      if (stationary_samples_[index] < 3U) {
        return {false, 5, "motor must be stationary for three consecutive samples"};
      }
    }
  }
  for (std::size_t index = 0; index < config_.transmissions.size(); ++index) {
    if (motor_id != 0 && config_.transmissions[index].motor_id != motor_id) {
      continue;
    }
    switch (operation) {
      case pi_arm_can::ManagementOperation::NONE:
        return {false, 1, "management operation NONE is not executable"};
      case pi_arm_can::ManagementOperation::MOTOR_OFF:
        sample_.enabled[index] = false;
        sample_.velocities_rad_s[index] = 0.0;
        sample_.in_motion[index] = false;
        break;
      case pi_arm_can::ManagementOperation::MOTOR_RUN:
        sample_.enabled[index] = true;
        break;
      case pi_arm_can::ManagementOperation::CLEAR_ERROR_FLAGS:
        sample_.has_error[index] = false;
        sample_.error_codes[index] = 0;
        break;
      case pi_arm_can::ManagementOperation::SET_CURRENT_POSITION_ZERO:
        sample_.positions_rad[index] = 0.0;
        break;
    }
  }
  return {true, 0, "mock management operation applied"};
}

}  // namespace pi_arm_hardware
