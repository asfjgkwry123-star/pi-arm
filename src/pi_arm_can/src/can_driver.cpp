#include "pi_arm_can/can_driver.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace pi_arm_can
{
namespace
{

Opcode management_opcode(const ManagementOperation operation)
{
  switch (operation) {
    case ManagementOperation::NONE:
      break;
    case ManagementOperation::MOTOR_OFF:
      return Opcode::MOTOR_OFF;
    case ManagementOperation::MOTOR_RUN:
      return Opcode::MOTOR_RUN;
    case ManagementOperation::CLEAR_ERROR_FLAGS:
      return Opcode::CLEAR_ERROR_FLAGS;
    case ManagementOperation::SET_CURRENT_POSITION_ZERO:
      return Opcode::SET_CURRENT_POSITION_ZERO;
  }
  throw std::invalid_argument("unknown management operation");
}

}  // namespace

const char * to_string(const DriverMode mode) noexcept
{
  switch (mode) {
    case DriverMode::OFFLINE: return "OFFLINE";
    case DriverMode::MONITORING: return "MONITORING";
    case DriverMode::CONTROL: return "CONTROL";
    case DriverMode::EXCLUSIVE_MANAGEMENT: return "EXCLUSIVE_MANAGEMENT";
    case DriverMode::FAULT: return "FAULT";
  }
  return "UNKNOWN";
}

const char * to_string(const ManagementOperation operation) noexcept
{
  switch (operation) {
    case ManagementOperation::NONE: return "NONE";
    case ManagementOperation::MOTOR_OFF: return "MOTOR_OFF";
    case ManagementOperation::MOTOR_RUN: return "MOTOR_RUN";
    case ManagementOperation::CLEAR_ERROR_FLAGS: return "CLEAR_ERROR_FLAGS";
    case ManagementOperation::SET_CURRENT_POSITION_ZERO: return "SET_CURRENT_POSITION_ZERO";
  }
  return "UNKNOWN";
}

CanDriver::CanDriver(DriverConfig config)
: config_(std::move(config))
{
  if (config_.interface_name.empty() || config_.interface_name.size() >= 16U) {
    throw std::invalid_argument("SocketCAN interface name is empty or too long");
  }
  if (config_.motor_ids.empty()) {
    throw std::invalid_argument("at least one motor ID is required");
  }
  if (config_.state_poll_interval.count() <= 0 || config_.status1_poll_divider == 0U ||
    config_.max_pending_batches == 0U)
  {
    throw std::invalid_argument("polling and queue configuration values must be positive");
  }

  std::unordered_set<std::uint8_t> unique_ids;
  for (const auto id : config_.motor_ids) {
    if (id == 0U || kCanIdBase + id > 0x7FFU || !unique_ids.insert(id).second) {
      throw std::invalid_argument("motor IDs must be unique valid standard CAN IDs");
    }
    MotorState state;
    state.motor_id = id;
    states_.emplace(id, state);
  }
}

CanDriver::~CanDriver()
{
  stop();
}

void CanDriver::start()
{
  if (running_.load()) {
    return;
  }
  stop_requested_.store(false);
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_.clear();
  }

  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    throw std::runtime_error("failed to create SocketCAN socket: " + std::string(std::strerror(errno)));
  }

  try {
    struct ifreq request {};
    std::strncpy(request.ifr_name, config_.interface_name.c_str(), IFNAMSIZ - 1U);
    if (::ioctl(fd, SIOCGIFINDEX, &request) < 0) {
      throw std::runtime_error(
              "SocketCAN interface '" + config_.interface_name + "' is unavailable: " +
              std::strerror(errno));
    }

    std::vector<struct can_filter> filters;
    filters.reserve(config_.motor_ids.size());
    for (const auto id : config_.motor_ids) {
      filters.push_back(can_filter{kCanIdBase + id, CAN_SFF_MASK});
    }
    if (::setsockopt(
        fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
        static_cast<socklen_t>(filters.size() * sizeof(can_filter))) < 0)
    {
      throw std::runtime_error("failed to install SocketCAN filters: " + std::string(std::strerror(errno)));
    }

    struct sockaddr_can address {};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(fd, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
      throw std::runtime_error("failed to bind SocketCAN socket: " + std::string(std::strerror(errno)));
    }
  } catch (...) {
    ::close(fd);
    throw;
  }

  socket_fd_ = fd;
  running_.store(true);
  mode_.store(DriverMode::MONITORING);
  try {
    rx_thread_ = std::thread(&CanDriver::rx_loop, this);
    tx_thread_ = std::thread(&CanDriver::tx_loop, this);
  } catch (...) {
    stop();
    throw;
  }
}

void CanDriver::stop() noexcept
{
  stop_requested_.store(true);
  tx_cv_.notify_all();
  if (socket_fd_ >= 0) {
    ::shutdown(socket_fd_, SHUT_RDWR);
  }
  if (tx_thread_.joinable()) {
    tx_thread_.join();
  }
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  close_socket();

  std::deque<TxBatch> abandoned;
  {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    abandoned.swap(tx_queue_);
  }
  for (auto & batch : abandoned) {
    if (batch.completion) {
      try {batch.completion->set_value(false);} catch (...) {}
    }
  }
  running_.store(false);
  mode_.store(DriverMode::OFFLINE);
  operation_.store(ManagementOperation::NONE);
}

bool CanDriver::set_mode(const DriverMode requested)
{
  const DriverMode current = mode_.load();
  if (requested == current) {
    return true;
  }
  if (!running_.load() || current == DriverMode::FAULT ||
    current == DriverMode::EXCLUSIVE_MANAGEMENT)
  {
    return false;
  }
  if (requested != DriverMode::MONITORING && requested != DriverMode::CONTROL) {
    return false;
  }
  mode_.store(requested);
  return true;
}

std::string CanDriver::last_error() const
{
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

bool CanDriver::validate_motor_id(const std::uint8_t motor_id) const noexcept
{
  return states_.find(motor_id) != states_.end();
}

bool CanDriver::enqueue_batch(TxBatch batch, const bool high_priority)
{
  if (!running_.load() || stop_requested_.load() || mode_.load() == DriverMode::FAULT) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(tx_mutex_);
    if (tx_queue_.size() >= config_.max_pending_batches) {
      return false;
    }
    if (high_priority) {
      tx_queue_.push_front(std::move(batch));
    } else {
      tx_queue_.push_back(std::move(batch));
    }
  }
  tx_cv_.notify_one();
  return true;
}

bool CanDriver::enqueue_position_batch(const std::vector<PositionCommand> & commands)
{
  if (mode_.load() != DriverMode::CONTROL || commands.empty()) {
    return false;
  }
  TxBatch batch;
  batch.frames.reserve(commands.size());
  try {
    for (const auto & command : commands) {
      if (!validate_motor_id(command.motor_id)) {
        return false;
      }
      batch.frames.push_back(
        make_position_command(
          command.motor_id, command.motor_angle_deg, command.max_motor_speed_dps));
    }
  } catch (const std::exception & error) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = error.what();
    return false;
  }
  return enqueue_batch(std::move(batch));
}

bool CanDriver::execute_management(
  const ManagementOperation operation, const std::vector<std::uint8_t> & motor_ids,
  const std::chrono::milliseconds timeout)
{
  if (operation == ManagementOperation::NONE || motor_ids.empty() || timeout.count() <= 0) {
    return false;
  }
  std::unique_lock<std::mutex> operation_lock(management_mutex_);
  const DriverMode previous = mode_.load();
  if (previous != DriverMode::MONITORING && previous != DriverMode::CONTROL) {
    return false;
  }
  for (const auto id : motor_ids) {
    if (!validate_motor_id(id)) {
      return false;
    }
  }
  std::unordered_map<std::uint8_t, std::chrono::steady_clock::time_point> previous_updates;
  {
    std::shared_lock<std::shared_mutex> lock(state_mutex_);
    for (const auto id : motor_ids) {
      const auto & state = states_.at(id);
      previous_updates[id] =
        operation == ManagementOperation::SET_CURRENT_POSITION_ZERO ?
        state.angle_update : state.status1_update;
    }
  }

  const bool exclusive = operation != ManagementOperation::CLEAR_ERROR_FLAGS;
  operation_.store(operation);
  if (exclusive) {
    mode_.store(DriverMode::EXCLUSIVE_MANAGEMENT);
  }
  TxBatch batch;
  batch.completion = std::make_shared<std::promise<bool>>();
  auto completion = batch.completion->get_future();
  batch.frames.reserve(motor_ids.size());
  const Opcode opcode = management_opcode(operation);
  for (const auto id : motor_ids) {
    batch.frames.push_back(make_request(id, opcode));
  }

  if (!enqueue_batch(std::move(batch), true)) {
    if (exclusive) {
      mode_.store(previous);
    }
    operation_.store(ManagementOperation::NONE);
    return false;
  }
  if (completion.wait_for(timeout) != std::future_status::ready) {
    operation_.store(ManagementOperation::NONE);
    set_fault("timed out while transmitting an exclusive management batch");
    return false;
  }
  const bool success = completion.get();
  if (exclusive && mode_.load() != DriverMode::FAULT) {
    mode_.store(previous);
  }
  if (!success) {
    operation_.store(ManagementOperation::NONE);
    return false;
  }

  std::unique_lock<std::shared_mutex> state_lock(state_mutex_);
  const bool confirmed = state_cv_.wait_for(
    state_lock, timeout,
    [this, operation, &motor_ids, &previous_updates]() {
      for (const auto id : motor_ids) {
        const auto & state = states_.at(id);
        if (operation == ManagementOperation::SET_CURRENT_POSITION_ZERO) {
          if (!state.angle_valid || state.angle_update <= previous_updates.at(id) ||
            std::abs(state.angle_deg) > 0.05)
          {
            return false;
          }
          continue;
        }
        if (!state.status1_valid || state.status1_update <= previous_updates.at(id)) {
          return false;
        }
        if (operation == ManagementOperation::MOTOR_RUN &&
          state.motor_state != 0x00U && state.motor_state != 0x30U)
        {
          return false;
        }
        if (operation == ManagementOperation::MOTOR_OFF &&
          (state.motor_state == 0x00U || state.motor_state == 0x30U))
        {
          return false;
        }
        if (operation == ManagementOperation::CLEAR_ERROR_FLAGS && state.error_state != 0U) {
          return false;
        }
      }
      return true;
    });
  operation_.store(ManagementOperation::NONE);
  if (!confirmed) {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = "management command was transmitted but device state was not confirmed";
  }
  return confirmed;
}

MotorState CanDriver::motor_state(const std::uint8_t motor_id) const
{
  std::shared_lock<std::shared_mutex> lock(state_mutex_);
  return states_.at(motor_id);
}

std::unordered_map<std::uint8_t, MotorState> CanDriver::snapshot() const
{
  std::shared_lock<std::shared_mutex> lock(state_mutex_);
  return states_;
}

std::vector<CanFrame> CanDriver::make_poll_batch(const std::size_t sequence) const
{
  std::vector<CanFrame> frames;
  const bool include_status1 = (sequence % config_.status1_poll_divider) == 0U;
  frames.reserve(config_.motor_ids.size() * (include_status1 ? 3U : 2U));
  for (const auto id : config_.motor_ids) {
    frames.push_back(make_request(id, Opcode::READ_STATUS_2));
    frames.push_back(make_request(id, Opcode::READ_MULTI_TURN_ANGLE));
    if (include_status1) {
      frames.push_back(make_request(id, Opcode::READ_STATUS_1));
    }
  }
  return frames;
}

void CanDriver::tx_loop() noexcept
{
  auto next_poll = std::chrono::steady_clock::now();
  std::size_t poll_sequence = 0U;
  while (!stop_requested_.load()) {
    TxBatch batch;
    bool have_batch = false;
    {
      std::unique_lock<std::mutex> lock(tx_mutex_);
      tx_cv_.wait_until(
        lock, next_poll, [this]() {return stop_requested_.load() || !tx_queue_.empty();});
      if (stop_requested_.load()) {
        break;
      }
      if (!tx_queue_.empty()) {
        batch = std::move(tx_queue_.front());
        tx_queue_.pop_front();
        have_batch = true;
      }
    }

    if (have_batch) {
      const bool success = write_batch(batch.frames);
      if (batch.completion) {
        try {batch.completion->set_value(success);} catch (...) {}
      }
      if (!success) {
        break;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_poll && mode_.load() != DriverMode::EXCLUSIVE_MANAGEMENT) {
      if (!write_batch(make_poll_batch(poll_sequence++))) {
        break;
      }
      do {
        next_poll += config_.state_poll_interval;
      } while (next_poll <= now);
    }
  }
}

bool CanDriver::write_batch(const std::vector<CanFrame> & frames) noexcept
{
  for (const auto & frame : frames) {
    struct can_frame native {};
    native.can_id = frame.id;
    native.can_dlc = static_cast<__u8>(frame.data.size());
    std::copy(frame.data.begin(), frame.data.end(), native.data);
    ssize_t written;
    do {
      written = ::write(socket_fd_, &native, sizeof(native));
    } while (written < 0 && errno == EINTR && !stop_requested_.load());
    if (written != static_cast<ssize_t>(sizeof(native))) {
      if (!stop_requested_.load()) {
        set_fault("SocketCAN write failed: " + std::string(std::strerror(errno)));
      }
      return false;
    }
  }
  return true;
}

void CanDriver::rx_loop() noexcept
{
  while (!stop_requested_.load()) {
    struct pollfd descriptor {socket_fd_, POLLIN, 0};
    const int result = ::poll(&descriptor, 1, 100);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      set_fault("SocketCAN poll failed: " + std::string(std::strerror(errno)));
      break;
    }
    if (result == 0) {
      continue;
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      if (!stop_requested_.load()) {
        set_fault("SocketCAN receive socket reported an error");
      }
      break;
    }
    if ((descriptor.revents & POLLIN) == 0) {
      continue;
    }

    struct can_frame native {};
    const ssize_t received = ::read(socket_fd_, &native, sizeof(native));
    if (received < 0 && errno == EINTR) {
      continue;
    }
    if (received != static_cast<ssize_t>(sizeof(native))) {
      if (!stop_requested_.load()) {
        set_fault("SocketCAN read returned an invalid frame");
      }
      break;
    }
    if ((native.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0U ||
      native.can_dlc != kCanPayloadSize)
    {
      continue;
    }
    CanFrame frame;
    frame.id = native.can_id & CAN_SFF_MASK;
    std::copy_n(native.data, frame.data.size(), frame.data.begin());
    try {
      process_received_frame(frame);
    } catch (...) {
      // A malformed device response must not terminate reception of later frames.
    }
  }
}

void CanDriver::process_received_frame(const CanFrame & frame)
{
  if (frame.id <= kCanIdBase || frame.id > 0x7FFU) {
    return;
  }
  const auto raw_id = frame.id - kCanIdBase;
  if (raw_id > 0xFFU || !validate_motor_id(static_cast<std::uint8_t>(raw_id))) {
    return;
  }
  const auto motor_id = static_cast<std::uint8_t>(raw_id);
  const auto opcode = static_cast<Opcode>(frame.data[0]);
  std::unique_lock<std::shared_mutex> lock(state_mutex_);
  auto & state = states_.at(motor_id);
  const auto now = std::chrono::steady_clock::now();
  switch (opcode) {
    case Opcode::READ_STATUS_1:
      state.temperature_c = static_cast<std::int8_t>(frame.data[1]);
      state.voltage_v = static_cast<double>(read_le<std::int16_t>(&frame.data[2])) * 0.01;
      state.current_a = static_cast<double>(read_le<std::int16_t>(&frame.data[4])) * 0.01;
      state.motor_state = frame.data[6];
      state.error_state = frame.data[7];
      state.status1_valid = true;
      state.status1_update = now;
      break;
    case Opcode::READ_STATUS_2:
    case Opcode::POSITION_CONTROL_2:
      state.temperature_c = static_cast<std::int8_t>(frame.data[1]);
      state.iq_or_power_raw = read_le<std::int16_t>(&frame.data[2]);
      state.speed_dps = static_cast<double>(read_le<std::int16_t>(&frame.data[4]));
      state.stationary_samples =
        state.speed_dps == 0.0 ? state.stationary_samples + 1U : 0U;
      state.encoder = read_le<std::uint16_t>(&frame.data[6]);
      state.status2_valid = true;
      state.status2_update = now;
      break;
    case Opcode::READ_MULTI_TURN_ANGLE:
      state.angle_deg = decode_multi_turn_angle_deg(frame);
      state.angle_valid = true;
      state.angle_update = now;
      break;
    default:
      return;
  }
  state.last_update = now;
  lock.unlock();
  state_cv_.notify_all();
}

void CanDriver::set_fault(const std::string & message) noexcept
{
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
  }
  mode_.store(DriverMode::FAULT);
  stop_requested_.store(true);
  tx_cv_.notify_all();
}

void CanDriver::close_socket() noexcept
{
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
  }
  socket_fd_ = -1;
}

}  // namespace pi_arm_can
