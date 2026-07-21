#pragma once

#include "pi_arm_can/protocol.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace pi_arm_can
{

enum class DriverMode
{
  OFFLINE,
  MONITORING,
  CONTROL,
  EXCLUSIVE_MANAGEMENT,
  FAULT,
};

enum class ManagementOperation
{
  NONE,
  MOTOR_OFF,
  MOTOR_RUN,
  CLEAR_ERROR_FLAGS,
  SET_CURRENT_POSITION_ZERO,
};

struct DriverConfig
{
  std::string interface_name{"can0"};
  std::vector<std::uint8_t> motor_ids;
  std::chrono::milliseconds state_poll_interval{20};
  std::size_t status1_poll_divider{5U};
  std::size_t max_pending_batches{64U};
};

struct MotorState
{
  std::uint8_t motor_id{0U};
  double angle_deg{0.0};
  double speed_dps{0.0};
  double voltage_v{0.0};
  double current_a{0.0};
  std::int16_t iq_or_power_raw{0};
  std::uint16_t encoder{0U};
  std::int8_t temperature_c{0};
  std::uint8_t motor_state{0U};
  std::uint8_t error_state{0U};
  bool angle_valid{false};
  bool status1_valid{false};
  bool status2_valid{false};
  std::size_t stationary_samples{0U};
  std::chrono::steady_clock::time_point angle_update{};
  std::chrono::steady_clock::time_point status1_update{};
  std::chrono::steady_clock::time_point status2_update{};
  std::chrono::steady_clock::time_point last_update{};
};

struct PositionCommand
{
  std::uint8_t motor_id;
  double motor_angle_deg;
  double max_motor_speed_dps;
};

class CanDriver final
{
public:
  explicit CanDriver(DriverConfig config);
  ~CanDriver();

  CanDriver(const CanDriver &) = delete;
  CanDriver & operator=(const CanDriver &) = delete;

  void start();
  void stop() noexcept;
  bool set_mode(DriverMode mode);
  DriverMode mode() const noexcept {return mode_.load();}
  ManagementOperation management_operation() const noexcept {return operation_.load();}
  bool running() const noexcept {return running_.load();}
  std::string last_error() const;

  bool enqueue_position_batch(const std::vector<PositionCommand> & commands);
  bool execute_management(
    ManagementOperation operation, const std::vector<std::uint8_t> & motor_ids,
    std::chrono::milliseconds timeout = std::chrono::milliseconds{1000});

  MotorState motor_state(std::uint8_t motor_id) const;
  std::unordered_map<std::uint8_t, MotorState> snapshot() const;

  // Public for deterministic protocol tests and replay tools.
  void process_received_frame(const CanFrame & frame);

private:
  struct TxBatch
  {
    std::vector<CanFrame> frames;
    std::shared_ptr<std::promise<bool>> completion;
  };

  bool validate_motor_id(std::uint8_t motor_id) const noexcept;
  bool enqueue_batch(TxBatch batch, bool high_priority = false);
  std::vector<CanFrame> make_poll_batch(std::size_t poll_sequence) const;
  void rx_loop() noexcept;
  void tx_loop() noexcept;
  bool write_batch(const std::vector<CanFrame> & frames) noexcept;
  void set_fault(const std::string & message) noexcept;
  void close_socket() noexcept;

  DriverConfig config_;
  std::unordered_map<std::uint8_t, MotorState> states_;
  mutable std::shared_mutex state_mutex_;
  std::condition_variable_any state_cv_;

  mutable std::mutex error_mutex_;
  std::string last_error_;
  std::atomic<DriverMode> mode_{DriverMode::OFFLINE};
  std::atomic<ManagementOperation> operation_{ManagementOperation::NONE};
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};

  std::mutex tx_mutex_;
  std::condition_variable tx_cv_;
  std::deque<TxBatch> tx_queue_;
  std::mutex management_mutex_;
  std::thread rx_thread_;
  std::thread tx_thread_;
  int socket_fd_{-1};
};

const char * to_string(DriverMode mode) noexcept;
const char * to_string(ManagementOperation operation) noexcept;

}  // namespace pi_arm_can
