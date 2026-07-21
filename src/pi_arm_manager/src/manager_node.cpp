#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "pi_arm_interfaces/action/direct_move.hpp"
#include "pi_arm_interfaces/action/move_j.hpp"
#include "pi_arm_interfaces/action/move_js.hpp"
#include "pi_arm_interfaces/action/move_l.hpp"
#include "pi_arm_interfaces/msg/hardware_state.hpp"
#include "pi_arm_interfaces/msg/robot_state.hpp"
#include "pi_arm_interfaces/msg/task_state.hpp"
#include "pi_arm_interfaces/srv/manage_motor.hpp"
#include "pi_arm_interfaces/srv/stop_motion.hpp"
#include "pi_arm_manager/state_aggregator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

namespace pi_arm_manager
{
namespace
{

// Filled at startup from the robot model (URDF position/velocity limits plus
// robot_description_planning acceleration limits) so the robot description
// stays the single source of truth.
struct JointLimits
{
  double lower{0.0};
  double upper{0.0};
  double velocity{0.0};
  double acceleration{0.0};
};

struct CartesianPeaks
{
  double linear_velocity{0.0};
  double angular_velocity{0.0};
  double linear_acceleration{0.0};
  double angular_acceleration{0.0};
};

// Peak TCP speed/accel along a timed joint trajectory (finite differences on EE pose).
CartesianPeaks measure_cartesian_peaks(
  robot_trajectory::RobotTrajectory & trajectory, const std::string & ee_link)
{
  CartesianPeaks peaks;
  const std::size_t n = trajectory.getWayPointCount();
  if (n < 2) {
    return peaks;
  }

  std::vector<double> times(n, 0.0);
  std::vector<double> linear_speeds(n, 0.0);
  std::vector<double> angular_speeds(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    trajectory.getWayPointPtr(i)->updateLinkTransforms();
    if (i > 0) {
      times[i] = times[i - 1] + trajectory.getWayPointDurationFromPrevious(i);
    }
  }

  Eigen::Isometry3d prev_pose = trajectory.getWayPoint(0).getGlobalLinkTransform(ee_link);
  for (std::size_t i = 1; i < n; ++i) {
    const Eigen::Isometry3d pose = trajectory.getWayPoint(i).getGlobalLinkTransform(ee_link);
    const double dt = std::max(trajectory.getWayPointDurationFromPrevious(i), 1e-9);
    const double d_lin = (pose.translation() - prev_pose.translation()).norm();
    const double d_ang = Eigen::Quaterniond(prev_pose.rotation()).angularDistance(
      Eigen::Quaterniond(pose.rotation()));
    linear_speeds[i] = d_lin / dt;
    angular_speeds[i] = d_ang / dt;
    peaks.linear_velocity = std::max(peaks.linear_velocity, linear_speeds[i]);
    peaks.angular_velocity = std::max(peaks.angular_velocity, angular_speeds[i]);
    prev_pose = pose;
  }
  for (std::size_t i = 2; i < n; ++i) {
    const double dt = std::max(times[i] - times[i - 1], 1e-9);
    peaks.linear_acceleration = std::max(
      peaks.linear_acceleration, std::abs(linear_speeds[i] - linear_speeds[i - 1]) / dt);
    peaks.angular_acceleration = std::max(
      peaks.angular_acceleration, std::abs(angular_speeds[i] - angular_speeds[i - 1]) / dt);
  }
  return peaks;
}

// Uniform time stretch: positions unchanged, v' = v/k, a' = a/k^2. k >= 1 slows motion.
void stretch_joint_trajectory(trajectory_msgs::msg::JointTrajectory & trajectory, double k)
{
  if (k <= 1.0 + 1e-12) {
    return;
  }
  for (auto & point : trajectory.points) {
    const double t = rclcpp::Duration(point.time_from_start).seconds() * k;
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    for (double & velocity : point.velocities) {
      velocity /= k;
    }
    for (double & acceleration : point.accelerations) {
      acceleration /= (k * k);
    }
  }
}

// Industry MoveL timing: joint-feasible TOTG first, then slow down so TCP respects
// the commanded Cartesian velocity/acceleration (never speed up past joint limits).
double cartesian_limit_stretch(
  const CartesianPeaks & peaks,
  double max_linear_velocity, double max_linear_acceleration,
  double max_angular_velocity, double max_angular_acceleration)
{
  double stretch = 1.0;
  if (peaks.linear_velocity > 1e-9) {
    stretch = std::max(stretch, peaks.linear_velocity / std::max(max_linear_velocity, 1e-9));
  }
  if (peaks.angular_velocity > 1e-9) {
    stretch = std::max(stretch, peaks.angular_velocity / std::max(max_angular_velocity, 1e-9));
  }
  if (peaks.linear_acceleration > 1e-9) {
    stretch = std::max(
      stretch,
      std::sqrt(peaks.linear_acceleration / std::max(max_linear_acceleration, 1e-9)));
  }
  if (peaks.angular_acceleration > 1e-9) {
    stretch = std::max(
      stretch,
      std::sqrt(peaks.angular_acceleration / std::max(max_angular_acceleration, 1e-9)));
  }
  return stretch;
}

}  // namespace

class ManagerNode : public rclcpp::Node
{
public:
  using MoveJ = pi_arm_interfaces::action::MoveJ;
  using MoveJs = pi_arm_interfaces::action::MoveJs;
  using MoveL = pi_arm_interfaces::action::MoveL;
  using DirectMove = pi_arm_interfaces::action::DirectMove;
  using Follow = control_msgs::action::FollowJointTrajectory;
  using GoalHandleMoveJ = rclcpp_action::ServerGoalHandle<MoveJ>;
  using GoalHandleMoveJs = rclcpp_action::ServerGoalHandle<MoveJs>;
  using GoalHandleMoveL = rclcpp_action::ServerGoalHandle<MoveL>;
  using GoalHandleDirect = rclcpp_action::ServerGoalHandle<DirectMove>;

  ManagerNode()
  : Node("pi_arm_manager"),
    task_claimed_(false),
    stop_requested_(false),
    next_task_id_(1),
    still_frames_(0)
  {
    hardware_timeout_ = declare_parameter("hardware_timeout_sec", 1.0);
    operation_timeout_ = declare_parameter("operation_timeout_sec", 5.0);
    motion_timeout_ = declare_parameter("motion_timeout_sec", 120.0);
    still_velocity_ = declare_parameter(
      "still_velocity_rad_s", 0.00017453292519943296);
    linear_velocity_limit_ = declare_parameter("linear_velocity_limit_m_s", 1.0);
    linear_acceleration_limit_ = declare_parameter("linear_acceleration_limit_m_s2", 2.0);
    angular_velocity_limit_ = declare_parameter("angular_velocity_limit_rad_s", 3.14);
    angular_acceleration_limit_ = declare_parameter("angular_acceleration_limit_rad_s2", 6.28);
    state_rate_ = declare_parameter("state_publish_rate_hz", 10.0);
    follow_action_name_ = declare_parameter(
      "follow_joint_trajectory_action",
      std::string("/joint_trajectory_controller/follow_joint_trajectory"));

    task_.status = pi_arm_interfaces::msg::TaskState::IDLE;
    task_.message = "idle";
    hardware_sub_ = create_subscription<pi_arm_interfaces::msg::HardwareState>(
      "/pi_arm/hardware_state", rclcpp::SensorDataQoS(),
      std::bind(&ManagerNode::hardware_callback, this, std::placeholders::_1));
    state_pub_ = create_publisher<pi_arm_interfaces::msg::RobotState>(
      "/pi_arm/state", rclcpp::QoS(10).reliable().transient_local());

    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // Reentrant group so worker threads can wait_for() action futures without
    // starving against the default MutuallyExclusive group (timers/subscriptions).
    action_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    enable_client_ = create_client<pi_arm_interfaces::srv::ManageMotor>(
      "/pi_arm/hardware/enable_motor", rmw_qos_profile_services_default, service_group_);
    disable_client_ = create_client<pi_arm_interfaces::srv::ManageMotor>(
      "/pi_arm/hardware/disable_motor", rmw_qos_profile_services_default, service_group_);
    reset_client_ = create_client<pi_arm_interfaces::srv::ManageMotor>(
      "/pi_arm/hardware/reset_motor", rmw_qos_profile_services_default, service_group_);
    zero_client_ = create_client<pi_arm_interfaces::srv::ManageMotor>(
      "/pi_arm/hardware/set_zero", rmw_qos_profile_services_default, service_group_);
    follow_client_ = rclcpp_action::create_client<Follow>(
      this, follow_action_name_, action_group_);

    enable_service_ = make_management_service(
      "/pi_arm/enable_motor", enable_client_, Operation::ENABLE);
    disable_service_ = make_management_service(
      "/pi_arm/disable_motor", disable_client_, Operation::DISABLE);
    reset_service_ = make_management_service(
      "/pi_arm/reset_motor", reset_client_, Operation::RESET);
    zero_service_ = make_management_service(
      "/pi_arm/set_zero", zero_client_, Operation::SET_ZERO);
    stop_service_ = create_service<pi_arm_interfaces::srv::StopMotion>(
      "/pi_arm/stop_motion",
      std::bind(
        &ManagerNode::stop_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, state_rate_));
    state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&ManagerNode::publish_state, this));
  }

  void initialize()
  {
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "pi_arm");
    move_group_->setEndEffectorLink("tool0");
    load_joint_limits();

    movej_server_ = rclcpp_action::create_server<MoveJ>(
      this, "/pi_arm/movej",
      std::bind(&ManagerNode::goal_movej, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManagerNode::cancel_movej, this, std::placeholders::_1),
      std::bind(&ManagerNode::accept_movej, this, std::placeholders::_1));
    movejs_server_ = rclcpp_action::create_server<MoveJs>(
      this, "/pi_arm/movejs",
      std::bind(&ManagerNode::goal_movejs, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManagerNode::cancel_movejs, this, std::placeholders::_1),
      std::bind(&ManagerNode::accept_movejs, this, std::placeholders::_1));
    movel_server_ = rclcpp_action::create_server<MoveL>(
      this, "/pi_arm/movel",
      std::bind(&ManagerNode::goal_movel, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManagerNode::cancel_movel, this, std::placeholders::_1),
      std::bind(&ManagerNode::accept_movel, this, std::placeholders::_1));
    direct_server_ = rclcpp_action::create_server<DirectMove>(
      this, "/pi_arm/direct_move",
      std::bind(&ManagerNode::goal_direct, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManagerNode::cancel_direct, this, std::placeholders::_1),
      std::bind(&ManagerNode::accept_direct, this, std::placeholders::_1));
  }

private:
  enum class Operation {ENABLE, DISABLE, RESET, SET_ZERO};

  // Reads position/velocity bounds from the robot model (URDF) and acceleration
  // bounds from the robot_description_planning joint limits that the launch
  // file loads from joint_limits.yaml. Called once before the action servers
  // are created; joint_limits_ is read-only afterwards.
  void load_joint_limits()
  {
    const auto robot_model = move_group_->getRobotModel();
    joint_limits_.clear();
    max_joint_velocity_limit_ = 0.0;
    max_joint_acceleration_limit_ = 0.0;
    for (const auto & name : move_group_->getActiveJoints()) {
      const auto * joint_model = robot_model->getJointModel(name);
      if (joint_model == nullptr || joint_model->getVariableBounds().empty()) {
        throw std::runtime_error("robot model has no bounds for joint " + name);
      }
      const auto & bounds = joint_model->getVariableBounds().front();
      if (!bounds.position_bounded_ || !bounds.velocity_bounded_ ||
        bounds.max_velocity_ <= 0.0)
      {
        throw std::runtime_error(
                "URDF must provide position and velocity limits for joint " + name);
      }
      if (!bounds.acceleration_bounded_ || bounds.max_acceleration_ <= 0.0) {
        throw std::runtime_error(
                "joint " + name + " has no acceleration limit; load joint_limits.yaml "
                "under the robot_description_planning namespace");
      }
      JointLimits limits;
      limits.lower = bounds.min_position_;
      limits.upper = bounds.max_position_;
      limits.velocity = bounds.max_velocity_;
      limits.acceleration = bounds.max_acceleration_;
      joint_limits_.emplace(name, limits);
      max_joint_velocity_limit_ = std::max(max_joint_velocity_limit_, limits.velocity);
      max_joint_acceleration_limit_ = std::max(max_joint_acceleration_limit_, limits.acceleration);
    }
    if (joint_limits_.empty()) {
      throw std::runtime_error("planning group pi_arm has no active joints");
    }
  }

  double min_velocity_limit(const std::vector<std::string> & names) const
  {
    double limit = max_joint_velocity_limit_;
    for (const auto & name : names) {
      const auto found = joint_limits_.find(name);
      if (found == joint_limits_.end()) {
        return 0.0;
      }
      limit = std::min(limit, found->second.velocity);
    }
    return limit;
  }

  double min_acceleration_limit(const std::vector<std::string> & names) const
  {
    double limit = max_joint_acceleration_limit_;
    for (const auto & name : names) {
      const auto found = joint_limits_.find(name);
      if (found == joint_limits_.end()) {
        return 0.0;
      }
      limit = std::min(limit, found->second.acceleration);
    }
    return limit;
  }

  rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr make_management_service(
    const std::string & name,
    const rclcpp::Client<pi_arm_interfaces::srv::ManageMotor>::SharedPtr & client,
    Operation operation)
  {
    return create_service<pi_arm_interfaces::srv::ManageMotor>(
      name,
      [this, client, operation](
        const std::shared_ptr<pi_arm_interfaces::srv::ManageMotor::Request> request,
        std::shared_ptr<pi_arm_interfaces::srv::ManageMotor::Response> response)
      {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        const auto state = current_state();
        const bool still = still_frames_.load() >= 3;
        bool allowed = false;
        switch (operation) {
          case Operation::ENABLE:
            allowed = state == pi_arm_interfaces::msg::RobotState::DISABLED &&
              !task_claimed_.load() && still;
            break;
          case Operation::DISABLE:
            allowed = state == pi_arm_interfaces::msg::RobotState::READY &&
              !task_claimed_.load();
            break;
          case Operation::SET_ZERO:
            allowed = (state == pi_arm_interfaces::msg::RobotState::READY ||
              state == pi_arm_interfaces::msg::RobotState::DISABLED) &&
              !task_claimed_.load() && still;
            break;
          case Operation::RESET:
            allowed = state != pi_arm_interfaces::msg::RobotState::DISCONNECTED;
            break;
        }
        if (!allowed) {
          response->success = false;
          response->code = 2001;
          response->message = "operation rejected by robot state";
          return;
        }
        if (!client->wait_for_service(std::chrono::duration<double>(operation_timeout_))) {
          response->success = false;
          response->code = 2002;
          response->message = "hardware service unavailable";
          return;
        }
        auto future = client->async_send_request(request);
        if (future.wait_for(std::chrono::duration<double>(operation_timeout_)) !=
          std::future_status::ready)
        {
          response->success = false;
          response->code = 2002;
          response->message = "hardware service timeout";
          return;
        }
        *response = *future.get();
      },
      rmw_qos_profile_services_default, service_group_);
  }

  void hardware_callback(const pi_arm_interfaces::msg::HardwareState::SharedPtr message)
  {
    const bool complete_motion_state =
      !message->joint_names.empty() &&
      message->velocities.size() == message->joint_names.size() &&
      message->moving.size() == message->joint_names.size() &&
      message->fresh.size() == message->joint_names.size() &&
      std::all_of(message->fresh.begin(), message->fresh.end(), [](bool v) {return v;});
    const bool no_motion = complete_motion_state &&
      !std::any_of(message->moving.begin(), message->moving.end(), [](bool v) {return v;}) &&
      std::all_of(
      message->velocities.begin(), message->velocities.end(),
      [this](double v) {return std::isfinite(v) && std::abs(v) <= still_velocity_;});
    still_frames_.store(no_motion ? std::min(3, still_frames_.load() + 1) : 0);
    bool stop_for_hardware = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      hardware_ = *message;
      last_hardware_time_ = now();
      const auto state = aggregate_state(hardware_, task_);
      stop_for_hardware = task_claimed_.load() &&
        (state == pi_arm_interfaces::msg::RobotState::DISCONNECTED ||
        state == pi_arm_interfaces::msg::RobotState::FAULT ||
        state == pi_arm_interfaces::msg::RobotState::DISABLED);
    }
    if (stop_for_hardware) {
      request_stop();
    }
  }

  uint8_t current_state()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto hardware = hardware_;
    if ((now() - last_hardware_time_).seconds() > hardware_timeout_) {
      hardware.connected = false;
    }
    const auto state = aggregate_state(hardware, task_);
    if (state == pi_arm_interfaces::msg::RobotState::READY && still_frames_.load() < 3) {
      return pi_arm_interfaces::msg::RobotState::RUNNING;
    }
    return state;
  }

  void publish_state()
  {
    pi_arm_interfaces::msg::RobotState message;
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      message.hardware = hardware_;
      stale = (now() - last_hardware_time_).seconds() > hardware_timeout_;
      message.task = task_;
    }
    message.stamp = now();
    auto state_input = message.hardware;
    if (stale) {
      state_input.connected = false;
    }
    message.state = aggregate_state(state_input, message.task);
    if (message.state == pi_arm_interfaces::msg::RobotState::READY &&
      still_frames_.load() < 3)
    {
      message.state = pi_arm_interfaces::msg::RobotState::RUNNING;
    }
    message.state_name = state_name(message.state);
    state_pub_->publish(message);
  }

  bool claim_goal(const std::string & command)
  {
    std::lock_guard<std::mutex> command_lock(command_mutex_);
    if (current_state() != pi_arm_interfaces::msg::RobotState::READY) {
      return false;
    }
    bool expected = false;
    if (!task_claimed_.compare_exchange_strong(expected, true)) {
      return false;
    }
    begin_task(command);
    return true;
  }

  void begin_task(const std::string & command)
  {
    stop_requested_.store(false);
    still_frames_.store(0);
    std::lock_guard<std::mutex> lock(state_mutex_);
    task_.task_id = next_task_id_.fetch_add(1);
    task_.command = command;
    task_.status = pi_arm_interfaces::msg::TaskState::RUNNING;
    task_.progress = 0.0F;
    task_.code = 0;
    task_.message = "running";
  }

  void finish_task(uint8_t status, int32_t code, const std::string & message)
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_.status = status;
      task_.progress = status == pi_arm_interfaces::msg::TaskState::SUCCEEDED ? 1.0F : task_.progress;
      task_.code = code;
      task_.message = message;
    }
    task_claimed_.store(false);
  }

  bool wait_still()
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(motion_timeout_));
    auto next_log = std::chrono::steady_clock::now();
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (stop_requested_.load()) {
        return false;
      }
      if (still_frames_.load() >= 3) {
        return true;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_log) {
        RCLCPP_INFO(
          get_logger(), "wait_still: still_frames=%d remain=%.1fs",
          still_frames_.load(),
          std::chrono::duration<double>(deadline - now).count());
        next_log = now + 1s;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool valid_joint_goal(
    const std::vector<std::string> & names, const std::vector<double> & positions,
    double velocity) const
  {
    if (names.empty() || names.size() != positions.size() ||
      !std::isfinite(velocity) || velocity <= 0.0 ||
      !std::all_of(positions.begin(), positions.end(), [](double v) {return std::isfinite(v);}))
    {
      return false;
    }
    for (size_t i = 0; i < names.size(); ++i) {
      const auto found = joint_limits_.find(names[i]);
      if (names[i].empty() ||
        found == joint_limits_.end() ||
        positions[i] < found->second.lower ||
        positions[i] > found->second.upper ||
        velocity > found->second.velocity + 1e-6 ||
        std::find(names.begin() + static_cast<std::ptrdiff_t>(i + 1), names.end(), names[i]) !=
        names.end())
      {
        return false;
      }
    }
    return true;
  }

  bool valid_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory)
  {
    const auto & names = trajectory.joint_names;
    if (names.empty() || trajectory.points.empty() || trajectory.points.size() > 50000U) {
      return false;
    }
    std::vector<double> previous_positions;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (hardware_.joint_names.empty() ||
        hardware_.positions.size() != hardware_.joint_names.size())
      {
        return false;
      }
      previous_positions.reserve(names.size());
      for (const auto & name : names) {
        const auto found = std::find(
          hardware_.joint_names.begin(), hardware_.joint_names.end(), name);
        if (found == hardware_.joint_names.end()) {
          return false;
        }
        previous_positions.push_back(
          hardware_.positions[static_cast<std::size_t>(
              std::distance(hardware_.joint_names.begin(), found))]);
      }
    }
    const double start_velocity_limit = min_velocity_limit(names);
    if (start_velocity_limit <= 0.0 ||
      !valid_joint_goal(names, previous_positions, start_velocity_limit))
    {
      return false;
    }
    std::vector<double> velocity_limits(names.size(), 0.0);
    std::vector<double> acceleration_limits(names.size(), 0.0);
    for (std::size_t index = 0; index < names.size(); ++index) {
      const auto & limits = joint_limits_.at(names[index]);
      velocity_limits[index] = limits.velocity;
      acceleration_limits[index] = limits.acceleration;
    }

    double previous_time = 0.0;
    std::vector<double> previous_segment_velocity(names.size(), 0.0);
    bool have_segment_velocity = false;
    for (const auto & point : trajectory.points) {
      const double time =
        static_cast<double>(point.time_from_start.sec) +
        static_cast<double>(point.time_from_start.nanosec) * 1e-9;
      const double dt = time - previous_time;
      if (dt <= 0.0 ||
        !valid_joint_goal(names, point.positions, start_velocity_limit))
      {
        return false;
      }
      if ((!point.velocities.empty() && point.velocities.size() != names.size()) ||
        (!point.accelerations.empty() && point.accelerations.size() != names.size()))
      {
        return false;
      }
      std::vector<double> segment_velocity(names.size(), 0.0);
      for (std::size_t index = 0; index < names.size(); ++index) {
        segment_velocity[index] =
          (point.positions[index] - previous_positions[index]) / dt;
        if (std::abs(segment_velocity[index]) > velocity_limits[index] + 1e-6 ||
          (!point.velocities.empty() &&
          (!std::isfinite(point.velocities[index]) ||
          std::abs(point.velocities[index]) > velocity_limits[index] + 1e-6)) ||
          (!point.accelerations.empty() &&
          (!std::isfinite(point.accelerations[index]) ||
          std::abs(point.accelerations[index]) > acceleration_limits[index] + 1e-6)))
        {
          return false;
        }
        if (have_segment_velocity &&
          std::abs(segment_velocity[index] - previous_segment_velocity[index]) / dt >
          acceleration_limits[index] + 1e-6)
        {
          return false;
        }
      }
      previous_positions = point.positions;
      previous_segment_velocity = std::move(segment_velocity);
      have_segment_velocity = true;
      previous_time = time;
    }
    return true;
  }

  rclcpp_action::GoalResponse goal_movej(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJ::Goal> goal)
  {
    if (!valid_joint_goal(goal->joint_names, goal->positions, goal->max_velocity) ||
      !std::isfinite(goal->max_acceleration) || goal->max_acceleration <= 0.0 ||
      goal->max_acceleration > min_acceleration_limit(goal->joint_names) + 1e-6)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return claim_goal("movej") ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse goal_movejs(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveJs::Goal> goal)
  {
    if (!valid_trajectory(goal->trajectory)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return claim_goal("movejs") ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse goal_movel(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveL::Goal> goal)
  {
    const auto & position = goal->target.pose.position;
    const auto & orientation = goal->target.pose.orientation;
    const double quaternion_norm =
      orientation.x * orientation.x + orientation.y * orientation.y +
      orientation.z * orientation.z + orientation.w * orientation.w;
    if (!std::isfinite(goal->max_linear_velocity) || goal->max_linear_velocity <= 0.0 ||
      goal->max_linear_velocity > linear_velocity_limit_ + 1e-6 ||
      !std::isfinite(goal->max_linear_acceleration) || goal->max_linear_acceleration <= 0.0 ||
      goal->max_linear_acceleration > linear_acceleration_limit_ + 1e-6 ||
      !std::isfinite(goal->max_angular_velocity) || goal->max_angular_velocity <= 0.0 ||
      goal->max_angular_velocity > angular_velocity_limit_ + 1e-6 ||
      !std::isfinite(goal->max_angular_acceleration) || goal->max_angular_acceleration <= 0.0 ||
      goal->max_angular_acceleration > angular_acceleration_limit_ + 1e-6 ||
      !std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
      !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
      !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
      std::abs(quaternion_norm - 1.0) > 1e-3)
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return claim_goal("movel") ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse goal_direct(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const DirectMove::Goal> goal)
  {
    if (!valid_joint_goal(goal->joint_names, goal->positions, goal->max_velocity)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return claim_goal("direct_move") ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
           rclcpp_action::GoalResponse::REJECT;
  }

  template<typename GoalHandleT>
  rclcpp_action::CancelResponse cancel_common(const std::shared_ptr<GoalHandleT>)
  {
    request_stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  rclcpp_action::CancelResponse cancel_movej(const std::shared_ptr<GoalHandleMoveJ> goal)
  {
    return cancel_common(goal);
  }
  rclcpp_action::CancelResponse cancel_movejs(const std::shared_ptr<GoalHandleMoveJs> goal)
  {
    return cancel_common(goal);
  }
  rclcpp_action::CancelResponse cancel_movel(const std::shared_ptr<GoalHandleMoveL> goal)
  {
    return cancel_common(goal);
  }
  rclcpp_action::CancelResponse cancel_direct(const std::shared_ptr<GoalHandleDirect> goal)
  {
    return cancel_common(goal);
  }

  void accept_movej(const std::shared_ptr<GoalHandleMoveJ> goal)
  {
    auto self = std::static_pointer_cast<ManagerNode>(shared_from_this());
    std::thread(
      [self, goal]() {
        try {
          self->execute_movej(goal);
        } catch (const std::exception & error) {
          self->complete_action<MoveJ>(goal, false, "movej", error.what());
        }
      }).detach();
  }
  void accept_movejs(const std::shared_ptr<GoalHandleMoveJs> goal)
  {
    auto self = std::static_pointer_cast<ManagerNode>(shared_from_this());
    std::thread(
      [self, goal]() {
        try {
          self->execute_movejs(goal);
        } catch (const std::exception & error) {
          self->complete_action<MoveJs>(goal, false, "movejs", error.what());
        }
      }).detach();
  }
  void accept_movel(const std::shared_ptr<GoalHandleMoveL> goal)
  {
    auto self = std::static_pointer_cast<ManagerNode>(shared_from_this());
    std::thread(
      [self, goal]() {
        try {
          self->execute_movel(goal);
        } catch (const std::exception & error) {
          self->complete_action<MoveL>(goal, false, "movel", error.what());
        }
      }).detach();
  }
  void accept_direct(const std::shared_ptr<GoalHandleDirect> goal)
  {
    auto self = std::static_pointer_cast<ManagerNode>(shared_from_this());
    std::thread(
      [self, goal]() {
        try {
          self->execute_direct(goal);
        } catch (const std::exception & error) {
          self->complete_action<DirectMove>(goal, false, "direct_move", error.what());
        }
      }).detach();
  }

  void execute_movej(const std::shared_ptr<GoalHandleMoveJ> handle)
  {
    publish_feedback<MoveJ>(handle, 0.0F, "running");
    const auto goal = handle->get_goal();
    std::map<std::string, double> targets;
    for (size_t i = 0; i < goal->joint_names.size(); ++i) {
      targets[goal->joint_names[i]] = goal->positions[i];
    }
    move_group_->setMaxVelocityScalingFactor(
      std::clamp(goal->max_velocity / std::max(1e-9, max_joint_velocity_limit_), 0.001, 1.0));
    move_group_->setMaxAccelerationScalingFactor(
      std::clamp(
        goal->max_acceleration / std::max(1e-9, max_joint_acceleration_limit_), 0.001, 1.0));
    if (!move_group_->setJointValueTarget(targets)) {
      throw std::runtime_error("movej target is invalid for planning group pi_arm");
    }
    const bool backend_ok = static_cast<bool>(move_group_->move());
    complete_action<MoveJ>(handle, backend_ok, "movej");
  }

  void execute_movel(const std::shared_ptr<GoalHandleMoveL> handle)
  {
    publish_feedback<MoveL>(handle, 0.0F, "running");
    const auto goal = handle->get_goal();
    const std::string frame =
      goal->target.header.frame_id.empty() ? "base_link" : goal->target.header.frame_id;
    const auto & pose = goal->target.pose;
    RCLCPP_INFO(
      get_logger(),
      "movel start: frame=%s target xyz=(%.4f, %.4f, %.4f) "
      "max_lin_v=%.4f m/s max_lin_a=%.4f m/s2 max_ang_v=%.4f rad/s max_ang_a=%.4f rad/s2",
      frame.c_str(), pose.position.x, pose.position.y, pose.position.z,
      goal->max_linear_velocity, goal->max_linear_acceleration,
      goal->max_angular_velocity, goal->max_angular_acceleration);
    move_group_->setPoseReferenceFrame(frame);
    // Path geometry only — do NOT map page TCP speed onto joint TOTG scaling.
    // Requested Cartesian limits are applied after joint-feasible timing (below).
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);
    moveit_msgs::msg::RobotTrajectory trajectory;
    const std::vector<geometry_msgs::msg::Pose> waypoints{goal->target.pose};
    RCLCPP_INFO(get_logger(), "movel calling computeCartesianPath...");
    const double fraction =
      move_group_->computeCartesianPath(waypoints, 0.005, trajectory, true);
    RCLCPP_INFO(
      get_logger(),
      "movel Cartesian path done: fraction=%.4f points=%zu joints=%zu",
      fraction, trajectory.joint_trajectory.points.size(),
      trajectory.joint_trajectory.joint_names.size());
    if (fraction < 0.999) {
      throw std::runtime_error(
              "movel Cartesian path is incomplete: fraction=" + std::to_string(fraction));
    }
    publish_feedback<MoveL>(handle, 0.2F, "cartesian_planned");
    RCLCPP_INFO(get_logger(), "movel fetching current robot state...");
    const auto current_state = move_group_->getCurrentState(operation_timeout_);
    if (!current_state) {
      throw std::runtime_error("movel current robot state is unavailable");
    }
    // 1) TOTG at full joint capability → feasible joint vel/acc + initial timing.
    // 2) Uniform time stretch so peak TCP speed/accel match the page command.
    //    Stretch only slows down; joint limits remain satisfied.
    RCLCPP_INFO(get_logger(), "movel TOTG at full joint limits, then Cartesian stretch...");
    robot_trajectory::RobotTrajectory timed_trajectory(
      move_group_->getRobotModel(), "pi_arm");
    timed_trajectory.setRobotTrajectoryMsg(*current_state, trajectory);
    trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
    if (!time_parameterization.computeTimeStamps(timed_trajectory, 1.0, 1.0)) {
      throw std::runtime_error("movel trajectory time parameterization failed");
    }
    const CartesianPeaks peaks = measure_cartesian_peaks(timed_trajectory, "tool0");
    const double stretch = cartesian_limit_stretch(
      peaks, goal->max_linear_velocity, goal->max_linear_acceleration,
      goal->max_angular_velocity, goal->max_angular_acceleration);
    timed_trajectory.getRobotTrajectoryMsg(trajectory);
    stretch_joint_trajectory(trajectory.joint_trajectory, stretch);
    RCLCPP_INFO(
      get_logger(),
      "movel Cartesian peaks before stretch: lin_v=%.4f m/s ang_v=%.4f rad/s "
      "lin_a=%.4f m/s2 ang_a=%.4f rad/s2; stretch=%.3f",
      peaks.linear_velocity, peaks.angular_velocity,
      peaks.linear_acceleration, peaks.angular_acceleration, stretch);
    // Do not use move_group_->execute() here: MoveGroupInterface blocking execute
    // is unreliable when this node is already spun by MultiThreadedExecutor and the
    // call runs on a worker thread (task stays RUNNING, joints never move). Send the
    // timed trajectory directly to the controller, same path as movejs/direct_move.
    Follow::Goal follow_goal;
    follow_goal.trajectory = trajectory.joint_trajectory;
    // stamp=0 means "execute as soon as received" for joint_trajectory_controller.
    follow_goal.trajectory.header.stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    if (follow_goal.trajectory.joint_names.empty() || follow_goal.trajectory.points.empty()) {
      throw std::runtime_error("movel produced an empty joint trajectory");
    }
    log_trajectory_summary("movel", follow_goal.trajectory);
    publish_feedback<MoveL>(handle, 0.3F, "sending_follow");
    complete_follow_action<MoveL>(handle, follow_goal, "movel");
  }

  void log_trajectory_summary(
    const std::string & tag, const trajectory_msgs::msg::JointTrajectory & trajectory) const
  {
    std::string joints;
    for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
      if (i > 0) {
        joints += ",";
      }
      joints += trajectory.joint_names[i];
    }
    const auto & first = trajectory.points.front();
    const auto & last = trajectory.points.back();
    const double duration = rclcpp::Duration(last.time_from_start).seconds();
    RCLCPP_INFO(
      get_logger(),
      "%s trajectory: joints=[%s] points=%zu duration=%.3fs "
      "first_has_vel=%d last_has_vel=%d stamp_sec=%d stamp_nanosec=%u",
      tag.c_str(), joints.c_str(), trajectory.points.size(), duration,
      first.velocities.empty() ? 0 : 1, last.velocities.empty() ? 0 : 1,
      trajectory.header.stamp.sec, trajectory.header.stamp.nanosec);
    if (!first.positions.empty() && !last.positions.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "%s trajectory ends: first_q0=%.4f last_q0=%.4f delta_q0=%.4f",
        tag.c_str(), first.positions.front(), last.positions.front(),
        last.positions.front() - first.positions.front());
    }
  }

  void execute_movejs(const std::shared_ptr<GoalHandleMoveJs> handle)
  {
    publish_feedback<MoveJs>(handle, 0.0F, "running");
    Follow::Goal goal;
    goal.trajectory = handle->get_goal()->trajectory;
    complete_follow_action<MoveJs>(handle, goal, "movejs");
  }

  void execute_direct(const std::shared_ptr<GoalHandleDirect> handle)
  {
    publish_feedback<DirectMove>(handle, 0.0F, "running");
    const auto request = handle->get_goal();
    Follow::Goal goal;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    double duration = 0.1;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (hardware_.joint_names.empty() ||
        hardware_.positions.size() != hardware_.joint_names.size())
      {
        throw std::runtime_error("hardware joint positions unavailable");
      }
      goal.trajectory.joint_names = hardware_.joint_names;
      point.positions = hardware_.positions;
      for (size_t i = 0; i < request->joint_names.size(); ++i) {
        const auto found = std::find(
          hardware_.joint_names.begin(), hardware_.joint_names.end(), request->joint_names[i]);
        if (found == hardware_.joint_names.end()) {
          throw std::runtime_error("direct_move contains unknown joint");
        }
        const auto index = static_cast<size_t>(
          std::distance(hardware_.joint_names.begin(), found));
        duration = std::max(
          duration, std::abs(request->positions[i] - point.positions[index]) /
          request->max_velocity);
        point.positions[index] = request->positions[i];
      }
    }
    point.time_from_start = rclcpp::Duration::from_seconds(duration);
    goal.trajectory.points.push_back(point);
    complete_follow_action<DirectMove>(handle, goal, "direct_move");
  }

  template<typename ActionT, typename GoalHandleT>
  void publish_feedback(
    const std::shared_ptr<GoalHandleT> handle, float progress, const std::string & message)
  {
    auto feedback = std::make_shared<typename ActionT::Feedback>();
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      task_.progress = progress;
      feedback->task_id = task_.task_id;
    }
    feedback->progress = progress;
    feedback->message = message;
    handle->publish_feedback(feedback);
  }

  // Wait on an rclcpp future with periodic logs so a hang is visible in journal.
  template<typename FutureT>
  std::future_status wait_future_logged(
    FutureT & future, double timeout_sec, const std::string & stage)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_sec));
    auto next_log = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        RCLCPP_ERROR(
          get_logger(), "follow wait TIMEOUT at stage=%s after %.1fs",
          stage.c_str(), timeout_sec);
        return std::future_status::timeout;
      }
      if (now >= next_log) {
        const double remain = std::chrono::duration<double>(deadline - now).count();
        RCLCPP_INFO(
          get_logger(), "follow waiting stage=%s remain=%.1fs still_frames=%d stop=%d",
          stage.c_str(), remain, still_frames_.load(), stop_requested_.load() ? 1 : 0);
        next_log = now + 1s;
      }
      const auto slice = std::min(
        std::chrono::duration<double>(0.2),
        std::chrono::duration<double>(deadline - now));
      const auto status = future.wait_for(
        std::chrono::duration_cast<std::chrono::nanoseconds>(slice));
      if (status == std::future_status::ready) {
        RCLCPP_INFO(get_logger(), "follow wait READY at stage=%s", stage.c_str());
        return status;
      }
      if (stop_requested_.load()) {
        RCLCPP_WARN(get_logger(), "follow wait interrupted by stop at stage=%s", stage.c_str());
        return std::future_status::timeout;
      }
    }
    return std::future_status::timeout;
  }

  template<typename ActionT, typename GoalHandleT>
  void complete_follow_action(
    const std::shared_ptr<GoalHandleT> handle, const Follow::Goal & goal,
    const std::string & command)
  {
    RCLCPP_INFO(
      get_logger(),
      "%s follow: waiting for action server '%s' (timeout=%.1fs)",
      command.c_str(), follow_action_name_.c_str(), operation_timeout_);
    if (!follow_client_->wait_for_action_server(std::chrono::duration<double>(operation_timeout_))) {
      RCLCPP_ERROR(
        get_logger(), "%s follow: action server unavailable: %s",
        command.c_str(), follow_action_name_.c_str());
      complete_action<ActionT>(handle, false, command, "trajectory controller unavailable");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "%s follow: sending goal points=%zu",
      command.c_str(), goal.trajectory.points.size());
    auto send_future = follow_client_->async_send_goal(goal);
    if (wait_future_logged(send_future, operation_timeout_, command + "/send_goal") !=
      std::future_status::ready)
    {
      complete_action<ActionT>(handle, false, command, "trajectory goal timeout");
      return;
    }
    auto follow_handle = send_future.get();
    if (!follow_handle) {
      RCLCPP_ERROR(get_logger(), "%s follow: goal REJECTED by controller", command.c_str());
      complete_action<ActionT>(handle, false, command, "trajectory goal rejected");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "%s follow: goal ACCEPTED, waiting for result (timeout=%.1fs)",
      command.c_str(), motion_timeout_);
    publish_feedback<ActionT>(handle, 0.5F, "follow_executing");
    {
      std::lock_guard<std::mutex> lock(follow_mutex_);
      active_follow_goal_ = follow_handle;
    }
    auto result_future = follow_client_->async_get_result(follow_handle);
    const auto status = wait_future_logged(
      result_future, motion_timeout_, command + "/get_result");
    bool backend_ok = false;
    std::string detail = command;
    if (status == std::future_status::ready) {
      const auto wrapped = result_future.get();
      const int error_code = wrapped.result ? wrapped.result->error_code : -1;
      backend_ok = wrapped.result &&
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped.result->error_code == Follow::Result::SUCCESSFUL;
      detail = wrapped.result ? wrapped.result->error_string : "trajectory returned no result";
      RCLCPP_INFO(
        get_logger(),
        "%s follow: result ready ok=%d result_code=%d error_code=%d detail=%s",
        command.c_str(), backend_ok ? 1 : 0, static_cast<int>(wrapped.code),
        error_code, detail.c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "%s follow: result TIMEOUT, canceling goal", command.c_str());
      follow_client_->async_cancel_goal(follow_handle);
      detail = "trajectory result timeout";
    }
    {
      std::lock_guard<std::mutex> lock(follow_mutex_);
      active_follow_goal_.reset();
    }
    complete_action<ActionT>(handle, backend_ok, command, detail);
  }

  template<typename ActionT, typename GoalHandleT>
  void complete_action(
    const std::shared_ptr<GoalHandleT> handle, bool backend_ok,
    const std::string & command, const std::string & detail = "")
  {
    RCLCPP_INFO(
      get_logger(),
      "%s complete_action: backend_ok=%d detail=%s stop=%d canceling=%d still_frames=%d",
      command.c_str(), backend_ok ? 1 : 0, detail.c_str(),
      stop_requested_.load() ? 1 : 0, handle->is_canceling() ? 1 : 0, still_frames_.load());
    auto result = std::make_shared<typename ActionT::Result>();
    if (handle->is_canceling() || stop_requested_.load()) {
      result->success = true;
      result->code = 0;
      result->message = "motion stopped";
      finish_task(pi_arm_interfaces::msg::TaskState::CANCELED, 0, result->message);
      if (handle->is_canceling()) {
        handle->canceled(result);
      } else {
        handle->abort(result);
      }
      return;
    }
    if (!backend_ok) {
      result->success = false;
      result->code = 3001;
      result->message = detail.empty() ? command + " backend failed" : detail;
      finish_task(pi_arm_interfaces::msg::TaskState::FAILED, result->code, result->message);
      handle->abort(result);
      return;
    }
    still_frames_.store(0);
    RCLCPP_INFO(get_logger(), "%s waiting for still (timeout=%.1fs)", command.c_str(), motion_timeout_);
    publish_feedback<ActionT>(handle, 0.9F, "waiting_still");
    if (!wait_still()) {
      RCLCPP_ERROR(
        get_logger(),
        "%s wait_still FAILED still_frames=%d stop=%d",
        command.c_str(), still_frames_.load(), stop_requested_.load() ? 1 : 0);
      if (stop_requested_.load() || handle->is_canceling()) {
        complete_action<ActionT>(handle, true, command, "motion stopped");
        return;
      }
      result->success = false;
      result->code = 3002;
      result->message = command + " did not become stationary";
      finish_task(pi_arm_interfaces::msg::TaskState::FAILED, result->code, result->message);
      handle->abort(result);
      return;
    }
    RCLCPP_INFO(get_logger(), "%s succeeded", command.c_str());
    result->success = true;
    result->code = 0;
    result->message = "ok";
    publish_feedback<ActionT>(handle, 1.0F, "completed");
    finish_task(pi_arm_interfaces::msg::TaskState::SUCCEEDED, 0, "ok");
    handle->succeed(result);
  }

  void request_stop()
  {
    stop_requested_.store(true);
    if (move_group_) {
      move_group_->stop();
    }
    rclcpp_action::ClientGoalHandle<Follow>::SharedPtr follow_goal;
    {
      std::lock_guard<std::mutex> lock(follow_mutex_);
      follow_goal = active_follow_goal_;
    }
    if (follow_goal) {
      auto cancel_future = follow_client_->async_cancel_goal(follow_goal);
      if (cancel_future.wait_for(std::chrono::duration<double>(operation_timeout_)) !=
        std::future_status::ready)
      {
        RCLCPP_WARN(get_logger(), "Timed out waiting for trajectory cancellation");
      }
      send_hold_trajectory();
    }
  }

  void send_hold_trajectory()
  {
    Follow::Goal hold;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (hardware_.joint_names.empty() ||
        hardware_.positions.size() != hardware_.joint_names.size())
      {
        RCLCPP_WARN(get_logger(), "Cannot send hold trajectory: joint state unavailable");
        return;
      }
      hold.trajectory.joint_names = hardware_.joint_names;
      point.positions = hardware_.positions;
    }
    point.time_from_start = rclcpp::Duration::from_seconds(0.1);
    hold.trajectory.points.push_back(point);
    follow_client_->async_send_goal(hold);
  }

  void stop_callback(
    const std::shared_ptr<pi_arm_interfaces::srv::StopMotion::Request>,
    std::shared_ptr<pi_arm_interfaces::srv::StopMotion::Response> response)
  {
    if (!task_claimed_.load()) {
      response->success = true;
      response->code = 0;
      response->message = "no active motion";
      return;
    }
    request_stop();
    response->success = true;
    response->code = 0;
    response->message = "stop requested";
  }

  double hardware_timeout_;
  double operation_timeout_;
  double motion_timeout_;
  double still_velocity_;
  std::map<std::string, JointLimits> joint_limits_;
  double max_joint_velocity_limit_{0.0};
  double max_joint_acceleration_limit_{0.0};
  double linear_velocity_limit_;
  double linear_acceleration_limit_;
  double angular_velocity_limit_;
  double angular_acceleration_limit_;
  double state_rate_;
  std::string follow_action_name_;

  std::mutex state_mutex_;
  std::mutex follow_mutex_;
  std::mutex command_mutex_;
  pi_arm_interfaces::msg::HardwareState hardware_;
  pi_arm_interfaces::msg::TaskState task_;
  rclcpp::Time last_hardware_time_{0, 0, RCL_ROS_TIME};
  std::atomic_bool task_claimed_;
  std::atomic_bool stop_requested_;
  std::atomic_uint64_t next_task_id_;
  std::atomic_int still_frames_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp_action::Client<Follow>::SharedPtr follow_client_;
  rclcpp_action::ClientGoalHandle<Follow>::SharedPtr active_follow_goal_;
  rclcpp::Subscription<pi_arm_interfaces::msg::HardwareState>::SharedPtr hardware_sub_;
  rclcpp::Publisher<pi_arm_interfaces::msg::RobotState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::Client<pi_arm_interfaces::srv::ManageMotor>::SharedPtr enable_client_;
  rclcpp::Client<pi_arm_interfaces::srv::ManageMotor>::SharedPtr disable_client_;
  rclcpp::Client<pi_arm_interfaces::srv::ManageMotor>::SharedPtr reset_client_;
  rclcpp::Client<pi_arm_interfaces::srv::ManageMotor>::SharedPtr zero_client_;
  rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr enable_service_;
  rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr disable_service_;
  rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr reset_service_;
  rclcpp::Service<pi_arm_interfaces::srv::ManageMotor>::SharedPtr zero_service_;
  rclcpp::Service<pi_arm_interfaces::srv::StopMotion>::SharedPtr stop_service_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp_action::Server<MoveJ>::SharedPtr movej_server_;
  rclcpp_action::Server<MoveJs>::SharedPtr movejs_server_;
  rclcpp_action::Server<MoveL>::SharedPtr movel_server_;
  rclcpp_action::Server<DirectMove>::SharedPtr direct_server_;
};

}  // namespace pi_arm_manager

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pi_arm_manager::ManagerNode>();
  try {
    node->initialize();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(node->get_logger(), "Manager initialization failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), std::max(4U, std::thread::hardware_concurrency()));
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
