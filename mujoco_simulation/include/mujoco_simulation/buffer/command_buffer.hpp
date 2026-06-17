#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>

#include "mujoco_simulation/buffer/command_snapshot.hpp"
#include "mujoco_simulation/component/joint/joint_command.hpp"
#include "mujoco_simulation/component/joint/joint_config.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_command.hpp"
#include "mujoco_simulation/status.hpp"

namespace mujoco_simulation {

enum class CommandTimeoutBehavior {
  KeepLast,
  ZeroCommand,
  HoldPosition,
};

struct CommandTimeoutConfig {
  bool enabled{true};
  double timeout_seconds{0.2};
  CommandTimeoutBehavior behavior{CommandTimeoutBehavior::ZeroCommand};
};

class CommandBuffer {
 public:
  using Clock = std::chrono::steady_clock;
  using JointModeResolver = std::function<CommandInterfaceType(std::string_view)>;

  Status set_joint_command(std::string_view component_name, const JointCommand& command) {
    if (component_name.empty()) {
      return Status::invalid_argument("Joint command target name must not be empty.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    joint_commands_[std::string(component_name)] = TimedJointCommand{command, Clock::now()};
    ++sequence_;
    return Status::Ok();
  }

  Status set_mobile_base_command(std::string_view component_name,
                                 const MobileBaseCommand& command) {
    if (component_name.empty()) {
      return Status::invalid_argument("Mobile base command target name must not be empty.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    mobile_base_commands_[std::string(component_name)] =
        TimedMobileBaseCommand{command, Clock::now()};
    ++sequence_;
    return Status::Ok();
  }

  CommandSnapshot snapshot() const { return snapshot(Clock::now(), {}); }

  CommandSnapshot snapshot(const Clock::time_point now,
                           const JointModeResolver& joint_mode_resolver) const {
    std::lock_guard<std::mutex> lock(mutex_);
    CommandSnapshot snapshot;
    snapshot.sequence = sequence_;
    for (const auto& [name, timed_command] : joint_commands_) {
      const CommandInterfaceType mode =
          joint_mode_resolver ? joint_mode_resolver(name) : CommandInterfaceType::None;
      snapshot.joint_commands.emplace(name,
                                      effective_joint_command(name, timed_command, mode, now));
    }
    for (const auto& [name, timed_command] : mobile_base_commands_) {
      snapshot.mobile_base_commands.emplace(name,
                                            effective_mobile_base_command(timed_command, now));
    }
    return snapshot;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    joint_commands_.clear();
    mobile_base_commands_.clear();
    ++sequence_;
  }

  void set_timeout_config(const CommandTimeoutConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    timeout_config_ = config;
  }

 private:
  struct TimedJointCommand {
    JointCommand command;
    Clock::time_point submission_time;
  };

  struct TimedMobileBaseCommand {
    MobileBaseCommand command;
    Clock::time_point submission_time;
  };

  bool timed_out(const Clock::time_point submission_time, const Clock::time_point now) const {
    if (!timeout_config_.enabled || timeout_config_.timeout_seconds <= 0.0) {
      return false;
    }
    return now - submission_time > std::chrono::duration<double>(timeout_config_.timeout_seconds);
  }

  JointCommand effective_joint_command(std::string_view name,
                                       const TimedJointCommand& timed_command,
                                       CommandInterfaceType mode,
                                       const Clock::time_point now) const {
    JointCommand command = timed_command.command;
    if (!timed_out(timed_command.submission_time, now)) {
      return command;
    }

    switch (timeout_config_.behavior) {
      case CommandTimeoutBehavior::KeepLast:
        return command;
      case CommandTimeoutBehavior::HoldPosition:
        if (mode == CommandInterfaceType::Position) {
          return command;
        }
        break;
      case CommandTimeoutBehavior::ZeroCommand:
        break;
    }

    command.name = std::string(name);
    if (mode == CommandInterfaceType::Velocity) {
      command.velocity = 0.0;
    } else if (mode == CommandInterfaceType::Effort) {
      command.effort = 0.0;
    }
    return command;
  }

  MobileBaseCommand effective_mobile_base_command(const TimedMobileBaseCommand& timed_command,
                                                  const Clock::time_point now) const {
    MobileBaseCommand command = timed_command.command;
    if (!timed_out(timed_command.submission_time, now) ||
        timeout_config_.behavior == CommandTimeoutBehavior::KeepLast) {
      return command;
    }

    command.linear = {0.0, 0.0, 0.0};
    command.angular = {0.0, 0.0, 0.0};
    command.linear_x = 0.0;
    command.linear_y = 0.0;
    command.angular_z = 0.0;
    return command;
  }

  mutable std::mutex mutex_;
  CommandTimeoutConfig timeout_config_{};
  std::unordered_map<std::string, TimedJointCommand> joint_commands_;
  std::unordered_map<std::string, TimedMobileBaseCommand> mobile_base_commands_;
  std::uint64_t sequence_{0};
};

}  // namespace mujoco_simulation
