#pragma once

#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/buffer/command_snapshot.hpp"
#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"
#include "mujoco_simulation/result_code.hpp"
#include "mujoco_simulation/simulation_status.hpp"

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
  using JointModeResolver = std::function<CommandInterfaceType(std::string)>;

  ResultCode write_joint_command(std::string component_name, const JointCommand& command);

  ResultCode write_mobile_base_command(std::string component_name,
                                       const MobileBaseCommand& command);

  CommandSnapshot read() const;

  CommandSnapshot read(const Clock::time_point now,
                       const JointModeResolver& joint_mode_resolver) const;

  void clear();

  void set_timeout_config(const CommandTimeoutConfig& config);

 private:
  struct TimedJointCommand {
    JointCommand command;
    Clock::time_point submission_time;
  };

  struct TimedMobileBaseCommand {
    MobileBaseCommand command;
    Clock::time_point submission_time;
  };

  bool timed_out(const Clock::time_point submission_time, const Clock::time_point now) const;

  JointCommand effective_joint_command(std::string name, const TimedJointCommand& timed_command,
                                       CommandInterfaceType mode,
                                       const Clock::time_point now) const;

  MobileBaseCommand effective_mobile_base_command(const TimedMobileBaseCommand& timed_command,
                                                  const Clock::time_point now) const;

  mutable std::mutex mutex_;
  CommandTimeoutConfig timeout_config_{};
  std::unordered_map<std::string, TimedJointCommand> joint_commands_;
  std::unordered_map<std::string, TimedMobileBaseCommand> mobile_base_commands_;
  std::uint64_t sequence_{0};
};

}  // namespace mujoco_simulation
