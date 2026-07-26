#include "mujoco_simulation/buffer/command_buffer.hpp"

#include <utility>

#include "mujoco_simulation/common/logging.hpp"

namespace mujoco_simulation {

bool CommandBuffer::write_joint_command(std::string component_name,
                                        const JointCommand &command) {
  if (component_name.empty()) {
    LOG_ERROR << "joint command component name must not be empty.";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  joint_commands_[std::string(component_name)] = command;
  ++sequence_;
  return true;
}

bool CommandBuffer::write_mobile_base_command(
    std::string component_name, const MobileBaseCommand &command) {
  if (component_name.empty()) {
    LOG_ERROR << "mobile base command component name must not be empty.";
    return false;
  }

  MobileBaseCommand stored_command = command;
  stored_command.mobile_base_name = component_name;
  std::lock_guard<std::mutex> lock(mutex_);
  mobile_base_commands_[std::string(component_name)] =
      std::move(stored_command);
  ++sequence_;
  return true;
}

bool CommandBuffer::write_command(const RobotCommand &command) {
  JointCommands joint_commands;
  MobileBaseCommands mobile_base_commands;
  joint_commands.reserve(command.joint_commands.size());
  mobile_base_commands.reserve(command.mobile_base_commands.size());

  for (const auto &[name, command_value] : command.joint_commands) {
    if (name.empty()) {
      LOG_ERROR << "joint command component name must not be empty.";
      return false;
    }
    JointCommand normalized_command = command_value;
    normalized_command.joint_name = name;
    joint_commands.emplace(name, std::move(normalized_command));
  }
  for (const auto &[name, command_value] : command.mobile_base_commands) {
    if (name.empty()) {
      LOG_ERROR << "mobile base command component name must not be empty.";
      return false;
    }
    MobileBaseCommand normalized_command = command_value;
    normalized_command.mobile_base_name = name;
    mobile_base_commands.emplace(name, std::move(normalized_command));
  }

  std::lock_guard<std::mutex> lock(mutex_);
  joint_commands_ = std::move(joint_commands);
  mobile_base_commands_ = std::move(mobile_base_commands);
  ++sequence_;
  return true;
}

RobotCommand CommandBuffer::read() const {
  std::lock_guard<std::mutex> lock(mutex_);
  RobotCommand snapshot;
  snapshot.sequence = sequence_;
  snapshot.joint_commands = joint_commands_;
  snapshot.mobile_base_commands = mobile_base_commands_;
  return snapshot;
}

void CommandBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  joint_commands_.clear();
  mobile_base_commands_.clear();
  ++sequence_;
}

} // namespace mujoco_simulation
