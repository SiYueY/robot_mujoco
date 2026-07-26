#include "mujoco_simulation/buffer/command_buffer.hpp"

#include <utility>

#include "common/logging.hpp"

namespace mujoco_simulation {

bool CommandBuffer::write_joint_command(JointId id,
                                        const JointCommand &command) {
  if (id == kInvalidComponentId) {
    LOG_ERROR << "joint command id is invalid.";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (joint_commands_.size() <= id) {
    joint_commands_.resize(id + 1U);
  }
  joint_commands_[id] = command;
  ++sequence_;
  return true;
}

bool CommandBuffer::write_mobile_base_command(
    MobileBaseId id, const MobileBaseCommand &command) {
  if (id == kInvalidComponentId) {
    LOG_ERROR << "mobile base command id is invalid.";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (mobile_base_commands_.size() <= id) {
    mobile_base_commands_.resize(id + 1U);
  }
  mobile_base_commands_[id] = command;
  ++sequence_;
  return true;
}

bool CommandBuffer::write_command(const RobotCommand &command) {
  std::lock_guard<std::mutex> lock(mutex_);
  joint_commands_ = command.joint_commands;
  mobile_base_commands_ = command.mobile_base_commands;
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
