#include "buffer/command_buffer.hpp"

#include <algorithm>

namespace mujoco_simulation {
namespace {

// Command channels are dense: every slot in [0, size) must be a valid
// component id, so a batch vector can be indexed directly by component id.
bool dense_layout(const CommandChannelLayout& layout) {
    return layout.empty() || std::find(layout.begin(), layout.end(), 0U) == layout.end();
}

}  // namespace

bool CommandBuffer::configure_channels(
    CommandChannelLayout joint_ids, CommandChannelLayout mobile_base_ids) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_ || !dense_layout(joint_ids) || !dense_layout(mobile_base_ids)) return false;
    joint_valid_ids_ = std::move(joint_ids);
    mobile_base_valid_ids_ = std::move(mobile_base_ids);
    auto snapshot = std::make_shared<RobotCommand>();
    snapshot->joints.assign(joint_valid_ids_.size(), JointCommand{});
    snapshot->mobile_bases.assign(mobile_base_valid_ids_.size(), MobileBaseCommand{});
    current_ = std::move(snapshot);
    return true;
}

bool CommandBuffer::finalize_configuration() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_ || current_ == nullptr) return false;
    initialized_ = true;
    auto updated = std::make_shared<RobotCommand>(*current_);
    updated->sequence = ++sequence_;
    current_ = std::move(updated);
    return true;
}

bool CommandBuffer::write(ComponentId id, const JointCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || current_ == nullptr) return false;
    if (id >= joint_valid_ids_.size() || !joint_valid_ids_[id] ||
        !CommandTraits<JointCommand>::validate(command))
        return false;
    auto updated = std::make_shared<RobotCommand>(*current_);
    updated->joints[id] = command;
    updated->sequence = ++sequence_;
    current_ = std::move(updated);
    return true;
}

bool CommandBuffer::write(ComponentId id, const MobileBaseCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || current_ == nullptr) return false;
    if (id >= mobile_base_valid_ids_.size() || !mobile_base_valid_ids_[id] ||
        !CommandTraits<MobileBaseCommand>::validate(command))
        return false;
    auto updated = std::make_shared<RobotCommand>(*current_);
    updated->mobile_bases[id] = command;
    updated->sequence = ++sequence_;
    current_ = std::move(updated);
    return true;
}

bool CommandBuffer::write(const JointCommands& commands) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || current_ == nullptr) return false;
    if (commands.empty()) return true;
    if (!validate(commands)) return false;
    auto updated = std::make_shared<RobotCommand>(*current_);
    updated->joints = commands;
    updated->sequence = ++sequence_;
    current_ = std::move(updated);
    return true;
}

bool CommandBuffer::write(const MobileBaseCommands& commands) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || current_ == nullptr) return false;
    if (commands.empty()) return true;
    if (!validate(commands)) return false;
    auto updated = std::make_shared<RobotCommand>(*current_);
    updated->mobile_bases = commands;
    updated->sequence = ++sequence_;
    current_ = std::move(updated);
    return true;
}

bool CommandBuffer::write(const RobotCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || current_ == nullptr) return false;
    if (command.joints.empty() && command.mobile_bases.empty()) return true;
    if ((!command.joints.empty() && !validate(command.joints)) ||
        (!command.mobile_bases.empty() && !validate(command.mobile_bases)))
        return false;
    auto updated = std::make_shared<RobotCommand>();
    updated->joints = command.joints;
    updated->mobile_bases = command.mobile_bases;
    updated->sequence = ++sequence_;
    current_ = std::move(updated);
    return true;
}

bool CommandBuffer::validate(const JointCommands& commands) const {
    if (commands.size() != joint_valid_ids_.size()) return false;
    for (std::size_t id = 0; id < commands.size(); ++id) {
        if (!joint_valid_ids_[id] || !CommandTraits<JointCommand>::validate(commands[id]))
            return false;
    }
    return true;
}

bool CommandBuffer::validate(const MobileBaseCommands& commands) const {
    if (commands.size() != mobile_base_valid_ids_.size()) return false;
    for (std::size_t id = 0; id < commands.size(); ++id) {
        if (!mobile_base_valid_ids_[id] ||
            !CommandTraits<MobileBaseCommand>::validate(commands[id]))
            return false;
    }
    return true;
}

std::shared_ptr<const RobotCommand> CommandBuffer::read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_;
}

bool CommandBuffer::read_if_updated(
    std::uint64_t last_sequence, std::shared_ptr<const RobotCommand>& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_ == nullptr || current_->sequence == last_sequence) return false;
    out = current_;
    return true;
}

void CommandBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (current_ == nullptr) return;
    auto cleared = std::make_shared<RobotCommand>();
    cleared->joints.assign(joint_valid_ids_.size(), JointCommand{});
    cleared->mobile_bases.assign(mobile_base_valid_ids_.size(), MobileBaseCommand{});
    cleared->sequence = ++sequence_;
    current_ = std::move(cleared);
}

void CommandBuffer::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    current_.reset();
    initialized_ = false;
    ++sequence_;
}

}  // namespace mujoco_simulation
