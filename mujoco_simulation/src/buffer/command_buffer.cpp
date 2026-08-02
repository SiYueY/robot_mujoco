#include "buffer/command_buffer.hpp"

namespace mujoco_simulation {

bool CommandBuffer::finalize_configuration() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_ || channels_.empty()) return false;
    initialized_ = true;
    ++snapshot_.sequence;
    return true;
}

bool CommandBuffer::write(const RobotCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) return false;
    if (command.joints.empty() && command.mobile_bases.empty()) return true;

    auto* joint_channel = find_channel<JointCommand>();
    auto* mobile_base_channel = find_channel<MobileBaseCommand>();
    if (joint_channel == nullptr || mobile_base_channel == nullptr ||
        (!command.joints.empty() && !joint_channel->channel.validate(command.joints)) ||
        (!command.mobile_bases.empty() &&
         !mobile_base_channel->channel.validate(command.mobile_bases))) {
        return false;
    }

    CommandSnapshot updated = snapshot_;
    std::shared_ptr<const CommandChannelSnapshot<JointCommand>> joints;
    std::shared_ptr<const CommandChannelSnapshot<MobileBaseCommand>> mobile_bases;
    if (!command.joints.empty()) {
        joints = joint_channel->channel.updated_snapshot(command.joints);
        updated.channels_.at(std::type_index(typeid(JointCommand))) = joints;
    }
    if (!command.mobile_bases.empty()) {
        mobile_bases = mobile_base_channel->channel.updated_snapshot(command.mobile_bases);
        updated.channels_.at(std::type_index(typeid(MobileBaseCommand))) = mobile_bases;
    }
    ++updated.sequence;

    if (joints != nullptr) joint_channel->channel.replace_snapshot(std::move(joints));
    if (mobile_bases != nullptr)
        mobile_base_channel->channel.replace_snapshot(std::move(mobile_bases));
    snapshot_ = std::move(updated);
    return true;
}

CommandSnapshot CommandBuffer::read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
}

bool CommandBuffer::read_if_updated(std::uint64_t last_sequence, CommandSnapshot& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (snapshot_.sequence == last_sequence) return false;
    out = snapshot_;
    return true;
}

void CommandBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [_, channel] : channels_) channel->clear();
    for (auto& [type, channel] : channels_) snapshot_.channels_[type] = channel->snapshot();
    ++snapshot_.sequence;
}

void CommandBuffer::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto& [_, channel] : channels_) channel->shutdown();
    channels_.clear();
    snapshot_.channels_.clear();
    initialized_ = false;
    ++snapshot_.sequence;
}

}  // namespace mujoco_simulation
