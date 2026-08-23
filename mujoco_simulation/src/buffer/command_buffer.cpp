#include "buffer/command_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace mujoco_simulation {
namespace {

constexpr std::size_t kInvalidCommandIndex = std::numeric_limits<std::size_t>::max();

bool is_valid(const JointCommand& command) {
    switch (static_cast<JointMode>(command.mode)) {
        case JointMode::Hybrid:
            return std::isfinite(command.position) && std::isfinite(command.velocity) &&
                   std::isfinite(command.effort) && std::isfinite(command.stiffness) &&
                   std::isfinite(command.damping);
        case JointMode::Position:
            return std::isfinite(command.position);
        case JointMode::Velocity:
            return std::isfinite(command.velocity);
        case JointMode::Effort:
            return std::isfinite(command.effort);
    }
    return false;
}

bool is_valid(const MobileBaseCommand& command) {
    switch (command.mode) {
        case MobileBaseControlMode::Twist:
            return std::isfinite(command.base_linear[0]) && std::isfinite(command.base_linear[1]) &&
                   std::isfinite(command.base_angular[2]);
        case MobileBaseControlMode::WheelLinear:
            for (double value : command.wheel_linear)
                if (!std::isfinite(value)) return false;
            return true;
        case MobileBaseControlMode::WheelAngular:
            for (double value : command.wheel_angular)
                if (!std::isfinite(value)) return false;
            return true;
    }
    return false;
}

template <typename Command, typename IsValid>
bool validate_commands(
    const std::vector<Command>& commands, const std::vector<std::size_t>& indices,
    IsValid is_valid) {
    std::vector<std::uint8_t> seen(indices.size(), 0U);
    for (const Command& command : commands) {
        if (command.id >= indices.size()) return false;
        if (indices[command.id] == kInvalidCommandIndex) return false;
        if (seen[command.id]) return false;
        if (!is_valid(command)) return false;
        seen[command.id] = 1U;
    }
    return true;
}

template <typename Command>
void configure_commands(const std::vector<std::size_t>& indices, std::vector<Command>& commands) {
    for (std::size_t id = 0; id < indices.size(); ++id) {
        if (indices[id] == kInvalidCommandIndex) continue;
        if (commands.size() <= indices[id]) commands.resize(indices[id] + 1U);
        commands[indices[id]].id = id;
    }
}

template <typename Command>
void write_commands(
    std::vector<Command>& target, const std::vector<Command>& updates,
    const std::vector<std::size_t>& indices) {
    for (const Command& command : updates) target[indices[command.id]] = command;
}

bool configure_active_joints(
    const ComponentConfigList& components, std::vector<std::size_t>& indices,
    std::vector<JointMode>& default_modes, std::vector<EnumMask<JointMode>>& allowed_modes) {
    std::vector<const JointInfo*> joints;
    for (const ComponentConfig& component : components) {
        const auto* joint = std::get_if<JointInfo>(&component);
        if (joint != nullptr && joint->actuation == JointActuation::Active) joints.push_back(joint);
    }
    std::sort(joints.begin(), joints.end(), [](const JointInfo* lhs, const JointInfo* rhs) {
        return lhs->id < rhs->id;
    });
    if (!joints.empty() &&
        (joints.back()->id > 255U ||
         std::adjacent_find(
             joints.begin(), joints.end(), [](const JointInfo* lhs, const JointInfo* rhs) {
                 return lhs->id == rhs->id;
             }) != joints.end()))
        return false;
    indices.assign(joints.empty() ? 0U : joints.back()->id + 1U, kInvalidCommandIndex);
    default_modes.resize(joints.size());
    allowed_modes.resize(joints.size());
    for (std::size_t slot = 0; slot < joints.size(); ++slot) {
        indices[joints[slot]->id] = slot;
        default_modes[slot] = joints[slot]->default_mode;
        allowed_modes[slot] = joints[slot]->allowed_modes;
    }
    return true;
}

}  // namespace

bool CommandBuffer::configure(
    std::shared_ptr<const ComponentIdResolver> id_resolver, const ComponentConfigList& components) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (id_resolver == nullptr) return false;
    if (initialized_) return false;
    if (!configure_active_joints(
            components, active_joint_indices_, active_joint_default_modes_,
            active_joint_allowed_modes_))
        return false;
    auto robot_command = std::make_shared<RobotCommand>();
    configure_commands(active_joint_indices_, robot_command->joints);
    for (std::size_t slot = 0; slot < robot_command->joints.size(); ++slot)
        robot_command->joints[slot].mode =
            static_cast<std::uint8_t>(active_joint_default_modes_[slot]);
    configure_commands(id_resolver->mobile_bases(), robot_command->mobile_bases);
    robot_command->sequence = ++sequence_;
    command_ = std::move(robot_command);
    id_resolver_ = std::move(id_resolver);
    initialized_ = true;
    return true;
}

void CommandBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_ == nullptr) return;
    // A stopped simulation may be started again without rebuilding its command
    // channels.  Preserve the last validated snapshot so an Active Joint never
    // resumes with JointMode::None or a zeroed target.
    auto preserved_command = std::make_shared<RobotCommand>(*command_);
    preserved_command->sequence = ++sequence_;
    command_ = std::move(preserved_command);
}

void CommandBuffer::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    command_.reset();
    id_resolver_.reset();
    active_joint_indices_.clear();
    active_joint_default_modes_.clear();
    active_joint_allowed_modes_.clear();
    initialized_ = false;
    ++sequence_;
}

bool CommandBuffer::write(const RobotCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || command_ == nullptr) return false;
    if (command.joints.empty() && command.mobile_bases.empty()) return true;
    if (!command.joints.empty() && !validate(command.joints)) return false;
    if (!command.mobile_bases.empty() && !validate(command.mobile_bases)) return false;
    auto robot_command = std::make_shared<RobotCommand>(*command_);
    write_commands(robot_command->joints, command.joints, active_joint_indices_);
    write_commands(robot_command->mobile_bases, command.mobile_bases, id_resolver_->mobile_bases());
    robot_command->sequence = ++sequence_;
    command_ = std::move(robot_command);
    return true;
}

bool CommandBuffer::write(const JointCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || command_ == nullptr) return false;
    if (!validate(JointCommands{command})) return false;
    auto robot_command = std::make_shared<RobotCommand>(*command_);
    std::size_t slot = 0;
    if (command.id >= active_joint_indices_.size() ||
        active_joint_indices_[command.id] == kInvalidCommandIndex)
        return false;
    slot = active_joint_indices_[command.id];
    robot_command->joints[slot] = command;
    robot_command->sequence = ++sequence_;
    command_ = std::move(robot_command);
    return true;
}

bool CommandBuffer::write(const JointCommands& commands) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || command_ == nullptr) return false;
    if (commands.empty()) return true;
    if (!validate(commands)) return false;
    auto robot_command = std::make_shared<RobotCommand>(*command_);
    write_commands(robot_command->joints, commands, active_joint_indices_);
    robot_command->sequence = ++sequence_;
    command_ = std::move(robot_command);
    return true;
}

bool CommandBuffer::write(const MobileBaseCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || command_ == nullptr) return false;
    if (!validate(MobileBaseCommands{command})) return false;
    auto robot_command = std::make_shared<RobotCommand>(*command_);
    robot_command->mobile_bases[id_resolver_->mobile_bases()[command.id]] = command;
    robot_command->sequence = ++sequence_;
    command_ = std::move(robot_command);
    return true;
}

bool CommandBuffer::write(const MobileBaseCommands& commands) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || command_ == nullptr) return false;
    if (commands.empty()) return true;
    if (!validate(commands)) return false;
    auto robot_command = std::make_shared<RobotCommand>(*command_);
    write_commands(robot_command->mobile_bases, commands, id_resolver_->mobile_bases());
    robot_command->sequence = ++sequence_;
    command_ = std::move(robot_command);
    return true;
}

std::shared_ptr<const RobotCommand> CommandBuffer::read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_;
}

bool CommandBuffer::read(
    std::uint64_t last_sequence, std::shared_ptr<const RobotCommand>& command) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_ == nullptr) return false;
    if (command_->sequence == last_sequence) return false;
    command = command_;
    return true;
}

bool CommandBuffer::validate(const JointCommands& commands) const {
    const auto& indices = active_joint_indices_;
    std::vector<std::uint8_t> seen(indices.size(), 0U);
    for (const JointCommand& command : commands) {
        if (command.id >= indices.size() || indices[command.id] == kInvalidCommandIndex)
            return false;
        const std::size_t slot = indices[command.id];
        const JointMode mode = static_cast<JointMode>(command.mode);
        if (seen[slot] || mode == JointMode::None ||
            !active_joint_allowed_modes_[slot].contains(mode) || !is_valid(command))
            return false;
        seen[slot] = 1U;
    }
    return true;
}

bool CommandBuffer::validate(const MobileBaseCommands& commands) const {
    return validate_commands(
        commands, id_resolver_->mobile_bases(),
        [](const MobileBaseCommand& command) { return is_valid(command); });
}

}  // namespace mujoco_simulation
