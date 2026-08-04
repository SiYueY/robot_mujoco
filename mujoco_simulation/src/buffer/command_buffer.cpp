#include "buffer/command_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace mujoco_simulation {
namespace {

constexpr std::size_t kNoCommandIndex = std::numeric_limits<std::size_t>::max();

bool is_valid(const JointCommand& command) {
    switch (command.mode) {
        case JointControlMode::Position:
            return std::isfinite(command.position);
        case JointControlMode::Velocity:
            return std::isfinite(command.velocity);
        case JointControlMode::Effort:
            return std::isfinite(command.effort);
        case JointControlMode::Hybrid:
            return std::isfinite(command.position) && std::isfinite(command.velocity) &&
                   std::isfinite(command.effort) && std::isfinite(command.stiffness) &&
                   std::isfinite(command.damping);
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
    IsValid is_valid_command) {
    std::vector<std::uint8_t> seen(indices.size(), 0U);
    for (const Command& command : commands) {
        if (command.id >= indices.size() || indices[command.id] == kNoCommandIndex ||
            seen[command.id] || !is_valid_command(command))
            return false;
        seen[command.id] = 1U;
    }
    return true;
}

template <typename Command>
void configure_commands(const std::vector<std::size_t>& indices, std::vector<Command>& commands) {
    for (std::size_t id = 0; id < indices.size(); ++id) {
        if (indices[id] == kNoCommandIndex) continue;
        if (commands.size() <= indices[id]) commands.resize(indices[id] + 1U);
        commands[indices[id]].id = id;
    }
}

template <typename Command>
void merge_commands(
    std::vector<Command>& target, const std::vector<Command>& updates,
    const std::vector<std::size_t>& indices) {
    for (const Command& command : updates) target[indices[command.id]] = command;
}

}  // namespace

bool CommandBuffer::configure(std::shared_ptr<const ComponentIndex> component_index) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_) return false;
    auto robot_command = std::make_shared<RobotCommand>();
    if (component_index == nullptr) return false;
    configure_commands(component_index->joints(), robot_command->joints);
    configure_commands(component_index->mobile_bases(), robot_command->mobile_bases);
    component_index_ = std::move(component_index);
    robot_command->sequence = ++sequence_;
    command_ = std::move(robot_command);
    initialized_ = true;
    return true;
}

bool CommandBuffer::write(const JointCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || command_ == nullptr) return false;
    if (!validate(JointCommands{command})) return false;
    auto robot_command = std::make_shared<RobotCommand>(*command_);
    robot_command->joints[component_index_->joints()[command.id]] = command;
    robot_command->sequence = ++sequence_;
    command_ = std::move(robot_command);
    return true;
}

bool CommandBuffer::write(const MobileBaseCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || command_ == nullptr) return false;
    if (!validate(MobileBaseCommands{command})) return false;
    auto robot_command = std::make_shared<RobotCommand>(*command_);
    robot_command->mobile_bases[component_index_->mobile_bases()[command.id]] = command;
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
    merge_commands(robot_command->joints, commands, component_index_->joints());
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
    merge_commands(robot_command->mobile_bases, commands, component_index_->mobile_bases());
    robot_command->sequence = ++sequence_;
    command_ = std::move(robot_command);
    return true;
}

bool CommandBuffer::write(const RobotCommand& command) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_ || command_ == nullptr) return false;
    if (command.joints.empty() && command.mobile_bases.empty()) return true;
    if ((!command.joints.empty() && !validate(command.joints)) ||
        (!command.mobile_bases.empty() && !validate(command.mobile_bases)))
        return false;
    auto robot_command = std::make_shared<RobotCommand>(*command_);
    merge_commands(robot_command->joints, command.joints, component_index_->joints());
    merge_commands(
        robot_command->mobile_bases, command.mobile_bases, component_index_->mobile_bases());
    robot_command->sequence = ++sequence_;
    command_ = std::move(robot_command);
    return true;
}

bool CommandBuffer::validate(const JointCommands& commands) const {
    return validate_commands(commands, component_index_->joints(), [](const JointCommand& command) {
        return is_valid(command);
    });
}

bool CommandBuffer::validate(const MobileBaseCommands& commands) const {
    return validate_commands(
        commands, component_index_->mobile_bases(),
        [](const MobileBaseCommand& command) { return is_valid(command); });
}

std::shared_ptr<const RobotCommand> CommandBuffer::read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_;
}

bool CommandBuffer::read(
    std::uint64_t last_sequence, std::shared_ptr<const RobotCommand>& command) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_ == nullptr || command_->sequence == last_sequence) return false;
    command = command_;
    return true;
}

void CommandBuffer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_ == nullptr) return;
    auto cleared = std::make_shared<RobotCommand>();
    cleared->joints.reserve(command_->joints.size());
    for (const JointCommand& previous : command_->joints) {
        JointCommand command;
        command.id = previous.id;
        cleared->joints.push_back(command);
    }
    cleared->mobile_bases.reserve(command_->mobile_bases.size());
    for (const MobileBaseCommand& previous : command_->mobile_bases) {
        MobileBaseCommand command;
        command.id = previous.id;
        cleared->mobile_bases.push_back(command);
    }
    cleared->sequence = ++sequence_;
    command_ = std::move(cleared);
}

void CommandBuffer::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    command_.reset();
    component_index_.reset();
    initialized_ = false;
    ++sequence_;
}

}  // namespace mujoco_simulation
