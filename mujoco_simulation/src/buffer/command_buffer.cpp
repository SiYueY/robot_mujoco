#include "buffer/command_buffer.hpp"

#include <algorithm>
#include <cmath>

namespace mujoco_simulation {
namespace {

constexpr std::size_t kInvalidCommandIndex = std::numeric_limits<std::size_t>::max();

bool is_valid(const JointCommand& command) {
    switch (command.mode) {
        case static_cast<std::uint8_t>(JointControlMode::Hybrid):
            return std::isfinite(command.position) && std::isfinite(command.velocity) &&
                   std::isfinite(command.effort) && std::isfinite(command.stiffness) &&
                   std::isfinite(command.damping);
        case static_cast<std::uint8_t>(JointControlMode::Position):
            return std::isfinite(command.position);
        case static_cast<std::uint8_t>(JointControlMode::Velocity):
            return std::isfinite(command.velocity);
        case static_cast<std::uint8_t>(JointControlMode::Effort):
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

}  // namespace

bool CommandBuffer::configure(std::shared_ptr<const ComponentIdResolver> id_resolver) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (id_resolver == nullptr) return false;
    if (initialized_) return false;
    auto robot_command = std::make_shared<RobotCommand>();
    configure_commands(id_resolver->joints(), robot_command->joints);
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
    auto cleared_command = std::make_shared<RobotCommand>();
    cleared_command->joints.reserve(command_->joints.size());
    for (const JointCommand& current_command : command_->joints) {
        JointCommand command;
        command.id = current_command.id;
        cleared_command->joints.push_back(command);
    }
    cleared_command->mobile_bases.reserve(command_->mobile_bases.size());
    for (const MobileBaseCommand& current_command : command_->mobile_bases) {
        MobileBaseCommand command;
        command.id = current_command.id;
        cleared_command->mobile_bases.push_back(command);
    }
    cleared_command->sequence = ++sequence_;
    command_ = std::move(cleared_command);
}

void CommandBuffer::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    command_.reset();
    id_resolver_.reset();
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
    write_commands(robot_command->joints, command.joints, id_resolver_->joints());
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
    robot_command->joints[id_resolver_->joints()[command.id]] = command;
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
    write_commands(robot_command->joints, commands, id_resolver_->joints());
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
    return validate_commands(commands, id_resolver_->joints(), [](const JointCommand& command) {
        return is_valid(command);
    });
}

bool CommandBuffer::validate(const MobileBaseCommands& commands) const {
    return validate_commands(
        commands, id_resolver_->mobile_bases(),
        [](const MobileBaseCommand& command) { return is_valid(command); });
}

}  // namespace mujoco_simulation
