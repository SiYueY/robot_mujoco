#pragma once
// Internal command buffering contract; not part of the installed API.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "component/component.hpp"
#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"
#include "mujoco_simulation/data/robot_command.hpp"

namespace mujoco_simulation {

template <typename Command>
struct CommandTraits;

template <>
struct CommandTraits<JointCommand> {
    static bool validate(const JointCommand& command) {
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
};

template <>
struct CommandTraits<MobileBaseCommand> {
    static bool validate(const MobileBaseCommand& command) {
        switch (command.mode) {
            case MobileBaseControlMode::Twist:
                return std::isfinite(command.base_linear[0]) &&
                       std::isfinite(command.base_linear[1]) &&
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
};

// Cold-path layout bitmap. Unlike vector<bool>, this has ordinary reference
// semantics and remains easy to inspect in diagnostics and tests.
using CommandChannelLayout = std::vector<std::uint8_t>;

class CommandBuffer {
public:
    bool configure_channels(CommandChannelLayout joint_ids, CommandChannelLayout mobile_base_ids);
    bool finalize_configuration();

    bool write(ComponentId id, const JointCommand& command);
    bool write(ComponentId id, const MobileBaseCommand& command);
    bool write(const JointCommands& commands);
    bool write(const MobileBaseCommands& commands);
    bool write(const RobotCommand& command);

    std::shared_ptr<const RobotCommand> read() const;
    bool read_if_updated(
        std::uint64_t last_sequence, std::shared_ptr<const RobotCommand>& out) const;
    void clear();
    void shutdown();

private:
    bool validate(const JointCommands& commands) const;
    bool validate(const MobileBaseCommands& commands) const;

    CommandChannelLayout joint_valid_ids_;
    CommandChannelLayout mobile_base_valid_ids_;
    std::uint64_t sequence_{0};
    bool initialized_{false};
    std::shared_ptr<const RobotCommand> current_;
    mutable std::mutex mutex_;
};

}  // namespace mujoco_simulation
