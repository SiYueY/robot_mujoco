#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include "romujoco/component/joint.hpp"
#include "romujoco/component/mobile_base.hpp"
#include "romujoco/data/robot_command.hpp"

#include "component/component_id_resolver.hpp"

namespace romujoco {

class CommandBuffer {
public:
    bool configure(
        std::shared_ptr<const ComponentIdResolver> id_resolver,
        const ComponentConfigList& components);
    void clear();
    void shutdown();

    bool write(const RobotCommand& command);
    bool write(const JointCommand& command);
    bool write(const JointCommands& commands);
    bool write(const GripperCommand& command);
    bool write(const GripperCommands& commands);
    bool write(const MobileBaseCommand& command);
    bool write(const MobileBaseCommands& commands);

    std::shared_ptr<const RobotCommand> read() const;
    bool read(std::uint64_t last_sequence, std::shared_ptr<const RobotCommand>& command) const;

private:
    std::shared_ptr<RobotCommand> writable_snapshot();
    bool validate(const JointCommand& command) const;
    bool validate(const JointCommands& commands) const;
    bool validate(const GripperCommand& command) const;
    bool validate(const GripperCommands& commands) const;
    bool validate(const MobileBaseCommand& command) const;
    bool validate(const MobileBaseCommands& commands) const;

private:
    bool initialized_{false};
    std::uint64_t sequence_{0};
    std::shared_ptr<const RobotCommand> command_;
    // A snapshot is writable only while the pool is its sole owner. Readers
    // and the physics thread may retain older immutable publications.
    std::vector<std::shared_ptr<RobotCommand>> snapshots_;
    std::shared_ptr<const ComponentIdResolver> id_resolver_;
    std::vector<std::size_t> active_joint_indices_;
    std::vector<JointMode> active_joint_default_modes_;
    std::vector<EnumMask<JointMode>> active_joint_allowed_modes_;
    mutable std::mutex mutex_;
};

}  // namespace romujoco
