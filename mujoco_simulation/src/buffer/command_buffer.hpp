#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"
#include "mujoco_simulation/data/robot_command.hpp"

#include "component/component_id.hpp"

namespace mujoco_simulation {

class CommandBuffer {
public:
    bool configure(std::shared_ptr<const ComponentIdResolver> id_resolver);
    void clear();
    void shutdown();

    bool write(const RobotCommand& command);
    bool write(const JointCommand& command);
    bool write(const JointCommands& commands);
    bool write(const MobileBaseCommand& command);
    bool write(const MobileBaseCommands& commands);

    std::shared_ptr<const RobotCommand> read() const;
    bool read(std::uint64_t last_sequence, std::shared_ptr<const RobotCommand>& command) const;

private:
    bool validate(const JointCommands& commands) const;
    bool validate(const MobileBaseCommands& commands) const;

private:
    bool initialized_{false};
    std::uint64_t sequence_{0};
    std::shared_ptr<const RobotCommand> command_;
    std::shared_ptr<const ComponentIdResolver> id_resolver_;
    mutable std::mutex mutex_;
};

}  // namespace mujoco_simulation
