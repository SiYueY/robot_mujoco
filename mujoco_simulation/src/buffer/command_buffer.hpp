#pragma once
// Internal command buffering contract; not part of the installed API.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <limits>
#include <vector>

#include "component/component_index.hpp"
#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"
#include "mujoco_simulation/data/robot_command.hpp"

namespace mujoco_simulation {

class CommandBuffer {
public:
    bool configure(std::shared_ptr<const ComponentIndex> component_index);
    void clear();
    void shutdown();

    bool write(const JointCommand& command);
    bool write(const MobileBaseCommand& command);
    bool write(const JointCommands& commands);
    bool write(const MobileBaseCommands& commands);
    bool write(const RobotCommand& command);

    std::shared_ptr<const RobotCommand> read() const;
    bool read(std::uint64_t last_sequence, std::shared_ptr<const RobotCommand>& command) const;

private:
    bool validate(const JointCommands& commands) const;
    bool validate(const MobileBaseCommands& commands) const;

    std::shared_ptr<const ComponentIndex> component_index_;
    std::uint64_t sequence_{0};
    bool initialized_{false};
    std::shared_ptr<const RobotCommand> command_;
    mutable std::mutex mutex_;
};

}  // namespace mujoco_simulation
