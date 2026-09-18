#pragma once

#include <cstdint>

#include "romujoco/component/gripper.hpp"
#include "romujoco/component/joint.hpp"
#include "romujoco/component/mobile_base.hpp"

namespace romujoco {

// The cross-component command state, mirroring RobotState.  Simulation::
// write_command() validates and publishes it as one command-buffer snapshot;
// the buffer assigns the sequence. Each command carries its component ID;
// batches are unordered incremental updates, and an empty batch preserves prior
// commands.
struct RobotCommand {
    std::uint64_t sequence{0};
    JointCommands joints;
    GripperCommands grippers;
    MobileBaseCommands mobile_bases;
};

}  // namespace romujoco
