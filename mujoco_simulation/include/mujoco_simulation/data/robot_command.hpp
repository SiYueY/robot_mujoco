#pragma once

#include <cstdint>

#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"

namespace mujoco_simulation {

// The cross-component command state, mirroring RobotState.  Simulation::
// write_command() validates and publishes it as one command-buffer snapshot;
// the buffer assigns the sequence.  Each batch is a dense command vector
// indexed by component id, must cover the configured channel size, and an
// empty batch is a no-op that preserves prior commands.
struct RobotCommand {
    std::uint64_t sequence{0};
    JointCommands joints;
    MobileBaseCommands mobile_bases;
};

}  // namespace mujoco_simulation
