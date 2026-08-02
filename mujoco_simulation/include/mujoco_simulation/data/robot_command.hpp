#pragma once

#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"

namespace mujoco_simulation {

// A cross-component command update.  Simulation::write_command() validates both
// batches and publishes them as one command-buffer snapshot.  A batch is a
// dense command vector indexed by component id: every slot is an explicit
// command, and the batch must cover the configured command channel size.  An
// empty batch is a no-op and preserves prior commands.
struct RobotCommand {
    JointCommands joints;
    MobileBaseCommands mobile_bases;
};

}  // namespace mujoco_simulation
