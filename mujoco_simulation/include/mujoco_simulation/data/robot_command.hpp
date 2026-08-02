#pragma once

#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"

namespace mujoco_simulation {

// A sparse, cross-component command update.  Simulation::write_command()
// validates both batches and publishes them as one command-buffer snapshot.
// Empty batches, and empty slots within a batch, preserve prior commands.
struct RobotCommand {
  JointCommandBatch joints;
  MobileBaseCommandBatch mobile_bases;
};

} // namespace mujoco_simulation
