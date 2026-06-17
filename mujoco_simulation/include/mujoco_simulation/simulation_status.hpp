#pragma once

namespace mujoco_simulation {

enum class SimulationStatus {
  Uninitialized,
  Stopped,
  Running,
  Paused,
  Stopping,
  Error,
};

}  // namespace mujoco_simulation
