#pragma once

namespace romujoco {

enum class SimulationStatus {
    Uninitialized,
    Stopped,
    Running,
    Paused,
    Stopping,
    Error,
};

}  // namespace romujoco
