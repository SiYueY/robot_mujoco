#pragma once

#include <chrono>

namespace mujoco_simulation {

using SimulationClock = std::chrono::steady_clock;
using SimulationTimePoint = SimulationClock::time_point;
using SimulationDuration = SimulationClock::duration;

}  // namespace mujoco_simulation
