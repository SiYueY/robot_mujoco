#include <mujoco_simulation/common/bitmask.hpp>
#include <mujoco_simulation/simulation.hpp>

int main() {
    mujoco_simulation::BitMask<> bits;
    if (!bits.set(0) || !bits.contains(0)) return 2;
    mujoco_simulation::Simulation simulation;
    return simulation.status() == mujoco_simulation::SimulationStatus::Uninitialized ? 0 : 1;
}
