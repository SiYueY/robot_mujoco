#include <mujoco_simulation/simulation.hpp>

int main() {
    mujoco_simulation::Simulation simulation;
    return simulation.status() == mujoco_simulation::SimulationStatus::Uninitialized ? 0 : 1;
}
