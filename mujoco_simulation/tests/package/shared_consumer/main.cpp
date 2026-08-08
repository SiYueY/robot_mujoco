#include <mujoco_simulation/log/logging.hpp>
#include <mujoco_simulation/simulation.hpp>

int main() {
    mujoco_simulation::logging::Policy logging_config;
    logging_config.console_enabled = false;
    logging_config.file_enabled = false;
    if (!mujoco_simulation::logging::configure(logging_config)) return 1;
    SIM_INFO << "installed consumer logging header";

    mujoco_simulation::Simulation simulation;
    return simulation.status() == mujoco_simulation::SimulationStatus::Uninitialized ? 0 : 2;
}
