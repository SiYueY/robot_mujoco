#include <romujoco/common/bitmask.hpp>
#include <romujoco/log/logging.hpp>
#include <romujoco/simulation.hpp>

int main() {
    romujoco::BitMask<> bits;
    if (!bits.set(0) || !bits.contains(0)) return 3;
    romujoco::logging::Policy logging_config;
    logging_config.console_enabled = false;
    logging_config.file_enabled = false;
    if (!romujoco::logging::configure(logging_config)) return 1;
    SIM_INFO << "installed consumer logging header";

    romujoco::Simulation simulation;
    return simulation.status() == romujoco::SimulationStatus::Uninitialized ? 0 : 2;
}
