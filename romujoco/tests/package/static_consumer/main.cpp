#include <romujoco/common/bitmask.hpp>
#include <romujoco/simulation.hpp>

int main() {
    romujoco::BitMask<> bits;
    if (!bits.set(0) || !bits.contains(0)) return 2;
    romujoco::Simulation simulation;
    return simulation.status() == romujoco::SimulationStatus::Uninitialized ? 0 : 1;
}
