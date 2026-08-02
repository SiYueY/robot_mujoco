#include <atomic>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

#include "buffer/state_buffer.hpp"

namespace {
bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}
}  // namespace

int main() {
    mujoco_simulation::StateBuffer buffer;
    std::atomic<bool> finished{false};
    std::atomic<bool> valid{true};
    std::thread writer([&] {
        for (std::uint64_t sequence = 1; sequence <= 10000U; ++sequence) {
            auto state = std::make_shared<mujoco_simulation::RobotState>();
            state->sequence = sequence;
            state->step = sequence;
            buffer.write(std::move(state));
        }
        finished.store(true);
    });
    std::thread reader([&] {
        std::uint64_t previous = 0;
        while (!finished.load()) {
            const auto state = buffer.read();
            if (state != nullptr && state->sequence < previous) valid.store(false);
            if (state != nullptr) previous = state->sequence;
        }
    });
    writer.join();
    reader.join();
    const auto final_state = buffer.read();
    return check(valid.load(), "state sequence regressed under concurrent access") &&
                   check(
                       final_state != nullptr && final_state->sequence == 10000U,
                       "final state was not published")
               ? 0
               : 1;
}
