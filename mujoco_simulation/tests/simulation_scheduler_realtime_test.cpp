#include <atomic>
#include <chrono>
#include <iostream>

#include "mujoco_simulation/runtime/simulation_scheduler.hpp"

namespace {

bool check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << '\n';
  }
  return condition;
}

} // namespace

int main() {
  using namespace std::chrono_literals;

  mujoco_simulation::SimulationScheduler scheduler;
  std::atomic<std::size_t> invocations{0};
  if (!check(scheduler.initialize(std::chrono::duration<double>(20ms)),
             "scheduler initialization failed") ||
      !check(scheduler.register_task([&invocations] {
        ++invocations;
        return true;
      }),
             "scheduler task registration failed") ||
      !check(scheduler.start(), "scheduler start failed")) {
    return 1;
  }

  std::this_thread::sleep_for(95ms);
  if (!check(scheduler.stop(), "scheduler stop failed")) {
    return 1;
  }
  const std::size_t realtime_count = invocations.load();
  if (!check(realtime_count >= 4U && realtime_count <= 6U,
             "scheduler did not run close to the configured real-time rate")) {
    return 1;
  }

  const auto step_begin = std::chrono::steady_clock::now();
  if (!check(scheduler.step(3), "manual scheduler stepping failed")) {
    return 1;
  }
  const auto step_elapsed = std::chrono::steady_clock::now() - step_begin;
  if (!check(step_elapsed < 20ms,
             "manual scheduler stepping was incorrectly throttled") ||
      !check(invocations.load() == realtime_count + 3U,
             "manual scheduler stepping ran an unexpected number of tasks")) {
    return 1;
  }

  return scheduler.shutdown() ? 0 : 1;
}
