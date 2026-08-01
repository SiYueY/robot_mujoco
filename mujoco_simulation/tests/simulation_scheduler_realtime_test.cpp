#include <atomic>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

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

  if (!check(scheduler.shutdown(), "scheduler shutdown failed")) {
    return 1;
  }

  mujoco_simulation::SimulationScheduler pause_scheduler;
  std::atomic<bool> task_started{false};
  std::atomic<bool> release_task{false};
  std::atomic<std::size_t> paused_invocations{0};
  if (!check(pause_scheduler.initialize(std::chrono::duration<double>(1ms)),
             "pause scheduler initialization failed") ||
      !check(pause_scheduler.register_task([&] {
        task_started.store(true);
        ++paused_invocations;
        while (!release_task.load()) {
          std::this_thread::yield();
        }
        return true;
      }),
             "pause scheduler task registration failed") ||
      !check(pause_scheduler.start(), "pause scheduler start failed")) {
    return 1;
  }
  const auto start_deadline = std::chrono::steady_clock::now() + 1s;
  while (!task_started.load() &&
         std::chrono::steady_clock::now() < start_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  if (!check(task_started.load(), "pause scheduler task did not start")) {
    return 1;
  }

  bool pause_result = false;
  std::atomic<bool> pause_returned{false};
  std::thread pause_thread([&] {
    pause_result = pause_scheduler.pause();
    pause_returned.store(true);
  });
  std::this_thread::sleep_for(10ms);
  if (!check(!pause_returned.load(),
             "pause returned before the running task completed")) {
    release_task.store(true);
    pause_thread.join();
    return 1;
  }
  release_task.store(true);
  pause_thread.join();
  const std::size_t paused_count = paused_invocations.load();
  std::this_thread::sleep_for(10ms);
  if (!check(pause_result, "synchronous pause failed") ||
      !check(pause_scheduler.status() ==
                 mujoco_simulation::SimulationStatus::Paused,
             "pause scheduler did not enter paused state") ||
      !check(paused_invocations.load() == paused_count,
             "scheduler ran another task after synchronous pause")) {
    return 1;
  }
  if (!check(pause_scheduler.stop(), "pause scheduler stop failed") ||
      !check(pause_scheduler.shutdown(), "pause scheduler shutdown failed")) {
    return 1;
  }

  mujoco_simulation::SimulationScheduler self_stop_scheduler;
  std::atomic<bool> self_stop_rejected{false};
  if (!check(self_stop_scheduler.initialize(std::chrono::duration<double>(1ms)),
             "self-stop scheduler initialization failed") ||
      !check(self_stop_scheduler.register_task([&] {
        self_stop_rejected.store(!self_stop_scheduler.stop());
        return true;
      }),
             "self-stop scheduler task registration failed") ||
      !check(self_stop_scheduler.start(), "self-stop scheduler start failed")) {
    return 1;
  }
  const auto stop_deadline = std::chrono::steady_clock::now() + 1s;
  while (self_stop_scheduler.status() !=
             mujoco_simulation::SimulationStatus::Stopped &&
         std::chrono::steady_clock::now() < stop_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  if (!check(self_stop_rejected.load(), "scheduler worker joined itself") ||
      !check(self_stop_scheduler.status() ==
                 mujoco_simulation::SimulationStatus::Stopped,
             "self-stop scheduler did not stop") ||
      !check(self_stop_scheduler.stop(), "self-stop scheduler join failed") ||
      !check(self_stop_scheduler.shutdown(),
             "self-stop scheduler shutdown failed")) {
    return 1;
  }

  mujoco_simulation::SimulationScheduler failing_scheduler;
  if (!check(failing_scheduler.initialize(std::chrono::duration<double>(1ms)),
             "failing scheduler initialization failed") ||
      !check(failing_scheduler.register_task([] { return false; }),
             "failing scheduler task registration failed") ||
      !check(failing_scheduler.start(), "failing scheduler start failed")) {
    return 1;
  }
  const auto error_deadline = std::chrono::steady_clock::now() + 1s;
  while (failing_scheduler.status() !=
             mujoco_simulation::SimulationStatus::Error &&
         std::chrono::steady_clock::now() < error_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  if (!check(failing_scheduler.status() ==
                 mujoco_simulation::SimulationStatus::Error,
             "false-returning task did not set scheduler error") ||
      !check(failing_scheduler.stop(), "failing scheduler stop failed") ||
      !check(failing_scheduler.shutdown(),
             "failing scheduler shutdown failed")) {
    return 1;
  }

  mujoco_simulation::SimulationScheduler throwing_scheduler;
  if (!check(throwing_scheduler.initialize(std::chrono::duration<double>(1ms)),
             "throwing scheduler initialization failed") ||
      !check(throwing_scheduler.register_task([]() -> bool {
        throw std::runtime_error("test task failure");
      }),
             "throwing scheduler task registration failed") ||
      !check(throwing_scheduler.start(), "throwing scheduler start failed")) {
    return 1;
  }
  while (throwing_scheduler.status() !=
             mujoco_simulation::SimulationStatus::Error &&
         std::chrono::steady_clock::now() < error_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  if (!check(throwing_scheduler.status() ==
                 mujoco_simulation::SimulationStatus::Error,
             "throwing task did not set scheduler error") ||
      !check(throwing_scheduler.stop(), "throwing scheduler stop failed") ||
      !check(throwing_scheduler.shutdown(),
             "throwing scheduler shutdown failed")) {
    return 1;
  }

  mujoco_simulation::SimulationScheduler stop_scheduler;
  std::atomic<bool> stop_task_started{false};
  std::atomic<bool> release_stop_task{false};
  if (!check(stop_scheduler.initialize(std::chrono::duration<double>(1ms)),
             "stop scheduler initialization failed") ||
      !check(stop_scheduler.register_task([&] {
        stop_task_started.store(true);
        while (!release_stop_task.load())
          std::this_thread::yield();
        return true;
      }),
             "stop scheduler task registration failed") ||
      !check(stop_scheduler.start(), "stop scheduler start failed")) {
    return 1;
  }
  while (!stop_task_started.load() &&
         std::chrono::steady_clock::now() < error_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  bool stop_result = false;
  std::atomic<bool> stop_returned{false};
  std::thread stop_thread([&] {
    stop_result = stop_scheduler.stop();
    stop_returned.store(true);
  });
  std::this_thread::sleep_for(10ms);
  if (!check(!stop_returned.load(),
             "stop returned before the executing task completed")) {
    release_stop_task.store(true);
    stop_thread.join();
    return 1;
  }
  release_stop_task.store(true);
  stop_thread.join();
  if (!check(stop_result, "stop scheduler stop failed") ||
      !check(stop_scheduler.status() ==
                 mujoco_simulation::SimulationStatus::Stopped,
             "stop scheduler did not stop after task release") ||
      !check(stop_scheduler.start(), "stop scheduler restart failed") ||
      !check(stop_scheduler.stop(), "stop scheduler second stop failed") ||
      !check(stop_scheduler.shutdown(), "stop scheduler shutdown failed")) {
    return 1;
  }

  return 0;
}
