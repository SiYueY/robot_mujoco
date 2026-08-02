#pragma once
// Internal simulation scheduler contract.

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

#include "mujoco_simulation/simulation_status.hpp"

namespace mujoco_simulation {

class SimulationScheduler {
public:
  bool initialize(std::chrono::duration<double> physics_period);
  bool register_task(std::function<bool()> task);
  bool shutdown();

  bool start();
  bool start_paused();
  bool stop();
  bool pause();
  bool resume();
  bool step(std::size_t count = 1);

  SimulationStatus status() const;

private:
  static bool invoke_task(const std::function<bool()> &callback);
  bool execute_task_once();
  void worker_loop();
  void set_error_locked();
  void reset_timing_locked();
  void log_deadline_miss(std::chrono::steady_clock::time_point now,
                         std::chrono::steady_clock::time_point deadline);

  // Task
  std::function<bool()> task_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_thread_;
  // Simulation state
  SimulationStatus status_{SimulationStatus::Uninitialized};
  bool stop_requested_{false};
  bool timing_reset_requested_{false};
  bool worker_task_executing_{false};
  std::chrono::steady_clock::duration physics_period_{};
  std::chrono::steady_clock::time_point timing_anchor_{};
  std::chrono::steady_clock::time_point last_deadline_log_{};
  std::uint64_t completed_steps_{0};
  std::uint64_t deadline_misses_{0};

  std::mutex execution_mutex_;
};

} // namespace mujoco_simulation
#include <chrono>
