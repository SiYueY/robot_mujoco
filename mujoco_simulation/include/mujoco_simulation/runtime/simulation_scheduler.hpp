#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

#include "mujoco_simulation/simulation_status.hpp"
#include "mujoco_simulation/visibility.hpp"

namespace mujoco_simulation {

class MUJOCO_SIMULATION_PUBLIC SimulationScheduler {
public:
  bool initialize();
  bool register_task(std::function<bool()> task);
  bool shutdown();

  bool start();
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

  // Task
  std::function<bool()> task_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_thread_;
  // Simulation state
  SimulationStatus status_{SimulationStatus::Uninitialized};
  bool stop_requested_{false};

  std::mutex execution_mutex_;
};

} // namespace mujoco_simulation
