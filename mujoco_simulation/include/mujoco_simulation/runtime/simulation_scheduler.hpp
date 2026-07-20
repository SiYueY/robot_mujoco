#pragma once

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
  bool initialize();
  bool register_cycle(std::function<bool()> cycle);
  bool shutdown();

  bool start();
  bool stop();
  bool pause();
  bool resume();
  bool step(std::size_t count = 1);

  SimulationStatus status() const;

 private:
  static bool invoke_cycle(const std::function<bool()>& callback);
  bool execute_cycle_once();
  void worker_loop();
  bool worker_should_wake_locked() const;
  void set_error_locked();

  std::function<bool()> cycle_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_thread_;
  // Simulation state
  SimulationStatus status_{SimulationStatus::Uninitialized};
  bool stop_requested_{false};

  std::mutex execution_mutex_;
};

}  // namespace mujoco_simulation
