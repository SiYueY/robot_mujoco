#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "mujoco_simulation/runtime/model_runtime.hpp"
#include "mujoco_simulation/runtime/scheduler_config.hpp"
#include "mujoco_simulation/runtime/simulation_clock.hpp"
#include "mujoco_simulation/runtime/simulation_request.hpp"
#include "mujoco_simulation/simulation_status.hpp"
#include "mujoco_simulation/status.hpp"

namespace mujoco_simulation {

class SimulationScheduler {
 public:
  Status initialize(const SchedulerConfig& config, SchedulerCallbacks callbacks);
  Status shutdown();

  Status start();
  Status stop();
  Status pause();
  Status resume();
  Status step(std::size_t count = 1);
  Status request_reset(const ResetRequest& request = {});
  std::future<Status> request_reset_waitable(ResetRequest request = {});
  Status set_realtime_factor(double realtime_factor);

  SimulationStatus status() const;
  SchedulerStatistics statistics() const;
  double realtime_factor() const;

 private:
  Status run_step_cycle(bool manual_step);
  Status process_pending_requests(bool* reset_deadline);
  Status process_reset_request(const ResetRequest& request, bool* reset_deadline);
  void resolve_reset_request(const ResetRequest& request, const Status& status);
  void fail_pending_reset_requests_locked(const Status& status);
  std::chrono::nanoseconds wall_period() const;
  void worker_loop();
  bool worker_should_wake_locked() const;
  void set_error_locked(const Status& status);

  SchedulerConfig config_{};
  SchedulerCallbacks callbacks_{};

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_thread_;
  std::deque<ResetRequest> reset_requests_;
  SimulationStatus status_{SimulationStatus::Uninitialized};
  SchedulerStatistics statistics_{};
  bool stop_requested_{false};
  bool deadline_reset_requested_{false};

  std::mutex execution_mutex_;
};

}  // namespace mujoco_simulation
