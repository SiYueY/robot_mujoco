#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#include "mujoco_simulation/reset_options.hpp"
#include "mujoco_simulation/result_code.hpp"
#include "mujoco_simulation/simulation_config.hpp"
#include "mujoco_simulation/simulation_status.hpp"

namespace mujoco_simulation {

struct ResetRequest {
  ResetOptions options;
  std::shared_ptr<std::promise<ResultCode>> completion;
};

struct SchedulerCallbacks {
  std::function<double()> timestep_provider;
  std::function<ResultCode()> write_commands;
  std::function<ResultCode()> step_physics;
  std::function<ResultCode()> read_components;
  std::function<ResultCode()> publish_state_snapshot;
  std::function<ResultCode()> sync_viewer_if_due;
  std::function<ResultCode(const ResetOptions&)> reset_runtime;
};

struct SchedulerStatistics {
  std::uint64_t physics_steps{0};
  std::uint64_t loop_iterations{0};
  std::uint64_t manual_step_calls{0};
  std::uint64_t reset_requests{0};
  std::uint64_t lag_recoveries{0};
  double last_loop_duration_sec{0.0};
  double last_step_duration_sec{0.0};
  double last_realtime_factor{0.0};
};

class SimulationScheduler {
 public:
  ResultCode initialize(const SchedulerConfig& config, SchedulerCallbacks callbacks);
  ResultCode shutdown();

  ResultCode start();
  ResultCode stop();
  ResultCode pause();
  ResultCode resume();
  ResultCode step(std::size_t count = 1);
  ResultCode request_reset(const ResetRequest& request = {});
  std::future<ResultCode> request_reset_waitable(ResetRequest request = {});
  ResultCode set_realtime_factor(double realtime_factor);

  SimulationStatus status() const;
  SchedulerStatistics statistics() const;
  double realtime_factor() const;

 private:
  ResultCode run_step_cycle(bool manual_step);
  ResultCode process_pending_requests(bool* reset_deadline);
  ResultCode process_reset_request(const ResetRequest& request, bool* reset_deadline);
  void resolve_reset_request(const ResetRequest& request, const ResultCode& status);
  void fail_pending_reset_requests_locked(const ResultCode& status);
  std::chrono::nanoseconds wall_period() const;
  void worker_loop();
  bool worker_should_wake_locked() const;
  void set_error_locked(const ResultCode& status);

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
