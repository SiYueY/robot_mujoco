#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "mujoco_simulation/config/simulation_config.hpp"
#include "mujoco_simulation/simulation_status.hpp"

namespace mujoco_simulation {

enum class ResetTargetType {
  Default,
  KeyframeName,
  KeyframeId,
};

struct ResetTarget {
  ResetTargetType type{ResetTargetType::Default};
  std::string keyframe_name;
  int keyframe_id{-1};
};

struct ResetRequest {
  ResetTarget target;
  std::shared_ptr<std::promise<bool>> completion;
};

class SimulationScheduler {
 public:
  bool initialize(const SchedulerConfig& config);
  bool register_timestep_provider(std::function<double()> provider);
  bool register_cycle_runner(std::function<bool()> runner);
  bool register_reset_handler(std::function<bool(const ResetRequest&)> handler);
  bool shutdown();

  bool start();
  bool stop();
  bool pause();
  bool resume();
  bool step(std::size_t count = 1);
  bool request_reset(const ResetRequest& request = {});
  std::future<bool> request_reset_waitable(ResetRequest request = {});
  bool set_realtime_factor(double realtime_factor);

  SimulationStatus status() const;
  double realtime_factor() const;

 private:
  static bool invoke_cycle(const std::function<bool()>& callback);
  static bool invoke_reset(const std::function<bool(const ResetRequest&)>& callback,
                           const ResetRequest& request);
  static bool invoke_timestep_provider(const std::function<double()>& callback, double* out);
  bool run_step_cycle();
  bool process_pending_requests(bool* reset_deadline);
  bool process_reset_request(const ResetRequest& request, bool* reset_deadline);
  void resolve_reset_request(const ResetRequest& request, bool success);
  void fail_pending_reset_requests_locked(bool success);
  std::chrono::nanoseconds wall_period() const;
  void worker_loop();
  bool worker_should_wake_locked() const;
  void set_error_locked();

  SchedulerConfig config_{};
  std::function<double()> timestep_provider_;
  std::function<bool()> cycle_runner_;
  std::function<bool(const ResetRequest&)> reset_handler_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::thread worker_thread_;
  std::deque<ResetRequest> reset_requests_;
  SimulationStatus status_{SimulationStatus::Uninitialized};
  bool stop_requested_{false};
  bool deadline_reset_requested_{false};

  std::mutex execution_mutex_;
};

}  // namespace mujoco_simulation
