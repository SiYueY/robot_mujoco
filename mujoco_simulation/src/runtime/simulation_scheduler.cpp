#include "mujoco_simulation/runtime/simulation_scheduler.hpp"

#include <exception>
#include <string>
#include <utility>

#include "mujoco_simulation/result_code.hpp"

namespace mujoco_simulation {
namespace {

ResultCode invoke_optional(const std::function<ResultCode()>& callback) {
  if (!callback) {
    return ResultCode::Ok;
  }
  try {
    return callback();
  } catch (const std::exception&) {
    return ResultCode::ThreadFailed;
  } catch (...) {
    return ResultCode::ThreadFailed;
  }
}

template <typename Callback>
ResultCode invoke_status_callback(Callback&& callback, std::string operation_name) {
  (void)operation_name;
  try {
    return callback();
  } catch (const std::exception&) {
    return ResultCode::ThreadFailed;
  } catch (...) {
    return ResultCode::ThreadFailed;
  }
}

ResultCode invoke_required_status_callback(const std::function<ResultCode()>& callback,
                                           std::string operation_name) {
  if (!callback) {
    return ResultCode::Internal;
  }
  return invoke_status_callback(callback, operation_name);
}

ResultCode invoke_reset_callback(const std::function<ResultCode(const ResetRequest&)>& callback,
                                 const ResetRequest& request) {
  if (!callback) {
    return ResultCode::Internal;
  }
  try {
    return callback(request);
  } catch (const std::exception&) {
    return ResultCode::ThreadFailed;
  } catch (...) {
    return ResultCode::ThreadFailed;
  }
}

ResultCode invoke_timestep_provider(const std::function<double()>& callback, double* out) {
  if (out == nullptr) {
    return ResultCode::InvalidArgument;
  }
  if (!callback) {
    return ResultCode::Internal;
  }
  try {
    *out = callback();
    return ResultCode::Ok;
  } catch (const std::exception&) {
    return ResultCode::ThreadFailed;
  } catch (...) {
    return ResultCode::ThreadFailed;
  }
}

double seconds_from_duration(const std::chrono::steady_clock::duration duration) {
  return std::chrono::duration<double>(duration).count();
}

}  // namespace

ResultCode SimulationScheduler::initialize(const SchedulerConfig& config,
                                           SchedulerCallbacks callbacks) {
  if (!callbacks.timestep_provider) {
    return ResultCode::InvalidArgument;
  }
  if (!callbacks.step_physics) {
    return ResultCode::InvalidArgument;
  }
  if (!callbacks.reset_runtime) {
    return ResultCode::InvalidArgument;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ != SimulationStatus::Uninitialized) {
    return ResultCode::FailedPrecondition;
  }

  config_ = config;
  callbacks_ = std::move(callbacks);
  reset_requests_.clear();
  statistics_ = {};
  stop_requested_ = false;
  deadline_reset_requested_ = false;
  status_ = SimulationStatus::Stopped;
  return ResultCode::Ok;
}

ResultCode SimulationScheduler::shutdown() {
  ResultCode stop_status = ResultCode::Ok;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return ResultCode::Ok;
    }
  }

  stop_status = stop();
  if (stop_status != ResultCode::Ok) {
    return stop_status;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  fail_pending_reset_requests_locked(ResultCode::FailedPrecondition);
  callbacks_ = {};
  reset_requests_.clear();
  statistics_ = {};
  stop_requested_ = false;
  deadline_reset_requested_ = false;
  status_ = SimulationStatus::Uninitialized;
  return ResultCode::Ok;
}

ResultCode SimulationScheduler::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    return ResultCode::FailedPrecondition;
  }
  if (status_ != SimulationStatus::Stopped) {
    return ResultCode::FailedPrecondition;
  }

  stop_requested_ = false;
  deadline_reset_requested_ = true;
  status_ = SimulationStatus::Running;

  try {
    worker_thread_ = std::thread([this]() { worker_loop(); });
  } catch (const std::exception&) {
    status_ = SimulationStatus::Stopped;
    return ResultCode::ThreadFailed;
  }

  return ResultCode::Ok;
}

ResultCode SimulationScheduler::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return ResultCode::FailedPrecondition;
    }
    if (status_ == SimulationStatus::Stopped) {
      return ResultCode::Ok;
    }
    stop_requested_ = true;
    if (status_ != SimulationStatus::Error) {
      status_ = SimulationStatus::Stopping;
    }
  }
  cv_.notify_all();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  std::lock_guard<std::mutex> lock(mutex_);
  fail_pending_reset_requests_locked(ResultCode::FailedPrecondition);
  reset_requests_.clear();
  stop_requested_ = false;
  deadline_reset_requested_ = false;
  if (status_ != SimulationStatus::Uninitialized) {
    status_ = SimulationStatus::Stopped;
  }
  return ResultCode::Ok;
}

ResultCode SimulationScheduler::pause() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    return ResultCode::FailedPrecondition;
  }
  if (status_ != SimulationStatus::Running) {
    return ResultCode::FailedPrecondition;
  }
  status_ = SimulationStatus::Paused;
  return ResultCode::Ok;
}

ResultCode SimulationScheduler::resume() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return ResultCode::FailedPrecondition;
    }
    if (status_ != SimulationStatus::Paused) {
      return ResultCode::FailedPrecondition;
    }
    status_ = SimulationStatus::Running;
    deadline_reset_requested_ = true;
  }
  cv_.notify_all();
  return ResultCode::Ok;
}

ResultCode SimulationScheduler::step(std::size_t count) {
  if (count == 0) {
    return ResultCode::InvalidArgument;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return ResultCode::FailedPrecondition;
    }
    if (status_ == SimulationStatus::Running || status_ == SimulationStatus::Stopping) {
      return ResultCode::FailedPrecondition;
    }
    if (status_ == SimulationStatus::Error) {
      return ResultCode::FailedPrecondition;
    }
  }

  for (std::size_t i = 0; i < count; ++i) {
    bool ignored_reset_deadline = false;
    ResultCode request_status = process_pending_requests(&ignored_reset_deadline);
    if (request_status != ResultCode::Ok) {
      return request_status;
    }

    ResultCode cycle_status = run_step_cycle(true);
    if (cycle_status != ResultCode::Ok) {
      std::lock_guard<std::mutex> lock(mutex_);
      set_error_locked(cycle_status);
      return cycle_status;
    }
  }

  return ResultCode::Ok;
}

ResultCode SimulationScheduler::request_reset(const ResetRequest& request) {
  bool process_immediately = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return ResultCode::FailedPrecondition;
    }

    reset_requests_.push_back(request);
    ++statistics_.reset_requests;
    process_immediately = status_ == SimulationStatus::Stopped && !worker_thread_.joinable();
  }

  if (process_immediately) {
    bool reset_deadline = false;
    return process_pending_requests(&reset_deadline);
  }

  cv_.notify_all();
  return ResultCode::Ok;
}

std::future<ResultCode> SimulationScheduler::request_reset_waitable(ResetRequest request) {
  auto completion = std::make_shared<std::promise<ResultCode>>();
  std::future<ResultCode> future = completion->get_future();
  request.completion = completion;

  const ResultCode status = request_reset(request);
  if (status != ResultCode::Ok) {
    resolve_reset_request(request, status);
  }
  return future;
}

ResultCode SimulationScheduler::set_realtime_factor(double realtime_factor) {
  if (realtime_factor <= 0.0) {
    return ResultCode::InvalidArgument;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    return ResultCode::FailedPrecondition;
  }
  config_.realtime_factor = realtime_factor;
  return ResultCode::Ok;
}

SimulationStatus SimulationScheduler::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

SchedulerStatistics SimulationScheduler::statistics() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return statistics_;
}

double SimulationScheduler::realtime_factor() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.realtime_factor;
}

ResultCode SimulationScheduler::run_step_cycle(bool manual_step) {
  const auto loop_start = std::chrono::steady_clock::now();
  double sim_timestep = 0.0;
  const ResultCode timestep_status =
      invoke_timestep_provider(callbacks_.timestep_provider, &sim_timestep);
  if (timestep_status != ResultCode::Ok) {
    return timestep_status;
  }

  std::lock_guard<std::mutex> execution_lock(execution_mutex_);

  ResultCode status = invoke_optional(callbacks_.write_commands);
  if (status != ResultCode::Ok) {
    return status;
  }

  const auto step_start = std::chrono::steady_clock::now();
  status = invoke_required_status_callback(callbacks_.step_physics, "step_physics");
  if (status != ResultCode::Ok) {
    return status;
  }
  const auto step_end = std::chrono::steady_clock::now();

  status = invoke_optional(callbacks_.update_components);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = invoke_optional(callbacks_.write_state_snapshot);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = invoke_optional(callbacks_.sync_viewer_if_due);
  if (status != ResultCode::Ok) {
    return status;
  }

  const auto loop_end = std::chrono::steady_clock::now();
  const double loop_duration_sec = seconds_from_duration(loop_end - loop_start);
  const double step_duration_sec = seconds_from_duration(step_end - step_start);

  std::lock_guard<std::mutex> lock(mutex_);
  ++statistics_.physics_steps;
  ++statistics_.loop_iterations;
  if (manual_step) {
    ++statistics_.manual_step_calls;
  }
  statistics_.last_loop_duration_sec = loop_duration_sec;
  statistics_.last_step_duration_sec = step_duration_sec;
  statistics_.last_realtime_factor =
      loop_duration_sec > 0.0 ? sim_timestep / loop_duration_sec : 0.0;
  return ResultCode::Ok;
}

ResultCode SimulationScheduler::process_pending_requests(bool* reset_deadline) {
  while (true) {
    ResetRequest request;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (reset_requests_.empty()) {
        break;
      }
      request = reset_requests_.front();
      reset_requests_.pop_front();
    }

    ResultCode status = process_reset_request(request, reset_deadline);
    resolve_reset_request(request, status);
    if (status != ResultCode::Ok) {
      std::lock_guard<std::mutex> lock(mutex_);
      set_error_locked(status);
      return status;
    }
  }

  return ResultCode::Ok;
}

ResultCode SimulationScheduler::process_reset_request(const ResetRequest& request,
                                                      bool* reset_deadline) {
  std::lock_guard<std::mutex> execution_lock(execution_mutex_);
  ResultCode status = invoke_reset_callback(callbacks_.reset_runtime, request);
  if (status != ResultCode::Ok) {
    return status;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (request.options.reset_statistics) {
    statistics_.physics_steps = 0;
    statistics_.loop_iterations = 0;
    statistics_.manual_step_calls = 0;
    statistics_.lag_recoveries = 0;
    statistics_.last_loop_duration_sec = 0.0;
    statistics_.last_step_duration_sec = 0.0;
    statistics_.last_realtime_factor = 0.0;
  }
  deadline_reset_requested_ = true;
  if (reset_deadline != nullptr) {
    *reset_deadline = true;
  }
  return ResultCode::Ok;
}

void SimulationScheduler::resolve_reset_request(const ResetRequest& request,
                                                const ResultCode& status) {
  if (request.completion == nullptr) {
    return;
  }
  request.completion->set_value(status);
}

void SimulationScheduler::fail_pending_reset_requests_locked(const ResultCode& status) {
  for (const ResetRequest& request : reset_requests_) {
    resolve_reset_request(request, status);
  }
}

std::chrono::nanoseconds SimulationScheduler::wall_period() const {
  double timestep = 0.0;
  (void)invoke_timestep_provider(callbacks_.timestep_provider, &timestep);
  const double realtime_factor = config_.realtime_factor > 0.0 ? config_.realtime_factor : 1.0;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timestep / realtime_factor));
}

void SimulationScheduler::worker_loop() {
  try {
    auto next_tick = std::chrono::steady_clock::now();

    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return worker_should_wake_locked(); });
        if (stop_requested_) {
          break;
        }
      }

      bool reset_deadline = false;
      ResultCode request_status = process_pending_requests(&reset_deadline);
      if (request_status != ResultCode::Ok) {
        break;
      }

      SimulationStatus current_status;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (deadline_reset_requested_) {
          reset_deadline = true;
          deadline_reset_requested_ = false;
        }
        current_status = status_;
      }

      if (reset_deadline) {
        next_tick = std::chrono::steady_clock::now();
      }
      if (current_status != SimulationStatus::Running) {
        continue;
      }

      ResultCode cycle_status = run_step_cycle(false);
      if (cycle_status != ResultCode::Ok) {
        std::lock_guard<std::mutex> lock(mutex_);
        set_error_locked(cycle_status);
        break;
      }

      const auto period = wall_period();
      next_tick += period;

      if (config_.realtime_sync && period.count() > 0) {
        std::this_thread::sleep_until(next_tick);
      }

      const auto now = std::chrono::steady_clock::now();
      if (now > next_tick + config_.max_schedule_lag) {
        std::lock_guard<std::mutex> lock(mutex_);
        next_tick = now;
        ++statistics_.lag_recoveries;
      }
    }
  } catch (const std::exception&) {
    std::lock_guard<std::mutex> lock(mutex_);
    set_error_locked(ResultCode::ThreadFailed);
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    set_error_locked(ResultCode::ThreadFailed);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  stop_requested_ = false;
  deadline_reset_requested_ = false;
  if (status_ != SimulationStatus::Error && status_ != SimulationStatus::Uninitialized) {
    status_ = SimulationStatus::Stopped;
  }
}

bool SimulationScheduler::worker_should_wake_locked() const {
  return stop_requested_ || status_ == SimulationStatus::Running || !reset_requests_.empty();
}

void SimulationScheduler::set_error_locked(const ResultCode& status) {
  status_ = SimulationStatus::Error;
}

}  // namespace mujoco_simulation
