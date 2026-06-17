#include "mujoco_simulation/runtime/simulation_scheduler.hpp"

#include <exception>
#include <string_view>
#include <utility>

#include "mujoco_simulation/result.hpp"

namespace mujoco_simulation {
namespace {

Status invoke_optional(const std::function<Status()>& callback) {
  if (!callback) {
    return Status::Ok();
  }
  try {
    return callback();
  } catch (const std::exception& error) {
    return Status::thread_failed(std::string("SimulationScheduler callback threw: ") +
                                 error.what());
  } catch (...) {
    return Status::thread_failed("SimulationScheduler callback threw an unknown exception.");
  }
}

template <typename Callback>
Status invoke_status_callback(Callback&& callback, std::string_view operation_name) {
  try {
    return callback();
  } catch (const std::exception& error) {
    return Status::thread_failed("SimulationScheduler " + std::string(operation_name) +
                                 " callback threw: " + error.what());
  } catch (...) {
    return Status::thread_failed("SimulationScheduler " + std::string(operation_name) +
                                 " callback threw an unknown exception.");
  }
}

Status invoke_required_status_callback(const std::function<Status()>& callback,
                                       std::string_view operation_name) {
  if (!callback) {
    return Status::internal("SimulationScheduler missing required callback for " +
                            std::string(operation_name) + ".");
  }
  return invoke_status_callback(callback, operation_name);
}

Status invoke_reset_callback(const std::function<Status(const ResetOptions&)>& callback,
                             const ResetOptions& options) {
  if (!callback) {
    return Status::internal("SimulationScheduler missing required reset callback.");
  }
  try {
    return callback(options);
  } catch (const std::exception& error) {
    return Status::thread_failed(std::string("SimulationScheduler reset callback threw: ") +
                                 error.what());
  } catch (...) {
    return Status::thread_failed("SimulationScheduler reset callback threw an unknown exception.");
  }
}

Result<double> invoke_timestep_provider(const std::function<double()>& callback) {
  if (!callback) {
    return Status::internal("SimulationScheduler missing required timestep provider callback.");
  }
  try {
    return callback();
  } catch (const std::exception& error) {
    return Status::thread_failed(std::string("SimulationScheduler timestep provider threw: ") +
                                 error.what());
  } catch (...) {
    return Status::thread_failed(
        "SimulationScheduler timestep provider threw an unknown exception.");
  }
}

double seconds_from_duration(const SimulationDuration duration) {
  return std::chrono::duration<double>(duration).count();
}

}  // namespace

Status SimulationScheduler::initialize(const SchedulerConfig& config,
                                       SchedulerCallbacks callbacks) {
  if (!callbacks.timestep_provider) {
    return Status::invalid_argument("Scheduler timestep_provider callback is required.");
  }
  if (!callbacks.step_physics) {
    return Status::invalid_argument("Scheduler step_physics callback is required.");
  }
  if (!callbacks.reset_runtime) {
    return Status::invalid_argument("Scheduler reset_runtime callback is required.");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ != SimulationStatus::Uninitialized) {
    return Status::failed_precondition("SimulationScheduler is already initialized.");
  }

  config_ = config;
  callbacks_ = std::move(callbacks);
  reset_requests_.clear();
  statistics_ = {};
  stop_requested_ = false;
  deadline_reset_requested_ = false;
  status_ = SimulationStatus::Stopped;
  return Status::Ok();
}

Status SimulationScheduler::shutdown() {
  Status stop_status = Status::Ok();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return Status::Ok();
    }
  }

  stop_status = stop();
  if (!stop_status.ok()) {
    return stop_status;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  fail_pending_reset_requests_locked(
      Status::failed_precondition("SimulationScheduler is shutting down."));
  callbacks_ = {};
  reset_requests_.clear();
  statistics_ = {};
  stop_requested_ = false;
  deadline_reset_requested_ = false;
  status_ = SimulationStatus::Uninitialized;
  return Status::Ok();
}

Status SimulationScheduler::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    return Status::failed_precondition("SimulationScheduler is not initialized.");
  }
  if (status_ != SimulationStatus::Stopped) {
    return Status::failed_precondition("SimulationScheduler can only start from Stopped.");
  }

  stop_requested_ = false;
  deadline_reset_requested_ = true;
  status_ = SimulationStatus::Running;

  try {
    worker_thread_ = std::thread([this]() { worker_loop(); });
  } catch (const std::exception& error) {
    status_ = SimulationStatus::Stopped;
    return Status::thread_failed(std::string("Failed to start scheduler thread: ") + error.what());
  }

  return Status::Ok();
}

Status SimulationScheduler::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return Status::failed_precondition("SimulationScheduler is not initialized.");
    }
    if (status_ == SimulationStatus::Stopped) {
      return Status::Ok();
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
  fail_pending_reset_requests_locked(
      Status::failed_precondition("SimulationScheduler stopped before reset request completed."));
  reset_requests_.clear();
  stop_requested_ = false;
  deadline_reset_requested_ = false;
  if (status_ != SimulationStatus::Uninitialized) {
    status_ = SimulationStatus::Stopped;
  }
  return Status::Ok();
}

Status SimulationScheduler::pause() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    return Status::failed_precondition("SimulationScheduler is not initialized.");
  }
  if (status_ != SimulationStatus::Running) {
    return Status::failed_precondition("SimulationScheduler can only pause from Running.");
  }
  status_ = SimulationStatus::Paused;
  return Status::Ok();
}

Status SimulationScheduler::resume() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return Status::failed_precondition("SimulationScheduler is not initialized.");
    }
    if (status_ != SimulationStatus::Paused) {
      return Status::failed_precondition("SimulationScheduler can only resume from Paused.");
    }
    status_ = SimulationStatus::Running;
    deadline_reset_requested_ = true;
  }
  cv_.notify_all();
  return Status::Ok();
}

Status SimulationScheduler::step(std::size_t count) {
  if (count == 0) {
    return Status::invalid_argument("Scheduler step count must be greater than zero.");
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return Status::failed_precondition("SimulationScheduler is not initialized.");
    }
    if (status_ == SimulationStatus::Running || status_ == SimulationStatus::Stopping) {
      return Status::failed_precondition(
          "SimulationScheduler manual step is only allowed in Stopped or Paused.");
    }
    if (status_ == SimulationStatus::Error) {
      return Status::failed_precondition(
          "SimulationScheduler manual step is not allowed in Error.");
    }
  }

  for (std::size_t i = 0; i < count; ++i) {
    bool ignored_reset_deadline = false;
    Status request_status = process_pending_requests(&ignored_reset_deadline);
    if (!request_status.ok()) {
      return request_status;
    }

    Status cycle_status = run_step_cycle(true);
    if (!cycle_status.ok()) {
      std::lock_guard<std::mutex> lock(mutex_);
      set_error_locked(cycle_status);
      return cycle_status;
    }
  }

  return Status::Ok();
}

Status SimulationScheduler::request_reset(const ResetRequest& request) {
  bool process_immediately = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return Status::failed_precondition("SimulationScheduler is not initialized.");
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
  return Status::Ok();
}

std::future<Status> SimulationScheduler::request_reset_waitable(ResetRequest request) {
  auto completion = std::make_shared<std::promise<Status>>();
  std::future<Status> future = completion->get_future();
  request.completion = completion;

  const Status status = request_reset(request);
  if (!status.ok()) {
    resolve_reset_request(request, status);
  }
  return future;
}

Status SimulationScheduler::set_realtime_factor(double realtime_factor) {
  if (realtime_factor <= 0.0) {
    return Status::invalid_argument(
        "SimulationScheduler realtime factor must be greater than zero.");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    return Status::failed_precondition("SimulationScheduler is not initialized.");
  }
  config_.realtime_factor = realtime_factor;
  return Status::Ok();
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

Status SimulationScheduler::run_step_cycle(bool manual_step) {
  const auto loop_start = SimulationClock::now();
  const Result<double> timestep_result = invoke_timestep_provider(callbacks_.timestep_provider);
  if (!timestep_result.ok()) {
    return timestep_result.status();
  }
  const double sim_timestep = timestep_result.value();

  std::lock_guard<std::mutex> execution_lock(execution_mutex_);

  Status status = invoke_optional(callbacks_.write_commands);
  if (!status.ok()) {
    return status;
  }

  const auto step_start = SimulationClock::now();
  status = invoke_required_status_callback(callbacks_.step_physics, "step_physics");
  if (!status.ok()) {
    return status;
  }
  const auto step_end = SimulationClock::now();

  status = invoke_optional(callbacks_.read_components);
  if (!status.ok()) {
    return status;
  }
  status = invoke_optional(callbacks_.publish_state_snapshot);
  if (!status.ok()) {
    return status;
  }
  status = invoke_optional(callbacks_.sync_viewer_if_due);
  if (!status.ok()) {
    return status;
  }

  const auto loop_end = SimulationClock::now();
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
  return Status::Ok();
}

Status SimulationScheduler::process_pending_requests(bool* reset_deadline) {
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

    Status status = process_reset_request(request, reset_deadline);
    resolve_reset_request(request, status);
    if (!status.ok()) {
      std::lock_guard<std::mutex> lock(mutex_);
      set_error_locked(status);
      return status;
    }
  }

  return Status::Ok();
}

Status SimulationScheduler::process_reset_request(const ResetRequest& request,
                                                  bool* reset_deadline) {
  std::lock_guard<std::mutex> execution_lock(execution_mutex_);
  Status status = invoke_reset_callback(callbacks_.reset_runtime, request.options);
  if (!status.ok()) {
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
  return Status::Ok();
}

void SimulationScheduler::resolve_reset_request(const ResetRequest& request, const Status& status) {
  if (request.completion == nullptr) {
    return;
  }
  request.completion->set_value(status);
}

void SimulationScheduler::fail_pending_reset_requests_locked(const Status& status) {
  for (const ResetRequest& request : reset_requests_) {
    resolve_reset_request(request, status);
  }
}

std::chrono::nanoseconds SimulationScheduler::wall_period() const {
  const Result<double> timestep_result = invoke_timestep_provider(callbacks_.timestep_provider);
  const double timestep = timestep_result.ok() ? timestep_result.value() : 0.0;
  const double realtime_factor = config_.realtime_factor > 0.0 ? config_.realtime_factor : 1.0;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(timestep / realtime_factor));
}

void SimulationScheduler::worker_loop() {
  try {
    auto next_tick = SimulationClock::now();

    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return worker_should_wake_locked(); });
        if (stop_requested_) {
          break;
        }
      }

      bool reset_deadline = false;
      Status request_status = process_pending_requests(&reset_deadline);
      if (!request_status.ok()) {
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
        next_tick = SimulationClock::now();
      }
      if (current_status != SimulationStatus::Running) {
        continue;
      }

      Status cycle_status = run_step_cycle(false);
      if (!cycle_status.ok()) {
        std::lock_guard<std::mutex> lock(mutex_);
        set_error_locked(cycle_status);
        break;
      }

      const auto period = wall_period();
      next_tick += period;

      if (config_.realtime_sync && period.count() > 0) {
        std::this_thread::sleep_until(next_tick);
      }

      const auto now = SimulationClock::now();
      if (now > next_tick + config_.max_schedule_lag) {
        std::lock_guard<std::mutex> lock(mutex_);
        next_tick = now;
        ++statistics_.lag_recoveries;
      }
    }
  } catch (const std::exception& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    set_error_locked(Status::thread_failed(
        std::string("SimulationScheduler worker thread failed: ") + error.what()));
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    set_error_locked(Status::thread_failed(
        "SimulationScheduler worker thread failed with an unknown exception."));
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

void SimulationScheduler::set_error_locked(const Status& status) {
  status_ = SimulationStatus::Error;
}

}  // namespace mujoco_simulation
