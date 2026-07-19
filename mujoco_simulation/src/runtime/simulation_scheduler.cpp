#include "mujoco_simulation/runtime/simulation_scheduler.hpp"

#include <exception>
#include <string>
#include <utility>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"

namespace mujoco_simulation {

bool SimulationScheduler::invoke_cycle(const std::function<bool()>& callback) {
  if (!callback) {
    LOG_ERROR << "required scheduler cycle host is not configured.";
    return false;
  }
  try {
    return callback();
  } catch (const std::exception&) {
    LOG_ERROR << "scheduler cycle callback threw an exception.";
  } catch (...) {
    LOG_ERROR << "scheduler cycle callback threw an unknown exception.";
  }
  return false;
}

bool SimulationScheduler::invoke_reset(const std::function<bool(const ResetRequest&)>& callback,
                                       const ResetRequest& request) {
  if (!callback) {
    LOG_ERROR << "required scheduler reset callback is not configured.";
    return false;
  }
  try {
    return callback(request);
  } catch (const std::exception&) {
    LOG_ERROR << "scheduler reset callback threw an exception.";
  } catch (...) {
    LOG_ERROR << "scheduler reset callback threw an unknown exception.";
  }
  return false;
}

bool SimulationScheduler::invoke_timestep_provider(const std::function<double()>& callback,
                                                   double* out) {
  if (out == nullptr) {
    LOG_ERROR << "scheduler timestep output is null.";
    return false;
  }
  if (!callback) {
    LOG_ERROR << "scheduler timestep provider is not configured.";
    return false;
  }
  try {
    *out = callback();
    return true;
  } catch (const std::exception&) {
    LOG_ERROR << "scheduler timestep provider threw an exception.";
  } catch (...) {
    LOG_ERROR << "scheduler timestep provider threw an unknown exception.";
  }
  return false;
}

bool SimulationScheduler::initialize(const SchedulerConfig& config) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ != SimulationStatus::Uninitialized) {
    LOG_ERROR << "scheduler is already initialized.";
    return false;
  }

  config_ = config;
  timestep_provider_ = {};
  cycle_runner_ = {};
  reset_handler_ = {};
  reset_requests_.clear();
  stop_requested_ = false;
  deadline_reset_requested_ = false;
  status_ = SimulationStatus::Stopped;
  return true;
}

bool SimulationScheduler::register_timestep_provider(std::function<double()> provider) {
  if (!provider) {
    LOG_ERROR << "scheduler timestep provider must not be empty.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    LOG_ERROR << "scheduler must be initialized before registering operations.";
    return false;
  }
  if (worker_thread_.joinable() || status_ == SimulationStatus::Running ||
      status_ == SimulationStatus::Paused || status_ == SimulationStatus::Stopping ||
      status_ == SimulationStatus::Error) {
    LOG_ERROR << "scheduler cannot register operations after execution has started.";
    return false;
  }
  timestep_provider_ = std::move(provider);
  return true;
}

bool SimulationScheduler::register_cycle_runner(std::function<bool()> runner) {
  if (!runner) {
    LOG_ERROR << "scheduler cycle runner must not be empty.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    LOG_ERROR << "scheduler must be initialized before registering operations.";
    return false;
  }
  if (worker_thread_.joinable() || status_ == SimulationStatus::Running ||
      status_ == SimulationStatus::Paused || status_ == SimulationStatus::Stopping ||
      status_ == SimulationStatus::Error) {
    LOG_ERROR << "scheduler cannot register operations after execution has started.";
    return false;
  }
  cycle_runner_ = std::move(runner);
  return true;
}

bool SimulationScheduler::register_reset_handler(std::function<bool(const ResetRequest&)> handler) {
  if (!handler) {
    LOG_ERROR << "scheduler reset handler must not be empty.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    LOG_ERROR << "scheduler must be initialized before registering operations.";
    return false;
  }
  if (worker_thread_.joinable() || status_ == SimulationStatus::Running ||
      status_ == SimulationStatus::Paused || status_ == SimulationStatus::Stopping ||
      status_ == SimulationStatus::Error) {
    LOG_ERROR << "scheduler cannot register operations after execution has started.";
    return false;
  }
  reset_handler_ = std::move(handler);
  return true;
}

bool SimulationScheduler::shutdown() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      return true;
    }
  }

  if (!stop()) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  fail_pending_reset_requests_locked(false);
  timestep_provider_ = {};
  cycle_runner_ = {};
  reset_handler_ = {};
  reset_requests_.clear();
  stop_requested_ = false;
  deadline_reset_requested_ = false;
  status_ = SimulationStatus::Uninitialized;
  return true;
}

bool SimulationScheduler::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    LOG_ERROR << "scheduler is not initialized.";
    return false;
  }
  if (status_ != SimulationStatus::Stopped) {
    LOG_ERROR << "scheduler is not stopped.";
    return false;
  }

  stop_requested_ = false;
  deadline_reset_requested_ = true;
  status_ = SimulationStatus::Running;

  try {
    worker_thread_ = std::thread([this]() { worker_loop(); });
  } catch (const std::exception&) {
    status_ = SimulationStatus::Stopped;
    LOG_ERROR << "failed to start scheduler thread.";
    return false;
  }

  return true;
}

bool SimulationScheduler::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      LOG_ERROR << "scheduler is not initialized.";
      return false;
    }
    if (status_ == SimulationStatus::Stopped) {
      return true;
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
  fail_pending_reset_requests_locked(false);
  reset_requests_.clear();
  stop_requested_ = false;
  deadline_reset_requested_ = false;
  if (status_ != SimulationStatus::Uninitialized) {
    status_ = SimulationStatus::Stopped;
  }
  return true;
}

bool SimulationScheduler::pause() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    LOG_ERROR << "scheduler is not initialized.";
    return false;
  }
  if (status_ != SimulationStatus::Running) {
    LOG_ERROR << "scheduler is not running.";
    return false;
  }
  status_ = SimulationStatus::Paused;
  return true;
}

bool SimulationScheduler::resume() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      LOG_ERROR << "scheduler is not initialized.";
      return false;
    }
    if (status_ != SimulationStatus::Paused) {
      LOG_ERROR << "scheduler is not paused.";
      return false;
    }
    status_ = SimulationStatus::Running;
    deadline_reset_requested_ = true;
  }
  cv_.notify_all();
  return true;
}

bool SimulationScheduler::step(std::size_t count) {
  if (count == 0) {
    LOG_ERROR << "scheduler step count must be positive.";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      LOG_ERROR << "scheduler is not initialized.";
      return false;
    }
    if (status_ == SimulationStatus::Running || status_ == SimulationStatus::Stopping) {
      LOG_ERROR << "scheduler cannot step while running.";
      return false;
    }
    if (status_ == SimulationStatus::Error) {
      LOG_ERROR << "scheduler is in an error state.";
      return false;
    }
  }

  for (std::size_t i = 0; i < count; ++i) {
    bool ignored_reset_deadline = false;
    if (!process_pending_requests(&ignored_reset_deadline)) {
      return false;
    }

    if (!run_step_cycle()) {
      std::lock_guard<std::mutex> lock(mutex_);
      set_error_locked();
      return false;
    }
  }

  return true;
}

bool SimulationScheduler::request_reset(const ResetRequest& request) {
  bool process_immediately = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (status_ == SimulationStatus::Uninitialized) {
      LOG_ERROR << "scheduler is not initialized.";
      return false;
    }

    reset_requests_.push_back(request);
    process_immediately = status_ == SimulationStatus::Stopped && !worker_thread_.joinable();
  }

  if (process_immediately) {
    bool reset_deadline = false;
    return process_pending_requests(&reset_deadline);
  }

  cv_.notify_all();
  return true;
}

std::future<bool> SimulationScheduler::request_reset_waitable(ResetRequest request) {
  auto completion = std::make_shared<std::promise<bool>>();
  std::future<bool> future = completion->get_future();
  request.completion = completion;

  if (!request_reset(request)) {
    resolve_reset_request(request, false);
  }
  return future;
}

bool SimulationScheduler::set_realtime_factor(double realtime_factor) {
  if (realtime_factor <= 0.0) {
    LOG_ERROR << "scheduler realtime factor must be positive.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    LOG_ERROR << "scheduler is not initialized.";
    return false;
  }
  config_.realtime_factor = realtime_factor;
  return true;
}

SimulationStatus SimulationScheduler::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

double SimulationScheduler::realtime_factor() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_.realtime_factor;
}

bool SimulationScheduler::run_step_cycle() {
  double sim_timestep = 0.0;
  if (!invoke_timestep_provider(timestep_provider_, &sim_timestep)) {
    return false;
  }
  UNUSED(sim_timestep);

  std::lock_guard<std::mutex> execution_lock(execution_mutex_);
  return invoke_cycle(cycle_runner_);
}

bool SimulationScheduler::process_pending_requests(bool* reset_deadline) {
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

    const bool success = process_reset_request(request, reset_deadline);
    resolve_reset_request(request, success);
    if (!success) {
      std::lock_guard<std::mutex> lock(mutex_);
      set_error_locked();
      return false;
    }
  }

  return true;
}

bool SimulationScheduler::process_reset_request(const ResetRequest& request, bool* reset_deadline) {
  std::lock_guard<std::mutex> execution_lock(execution_mutex_);
  if (!invoke_reset(reset_handler_, request)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  deadline_reset_requested_ = true;
  if (reset_deadline != nullptr) {
    *reset_deadline = true;
  }
  return true;
}

void SimulationScheduler::resolve_reset_request(const ResetRequest& request, bool success) {
  if (request.completion == nullptr) {
    return;
  }
  request.completion->set_value(success);
}

void SimulationScheduler::fail_pending_reset_requests_locked(bool success) {
  for (const ResetRequest& request : reset_requests_) {
    resolve_reset_request(request, success);
  }
}

std::chrono::nanoseconds SimulationScheduler::wall_period() const {
  double timestep = 0.0;
  UNUSED(invoke_timestep_provider(timestep_provider_, &timestep));
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
      if (!process_pending_requests(&reset_deadline)) {
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

      if (!run_step_cycle()) {
        std::lock_guard<std::mutex> lock(mutex_);
        set_error_locked();
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
      }
    }
  } catch (const std::exception&) {
    LOG_ERROR << "scheduler worker thread threw an exception.";
    std::lock_guard<std::mutex> lock(mutex_);
    set_error_locked();
  } catch (...) {
    LOG_ERROR << "scheduler worker thread threw an unknown exception.";
    std::lock_guard<std::mutex> lock(mutex_);
    set_error_locked();
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

void SimulationScheduler::set_error_locked() { status_ = SimulationStatus::Error; }

}  // namespace mujoco_simulation
