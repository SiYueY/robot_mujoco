#include "mujoco_simulation/runtime/simulation_scheduler.hpp"

#include <cmath>
#include <exception>
#include <utility>

#include "common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"

namespace mujoco_simulation {

bool SimulationScheduler::invoke_task(const std::function<bool()> &task) {
  if (!task) {
    LOG_ERROR << "required scheduler task host is not configured.";
    return false;
  }
  try {
    return task();
  } catch (const std::exception &) {
    LOG_ERROR << "scheduler task task threw an exception.";
  } catch (...) {
    LOG_ERROR << "scheduler task task threw an unknown exception.";
  }
  return false;
}

bool SimulationScheduler::initialize(
    std::chrono::duration<double> physics_period) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ != SimulationStatus::Uninitialized) {
    LOG_ERROR << "scheduler is already initialized.";
    return false;
  }

  if (!std::isfinite(physics_period.count()) || physics_period.count() <= 0.0) {
    LOG_ERROR << "scheduler physics period must be finite and positive.";
    return false;
  }
  physics_period_ =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          physics_period);
  if (physics_period_ <= std::chrono::steady_clock::duration::zero()) {
    LOG_ERROR << "scheduler physics period is below clock resolution.";
    return false;
  }
  task_ = {};
  stop_requested_ = false;
  reset_timing_locked();
  status_ = SimulationStatus::Stopped;
  return true;
}

bool SimulationScheduler::register_task(std::function<bool()> task) {
  if (!task) {
    LOG_ERROR << "scheduler task must not be empty.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ == SimulationStatus::Uninitialized) {
    LOG_ERROR << "scheduler must be initialized before registering operations.";
    return false;
  }
  if (worker_thread_.joinable() || status_ == SimulationStatus::Running ||
      status_ == SimulationStatus::Paused ||
      status_ == SimulationStatus::Stopping ||
      status_ == SimulationStatus::Error) {
    LOG_ERROR
        << "scheduler cannot register operations after execution has started.";
    return false;
  }
  task_ = std::move(task);
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
  task_ = {};
  stop_requested_ = false;
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
  reset_timing_locked();
  status_ = SimulationStatus::Running;

  try {
    worker_thread_ = std::thread([this]() { worker_loop(); });
  } catch (const std::exception &) {
    status_ = SimulationStatus::Stopped;
    LOG_ERROR << "failed to start scheduler thread.";
    return false;
  }

  return true;
}

bool SimulationScheduler::start_paused() {
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
  reset_timing_locked();
  status_ = SimulationStatus::Paused;
  try {
    worker_thread_ = std::thread([this]() { worker_loop(); });
  } catch (const std::exception &) {
    status_ = SimulationStatus::Stopped;
    LOG_ERROR << "failed to start paused scheduler thread.";
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
  stop_requested_ = false;
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
  timing_reset_requested_ = true;
  cv_.notify_all();
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
    reset_timing_locked();
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
    if (status_ == SimulationStatus::Running ||
        status_ == SimulationStatus::Stopping) {
      LOG_ERROR << "scheduler cannot step while running.";
      return false;
    }
    if (status_ == SimulationStatus::Error) {
      LOG_ERROR << "scheduler is in an error state.";
      return false;
    }
  }

  for (std::size_t i = 0; i < count; ++i) {
    if (!execute_task_once()) {
      std::lock_guard<std::mutex> lock(mutex_);
      set_error_locked();
      return false;
    }
  }

  return true;
}

SimulationStatus SimulationScheduler::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

bool SimulationScheduler::execute_task_once() {
  {
    std::lock_guard<std::mutex> execution_lock(execution_mutex_);
    if (!invoke_task(task_)) {
      return false;
    }
  }
  return true;
}

void SimulationScheduler::worker_loop() {
  using Clock = std::chrono::steady_clock;
  Clock::time_point next_deadline = Clock::now();
  try {
    while (true) {
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() {
          return stop_requested_ || status_ == SimulationStatus::Running;
        });
        if (stop_requested_) {
          break;
        }
        if (timing_reset_requested_) {
          next_deadline = Clock::now();
          timing_reset_requested_ = false;
        }
      }

      next_deadline += physics_period_;
      if (!execute_task_once()) {
        std::lock_guard<std::mutex> lock(mutex_);
        set_error_locked();
        break;
      }

      const Clock::time_point step_end = Clock::now();
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ++completed_steps_;
        if (step_end >= next_deadline + physics_period_) {
          log_deadline_miss(step_end, next_deadline);
          next_deadline = step_end;
        }
        cv_.wait_until(lock, next_deadline, [this] {
          return stop_requested_ || status_ != SimulationStatus::Running ||
                 timing_reset_requested_;
        });
        if (stop_requested_) {
          break;
        }
      }
    }
  } catch (const std::exception &) {
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
  if (status_ != SimulationStatus::Error &&
      status_ != SimulationStatus::Uninitialized) {
    status_ = SimulationStatus::Stopped;
  }
}

void SimulationScheduler::set_error_locked() {
  status_ = SimulationStatus::Error;
}

void SimulationScheduler::reset_timing_locked() {
  timing_reset_requested_ = true;
  timing_anchor_ = std::chrono::steady_clock::now();
  last_deadline_log_ = {};
  completed_steps_ = 0;
  deadline_misses_ = 0;
}

void SimulationScheduler::log_deadline_miss(
    std::chrono::steady_clock::time_point now,
    std::chrono::steady_clock::time_point deadline) {
  ++deadline_misses_;
  if (last_deadline_log_ != std::chrono::steady_clock::time_point{} &&
      now - last_deadline_log_ < std::chrono::seconds(1)) {
    return;
  }
  const double wall_seconds =
      std::chrono::duration<double>(now - timing_anchor_).count();
  const double simulation_seconds =
      std::chrono::duration<double>(physics_period_).count() * completed_steps_;
  const double realtime_factor =
      wall_seconds > 0.0 ? simulation_seconds / wall_seconds : 0.0;
  const double lateness = std::chrono::duration<double>(now - deadline).count();
  LOG_WARNING << "physics deadline miss: lateness=" << lateness
              << " s, realtime_factor=" << realtime_factor
              << ", misses=" << deadline_misses_;
  last_deadline_log_ = now;
}

} // namespace mujoco_simulation
