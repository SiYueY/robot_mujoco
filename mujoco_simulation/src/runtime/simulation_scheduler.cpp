#include "mujoco_simulation/runtime/simulation_scheduler.hpp"

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

bool SimulationScheduler::initialize() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (status_ != SimulationStatus::Uninitialized) {
    LOG_ERROR << "scheduler is already initialized.";
    return false;
  }

  task_ = {};
  stop_requested_ = false;
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
      }

      SimulationStatus current_status;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        current_status = status_;
      }
      if (current_status != SimulationStatus::Running) {
        continue;
      }

      if (!execute_task_once()) {
        std::lock_guard<std::mutex> lock(mutex_);
        set_error_locked();
        break;
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

} // namespace mujoco_simulation
