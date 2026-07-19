#include "mujoco_simulation/viewer/simulation_viewer.hpp"

#include <chrono>
#include <cstring>
#include <exception>

#include "glfw_adapter.h"
#include "mujoco_simulation/common/logging.hpp"
#include "simulate.h"

namespace mujoco_simulation {

void SimulationViewer::delete_simulate(mujoco::Simulate* simulate) { delete simulate; }

SimulationViewer::SimulationViewer() : SimulationViewer(std::chrono::milliseconds{5000}) {}

SimulationViewer::SimulationViewer(std::chrono::milliseconds startup_timeout)
    : startup_timeout_(startup_timeout), simulate_(nullptr, &SimulationViewer::delete_simulate) {}

SimulationViewer::~SimulationViewer() { stop(); }

void SimulationViewer::mark_ready() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_ = true;
  }
  cv_.notify_all();
}

void SimulationViewer::record_async_failure() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_ = false;
    async_failed_ = true;
    if (simulate_ != nullptr) {
      simulate_->exitrequest.store(true);
    }
  }
  cv_.notify_all();
}

bool SimulationViewer::start(const mjContext& context, const std::string& displayed_filename) {
  if (!context.valid()) {
    LOG_ERROR << "viewer requires a valid MuJoCo context.";
    return false;
  }
  const mjModel* model = context.model;
  mjData* data = context.data;
  if (render_thread_.joinable()) {
    return true;
  }

  try {
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&visual_options_);
    mjv_defaultPerturb(&perturb_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ready_ = false;
      stop_requested_ = false;
      async_failed_ = false;
      model_ = model;
      data_ = data;
    }

    render_thread_ = std::thread([this, displayed_filename]() {
      try {
        if (render_thread_entry_) {
          render_thread_entry_(*this, const_cast<mjModel*>(model_), data_, displayed_filename);
          return;
        }

        mjModel* model = const_cast<mjModel*>(model_);
        mjData* data = data_;
        if (model == nullptr || data == nullptr) {
          LOG_ERROR << "viewer runtime became invalid.";
          record_async_failure();
          return;
        }

        auto simulate =
            SimulateHandle(new mujoco::Simulate(std::make_unique<mujoco::GlfwAdapter>(), &camera_,
                                                &visual_options_, &perturb_, true),
                           delete_simulate);
        simulate->exitrequest.store(false);

        {
          const ::mujoco::MutexLock simulate_lock(simulate->mtx);
          simulate->mnew_ = model;
          simulate->dnew_ = data;
          std::strncpy(simulate->filename, displayed_filename.c_str(),
                       sizeof(simulate->filename) - 1);
          simulate->filename[sizeof(simulate->filename) - 1] = '\0';
          simulate->loadrequest = 1;
        }

        {
          std::lock_guard<std::mutex> lock(mutex_);
          simulate_ = std::move(simulate);
        }
        mark_ready();

        simulate_->RenderLoop();
      } catch (const std::exception&) {
        LOG_ERROR << "viewer render thread failed.";
        record_async_failure();
        return;
      } catch (...) {
        LOG_ERROR << "viewer render thread failed.";
        record_async_failure();
        return;
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        ready_ = false;
      }
      cv_.notify_all();
    });
  } catch (const std::exception&) {
    if (render_thread_.joinable()) {
      render_thread_.join();
    }
    LOG_ERROR << "failed to start viewer render thread.";
    return false;
  }

  const auto ready_deadline = std::chrono::steady_clock::now() + startup_timeout_;
  std::unique_lock<std::mutex> lock(mutex_);
  const bool ready =
      cv_.wait_until(lock, ready_deadline, [this]() { return ready_ || async_failed_; });
  if (async_failed_) {
    lock.unlock();
    stop();
    return false;
  }
  if (!ready) {
    lock.unlock();
    stop();
    LOG_ERROR << "viewer startup timed out.";
    return false;
  }
  return true;
}

void SimulationViewer::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_requested_ = true;
    ready_ = false;
    if (simulate_ != nullptr) {
      simulate_->exitrequest.store(true);
    }
  }
  cv_.notify_all();
  if (render_thread_.joinable()) {
    render_thread_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  simulate_.reset();
  model_ = nullptr;
  data_ = nullptr;
}

bool SimulationViewer::sync(bool state_only) {
  std::lock_guard<std::mutex> state_lock(mutex_);
  if (async_failed_) {
    LOG_ERROR << "viewer render thread failed.";
    return false;
  }
  if (simulate_ == nullptr || !ready_) {
    LOG_ERROR << "viewer is not ready.";
    return false;
  }

  try {
    std::unique_lock<std::recursive_mutex> simulate_lock(simulate_->mtx);
    if (simulate_->exitrequest.load()) {
      LOG_ERROR << "viewer render loop has stopped.";
      return false;
    }
    simulate_->Sync(state_only);
  } catch (const std::exception&) {
    LOG_ERROR << "viewer synchronization failed.";
    return false;
  }

  return true;
}

bool SimulationViewer::is_running() const { return render_thread_.joinable(); }

bool SimulationViewer::is_ready() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ready_;
}

}  // namespace mujoco_simulation
