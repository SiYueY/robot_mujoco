#include "mujoco_simulation/viewer/viewer.hpp"

#include <chrono>
#include <cstring>
#include <exception>

#include "glfw_adapter.h"
#include "simulate.h"

namespace mujoco_simulation {
namespace {

void delete_simulate(mujoco::Simulate* simulate) { delete simulate; }

}  // namespace

Viewer::Viewer() : Viewer(ViewerConfig{}) {}

Viewer::Viewer(ViewerConfig config) : config_(config), simulate_(nullptr, delete_simulate) {}

Viewer::~Viewer() { stop(); }

void Viewer::mark_ready() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_ = true;
  }
  cv_.notify_all();
}

void Viewer::record_async_failure(Status status) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_ = false;
    async_failure_ = std::move(status);
    if (simulate_ != nullptr) {
      simulate_->exitrequest.store(true);
    }
  }
  cv_.notify_all();
}

Status Viewer::start(mjModel* model, mjData* data, const std::string& displayed_filename) {
  if (model == nullptr || data == nullptr) {
    return Status::invalid_argument("Viewer requires non-null MuJoCo model and data.");
  }
  if (render_thread_.joinable()) {
    return Status::Ok();
  }

  try {
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&visual_options_);
    mjv_defaultPerturb(&perturb_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ready_ = false;
      stop_requested_ = false;
      async_failure_.reset();
    }

    render_thread_ = std::thread([this, model, data, displayed_filename]() {
      try {
        if (render_thread_entry_) {
          render_thread_entry_(*this, model, data, displayed_filename);
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
      } catch (const std::exception& exc) {
        record_async_failure(
            Status::thread_failed(std::string("MuJoCo viewer thread failed: ") + exc.what()));
        return;
      } catch (...) {
        record_async_failure(
            Status::thread_failed("MuJoCo viewer thread failed with an unknown exception."));
        return;
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        ready_ = false;
      }
      cv_.notify_all();
    });
  } catch (const std::exception& exc) {
    if (render_thread_.joinable()) {
      render_thread_.join();
    }
    return Status::thread_failed(std::string("Failed to start MuJoCo viewer: ") + exc.what());
  }

  const auto ready_deadline = std::chrono::steady_clock::now() + config_.startup_timeout;
  std::unique_lock<std::mutex> lock(mutex_);
  const bool ready = cv_.wait_until(lock, ready_deadline,
                                    [this]() { return ready_ || async_failure_.has_value(); });
  if (async_failure_.has_value()) {
    const Status failure = *async_failure_;
    lock.unlock();
    stop();
    return failure;
  }
  if (!ready) {
    lock.unlock();
    stop();
    return Status::timeout(
        "MuJoCo viewer failed to become ready before the startup timeout expired.");
  }
  return Status::Ok();
}

void Viewer::stop() {
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
}

Status Viewer::sync(bool state_only) {
  std::lock_guard<std::mutex> state_lock(mutex_);
  if (async_failure_.has_value()) {
    return *async_failure_;
  }
  if (simulate_ == nullptr || !ready_) {
    return Status::invalid_state(stop_requested_ ? "Viewer is stopping." : "Viewer is not ready.");
  }

  try {
    std::unique_lock<std::recursive_mutex> simulate_lock(simulate_->mtx);
    if (simulate_->exitrequest.load()) {
      return Status::invalid_state("Viewer is stopping.");
    }
    simulate_->Sync(state_only);
  } catch (const std::exception& exc) {
    return Status::render_failed(std::string("Failed to sync MuJoCo viewer: ") + exc.what());
  }

  return Status::Ok();
}

bool Viewer::is_running() const { return render_thread_.joinable(); }

bool Viewer::is_ready() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ready_;
}

}  // namespace mujoco_simulation
