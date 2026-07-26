#include "mujoco_simulation/viewer/simulation_viewer.hpp"

#include <chrono>
#include <cstring>
#include <exception>
#include <utility>

#include "glfw_adapter.h"
#include "mujoco_simulation/common/logging.hpp"
#include "simulate.h"

namespace mujoco_simulation {

SimulationViewer::SimulationViewer() : SimulationViewer(std::chrono::milliseconds{5000}) {}

SimulationViewer::SimulationViewer(std::chrono::milliseconds startup_timeout)
    : startup_timeout_(startup_timeout) {}

SimulationViewer::~SimulationViewer() { stop(); }

void SimulationViewer::set_ready() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == ViewerState::Starting) {
      state_ = ViewerState::Ready;
    }
  }
  cv_.notify_all();
}

void SimulationViewer::set_failed() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (simulate_ != nullptr) {
      simulate_->exitrequest.store(true);
    }
    if (state_ != ViewerState::Stopping && state_ != ViewerState::Stopped) {
      state_ = ViewerState::Failed;
    }
  }
  cv_.notify_all();
}

void SimulationViewer::finish_render_thread() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == ViewerState::Stopping) {
      state_ = ViewerState::Stopped;
    } else if (state_ != ViewerState::Stopped && state_ != ViewerState::Failed) {
      state_ = ViewerState::Failed;
    }
  }
  cv_.notify_all();
}

void SimulationViewer::set_stop() {
  if (state_ != ViewerState::Stopped) {
    state_ = ViewerState::Stopping;
  }
  if (simulate_ != nullptr) {
    simulate_->exitrequest.store(true);
  }
  cv_.notify_all();
}

void SimulationViewer::join_render_thread() {
  std::thread render_thread;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!render_thread_.joinable()) {
      return;
    }
    render_thread = std::move(render_thread_);
  }
  render_thread.join();
}

void SimulationViewer::cleanup() {
  std::lock_guard<std::mutex> lock(mutex_);
  simulate_.reset();
  state_ = ViewerState::Stopped;
  cv_.notify_all();
}

void SimulationViewer::stop_viewer() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    set_stop();
  }
  join_render_thread();
  cleanup();
}

void SimulationViewer::render_task(const mjContext& context, std::string displayed_filename) {
  try {
    if (!context.valid()) {
      LOG_ERROR << "viewer runtime became invalid.";
      set_failed();
      return;
    }

    auto simulate = std::make_unique<mujoco::Simulate>(std::make_unique<mujoco::GlfwAdapter>(),
                                                       &camera_, &visual_options_, &perturb_, true);
    simulate->exitrequest.store(false);

    {
      const ::mujoco::MutexLock simulate_lock(simulate->mtx);
      simulate->mnew_ = const_cast<mjModel*>(context.model);
      simulate->dnew_ = context.data;
      std::strncpy(simulate->filename, displayed_filename.c_str(), sizeof(simulate->filename) - 1);
      simulate->filename[sizeof(simulate->filename) - 1] = '\0';
      simulate->loadrequest = 1;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == ViewerState::Stopping) {
        simulate->exitrequest.store(true);
      }
      simulate_ = std::move(simulate);
    }
    set_ready();

    simulate_->RenderLoop();
    finish_render_thread();
  } catch (const std::exception&) {
    LOG_ERROR << "viewer render thread failed.";
    set_failed();
  } catch (...) {
    LOG_ERROR << "viewer render thread failed.";
    set_failed();
  }
}

bool SimulationViewer::start(const mjContext& context, const std::string& displayed_filename) {
  if (!context.valid()) {
    LOG_ERROR << "failed to start simulation viewer, context is invalid.";
    return false;
  }

  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == ViewerState::Ready) {
      return true;
    }
  }

  stop_viewer();

  try {
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&visual_options_);
    mjv_defaultPerturb(&perturb_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = ViewerState::Starting;
    }

    std::thread render_thread(
        [this, &context, displayed_filename]() { render_task(context, displayed_filename); });
    {
      std::lock_guard<std::mutex> lock(mutex_);
      render_thread_ = std::move(render_thread);
    }
  } catch (const std::exception&) {
    LOG_ERROR << "failed to start viewer render thread.";
    cleanup();
    return false;
  }

  const auto ready_deadline = std::chrono::steady_clock::now() + startup_timeout_;
  std::unique_lock<std::mutex> lock(mutex_);
  const bool completed = cv_.wait_until(lock, ready_deadline, [this]() {
    return state_ == ViewerState::Ready || state_ == ViewerState::Failed ||
           state_ == ViewerState::Stopped;
  });
  const bool ready = completed && state_ == ViewerState::Ready;
  lock.unlock();
  if (!ready) {
    if (!completed) {
      LOG_ERROR << "viewer startup timed out.";
    }
    stop_viewer();
  }
  return ready;
}

void SimulationViewer::stop() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  stop_viewer();
}

bool SimulationViewer::sync(bool state_only) {
  std::lock_guard<std::mutex> state_lock(mutex_);
  if (state_ != ViewerState::Ready || simulate_ == nullptr) {
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

bool SimulationViewer::is_running() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_ == ViewerState::Starting || state_ == ViewerState::Ready ||
         state_ == ViewerState::Stopping;
}

bool SimulationViewer::is_ready() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_ == ViewerState::Ready;
}

}  // namespace mujoco_simulation
