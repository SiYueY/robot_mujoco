#include "mujoco_simulation/viewer/simulation_viewer.hpp"

#include <chrono>
#include <cstring>
#include <exception>
#include <utility>

#include "common/logging.hpp"
#include "glfw_adapter.h"
#include "simulate.h"

namespace mujoco_simulation {

SimulationViewer::SimulationViewer()
    : SimulationViewer(std::chrono::milliseconds{5000}) {}

SimulationViewer::SimulationViewer(std::chrono::milliseconds startup_timeout)
    : startup_timeout_(startup_timeout) {}

SimulationViewer::~SimulationViewer() { stop(); }

bool SimulationViewer::create_viewer_data(const mjContext &context) {
  if (!context.valid()) {
    LOG_ERROR << "cannot create viewer data from an invalid context.";
    return false;
  }
  viewer_model_ = mj_copyModel(nullptr, context.model);
  if (viewer_model_ == nullptr) {
    LOG_ERROR << "failed to copy the MuJoCo model for the viewer.";
    return false;
  }
  viewer_data_ = mj_makeData(viewer_model_);
  pending_data_ = mj_makeData(viewer_model_);
  render_data_ = mj_makeData(viewer_model_);
  if (viewer_data_ == nullptr || pending_data_ == nullptr ||
      render_data_ == nullptr ||
      mj_copyData(viewer_data_, viewer_model_, context.data) == nullptr ||
      mj_copyData(pending_data_, viewer_model_, context.data) == nullptr ||
      mj_copyData(render_data_, viewer_model_, context.data) == nullptr) {
    LOG_ERROR << "failed to create viewer data buffers.";
    release_viewer_data();
    return false;
  }
  return true;
}

void SimulationViewer::release_viewer_data() {
  if (pending_data_ != nullptr) {
    mj_deleteData(pending_data_);
    pending_data_ = nullptr;
  }
  if (render_data_ != nullptr) {
    mj_deleteData(render_data_);
    render_data_ = nullptr;
  }
  if (viewer_data_ != nullptr) {
    mj_deleteData(viewer_data_);
    viewer_data_ = nullptr;
  }
  if (viewer_model_ != nullptr) {
    mj_deleteModel(viewer_model_);
    viewer_model_ = nullptr;
  }
}

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
    } else if (state_ != ViewerState::Stopped &&
               state_ != ViewerState::Failed) {
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

bool SimulationViewer::start_sync_worker() {
  try {
    {
      std::lock_guard<std::mutex> lock(sync_mutex_);
      sync_stopping_ = false;
      sync_pending_ = false;
    }
    sync_thread_ = std::thread([this] { sync_worker_loop(); });
  } catch (const std::exception &) {
    LOG_ERROR << "failed to start viewer synchronization worker.";
    return false;
  }
  return true;
}

void SimulationViewer::stop_sync_worker() {
  {
    std::lock_guard<std::mutex> lock(sync_mutex_);
    sync_stopping_ = true;
    sync_pending_ = false;
  }
  sync_cv_.notify_all();
  if (sync_thread_.joinable()) {
    sync_thread_.join();
  }
}

void SimulationViewer::sync_worker_loop() {
  while (true) {
    mjData *data = nullptr;
    {
      std::unique_lock<std::mutex> lock(sync_mutex_);
      sync_cv_.wait(lock, [this] { return sync_stopping_ || sync_pending_; });
      if (sync_stopping_) {
        return;
      }
      std::swap(pending_data_, render_data_);
      data = render_data_;
      sync_pending_ = false;
    }
    if (data != nullptr && !sync_render_data(*data)) {
      LOG_WARNING << "viewer synchronization failed; disabling viewer.";
      stop_viewer();
      return;
    }
  }
}

void SimulationViewer::render_task(const mjModel *model, mjData *data,
                                   std::string displayed_filename) {
  try {
    if (model == nullptr || data == nullptr) {
      LOG_ERROR << "viewer runtime became invalid.";
      set_failed();
      return;
    }

    auto simulate = std::make_unique<mujoco::Simulate>(
        std::make_unique<mujoco::GlfwAdapter>(), &camera_, &visual_options_,
        &perturb_, true);
    simulate->exitrequest.store(false);

    {
      const ::mujoco::MutexLock simulate_lock(simulate->mtx);
      simulate->mnew_ = const_cast<mjModel *>(model);
      simulate->dnew_ = data;
      std::strncpy(simulate->filename, displayed_filename.c_str(),
                   sizeof(simulate->filename) - 1);
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
  } catch (const std::exception &) {
    LOG_ERROR << "viewer render thread failed.";
    set_failed();
  } catch (...) {
    LOG_ERROR << "viewer render thread failed.";
    set_failed();
  }
}

bool SimulationViewer::start(const mjContext &context,
                             const std::string &displayed_filename) {
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

  stop_sync_worker();
  stop_viewer();
  release_viewer_data();

  if (!create_viewer_data(context)) {
    return false;
  }

  try {
    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&visual_options_);
    mjv_defaultPerturb(&perturb_);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = ViewerState::Starting;
    }

    std::thread render_thread([this, displayed_filename]() {
      render_task(viewer_model_, viewer_data_, displayed_filename);
    });
    {
      std::lock_guard<std::mutex> lock(mutex_);
      render_thread_ = std::move(render_thread);
    }
  } catch (const std::exception &) {
    LOG_ERROR << "failed to start viewer render thread.";
    cleanup();
    release_viewer_data();
    return false;
  }

  const auto ready_deadline =
      std::chrono::steady_clock::now() + startup_timeout_;
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
    release_viewer_data();
    return false;
  }
  if (!start_sync_worker()) {
    stop_viewer();
    release_viewer_data();
    return false;
  }
  return true;
}

void SimulationViewer::stop() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  stop_sync_worker();
  stop_viewer();
  release_viewer_data();
}

bool SimulationViewer::submit(const mjContext &context) {
  if (!context.valid()) {
    LOG_WARNING << "viewer synchronization request has an invalid context.";
    return false;
  }
  std::lock_guard<std::mutex> lock(sync_mutex_);
  if (sync_stopping_ || pending_data_ == nullptr || !is_ready()) {
    return false;
  }
  if (mj_copyData(pending_data_, viewer_model_, context.data) == nullptr) {
    LOG_WARNING << "failed to copy simulation data for viewer synchronization.";
    return false;
  }
  sync_pending_ = true;
  sync_cv_.notify_one();
  return true;
}

bool SimulationViewer::sync_render_data(const mjData &data) {
  std::lock_guard<std::mutex> state_lock(mutex_);
  if (state_ != ViewerState::Ready || simulate_ == nullptr ||
      viewer_data_ == nullptr || viewer_model_ == nullptr) {
    return false;
  }

  try {
    std::unique_lock<std::recursive_mutex> simulate_lock(simulate_->mtx);
    if (simulate_->exitrequest.load()) {
      return false;
    }
    if (mj_copyData(viewer_data_, viewer_model_, &data) == nullptr) {
      return false;
    }
    simulate_->Sync(false);
  } catch (const std::exception &) {
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

} // namespace mujoco_simulation
