#include "viewer/simulation_viewer.hpp"

#include <chrono>
#include <cstring>
#include <exception>
#include <memory>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>

#include "common/logging.hpp"
#include "glfw_adapter.h"
#include "simulate.h"

namespace mujoco_simulation {

struct SimulationViewer::SnapshotPool
    : std::enable_shared_from_this<SimulationViewer::SnapshotPool> {
  explicit SnapshotPool(const mjModel *model) : model(model) {}

  ~SnapshotPool() { shutdown(); }

  std::unique_ptr<ViewerSnapshot::Lease> acquire();
  void release(mjData *data);
  void shutdown();

  const mjModel *model{nullptr};
  std::mutex mutex;
  bool accepting{true};
  std::vector<mjData *> available;
};

struct SimulationViewer::ViewerSnapshot::Lease {
  Lease(std::shared_ptr<SnapshotPool> pool, mjData *data)
      : pool(std::move(pool)), data(data) {}
  ~Lease() {
    if (pool != nullptr)
      pool->release(data);
  }

  std::shared_ptr<SnapshotPool> pool;
  mjData *data{nullptr};
};

SimulationViewer::ViewerSnapshot::ViewerSnapshot() = default;
SimulationViewer::ViewerSnapshot::ViewerSnapshot(std::unique_ptr<Lease> lease)
    : lease_(std::move(lease)) {}
SimulationViewer::ViewerSnapshot::~ViewerSnapshot() = default;
SimulationViewer::ViewerSnapshot::ViewerSnapshot(ViewerSnapshot &&) noexcept =
    default;
SimulationViewer::ViewerSnapshot &SimulationViewer::ViewerSnapshot::operator=(
    ViewerSnapshot &&) noexcept = default;
SimulationViewer::ViewerSnapshot::operator bool() const noexcept {
  return lease_ != nullptr;
}

std::unique_ptr<SimulationViewer::ViewerSnapshot::Lease>
SimulationViewer::SnapshotPool::acquire() {
  std::lock_guard<std::mutex> lock(mutex);
  if (!accepting || available.empty())
    return nullptr;
  mjData *data = available.back();
  available.pop_back();
  return std::make_unique<ViewerSnapshot::Lease>(shared_from_this(), data);
}

void SimulationViewer::SnapshotPool::release(mjData *data) {
  if (data == nullptr)
    return;
  std::lock_guard<std::mutex> lock(mutex);
  if (accepting) {
    available.push_back(data);
  } else {
    mj_deleteData(data);
  }
}

void SimulationViewer::SnapshotPool::shutdown() {
  std::lock_guard<std::mutex> lock(mutex);
  accepting = false;
  for (mjData *data : available)
    mj_deleteData(data);
  available.clear();
}

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
  if (viewer_data_ == nullptr ||
      mj_copyData(viewer_data_, viewer_model_, context.data) == nullptr) {
    LOG_ERROR << "failed to create viewer data buffers.";
    release_viewer_data();
    return false;
  }
  snapshot_pool_ = std::make_shared<SnapshotPool>(viewer_model_);
  for (std::size_t index = 0; index < 3U; ++index) {
    mjData *snapshot = mj_makeData(viewer_model_);
    if (snapshot == nullptr) {
      LOG_ERROR << "failed to allocate viewer snapshot buffers.";
      release_viewer_data();
      return false;
    }
    snapshot_pool_->available.push_back(snapshot);
  }
  return true;
}

void SimulationViewer::release_viewer_data() {
  pending_snapshot_.reset();
  if (snapshot_pool_ != nullptr) {
    snapshot_pool_->shutdown();
    snapshot_pool_.reset();
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
    pending_snapshot_.reset();
  }
  sync_cv_.notify_all();
  if (sync_thread_.joinable()) {
    sync_thread_.join();
  }
}

void SimulationViewer::sync_worker_loop() {
  while (true) {
    std::unique_ptr<ViewerSnapshot::Lease> snapshot;
    {
      std::unique_lock<std::mutex> lock(sync_mutex_);
      sync_cv_.wait(lock, [this] { return sync_stopping_ || sync_pending_; });
      if (sync_stopping_) {
        return;
      }
      snapshot = std::move(pending_snapshot_);
      sync_pending_ = false;
    }
    if (snapshot != nullptr && !sync_render_data(*snapshot->data)) {
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

bool SimulationViewer::prepare(const mjContext &context) {
  if (!context.valid()) {
    LOG_ERROR << "cannot prepare simulation viewer from an invalid context.";
    return false;
  }

  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != ViewerState::Stopped) {
      LOG_ERROR << "simulation viewer must be stopped before preparation.";
      return false;
    }
  }
  if (viewer_model_ != nullptr || viewer_data_ != nullptr ||
      snapshot_pool_ != nullptr) {
    LOG_ERROR << "simulation viewer data is already prepared.";
    return false;
  }
  if (!create_viewer_data(context)) {
    return false;
  }
  return true;
}

bool SimulationViewer::start(const std::string &displayed_filename) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (viewer_model_ == nullptr || viewer_data_ == nullptr) {
    LOG_ERROR << "simulation viewer must be prepared before it is started.";
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == ViewerState::Ready)
      return true;
  }
  // GLFW is process-lifetime state.  Do not call glfwTerminate() here: the
  // unmodified vendored viewer registers the process-exit termination hook.
  if (glfwInit() == GLFW_FALSE) {
    LOG_ERROR << "failed to initialize GLFW for the simulation viewer.";
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

bool SimulationViewer::start(const mjContext &context,
                             const std::string &displayed_filename) {
  // Compatibility entry point.  Simulation uses the explicit two-phase API
  // so it can release its MuJoCo lock before GUI startup.
  stop();
  return prepare(context) && start(displayed_filename);
}

void SimulationViewer::stop() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  stop_sync_worker();
  stop_viewer();
  release_viewer_data();
}

bool SimulationViewer::capture_snapshot(const mjContext &context,
                                        ViewerSnapshot &snapshot) {
  snapshot = ViewerSnapshot{};
  if (!context.valid()) {
    LOG_WARNING << "viewer synchronization request has an invalid context.";
    return false;
  }
  std::lock_guard<std::mutex> lock(sync_mutex_);
  if (sync_stopping_ || snapshot_pool_ == nullptr || !is_ready()) {
    return false;
  }
  auto lease = snapshot_pool_->acquire();
  if (lease == nullptr) {
    LOG_WARNING
        << "viewer snapshot pool is exhausted; dropped synchronization.";
    return false;
  }
  if (mj_copyData(lease->data, viewer_model_, context.data) == nullptr) {
    LOG_WARNING << "failed to copy simulation data for viewer synchronization.";
    return false;
  }
  snapshot = ViewerSnapshot(std::move(lease));
  return true;
}

bool SimulationViewer::submit(ViewerSnapshot &&snapshot) {
  if (!snapshot)
    return false;
  std::lock_guard<std::mutex> lock(sync_mutex_);
  if (sync_stopping_) {
    return false;
  }
  pending_snapshot_ = std::move(snapshot.lease_);
  sync_pending_ = true;
  sync_cv_.notify_one();
  return true;
}

bool SimulationViewer::submit(const mjContext &context) {
  ViewerSnapshot snapshot;
  return capture_snapshot(context, snapshot) && submit(std::move(snapshot));
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
