#include "mujoco_simulation/viewer/mujoco_viewer.hpp"

#include <chrono>
#include <cstring>
#include <exception>

#include "glfw_adapter.h"
#include "simulate.h"

namespace mujoco_simulation {
namespace {

void delete_simulate(mujoco::Simulate* simulate) { delete simulate; }

}  // namespace

MuJoCoViewer::MuJoCoViewer() : MuJoCoViewer(std::chrono::milliseconds{5000}) {}

MuJoCoViewer::MuJoCoViewer(std::chrono::milliseconds startup_timeout)
    : startup_timeout_(startup_timeout), simulate_(nullptr, delete_simulate) {}

MuJoCoViewer::~MuJoCoViewer() { stop(); }

void MuJoCoViewer::mark_ready() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_ = true;
  }
  cv_.notify_all();
}

void MuJoCoViewer::record_async_failure(ResultCode status) {
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

ResultCode MuJoCoViewer::start(const ViewerRuntimeHandle& runtime_handle,
                               const std::string& displayed_filename) {
  const mjModel* model = runtime_handle.model();
  const mjData* data = runtime_handle.data();
  if (model == nullptr || data == nullptr) {
    return ResultCode::InvalidArgument;
  }
  if (render_thread_.joinable()) {
    return ResultCode::Ok;
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
      runtime_handle_ = runtime_handle;
    }

    render_thread_ = std::thread([this, displayed_filename]() {
      try {
        if (render_thread_entry_) {
          render_thread_entry_(*this, const_cast<mjModel*>(runtime_handle_.model()),
                               const_cast<mjData*>(runtime_handle_.data()), displayed_filename);
          return;
        }

        mjModel* model = const_cast<mjModel*>(runtime_handle_.model());
        mjData* data = const_cast<mjData*>(runtime_handle_.data());
        if (model == nullptr || data == nullptr) {
          record_async_failure(ResultCode::InvalidState);
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
        record_async_failure(ResultCode::ThreadFailed);
        return;
      } catch (...) {
        record_async_failure(ResultCode::ThreadFailed);
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
    return ResultCode::ThreadFailed;
  }

  const auto ready_deadline = std::chrono::steady_clock::now() + startup_timeout_;
  std::unique_lock<std::mutex> lock(mutex_);
  const bool ready = cv_.wait_until(lock, ready_deadline,
                                    [this]() { return ready_ || async_failure_.has_value(); });
  if (async_failure_.has_value()) {
    const ResultCode failure = *async_failure_;
    lock.unlock();
    stop();
    return failure;
  }
  if (!ready) {
    lock.unlock();
    stop();
    return ResultCode::Timeout;
  }
  return ResultCode::Ok;
}

void MuJoCoViewer::stop() {
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
  runtime_handle_ = ViewerRuntimeHandle{};
}

ResultCode MuJoCoViewer::sync(bool state_only) {
  std::lock_guard<std::mutex> state_lock(mutex_);
  if (async_failure_.has_value()) {
    return *async_failure_;
  }
  if (simulate_ == nullptr || !ready_) {
    return ResultCode::InvalidState;
  }

  try {
    std::unique_lock<std::recursive_mutex> simulate_lock(simulate_->mtx);
    if (simulate_->exitrequest.load()) {
      return ResultCode::InvalidState;
    }
    simulate_->Sync(state_only);
  } catch (const std::exception&) {
    return ResultCode::RenderFailed;
  }

  return ResultCode::Ok;
}

bool MuJoCoViewer::is_running() const { return render_thread_.joinable(); }

bool MuJoCoViewer::is_ready() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ready_;
}

}  // namespace mujoco_simulation
