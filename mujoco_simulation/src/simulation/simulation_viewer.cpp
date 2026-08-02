#include "simulation/simulation_impl.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <utility>

#include "common/logging.hpp"

namespace mujoco_simulation {

bool Simulation::Impl::start_viewer() {
    auto viewer = std::make_shared<SimulationViewer>(config_.viewer_startup_timeout);
    {
        std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
        if (runtime_ == nullptr) {
            LOG_ERROR << "simulation runtime is not available.";
            return false;
        }
        if (!runtime_->is_initialized()) {
            LOG_ERROR << "simulation runtime is not initialized.";
            return false;
        }
        if (!viewer->prepare(runtime_->context())) {
            LOG_ERROR << "viewer preparation failed.";
            return false;
        }
    }
    if (!viewer->start(config_.model.model_path)) {
        LOG_ERROR << "viewer startup failed.";
        return false;
    }
    std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
    viewer_ = std::move(viewer);
    next_sync_time_ = std::chrono::steady_clock::now();
    return true;
}

bool Simulation::Impl::scheduler_submit_viewer_sync_if_due() {
    const auto now = std::chrono::steady_clock::now();
    if (now < next_sync_time_) return true;
    std::shared_ptr<SimulationViewer> viewer;
    {
        std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
        viewer = viewer_;
    }
    if (viewer != nullptr) {
        SimulationViewer::ViewerSnapshot snapshot;
        {
            std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
            if (runtime_ != nullptr && runtime_->is_initialized() &&
                !viewer->capture_snapshot(runtime_->context(), snapshot)) {
                LOG_WARNING << "failed to capture a viewer synchronization snapshot.";
            }
        }
        if (snapshot && !viewer->submit(std::move(snapshot)))
            LOG_WARNING << "failed to enqueue a viewer synchronization request.";
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(config_.scheduler.viewer_period));
    next_sync_time_ = now + period;
    return true;
}

bool Simulation::Impl::stop_viewer() {
    std::shared_ptr<SimulationViewer> viewer;
    {
        std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
        viewer = std::move(viewer_);
    }
    if (viewer != nullptr) viewer->stop();
    return true;
}

}  // namespace mujoco_simulation
