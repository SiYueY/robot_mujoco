#include "simulation/simulation_impl.hpp"

#include <chrono>
#include <mutex>
#include <optional>
#include <utility>

#include "common/logging.hpp"
#include "common/macro.hpp"
#include "config/simulation_config_parser.hpp"
#include "config/simulation_config_validator.hpp"

namespace mujoco_simulation {
bool Simulation::Impl::initialize(const std::string& config_path) {
    SimulationConfig config;
    SimulationConfigParser parser;
    if (!parser.load_file(config_path, config)) return false;
    return initialize(config);
}

bool Simulation::Impl::initialize(const SimulationConfig& config) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (runtime_ != nullptr) {
        LOG_ERROR << "simulation is already initialized.";
        return false;
    }
    if (!SimulationConfigValidator::validate(config)) return false;
    config_ = config;
    applied_command_ = {};
    command_buffer_.shutdown();
    state_buffer_.shutdown();
    id_resolver_.reset();
    const auto cleanup = [this] {
        if (scheduler_ != nullptr) {
            UNUSED(scheduler_->shutdown());
            scheduler_.reset();
        }
        if (camera_render_service_ != nullptr) UNUSED(camera_render_service_->shutdown());
        UNUSED(stop_viewer());
        std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
        camera_render_service_.reset();
        component_manager_.clear();
        runtime_.reset();
        runtime_failed_.store(false);
        viewer_stop_requested_.store(false);
        step_.store(0);
        sequence_ = 0;
        command_buffer_.shutdown();
        state_buffer_.shutdown();
        id_resolver_.reset();
        applied_command_ = {};
    };
    if (!load_model(config.model)) {
        LOG_ERROR << "failed to load the simulation model.";
        cleanup();
        return false;
    }
    bool physics_period_applied = false;
    {
        std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
        physics_period_applied =
            runtime_ != nullptr && runtime_->set_timestep(config_.scheduler.physics_period);
    }
    if (!physics_period_applied) {
        LOG_ERROR << "failed to apply the configured physics period.";
        cleanup();
        return false;
    }
    if (!initialize_camera_renderer()) {
        LOG_ERROR << "failed to initialize the camera renderer.";
        cleanup();
        return false;
    }
    if (!initialize_components()) {
        LOG_ERROR << "failed to initialize simulation components.";
        cleanup();
        return false;
    }
    bool initial_state_published = false;
    {
        std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
        initial_state_published = write_state_snapshot_locked();
    }
    if (!initial_state_published) {
        LOG_ERROR << "failed to publish the initial simulation state.";
        cleanup();
        return false;
    }
    if (!initialize_scheduler()) {
        LOG_ERROR << "failed to initialize the simulation scheduler.";
        cleanup();
        return false;
    }
    if (config_.viewer_enabled && !start_viewer()) {
        LOG_ERROR << "failed to start the simulation viewer.";
        cleanup();
        runtime_failed_.store(true);
        return false;
    }
    return true;
}

bool Simulation::Impl::shutdown() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (scheduler_ != nullptr) {
        UNUSED(scheduler_->shutdown());
        scheduler_.reset();
    }
    if (camera_render_service_ != nullptr) UNUSED(camera_render_service_->shutdown());
    UNUSED(stop_viewer());
    {
        std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
        camera_render_service_.reset();
        component_manager_.clear();
        runtime_.reset();
        runtime_failed_.store(false);
        viewer_stop_requested_.store(false);
        step_.store(0);
        sequence_ = 0;
    }
    command_buffer_.shutdown();
    state_buffer_.shutdown();
    id_resolver_.reset();
    applied_command_ = {};
    return true;
}

bool Simulation::Impl::start() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (scheduler_ == nullptr) {
        LOG_ERROR << "simulation scheduler is not initialized.";
        return false;
    }
    if (runtime_failed_.load()) {
        LOG_ERROR << "simulation is in an error state; reset or shutdown is required "
                     "before starting again.";
        return false;
    }
    bool needs_viewer = false;
    bool viewer_started_here = false;
    bool camera_started_here = false;
    const auto rollback_started_resources = [this, &viewer_started_here, &camera_started_here] {
        if (camera_started_here && camera_render_service_ != nullptr)
            UNUSED(camera_render_service_->reset());
        if (viewer_started_here) UNUSED(stop_viewer());
    };
    if (config_.viewer_enabled) {
        std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
        needs_viewer = viewer_ == nullptr;
    }
    if (needs_viewer) {
        if (start_viewer()) {
            viewer_started_here = true;
            viewer_stop_requested_.store(false);
        } else {
            LOG_ERROR << "failed to restart the simulation viewer.";
            runtime_failed_.store(true);
            return false;
        }
    }
    bool camera_start_failed = false;
    if (component_manager_.has_cameras()) {
        std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
        if (runtime_ == nullptr) {
            LOG_ERROR << "simulation runtime is not available.";
            camera_start_failed = true;
        }
        if (!camera_start_failed && !runtime_->is_initialized()) {
            LOG_ERROR << "simulation runtime is not initialized.";
            camera_start_failed = true;
        }
        if (!camera_start_failed && camera_render_service_ == nullptr) {
            LOG_ERROR << "camera renderer is not available.";
            camera_start_failed = true;
        }
        if (!camera_start_failed &&
            !camera_render_service_->initialize(config_, runtime_->context().model)) {
            LOG_ERROR << "failed to restart the camera render worker.";
            camera_start_failed = true;
        } else if (!camera_start_failed) {
            camera_started_here = true;
        }
    }
    if (camera_start_failed) {
        rollback_started_resources();
        return false;
    }
    if (!scheduler_->start()) {
        LOG_ERROR << "failed to start the simulation scheduler.";
        rollback_started_resources();
        return false;
    }
    return true;
}

bool Simulation::Impl::stop() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    const bool was_error =
        runtime_failed_.load() ||
        (scheduler_ != nullptr && scheduler_->status() == SimulationStatus::Error);
    if (scheduler_ != nullptr && !scheduler_->stop()) {
        LOG_ERROR << "failed to stop the simulation scheduler.";
        return false;
    }
    runtime_failed_.store(was_error);
    if (camera_render_service_ != nullptr) UNUSED(camera_render_service_->reset());
    {
        std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
        component_manager_.clear_camera_states();
    }
    command_buffer_.clear();
    UNUSED(stop_viewer());
    return true;
}

bool Simulation::Impl::pause() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (scheduler_ == nullptr) {
        LOG_ERROR << "simulation scheduler is not initialized.";
        return false;
    }
    if (!scheduler_->pause()) {
        LOG_ERROR << "failed to pause the simulation scheduler.";
        return false;
    }
    return true;
}

bool Simulation::Impl::resume() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (scheduler_ == nullptr) {
        LOG_ERROR << "simulation scheduler is not initialized.";
        return false;
    }
    if (!scheduler_->resume()) {
        LOG_ERROR << "failed to resume the simulation scheduler.";
        return false;
    }
    return true;
}

bool Simulation::Impl::reset(const std::string* keyframe_name) {
    std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (scheduler_ == nullptr) {
        LOG_ERROR << "simulation scheduler is not initialized.";
        return false;
    }
    if (runtime_ == nullptr) {
        LOG_ERROR << "simulation must be initialized before reset.";
        return false;
    }
    const auto fail_reset = [this] {
        command_buffer_.clear();
        runtime_failed_.store(true);
        return false;
    };
    const SimulationStatus previous_status = scheduler_->status();
    if (previous_status == SimulationStatus::Uninitialized) {
        LOG_ERROR << "simulation scheduler is not initialized.";
        return false;
    }
    const bool restart_running = previous_status == SimulationStatus::Running;
    const bool restart_paused = previous_status == SimulationStatus::Paused;
    const bool recover_from_error = previous_status == SimulationStatus::Error;
    if ((restart_running || restart_paused || recover_from_error) && !scheduler_->stop()) {
        LOG_ERROR << "failed to stop the scheduler before reset.";
        return fail_reset();
    }
    if (camera_render_service_ != nullptr && !camera_render_service_->reset()) {
        LOG_ERROR << "failed to stop the camera render worker before reset.";
        return fail_reset();
    }

    bool succeeded = true;
    {
        std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
        if (!runtime_->is_initialized()) {
            LOG_ERROR << "simulation runtime is not initialized.";
            succeeded = false;
        } else if (keyframe_name == nullptr) {
            succeeded = runtime_->reset();
        } else {
            succeeded = runtime_->reset(*keyframe_name);
        }
        if (succeeded) {
            succeeded = component_manager_.reset(runtime_->context());
            if (!succeeded) LOG_ERROR << "failed to reset simulation components.";
        }
        if (succeeded) {
            command_buffer_.clear();
            step_.store(0);
            sequence_ = 0;
            if (component_manager_.has_cameras()) {
                if (camera_render_service_ == nullptr) {
                    LOG_ERROR << "camera renderer is not available after reset.";
                    succeeded = false;
                } else if (!camera_render_service_->initialize(
                               config_, runtime_->context().model)) {
                    LOG_ERROR << "failed to initialize camera rendering after reset.";
                    succeeded = false;
                }
            }
        }
        if (succeeded && !component_manager_.update(runtime_->context())) {
            LOG_ERROR << "failed to update components after reset.";
            succeeded = false;
        }
    }
    if (succeeded && !component_manager_.wait_for_camera_results()) {
        LOG_ERROR << "failed to obtain the reset camera frame.";
        succeeded = false;
    }
    if (succeeded) {
        std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
        succeeded = write_state_snapshot_locked();
    }
    if (!succeeded) {
        LOG_ERROR << "failed to publish the reset simulation state.";
        return fail_reset();
    }
    runtime_failed_.store(false);
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
                LOG_WARNING << "failed to capture the reset viewer snapshot.";
            }
        }
        if (snapshot && !viewer->submit(std::move(snapshot))) {
            LOG_WARNING << "failed to enqueue the reset viewer snapshot.";
        }
    }
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(config_.scheduler.viewer_period));
    next_sync_time_ = std::chrono::steady_clock::now() + period;
    if (restart_running && !scheduler_->start()) {
        LOG_ERROR << "failed to restart the scheduler after reset.";
        return fail_reset();
    }
    if (restart_paused && !scheduler_->start_paused()) {
        LOG_ERROR << "failed to restore the scheduler paused state after reset.";
        return fail_reset();
    }
    return true;
}

}  // namespace mujoco_simulation
