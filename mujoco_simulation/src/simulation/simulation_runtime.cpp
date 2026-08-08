#include "simulation/simulation_impl.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <mutex>
#include <utility>

#include "common/logging.hpp"
#include "render/camera_render_service_impl.hpp"

namespace mujoco_simulation {
bool Simulation::Impl::initialize_camera_renderer() {
    if (camera_render_service_ != nullptr) {
        LOG_ERROR << "camera renderer is already initialized.";
        return false;
    }
    try {
        CameraRendererConfig renderer_config = config_.camera_renderer;
        camera_render_service_ =
            std::make_unique<CameraRenderServiceImpl>(std::move(renderer_config));
    } catch (const std::exception&) {
        LOG_ERROR << "failed to create the camera renderer.";
        return false;
    } catch (...) {
        LOG_ERROR << "failed to create the camera renderer.";
        return false;
    }
    return true;
}

bool Simulation::Impl::initialize_scheduler() {
    if (scheduler_ != nullptr) {
        LOG_ERROR << "simulation scheduler is already initialized.";
        return false;
    }
    auto scheduler = std::make_unique<SimulationScheduler>();
    if (!scheduler->initialize(std::chrono::duration<double>(config_.scheduler.physics_period))) {
        LOG_ERROR << "failed to initialize the simulation scheduler.";
        return false;
    }
    if (!scheduler->register_task([this] { return scheduler_run_task(); })) {
        LOG_ERROR << "failed to register the simulation scheduler task.";
        return false;
    }
    scheduler_ = std::move(scheduler);
    return true;
}

bool Simulation::Impl::initialize_components() {
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
        if (camera_render_service_ == nullptr) {
            LOG_ERROR << "camera renderer is not initialized.";
            return false;
        }
        if (!component_manager_.init(
                runtime_->context(), config_.components, *camera_render_service_)) {
            LOG_ERROR << "failed to initialize simulation components.";
            return false;
        }
        if (component_manager_.has_cameras() &&
            !camera_render_service_->initialize(config_, runtime_->context().model)) {
            LOG_ERROR << "failed to initialize the camera render worker.";
            return false;
        }
        if (!component_manager_.update(runtime_->context())) {
            LOG_ERROR << "failed to update initial component state.";
            return false;
        }
    }
    if (!component_manager_.wait_for_camera_results()) {
        LOG_ERROR << "failed to obtain the initial camera frame.";
        return false;
    }
    auto id_resolver = ComponentIdResolver::create(config_.components);
    if (id_resolver == nullptr || !command_buffer_.configure(id_resolver)) {
        LOG_ERROR << "failed to configure command channels.";
        return false;
    }
    if (!state_buffer_.configure(id_resolver)) {
        LOG_ERROR << "failed to configure state channels.";
        return false;
    }
    id_resolver_ = std::move(id_resolver);
    return true;
}

bool Simulation::Impl::load_model(const ModelConfig& model_config) {
    auto runtime = std::make_unique<SimulationRuntime>();
    if (!runtime->init(model_config)) {
        LOG_ERROR << "failed to initialize the MuJoCo runtime.";
        return false;
    }
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    runtime_ = std::move(runtime);
    component_manager_.clear();
    next_sync_time_ = std::chrono::steady_clock::now();
    runtime_failed_.store(false);
    viewer_stop_requested_.store(false);
    step_.store(0);
    sequence_ = 0;
    return true;
}

bool Simulation::Impl::scheduler_run_task() {
    if (viewer_stop_requested_.load()) {
        return scheduler_ != nullptr && scheduler_->request_stop();
    }
    if (runtime_failed_.load()) {
        LOG_ERROR << "simulation runtime is in the error state.";
        return false;
    }
    if (command_buffer_.read(applied_command_sequence_, applied_command_)) {
        applied_command_sequence_ = applied_command_->sequence;
    }
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
        if (applied_command_ == nullptr) {
            LOG_ERROR << "command snapshot is not available.";
            return false;
        }
        if (!component_manager_.write_command(runtime_->context(), *applied_command_)) {
            LOG_ERROR << "component manager rejected a command snapshot.";
            return false;
        }
        if (!runtime_->step()) {
            LOG_ERROR << "MuJoCo physics step failed.";
            return false;
        }
        ++step_;
        if (!component_manager_.update(runtime_->context())) {
            LOG_ERROR << "component manager update failed.";
            return false;
        }
        if (!write_state_snapshot_locked()) {
            LOG_ERROR << "failed to publish the simulation state.";
            return false;
        }
    }
    if (!scheduler_submit_viewer_sync_if_due()) {
        LOG_ERROR << "failed to submit a simulation viewer synchronization request.";
        return false;
    }
    return !viewer_stop_requested_.load() || (scheduler_ != nullptr && scheduler_->request_stop());
}

bool Simulation::Impl::step(std::size_t count) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (scheduler_ == nullptr) {
        LOG_ERROR << "simulation scheduler is not initialized.";
        return false;
    }
    if (runtime_failed_.load()) {
        LOG_ERROR << "simulation is in an error state; reset or shutdown is required "
                     "before stepping again.";
        return false;
    }
    return scheduler_->step(count);
}

bool Simulation::Impl::write_state_snapshot_locked() {
    if (runtime_ == nullptr) {
        LOG_ERROR << "simulation runtime is not available.";
        return false;
    }
    if (!runtime_->is_initialized()) {
        LOG_ERROR << "simulation runtime is not initialized.";
        return false;
    }
    return create_state_snapshot_locked(step_.load(), runtime_->time());
}

bool Simulation::Impl::create_state_snapshot_locked(std::uint64_t step, double simulation_time) {
    auto snapshot = std::make_shared<RobotState>();
    if (!create_state_snapshot(*snapshot)) {
        LOG_ERROR << "component manager failed to create the state snapshot.";
        return false;
    }
    snapshot->sequence = ++sequence_;
    snapshot->simulation_time = simulation_time;
    snapshot->timestamp =
        simulation_time > 0.0 ? static_cast<std::uint64_t>(simulation_time * 1.0e9) : 0;
    snapshot->step = step;
    if (!state_buffer_.write(std::move(snapshot))) {
        LOG_ERROR << "state snapshot does not match configured component IDs.";
        return false;
    }
    return true;
}

bool Simulation::Impl::create_state_snapshot(RobotState& snapshot) const {
    if (runtime_ == nullptr) {
        LOG_ERROR << "simulation runtime is not available.";
        return false;
    }
    if (!runtime_->is_initialized()) {
        LOG_ERROR << "simulation runtime is not initialized.";
        return false;
    }
    if (!component_manager_.read_state(runtime_->context(), snapshot)) {
        LOG_ERROR << "component manager failed to read the simulation state.";
        return false;
    }
    return true;
}

}  // namespace mujoco_simulation
