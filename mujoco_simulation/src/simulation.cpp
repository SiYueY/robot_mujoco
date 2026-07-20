#include "mujoco_simulation/simulation.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>

#include "mujoco_simulation/common/macro.hpp"
#include "mujoco_simulation/runtime/simulation_runtime.hpp"
#include "mujoco_simulation/runtime/simulation_scheduler.hpp"
#include "mujoco_simulation/viewer/simulation_viewer.hpp"

namespace mujoco_simulation {

Simulation::Simulation() = default;

Simulation::~Simulation() { UNUSED(shutdown()); }

ResultCode Simulation::initialize(const SimulationConfig& config) {
  if (runtime_ != nullptr && runtime_->is_initialized()) {
    return ResultCode::AlreadyExists;
  }
  if (config.scheduler.viewer_update_rate <= 0.0) {
    return ResultCode::InvalidArgument;
  }

  config_ = config;

  if (!load_model(config.model)) {
    UNUSED(shutdown());
    return ResultCode::Internal;
  }

  if (config.render_mode == RenderMode::Viewer) {
    if (!start_viewer()) {
      UNUSED(shutdown());
      return ResultCode::Internal;
    }
  }

  if (!initialize_scheduler()) {
    UNUSED(shutdown());
    return ResultCode::Internal;
  }

  if (!initialize_components()) {
    UNUSED(shutdown());
    return ResultCode::Internal;
  }

  return ResultCode::Ok;
}

ResultCode Simulation::shutdown() {
  UNUSED(stop());
  UNUSED(stop_viewer());

  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    if (scheduler_ != nullptr) {
      UNUSED(scheduler_->shutdown());
      scheduler_.reset();
    }
    camera_renderer_.reset();
    camera_buffer_.reset();
    state_buffer_.reset();
    command_buffer_.reset();
    component_manager_.clear();
    runtime_.reset();
    runtime_failed_ = false;
    step_ = 0;
    sequence_ = 0;
  }

  return ResultCode::Ok;
}

ResultCode Simulation::start() {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }

  bool should_start_viewer = false;
  if (config_.render_mode == RenderMode::Viewer) {
    {
      std::lock_guard<std::mutex> runtime_lock(runtime_mutex_);
      if (runtime_ == nullptr || !runtime_->is_initialized()) {
        return ResultCode::FailedPrecondition;
      }
    }
    std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
    if (viewer_ == nullptr) {
      should_start_viewer = true;
    }
  }

  if (should_start_viewer) {
    if (!start_viewer()) {
      return ResultCode::Internal;
    }
  }

  return scheduler_->start() ? ResultCode::Ok : ResultCode::Internal;
}

ResultCode Simulation::stop() {
  if (scheduler_ != nullptr) {
    if (!scheduler_->stop()) {
      return ResultCode::Internal;
    }
  }
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    runtime_failed_ = false;
  }
  UNUSED(stop_viewer());
  return ResultCode::Ok;
}

ResultCode Simulation::pause() {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }
  return scheduler_->pause() ? ResultCode::Ok : ResultCode::Internal;
}

ResultCode Simulation::resume() {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }
  return scheduler_->resume() ? ResultCode::Ok : ResultCode::Internal;
}

ResultCode Simulation::request_reset() { return ResultCode::Unimplemented; }

ResultCode Simulation::request_reset_to_keyframe_name(std::string_view keyframe_name) {
  UNUSED(keyframe_name);
  return ResultCode::Unimplemented;
}

ResultCode Simulation::request_reset_to_keyframe_id(int keyframe_id) {
  UNUSED(keyframe_id);
  return ResultCode::Unimplemented;
}

ResultCode Simulation::reset() { return ResultCode::Unimplemented; }

ResultCode Simulation::reset_to_keyframe_name(std::string_view keyframe_name) {
  UNUSED(keyframe_name);
  return ResultCode::Unimplemented;
}

ResultCode Simulation::reset_to_keyframe_id(int keyframe_id) {
  UNUSED(keyframe_id);
  return ResultCode::Unimplemented;
}

ResultCode Simulation::set_joint_command(const JointCommand& command) {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  CommandBuffer* command_buffer = command_buffer_.get();
  if (command_buffer == nullptr) {
    return ResultCode::InvalidState;
  }
  return command_buffer->write_joint_command(command.joint_name, command)
             ? ResultCode::Ok
             : ResultCode::InvalidArgument;
}

bool Simulation::joint_state(std::string joint_name, JointState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  StateBuffer* state_buffer = state_buffer_.get();
  if (state_buffer == nullptr) {
    return false;
  }
  return state_buffer->read_joint_state(joint_name, out);
}

bool Simulation::imu_state(std::string imu_name, ImuState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  StateBuffer* state_buffer = state_buffer_.get();
  if (state_buffer == nullptr) {
    return false;
  }
  return state_buffer->read_imu_state(imu_name, out);
}

bool Simulation::camera_state(std::string camera_name, CameraState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  CameraBuffer* camera_buffer = camera_buffer_.get();
  if (camera_buffer == nullptr) {
    return false;
  }
  return camera_buffer->read(camera_name, out);
}

bool Simulation::lidar_state(std::string lidar_name, LidarState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  StateBuffer* state_buffer = state_buffer_.get();
  if (state_buffer == nullptr) {
    return false;
  }
  return state_buffer->read_lidar_state(lidar_name, out);
}

ResultCode Simulation::set_mobile_base_command(std::string name, const MobileBaseCommand& command) {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  CommandBuffer* command_buffer = command_buffer_.get();
  if (command_buffer == nullptr) {
    return ResultCode::InvalidState;
  }
  return command_buffer->write_mobile_base_command(name, command) ? ResultCode::Ok
                                                                  : ResultCode::InvalidArgument;
}

bool Simulation::mobile_base_state(std::string name, MobileBaseState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  StateBuffer* state_buffer = state_buffer_.get();
  if (state_buffer == nullptr) {
    return false;
  }
  return state_buffer->read_mobile_base_state(name, out);
}

uint64_t Simulation::step_count() const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  return step_;
}

SimulationStatus Simulation::status() const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_failed_) {
    return SimulationStatus::Error;
  }
  if (scheduler_ != nullptr) {
    return scheduler_->status();
  }
  return runtime_ != nullptr && runtime_->is_initialized() ? SimulationStatus::Stopped
                                                           : SimulationStatus::Uninitialized;
}

double Simulation::time() const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_ == nullptr || !runtime_->is_initialized()) {
    return 0.0;
  }
  return runtime_->time();
}

std::shared_ptr<const StateSnapshot> Simulation::state_snapshot() const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  StateBuffer* state_buffer = state_buffer_.get();
  return state_buffer == nullptr ? nullptr : state_buffer->read();
}

void Simulation::set_snapshot_observer(SnapshotObserver observer) {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  snapshot_observer_ = std::move(observer);
}

bool Simulation::initialize_scheduler() {
  camera_buffer_ = std::make_unique<CameraBuffer>();
  camera_renderer_ = std::make_unique<CameraRenderer>(config_.camera_renderer);
  command_buffer_ = std::make_unique<CommandBuffer>();
  state_buffer_ = std::make_unique<StateBuffer>();
  step_ = 0;
  sequence_ = 0;

  auto scheduler = std::make_unique<SimulationScheduler>();
  if (!scheduler->initialize()) {
    return false;
  }
  if (!scheduler->register_cycle([this]() { return scheduler_run_cycle(); })) {
    return false;
  }

  scheduler_ = std::move(scheduler);
  return true;
}

bool Simulation::scheduler_run_cycle() {
  if (!scheduler_write_commands()) {
    return false;
  }
  if (!scheduler_step_physics()) {
    return false;
  }
  if (!scheduler_update_components()) {
    return false;
  }
  if (!write_state_snapshot()) {
    return false;
  }
  return scheduler_sync_viewer_if_due();
}

bool Simulation::initialize_components() {
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    if (runtime_ == nullptr || !runtime_->is_initialized()) {
      return false;
    }
    const mjContext& context = runtime_->context();
    if (!component_manager_.init(context, config_.components)) {
      return false;
    }
    const double simulation_time = runtime_->time();
    if (!update_components_for_step_locked(0, simulation_time)) {
      return false;
    }
  }
  return write_state_snapshot();
}

bool Simulation::load_model(const ModelConfig& model_config) {
  auto runtime = std::make_unique<SimulationRuntime>();
  if (!runtime->init(model_config)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(runtime_mutex_);
  runtime_ = std::move(runtime);
  component_manager_.clear();
  next_sync_time_ = std::chrono::steady_clock::now();
  runtime_failed_ = false;
  step_ = 0;
  sequence_ = 0;
  return true;
}

bool Simulation::scheduler_step_physics() {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_ == nullptr || !runtime_->is_initialized()) {
    return false;
  }
  if (!runtime_->step()) {
    return false;
  }
  ++step_;
  return true;
}

bool Simulation::scheduler_write_commands() {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (command_buffer_ == nullptr || runtime_ == nullptr || !runtime_->is_initialized()) {
    return false;
  }

  const CommandSnapshot snapshot = command_buffer_->read(CommandBuffer::Clock::now());
  const mjContext& context = runtime_->context();
  return component_manager_.write_command(context, snapshot);
}

bool Simulation::scheduler_update_components() {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (runtime_ == nullptr || !runtime_->is_initialized()) {
    return false;
  }

  const double simulation_time = runtime_->time();
  return update_components_for_step_locked(step_, simulation_time);
}

bool Simulation::update_components_for_step_locked(std::uint64_t step_count,
                                                   double simulation_time) {
  if (runtime_ == nullptr || !runtime_->is_initialized()) {
    return false;
  }
  const mjContext& context = runtime_->context();
  return component_manager_.update(context, camera_renderer_.get(), camera_buffer_.get());
}

bool Simulation::write_state_snapshot() {
  std::shared_ptr<const StateSnapshot> published_snapshot;
  SnapshotObserver snapshot_observer;
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    if (!write_state_snapshot_locked(&published_snapshot)) {
      return false;
    }
    snapshot_observer = snapshot_observer_;
  }
  if (snapshot_observer != nullptr && published_snapshot != nullptr) {
    snapshot_observer(std::move(published_snapshot));
  }
  return true;
}

bool Simulation::write_state_snapshot_locked(
    std::shared_ptr<const StateSnapshot>* published_snapshot) {
  if (state_buffer_ == nullptr || runtime_ == nullptr || !runtime_->is_initialized()) {
    return false;
  }

  const double simulation_time = runtime_->time();
  return build_state_snapshot_locked(step_, simulation_time, published_snapshot);
}

bool Simulation::build_state_snapshot_locked(
    std::uint64_t step_count, double simulation_time,
    std::shared_ptr<const StateSnapshot>* published_snapshot) {
  auto snapshot = std::make_shared<StateSnapshot>();
  if (!build_state_snapshot(snapshot.get())) {
    return false;
  }
  snapshot->sequence = ++sequence_;
  snapshot->simulation_time = simulation_time;
  snapshot->timestamp_ns =
      simulation_time <= 0.0 ? 0 : static_cast<std::uint64_t>(simulation_time * 1.0e9);
  snapshot->step_count = step_count;
  std::shared_ptr<const StateSnapshot> published = snapshot;
  state_buffer_->write(published);
  if (published_snapshot != nullptr) {
    *published_snapshot = std::move(published);
  }
  return true;
}

bool Simulation::start_viewer() {
  auto viewer = std::make_unique<SimulationViewer>(config_.viewer_startup_timeout);
  {
    std::lock_guard<std::mutex> runtime_lock(runtime_mutex_);
    if (runtime_ == nullptr || !runtime_->is_initialized()) {
      return false;
    }
    if (!viewer->start(runtime_->context(), config_.model.model_path)) {
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock(viewer_mutex_);
    viewer_ = std::move(viewer);
  }
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    next_sync_time_ = std::chrono::steady_clock::now();
    runtime_failed_ = false;
  }
  return true;
}

bool Simulation::scheduler_sync_viewer_if_due() {
  std::unique_ptr<SimulationViewer> viewer_to_stop = nullptr;
  bool failed = false;
  std::chrono::nanoseconds period = kDefaultViewerPeriod;

  {
    std::lock_guard<std::mutex> runtime_lock(runtime_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (config_.scheduler.viewer_update_rate > 0.0) {
      period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / config_.scheduler.viewer_update_rate));
    }
    if (now < next_sync_time_) {
      return true;
    }
  }

  {
    std::lock_guard<std::mutex> lock(viewer_mutex_);
    if (viewer_ == nullptr) {
      return true;
    }
    if (!viewer_->is_running()) {
      failed = true;
      viewer_to_stop = std::move(viewer_);
    } else if (!viewer_->is_ready()) {
      failed = true;
      viewer_to_stop = std::move(viewer_);
    } else {
      if (!viewer_->sync(false)) {
        failed = true;
        viewer_to_stop = std::move(viewer_);
      } else {
        std::lock_guard<std::mutex> runtime_lock(runtime_mutex_);
        const auto now = std::chrono::steady_clock::now();
        next_sync_time_ = now + period;
        return true;
      }
    }
  }
  {
    std::lock_guard<std::mutex> runtime_lock(runtime_mutex_);
    runtime_failed_ = failed;
  }

  if (viewer_to_stop != nullptr) {
    viewer_to_stop->stop();
  }
  return false;
}

bool Simulation::stop_viewer() {
  std::unique_ptr<SimulationViewer> viewer_to_stop;
  {
    std::lock_guard<std::mutex> lock(viewer_mutex_);
    viewer_to_stop = std::move(viewer_);
  }
  if (viewer_to_stop != nullptr) {
    viewer_to_stop->stop();
  }
  return true;
}

bool Simulation::build_state_snapshot(StateSnapshot* snapshot) const {
  if (snapshot == nullptr || runtime_ == nullptr || !runtime_->is_initialized()) {
    return false;
  }
  const mjContext& context = runtime_->context();
  return component_manager_.read_state(context, *snapshot);
}

RenderMode parse_render_mode(const std::string& value) {
  if (value == "headless") {
    return RenderMode::Headless;
  }
  if (value == "viewer") {
    return RenderMode::Viewer;
  }
  throw std::invalid_argument("render_mode must be 'headless' or 'viewer'.");
}

const char* to_string(RenderMode mode) {
  switch (mode) {
    case RenderMode::Headless:
      return "headless";
    case RenderMode::Viewer:
      return "viewer";
  }
  return "unknown";
}

}  // namespace mujoco_simulation
