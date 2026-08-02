#include "component/camera/camera_component.hpp"

#include <utility>

#include "common/logging.hpp"
#include "common/macro.hpp"

namespace mujoco_simulation {

CameraComponent::CameraComponent(CameraConfig config)
    : SimulationComponent(config.name, config.period),
      config_(std::move(config)) {}

bool CameraComponent::init(const mjContext &context) {
  if (!configure(context)) {
    return false;
  }

  const mjModel &model = *context.model;
  if (model.opt.timestep <= 0.0) {
    LOG_ERROR << "model timestep must be positive.";
    return false;
  }
  if (config_.camera_name.empty()) {
    LOG_ERROR << "camera name must not be empty.";
    return false;
  }
  if (config_.width <= 0 || config_.height <= 0) {
    LOG_ERROR << "camera width and height must be positive.";
    return false;
  }
  if (!config_.enable_rgb && !config_.enable_depth) {
    LOG_ERROR << "camera must enable rgb or depth output.";
    return false;
  }

  camera_id_ = mj_name2id(&model, mjOBJ_CAMERA, config_.camera_name.c_str());
  if (camera_id_ < 0) {
    LOG_ERROR << "camera was not found in model.";
    return false;
  }
  sample_sequence_ = 0;
  last_applied_sequence_ = 0;
  state_.reset();
  return true;
}

bool CameraComponent::reset(const mjContext &context) {
  UNUSED(context);
  sample_sequence_ = 0;
  last_applied_sequence_ = 0;
  state_.reset();
  return true;
}

bool CameraComponent::read_state(
    std::shared_ptr<const CameraState> &state) const {
  state = state_;
  return state != nullptr;
}

bool CameraComponent::update(const mjContext &context) {
  UNUSED(context);
  if (camera_id_ < 0) {
    LOG_ERROR << "camera must be bound before update.";
    return false;
  }
  return true;
}

CameraRenderTask CameraComponent::make_render_task(std::uint64_t timestamp) {
  CameraRenderTask task;
  task.camera_id = config_.id;
  task.mujoco_camera_id = camera_id_;
  task.width = static_cast<std::uint32_t>(config_.width);
  task.height = static_cast<std::uint32_t>(config_.height);
  task.render_depth = config_.enable_depth;
  task.config = config_;
  task.sequence = ++sample_sequence_;
  task.timestamp = timestamp;
  return task;
}

bool CameraComponent::apply_render_result(
    const CameraRenderTaskResult &result) {
  if (result.camera_id != config_.id ||
      result.status != CameraTaskStatus::Completed ||
      result.sequence <= last_applied_sequence_) {
    return false;
  }
  auto state = std::make_shared<CameraState>();
  state->sequence = result.sequence;
  state->timestamp = result.timestamp;
  state->frame_id = config_.frame_id;
  state->optical_frame_id = config_.optical_frame_id;
  state->image = result.frame.image;
  state->depth_image = result.frame.depth_image;
  state->camera_info = result.frame.camera_info;
  state_ = std::move(state);
  last_applied_sequence_ = result.sequence;
  return true;
}

void CameraComponent::clear_render_state() noexcept {
  last_applied_sequence_ = 0;
  state_.reset();
}

} // namespace mujoco_simulation
