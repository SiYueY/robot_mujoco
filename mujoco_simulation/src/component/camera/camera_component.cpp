#include "mujoco_simulation/component/camera/camera_component.hpp"

#include <cstring>
#include <utility>

#include "common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"

namespace mujoco_simulation {

CameraInfo camera_info_from_intrinsics(const CameraRenderIntrinsics &intrinsics,
                                       std::uint32_t width,
                                       std::uint32_t height) {
  CameraInfo info;
  info.width = width;
  info.height = height;
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  info.k = intrinsics.k;
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = intrinsics.p;
  return info;
}

std::shared_ptr<const CameraState>
camera_state_from_render_state(const CameraRenderState &render_state) {
  auto state = std::make_shared<CameraState>();
  state->sequence = render_state.sequence;
  state->timestamp = render_state.timestamp;
  state->frame_id = render_state.frame_id;
  state->optical_frame_id = render_state.optical_frame_id;
  state->image.timestamp = render_state.timestamp;
  state->image.frame_id = render_state.optical_frame_id.empty()
                              ? render_state.frame_id
                              : render_state.optical_frame_id;
  state->image.width = render_state.color.width;
  state->image.height = render_state.color.height;
  state->image.encoding = "rgb8";
  state->image.step = render_state.color.step;
  state->image.data = render_state.color.data;

  state->depth_image.timestamp = render_state.timestamp;
  state->depth_image.frame_id = render_state.optical_frame_id.empty()
                                    ? render_state.frame_id
                                    : render_state.optical_frame_id;
  state->depth_image.width = render_state.depth.width;
  state->depth_image.height = render_state.depth.height;
  state->depth_image.encoding = "32FC1";
  state->depth_image.step =
      render_state.depth.width * static_cast<std::uint32_t>(sizeof(float));
  state->depth_image.data.resize(render_state.depth.data.size() *
                                 sizeof(float));
  std::memcpy(state->depth_image.data.data(), render_state.depth.data.data(),
              state->depth_image.data.size());

  const std::uint32_t info_width = render_state.color.width != 0U
                                       ? render_state.color.width
                                       : render_state.depth.width;
  const std::uint32_t info_height = render_state.color.height != 0U
                                        ? render_state.color.height
                                        : render_state.depth.height;
  state->camera_info = camera_info_from_intrinsics(render_state.intrinsics,
                                                   info_width, info_height);
  return state;
}

CameraComponent::CameraComponent(CameraConfig config)
    : SimulationComponent(config.name, config.update_rate),
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
  task.config = config_;
  task.sequence = ++sample_sequence_;
  task.timestamp = timestamp;
  return task;
}

bool CameraComponent::apply_render_result(
    const CameraRenderStatePtr &rendered) {
  if (rendered == nullptr) {
    LOG_WARNING << "received an empty camera render result.";
    return false;
  }
  if (rendered->sequence <= last_applied_sequence_) {
    return false;
  }
  state_ = camera_state_from_render_state(*rendered);
  last_applied_sequence_ = rendered->sequence;
  return true;
}

void CameraComponent::clear_render_state() noexcept {
  last_applied_sequence_ = 0;
  state_.reset();
}

} // namespace mujoco_simulation
