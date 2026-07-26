#include "mujoco_simulation/component/camera/camera_component.hpp"

#include <cmath>
#include <cstring>
#include <utility>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"

namespace mujoco_simulation {

CameraRenderIntrinsics
CameraComponent::compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                    std::uint32_t height) {
  CameraRenderIntrinsics intrinsics;
  if (width == 0U || height == 0U) {
    return intrinsics;
  }

  const double aspect =
      static_cast<double>(width) / static_cast<double>(height);
  const double fovy_radians = fovy_degrees * M_PI / 180.0;
  intrinsics.fy =
      static_cast<double>(height) / (2.0 * std::tan(fovy_radians / 2.0));
  const double fovx_radians =
      2.0 * std::atan(aspect * std::tan(fovy_radians / 2.0));
  intrinsics.fx =
      static_cast<double>(width) / (2.0 * std::tan(fovx_radians / 2.0));
  intrinsics.cx = (static_cast<double>(width) - 1.0) / 2.0;
  intrinsics.cy = (static_cast<double>(height) - 1.0) / 2.0;
  intrinsics.k = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0,           1.0};
  intrinsics.p = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0,           0.0, 1.0, 0.0};
  return intrinsics;
}

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
  fovy_degrees_ = static_cast<double>(model.cam_fovy[camera_id_]);
  sample_sequence_ = 0;
  return true;
}

bool CameraComponent::reset(const mjContext &context) {
  UNUSED(context);
  sample_sequence_ = 0;
  return true;
}

bool CameraComponent::configure_rendering(CameraRenderer &renderer) {
  camera_renderer_ = &renderer;
  return true;
}

bool CameraComponent::prepare_rendering(const mjContext &context) {
  if (camera_renderer_ == nullptr) {
    LOG_ERROR << "rendering must be configured before update.";
    return false;
  }
  return camera_renderer_->copy_simulation_data(context);
}

bool CameraComponent::read_state(
    std::shared_ptr<const CameraState> &state) const {
  state = state_;
  return state != nullptr;
}

bool CameraComponent::update(const mjContext &context) {
  if (camera_id_ < 0) {
    LOG_ERROR << "camera must be bound before update.";
    return false;
  }
  if (camera_renderer_ == nullptr) {
    LOG_ERROR << "rendering must be configured before update.";
    return false;
  }

  const std::uint64_t timestamp =
      context.data->time <= 0.0
          ? 0
          : static_cast<std::uint64_t>(context.data->time * 1.0e9);
  std::shared_ptr<const CameraRenderState> rendered;
  if (!camera_renderer_->render(context, config_, ++sample_sequence_, timestamp,
                                rendered)) {
    LOG_ERROR << "camera render failed.";
    return false;
  }
  state_ = camera_state_from_render_state(*rendered);
  return true;
}

} // namespace mujoco_simulation
