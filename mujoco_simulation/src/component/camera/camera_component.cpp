#include "mujoco_simulation/component/camera/camera_component.hpp"

#include <cmath>
#include <cstring>
#include <utility>

#include "mujoco_simulation/component/update_context.hpp"

namespace mujoco_simulation {
namespace {

CameraRenderIntrinsics compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                          std::uint32_t height) {
  CameraRenderIntrinsics intrinsics;
  if (width == 0U || height == 0U) {
    return intrinsics;
  }

  const double aspect = static_cast<double>(width) / static_cast<double>(height);
  const double fovy_radians = fovy_degrees * M_PI / 180.0;
  intrinsics.fy = static_cast<double>(height) / (2.0 * std::tan(fovy_radians / 2.0));
  const double fovx_radians = 2.0 * std::atan(aspect * std::tan(fovy_radians / 2.0));
  intrinsics.fx = static_cast<double>(width) / (2.0 * std::tan(fovx_radians / 2.0));
  intrinsics.cx = (static_cast<double>(width) - 1.0) / 2.0;
  intrinsics.cy = (static_cast<double>(height) - 1.0) / 2.0;
  intrinsics.k = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0,           1.0};
  intrinsics.p = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0,           0.0, 1.0, 0.0};
  return intrinsics;
}

}  // namespace

CameraInfo camera_info_from_intrinsics(const CameraRenderIntrinsics& intrinsics,
                                       std::uint32_t width, std::uint32_t height) {
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

CameraState camera_state_from_render_state(const CameraRenderState& render_state) {
  CameraState state;
  state.sequence = render_state.sequence;
  state.timestamp_ns = render_state.timestamp_ns;
  state.frame_id = render_state.frame_id;
  state.optical_frame_id = render_state.optical_frame_id;
  state.image.timestamp = render_state.timestamp_ns;
  state.image.frame_id =
      render_state.optical_frame_id.empty() ? render_state.frame_id : render_state.optical_frame_id;
  state.image.width = render_state.color.width;
  state.image.height = render_state.color.height;
  state.image.encoding = "rgb8";
  state.image.step = render_state.color.step;
  state.image.data = render_state.color.data;

  state.depth_image.timestamp = render_state.timestamp_ns;
  state.depth_image.frame_id =
      render_state.optical_frame_id.empty() ? render_state.frame_id : render_state.optical_frame_id;
  state.depth_image.width = render_state.depth.width;
  state.depth_image.height = render_state.depth.height;
  state.depth_image.encoding = "32FC1";
  state.depth_image.step = render_state.depth.width * static_cast<std::uint32_t>(sizeof(float));
  state.depth_image.data.resize(render_state.depth.data.size() * sizeof(float));
  std::memcpy(state.depth_image.data.data(), render_state.depth.data.data(),
              state.depth_image.data.size());

  const std::uint32_t info_width =
      render_state.color.width != 0U ? render_state.color.width : render_state.depth.width;
  const std::uint32_t info_height =
      render_state.color.height != 0U ? render_state.color.height : render_state.depth.height;
  state.camera_info = camera_info_from_intrinsics(render_state.intrinsics, info_width, info_height);
  return state;
}

CameraComponent::CameraComponent(CameraConfig config) : config_(std::move(config)) {}

std::string CameraComponent::name() const noexcept { return config_.common.name; }

ResultCode CameraComponent::bind(const mjModel& model) {
  if (config_.common.name.empty()) {
    return ResultCode::InvalidArgument;
  }
  if (model.opt.timestep <= 0.0) {
    return ResultCode::InvalidArgument;
  }
  if (config_.camera_name.empty()) {
    return ResultCode::InvalidArgument;
  }
  if (config_.width <= 0 || config_.height <= 0) {
    return ResultCode::InvalidArgument;
  }
  if (config_.common.update_rate <= 0.0) {
    return ResultCode::InvalidArgument;
  }
  if (!config_.enable_rgb && !config_.enable_depth) {
    return ResultCode::InvalidArgument;
  }

  binding_.camera_id = mj_name2id(&model, mjOBJ_CAMERA, config_.camera_name.c_str());
  if (binding_.camera_id < 0) {
    return ResultCode::BindingFailed;
  }
  binding_.fovy_degrees = static_cast<double>(model.cam_fovy[binding_.camera_id]);
  const ResultCode schedule_status =
      set_update_rate(config_.common.update_rate, 1.0 / model.opt.timestep);
  if (schedule_status != ResultCode::Ok) {
    return schedule_status;
  }

  sample_sequence_ = 0;
  return ResultCode::Ok;
}

ResultCode CameraComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  (void)data;
  sample_sequence_ = 0;
  return ResultCode::Ok;
}

ResultCode CameraComponent::update(const UpdateContext& context) {
  (void)context.step_count;
  if (binding_.camera_id < 0) {
    return ResultCode::FailedPrecondition;
  }
  if (context.camera_renderer == nullptr || context.camera_buffer == nullptr) {
    return ResultCode::FailedPrecondition;
  }

  const std::uint64_t timestamp_ns =
      context.simulation_time <= 0.0 ? 0
                                     : static_cast<std::uint64_t>(context.simulation_time * 1.0e9);
  std::shared_ptr<const CameraRenderState> rendered;
  const ResultCode render_status = context.camera_renderer->render(
      context.model, config_, ++sample_sequence_, timestamp_ns, &rendered);
  if (render_status != ResultCode::Ok) {
    return render_status;
  }
  context.camera_buffer->write(config_.common.name, camera_state_from_render_state(*rendered));
  return ResultCode::Ok;
}

}  // namespace mujoco_simulation
