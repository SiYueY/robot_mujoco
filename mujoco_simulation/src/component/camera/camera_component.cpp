#include "mujoco_simulation/component/camera/camera_component.hpp"

#include <cmath>
#include <cstring>
#include <utility>

namespace mujoco_simulation {
namespace {

CameraIntrinsics compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                    std::uint32_t height) {
  CameraIntrinsics intrinsics;
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

CameraInfo camera_info_from_intrinsics(const CameraIntrinsics& intrinsics, std::uint32_t width,
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

CameraState camera_state_from_sample(const CameraSample& sample) {
  CameraState state;

  if (sample.color.has_value()) {
    const ImageBuffer& color = *sample.color;
    state.image.timestamp = sample.timestamp_ns;
    state.image.frame_id =
        sample.optical_frame_id.empty() ? sample.frame_id : sample.optical_frame_id;
    state.image.width = color.width;
    state.image.height = color.height;
    state.image.encoding = "rgb8";
    state.image.step = color.step;
    state.image.data = color.data;
  }

  if (sample.depth.has_value()) {
    const DepthBuffer& depth = *sample.depth;
    state.depth_image.timestamp = sample.timestamp_ns;
    state.depth_image.frame_id =
        sample.optical_frame_id.empty() ? sample.frame_id : sample.optical_frame_id;
    state.depth_image.width = depth.width;
    state.depth_image.height = depth.height;
    state.depth_image.encoding = "32FC1";
    state.depth_image.step = depth.width * static_cast<std::uint32_t>(sizeof(float));
    state.depth_image.data.resize(depth.data.size() * sizeof(float));
    std::memcpy(state.depth_image.data.data(), depth.data.data(), state.depth_image.data.size());
  }

  const std::uint32_t info_width = sample.color.has_value()
                                       ? sample.color->width
                                       : (sample.depth.has_value() ? sample.depth->width : 0U);
  const std::uint32_t info_height = sample.color.has_value()
                                        ? sample.color->height
                                        : (sample.depth.has_value() ? sample.depth->height : 0U);
  state.camera_info = camera_info_from_intrinsics(sample.intrinsics, info_width, info_height);
  return state;
}

CameraComponent::CameraComponent(CameraConfig config) : config_(std::move(config)) {}

std::string_view CameraComponent::name() const noexcept { return config_.common.name; }

Status CameraComponent::bind(const mjModel& model) {
  if (config_.common.name.empty()) {
    return Status::invalid_argument("Camera name must not be empty.");
  }
  if (config_.camera_name.empty()) {
    return Status::invalid_argument("MuJoCo camera name must not be empty for camera '" +
                                    config_.common.name + "'.");
  }
  if (config_.width <= 0 || config_.height <= 0) {
    return Status::invalid_argument("Camera '" + config_.common.name +
                                    "' requires positive width and height.");
  }
  if (config_.common.update_rate <= 0.0) {
    return Status::invalid_argument("Camera '" + config_.common.name +
                                    "' requires a positive update_rate.");
  }
  if (!config_.enable_rgb && !config_.enable_depth) {
    return Status::invalid_argument("Camera '" + config_.common.name +
                                    "' must enable rgb or depth output.");
  }

  binding_.camera_id = mj_name2id(&model, mjOBJ_CAMERA, config_.camera_name.c_str());
  if (binding_.camera_id < 0) {
    return Status::binding_failed("Camera '" + config_.common.name +
                                  "' could not bind to MuJoCo camera '" + config_.camera_name +
                                  "'.");
  }
  binding_.fovy_degrees = static_cast<double>(model.cam_fovy[binding_.camera_id]);

  sample_sequence_ = 0;
  return Status::Ok();
}

Status CameraComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  (void)data;
  sample_sequence_ = 0;
  return Status::Ok();
}

double CameraComponent::update_rate() const noexcept { return config_.common.update_rate; }

Status CameraComponent::sample(const SensorSampleContext& context) {
  (void)context.step_count;
  if (binding_.camera_id < 0) {
    return Status::failed_precondition("CameraComponent is not bound: " + config_.common.name);
  }
  if (context.camera_renderer == nullptr || context.camera_buffer == nullptr) {
    return Status::failed_precondition("Camera runtime resources are not available for '" +
                                       config_.common.name + "'.");
  }

  const std::uint64_t timestamp_ns =
      context.simulation_time <= 0.0 ? 0
                                     : static_cast<std::uint64_t>(context.simulation_time * 1.0e9);
  Result<std::shared_ptr<const CameraSample>> rendered =
      context.camera_renderer->render(context.model, config_, ++sample_sequence_, timestamp_ns);
  if (!rendered.ok()) {
    return rendered.status();
  }
  context.camera_buffer->publish(config_.common.name, rendered.value());
  return Status::Ok();
}

}  // namespace mujoco_simulation
