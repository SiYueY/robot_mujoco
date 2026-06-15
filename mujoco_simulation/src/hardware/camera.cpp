#include "mujoco_simulation/hardware/camera.hpp"

#include <algorithm>
#include <cstring>

namespace mujoco_simulation {

Camera::Camera(const MjContext& context) : context_(context) {}

bool Camera::init(const CameraSpec& data) {
  data_ = data;
  state_ = {};
  last_error_.clear();

  if (context_.model == nullptr || context_.data == nullptr || context_.scene == nullptr ||
      context_.render == nullptr) {
    last_error_ = "MuJoCo rendering state is not available for camera '" + data.name + "'.";
    return false;
  }

  camera_id_ = mj_name2id(context_.model, mjOBJ_CAMERA, data.camera_name.c_str());
  if (camera_id_ < 0) {
    last_error_ = "MuJoCo camera not found: " + data.camera_name;
    return false;
  }

  if (data_.width <= 0 || data_.height <= 0) {
    last_error_ = "Camera '" + data.name + "' requires positive width and height.";
    return false;
  }

  mjv_defaultCamera(&mj_camera_);
  mjv_defaultOption(&mj_option_);
  mj_camera_.type = mjCAMERA_FIXED;
  mj_camera_.fixedcamid = camera_id_;
  viewport_ = {0, 0, data_.width, data_.height};

  const std::size_t rgb_size =
      static_cast<std::size_t>(data_.width) * static_cast<std::size_t>(data_.height) * 3U;
  image_buffer_.assign(rgb_size, 0U);
  depth_image_buffer_.assign(
      static_cast<std::size_t>(data_.width) * static_cast<std::size_t>(data_.height), 0.0F);

  state_.image.width = static_cast<uint32_t>(data_.width);
  state_.image.height = static_cast<uint32_t>(data_.height);
  state_.image.step = static_cast<uint32_t>(data_.width * 3);
  state_.image.encoding = "rgb8";
  state_.image.data.assign(rgb_size, 0U);

  state_.depth_image.width = static_cast<uint32_t>(data_.width);
  state_.depth_image.height = static_cast<uint32_t>(data_.height);
  state_.depth_image.step = static_cast<uint32_t>(data_.width * static_cast<int>(sizeof(float)));
  state_.depth_image.encoding = "32FC1";
  state_.depth_image.data.assign(depth_image_buffer_.size() * sizeof(float), 0U);

  state_.camera_info.width = static_cast<uint32_t>(data_.width);
  state_.camera_info.height = static_cast<uint32_t>(data_.height);
  state_.camera_info.distortion_model = "plumb_bob";
  state_.camera_info.d.assign(5, 0.0);
  state_.camera_info.k.fill(0.0);
  state_.camera_info.r.fill(0.0);
  state_.camera_info.p.fill(0.0);
  state_.camera_info.k[0] = state_.camera_info.k[4] = 1.0;
  state_.camera_info.k[8] = 1.0;
  state_.camera_info.r[0] = state_.camera_info.r[4] = state_.camera_info.r[8] = 1.0;
  state_.camera_info.p[0] = state_.camera_info.p[5] = state_.camera_info.p[10] = 1.0;

  return true;
}

bool Camera::reset() {
  last_error_.clear();
  std::fill(image_buffer_.begin(), image_buffer_.end(), 0U);
  std::fill(depth_image_buffer_.begin(), depth_image_buffer_.end(), 0.0F);
  std::fill(state_.image.data.begin(), state_.image.data.end(), 0U);
  std::fill(state_.depth_image.data.begin(), state_.depth_image.data.end(), 0U);
  return true;
}

bool Camera::write(const CameraCommand&) {
  last_error_.clear();
  return true;
}

bool Camera::read(CameraState& state) {
  last_error_.clear();
  if (context_.model == nullptr || context_.data == nullptr || context_.scene == nullptr ||
      context_.render == nullptr || camera_id_ < 0) {
    last_error_ = "Camera '" + data_.name + "' is not initialized.";
    return false;
  }

  mjv_updateScene(context_.model, context_.data, &mj_option_, nullptr, &mj_camera_, mjCAT_ALL,
                  context_.scene);
  mjr_render(viewport_, context_.scene, context_.render);

  unsigned char* rgb_ptr = data_.enable_rgb ? image_buffer_.data() : nullptr;
  float* depth_ptr = data_.enable_depth ? depth_image_buffer_.data() : nullptr;
  mjr_readPixels(rgb_ptr, depth_ptr, viewport_, context_.render);

  if (data_.enable_rgb) {
    state_.image.data = image_buffer_;
  }
  if (data_.enable_depth) {
    std::memcpy(state_.depth_image.data.data(), depth_image_buffer_.data(),
                depth_image_buffer_.size() * sizeof(float));
  }

  state = state_;
  return true;
}

}  // namespace mujoco_simulation
