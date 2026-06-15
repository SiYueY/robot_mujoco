#pragma once

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "mujoco_simulation/hardware/data.hpp"
#include "mujoco_simulation/hardware/hardware_interface.hpp"
#include "mujoco_simulation/hardware/mj_context.hpp"

namespace mujoco_simulation {

struct CameraSpec {
  std::string name;
  std::string camera_name;

  int height{0};
  int width{0};

  bool enable_rgb{true};
  bool enable_depth{false};
};

struct CameraCommand {
  // Camera typically has no command
};

// https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg
struct Image {
  uint64_t timestamp{0};
  std::string frame_id;
  uint32_t height{0};         // image height
  uint32_t width{0};          // image width
  std::string encoding;       // encoding of pixels
  uint8_t is_bigendian{0};    // data bigendian
  uint32_t step{0};           // full row length in bytes
  std::vector<uint8_t> data;  // actual matrix data, size is (step * rows)
};

// https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg
struct CameraInfo {
  uint32_t height{0};
  uint32_t width{0};
  std::string distortion_model;
  std::vector<double> d;
  Vector9d k{};
  Vector9d r{};
  Vector12d p{};
  uint32_t binning_x{0};
  uint32_t binning_y{0};
};

struct CameraState {
  Image image;
  Image depth_image;
  CameraInfo camera_info;
};

class Camera : public HardwareInterface<CameraSpec, CameraCommand, CameraState> {
 public:
  explicit Camera(const MjContext& context);
  ~Camera() override = default;

  bool init(const CameraSpec& data) override;
  bool reset() override;
  bool write(const CameraCommand&) override;
  bool read(CameraState& state) override;
  const std::string& last_error() const override { return last_error_; }

 private:
  MjContext context_{};

  CameraSpec data_;
  int camera_id_{-1};

  mjvCamera mj_camera_{};
  mjvOption mj_option_{};
  mjrRect viewport_{};

  std::vector<uint8_t> image_buffer_;
  std::vector<float> depth_image_buffer_;
  CameraState state_;
  std::string last_error_;
};

}  // namespace mujoco_simulation
