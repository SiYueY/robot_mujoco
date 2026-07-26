#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mujoco_simulation/common/math.hpp"
#include "mujoco_simulation/component/component_id.hpp"

namespace mujoco_simulation {

struct CameraConfig {
  CameraId id{kInvalidComponentId};
  std::string name;
  std::string frame_id;
  double update_rate{30.0};
  std::string camera_name;
  std::string optical_frame_id;

  int height{0};
  int width{0};

  bool enable_rgb{true};
  bool enable_depth{false};
};

struct Image {
  std::uint64_t timestamp{0};
  std::string frame_id;
  std::uint32_t height{0};
  std::uint32_t width{0};
  std::string encoding;
  std::uint8_t is_bigendian{0};
  std::uint32_t step{0};
  std::vector<std::uint8_t> data;
};

struct CameraInfo {
  std::uint32_t height{0};
  std::uint32_t width{0};
  std::string distortion_model;
  std::vector<double> d;
  Vector9d k{};
  Vector9d r{};
  Vector12d p{};
  std::uint32_t binning_x{0};
  std::uint32_t binning_y{0};
};

struct CameraState {
  std::uint64_t sequence{0};
  std::uint64_t timestamp{0};
  std::string frame_id;
  std::string optical_frame_id;
  Image image;
  Image depth_image;
  CameraInfo camera_info;
};

} // namespace mujoco_simulation
