#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mujoco_simulation/hardware/data.hpp"

namespace mujoco_simulation {

enum class PixelFormat {
  Rgb8,
};

enum class DepthFormat {
  Float32Meters,
};

struct ImageBuffer {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t step{0};
  PixelFormat format{PixelFormat::Rgb8};
  std::vector<std::uint8_t> data;
};

struct DepthBuffer {
  std::uint32_t width{0};
  std::uint32_t height{0};
  DepthFormat format{DepthFormat::Float32Meters};
  std::vector<float> data;
};

struct CameraIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  Vector9d k{};
  Vector12d p{};
};

struct CameraSample {
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  std::string frame_id;
  std::string optical_frame_id;
  std::optional<ImageBuffer> color;
  std::optional<DepthBuffer> depth;
  CameraIntrinsics intrinsics;
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
  Image image;
  Image depth_image;
  CameraInfo camera_info;
};

CameraInfo camera_info_from_intrinsics(const CameraIntrinsics& intrinsics, std::uint32_t width,
                                       std::uint32_t height);
CameraState camera_state_from_sample(const CameraSample& sample);

}  // namespace mujoco_simulation
