#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/camera/camera_renderer_config.hpp"
#include "mujoco_simulation/component/camera/offscreen_gl_context.hpp"

namespace mujoco_simulation {

struct CameraRenderImage {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t step{0};
  std::vector<std::uint8_t> data;
};

struct CameraRenderDepthImage {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<float> data;
};

struct CameraRenderIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  Vector9d k{};
  Vector12d p{};
};

struct CameraRenderState {
  std::uint64_t sequence{0};
  std::uint64_t timestamp{0};
  std::string frame_id;
  std::string optical_frame_id;
  CameraRenderImage color;
  CameraRenderDepthImage depth;
  CameraRenderIntrinsics intrinsics;
};

class CameraRenderer {
 public:
  CameraRenderer();
  explicit CameraRenderer(CameraRendererConfig config);
  ~CameraRenderer();

  CameraRenderer(const CameraRenderer&) = delete;
  CameraRenderer& operator=(const CameraRenderer&) = delete;

  bool initialize(const mjModel& model);
  bool shutdown();

  bool copy_simulation_data(const mjModel& model, const mjData& source);
  bool render(const mjModel& model, const CameraConfig& spec, std::uint64_t sequence,
              std::uint64_t timestamp, std::shared_ptr<const CameraRenderState>& out);

  bool is_initialized() const noexcept { return initialized_; }

 private:
  static bool ensure_glfw_initialized();
  bool ensure_gl_context();
  bool ensure_offscreen_capacity(int width, int height);
  bool ensure_camera_binding(const mjModel& model, const CameraConfig& spec, int& camera_id,
                             double& fovy_degrees);
  CameraRenderIntrinsics compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                            std::uint32_t height) const;
  void flip_rgb(const std::vector<std::uint8_t>& source, std::uint32_t width, std::uint32_t height,
                std::vector<std::uint8_t>& dest) const;
  void convert_and_flip_depth(const mjModel& model, const std::vector<float>& source,
                              std::uint32_t width, std::uint32_t height,
                              std::vector<float>& dest) const;
  bool initialize_egl_context();
  void release_current_context();

  CameraRendererConfig config_{};
  OffscreenGlContext gl_context_{};
  mjData* render_data_{nullptr};
  mjvScene scene_{};
  mjvOption option_{};
  mjrContext render_context_{};
  bool initialized_{false};
  int offscreen_width_{0};
  int offscreen_height_{0};
};

}  // namespace mujoco_simulation
