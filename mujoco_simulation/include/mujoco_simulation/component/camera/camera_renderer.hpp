#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mujoco_simulation/component/camera/camera_config.hpp"
#include "mujoco_simulation/component/camera/camera_renderer_config.hpp"
#include "mujoco_simulation/component/camera/camera_sample.hpp"
#include "mujoco_simulation/component/camera/offscreen_gl_context.hpp"
#include "mujoco_simulation/result.hpp"
#include "mujoco_simulation/status.hpp"

namespace mujoco_simulation {

class CameraRenderer {
 public:
  CameraRenderer();
  explicit CameraRenderer(CameraRendererConfig config);
  ~CameraRenderer();

  CameraRenderer(const CameraRenderer&) = delete;
  CameraRenderer& operator=(const CameraRenderer&) = delete;

  Status initialize(const mjModel& model);
  Status shutdown();

  Status copy_simulation_data(const mjModel& model, const mjData& source);
  Result<std::shared_ptr<const CameraSample>> render(const mjModel& model, const CameraConfig& spec,
                                                     std::uint64_t sequence,
                                                     std::uint64_t timestamp_ns);

  bool is_initialized() const noexcept { return initialized_; }

 private:
  Status ensure_gl_context();
  Status ensure_offscreen_capacity(int width, int height);
  Status ensure_camera_binding(const mjModel& model, const CameraConfig& spec, int* camera_id,
                               double* fovy_degrees);
  CameraIntrinsics compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                      std::uint32_t height) const;
  void flip_rgb(const std::vector<std::uint8_t>& source, std::uint32_t width, std::uint32_t height,
                std::vector<std::uint8_t>* dest) const;
  void convert_and_flip_depth(const mjModel& model, const std::vector<float>& source,
                              std::uint32_t width, std::uint32_t height,
                              std::vector<float>* dest) const;
  Status initialize_egl_context();
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
