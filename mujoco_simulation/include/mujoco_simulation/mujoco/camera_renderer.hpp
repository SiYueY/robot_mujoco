#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/mujoco/context.hpp"

struct GLFWwindow;

namespace mujoco_simulation {

/// 共享的 MuJoCo 离屏相机渲染器配置。
struct CameraRendererConfig {
  /// MuJoCo 渲染场景可容纳的最大几何体数量。
  int max_scene_geometries{2000};
  /// 可用时优先使用隐藏的 GLFW OpenGL context。
  bool allow_glfw_backend{true};
  /// GLFW 无法创建 OpenGL context 时允许回退至 EGL。
  bool allow_egl_backend{true};
};

/// CameraRenderer 使用的内部离屏 OpenGL 后端。
enum class OffscreenGlBackend {
  None,
  Glfw,
  Egl,
};

/// 已选离屏 OpenGL context 的内部资源记录。
struct OffscreenGlContext {
  OffscreenGlBackend backend{OffscreenGlBackend::None};
  GLFWwindow* window{nullptr};
  void* egl_display{nullptr};
  void* egl_context{nullptr};
  void* egl_surface{nullptr};
};

/// CameraRenderer 输出的紧凑 RGB8 图像。
struct CameraRenderImage {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t step{0};
  std::vector<std::uint8_t> data;
};

/// CameraRenderer 输出的、单位为米的深度图像。
struct CameraRenderDepthImage {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<float> data;
};

/// 由 MuJoCo 相机视场角推导出的针孔相机内参。
struct CameraRenderIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  Vector9d k{};
  Vector12d p{};
};

/// 单次相机渲染生成的不可变结果。
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

  /// 创建 MuJoCo 和 OpenGL 资源；成功后重复调用不执行额外操作。
  bool initialize(const mjContext& context);
  /// 释放全部资源；未初始化时重复调用不执行额外操作。
  bool release();

  /// 复制后续渲染所使用的仿真状态。
  bool copy_simulation_data(const mjContext& context);
  /// 渲染一台相机；失败时不修改 out。
  bool render(const mjContext& context, const CameraConfig& spec, std::uint64_t sequence,
              std::uint64_t timestamp, std::shared_ptr<const CameraRenderState>& out);

  bool is_initialized() const noexcept { return initialized_; }

 private:
  bool create_context();
  bool create_glfw_context();
  bool create_egl_context();
  void destroy_context();
  void destroy_glfw_context();
  void destroy_egl_context();

  bool activate_context();
  void deactivate_context();

  bool create_render_resources(const mjContext& context);
  void destroy_render_resources();
  bool resize_offscreen_buffer(int width, int height);

  CameraRenderIntrinsics compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                            std::uint32_t height) const;
  void transform_rgb(const std::vector<std::uint8_t>& source, std::uint32_t width,
                     std::uint32_t height, std::vector<std::uint8_t>& dest) const;
  bool transform_depth(const mjContext& context, const std::vector<float>& source,
                       std::uint32_t width, std::uint32_t height, std::vector<float>& dest) const;
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
