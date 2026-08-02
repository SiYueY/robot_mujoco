#pragma once
// Internal resources for the CameraRenderer-owned offscreen context.

struct GLFWwindow;

namespace mujoco_simulation {

enum class OffscreenGlBackend {
  None,
  Glfw,
  Egl,
};

struct OffscreenGlContext {
  OffscreenGlBackend backend{OffscreenGlBackend::None};
  GLFWwindow *window{nullptr};
  void *egl_display{nullptr};
  void *egl_context{nullptr};
  void *egl_surface{nullptr};
};

} // namespace mujoco_simulation
