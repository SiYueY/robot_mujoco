#pragma once

struct GLFWwindow;

using CameraRendererEglContextHandle = void*;
using CameraRendererEglDisplayHandle = void*;
using CameraRendererEglSurfaceHandle = void*;

namespace mujoco_simulation {

enum class OffscreenGlBackend {
  None,
  Glfw,
  Egl,
};

struct OffscreenGlContext {
  OffscreenGlBackend backend{OffscreenGlBackend::None};
  GLFWwindow* window{nullptr};
  CameraRendererEglDisplayHandle egl_display{nullptr};
  CameraRendererEglContextHandle egl_context{nullptr};
  CameraRendererEglSurfaceHandle egl_surface{nullptr};
};

}  // namespace mujoco_simulation
