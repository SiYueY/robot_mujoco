#pragma once

struct GLFWwindow;

namespace romujoco {

enum class OffscreenGlBackend {
    None,
    Glfw,
    Egl,
};

struct OffscreenGlContext {
    OffscreenGlBackend backend{OffscreenGlBackend::None};
    GLFWwindow* window{nullptr};
    void* egl_display{nullptr};
    void* egl_context{nullptr};
    void* egl_surface{nullptr};
};

}  // namespace romujoco
