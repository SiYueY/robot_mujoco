#pragma once

namespace mujoco_simulation {

struct CameraRendererConfig {
  int max_scene_geometries{2000};
  bool allow_glfw_backend{true};
  bool allow_egl_backend{true};
};

}  // namespace mujoco_simulation
