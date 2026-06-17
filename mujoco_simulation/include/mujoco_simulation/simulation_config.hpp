#pragma once

#include <string>

#include "mujoco_simulation/component/camera/camera_renderer_config.hpp"
#include "mujoco_simulation/component/component_config.hpp"
#include "mujoco_simulation/runtime/model_runtime.hpp"
#include "mujoco_simulation/runtime/scheduler_config.hpp"
#include "mujoco_simulation/viewer/viewer_config.hpp"

namespace mujoco_simulation {

enum class RenderMode {
  Headless,
  Viewer,
};

struct SimulationConfig {
  ModelConfig model;
  SchedulerConfig scheduler;
  ComponentConfigList components;
  ViewerConfig viewer;
  CameraRendererConfig camera_renderer;
  RenderMode render_mode = RenderMode::Headless;
};

RenderMode parse_render_mode(const std::string& value);
const char* to_string(RenderMode mode);

}  // namespace mujoco_simulation
