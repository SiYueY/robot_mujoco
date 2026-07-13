#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "mujoco_simulation/component/camera/camera_renderer_config.hpp"
#include "mujoco_simulation/component/component_config.hpp"

namespace mujoco_simulation {

enum class RenderMode {
  Headless,
  Viewer,
};

struct ModelConfig {
  std::string model_path;
  std::string initial_keyframe;
};

struct SchedulerConfig {
  bool realtime_sync{true};
  double realtime_factor{1.0};
  double state_update_rate{1000.0};
  double viewer_update_rate{60.0};
  std::chrono::milliseconds max_schedule_lag{100};
};

struct SimulationConfig {
  ModelConfig model;
  SchedulerConfig scheduler;
  ComponentConfigList components;
  std::chrono::milliseconds viewer_startup_timeout{5000};
  CameraRendererConfig camera_renderer;
  RenderMode render_mode = RenderMode::Headless;
};

RenderMode parse_render_mode(const std::string& value);
const char* to_string(RenderMode mode);

}  // namespace mujoco_simulation
