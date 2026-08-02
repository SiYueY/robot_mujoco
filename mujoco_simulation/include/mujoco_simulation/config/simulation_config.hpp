#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "mujoco_simulation/component/camera.hpp"
#include "mujoco_simulation/component/component_id.hpp"
#include "mujoco_simulation/component/imu.hpp"
#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/lidar.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"
#include "mujoco_simulation/config/config_limits.hpp"

namespace mujoco_simulation {

using ComponentConfig =
    std::variant<JointInfo, ImuInfo, CameraConfig, LidarInfo, MobileBaseInfo>;
using ComponentConfigList = std::vector<ComponentConfig>;

// Configuration contract for the always-built internal camera renderer.  It
// intentionally describes policy only and does not expose any GL or MuJoCo
// rendering type.
struct CameraRendererConfig {
  int max_scene_geometries{2000};
  bool allow_glfw_backend{true};
  bool allow_egl_backend{true};
  ComponentId max_camera_id{SimulationConfigLimits::kMaximumComponentId};
  std::size_t completed_ticket_history{8};
};

struct ModelConfig {
  std::string model_path;
  std::string initial_keyframe;
};

struct SchedulerConfig {
  double physics_period{0.001};
  double viewer_period{1.0 / 60.0};
};

struct SimulationConfig {
  ModelConfig model;
  SchedulerConfig scheduler;
  ComponentConfigList components;
  ComponentId max_component_id{256};
  // Disables GUI viewer creation while leaving camera rendering available.
  bool viewer_enabled{true};
  std::chrono::milliseconds viewer_startup_timeout{5000};
  CameraRendererConfig camera_renderer;
};

} // namespace mujoco_simulation
