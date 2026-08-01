#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/component_id.hpp"
#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"
#include "mujoco_simulation/config/config_limits.hpp"
#include "mujoco_simulation/mujoco/camera_renderer.hpp"
#include "mujoco_simulation/visibility.hpp"

namespace mujoco_simulation {

using ComponentConfig =
    std::variant<JointInfo, ImuInfo, CameraConfig, LidarInfo, MobileBaseInfo>;
using ComponentConfigList = std::vector<ComponentConfig>;

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

struct ConfigError {
  static constexpr std::size_t kNoComponent = static_cast<std::size_t>(-1);
  int line{0};
  std::size_t component_index{kNoComponent};
  std::string element;
  std::string attribute;
  std::string message;
};

class MUJOCO_SIMULATION_PUBLIC SimulationConfigValidator {
public:
  static bool validate(const SimulationConfig &config,
                       ConfigError *error = nullptr);
};

class MUJOCO_SIMULATION_PUBLIC SimulationConfigParser {
public:
  bool load_file(const std::string &path, SimulationConfig &config,
                 ConfigError *error = nullptr) const;
};

} // namespace mujoco_simulation
