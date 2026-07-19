#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_set>
#include <variant>
#include <vector>

#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/camera/camera_renderer_config.hpp"
#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"
#include "tinyxml2.h"

namespace mujoco_simulation {

enum class RenderMode {
  Headless,
  Viewer,
};

using ComponentConfig = std::variant<JointInfo, ImuInfo, CameraConfig, LidarInfo, MobileBaseInfo>;
using ComponentConfigList = std::vector<ComponentConfig>;

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

class SimulationConfigParser {
 public:
  bool load_file(const std::string& path, SimulationConfig* config) const;

 private:
  static std::string trim_copy(const std::string& value);
  static std::string element_text(const tinyxml2::XMLElement& element);
  static bool parse_required_double(const tinyxml2::XMLElement& element, double* out);
  static bool reject_unknown_children(const tinyxml2::XMLElement& element,
                                      const std::unordered_set<std::string>& allowed_names);
  static bool parse_limit_axis(const tinyxml2::XMLElement* element, Limit* limits);
  static bool parse_position_config(const tinyxml2::XMLElement* element, JointInfo* info);
  static bool parse_velocity_config(const tinyxml2::XMLElement* element, JointInfo* info);
  static bool parse_joint_config(const tinyxml2::XMLElement& element, JointInfo* info);
  static bool parse_robot_section(const tinyxml2::XMLElement* robot,
                                  ComponentConfigList* components);
  static std::optional<std::filesystem::path> resolve_model_path(
      const std::filesystem::path& config_path, const std::string& mjcf_path);
};

RenderMode parse_render_mode(const std::string& value);
const char* to_string(RenderMode mode);

}  // namespace mujoco_simulation
