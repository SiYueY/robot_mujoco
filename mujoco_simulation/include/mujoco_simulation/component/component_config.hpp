#pragma once

#include <string>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

#include "mujoco_simulation/component/camera/camera_config.hpp"
#include "mujoco_simulation/component/imu/imu_config.hpp"
#include "mujoco_simulation/component/joint/joint_config.hpp"
#include "mujoco_simulation/component/lidar/lidar_config.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_config.hpp"

namespace mujoco_simulation {

using ComponentConfig =
    std::variant<JointConfig, ImuConfig, CameraConfig, LidarConfig, MobileBaseConfig>;
using ComponentConfigList = std::vector<ComponentConfig>;

inline std::string_view component_config_name(const ComponentConfig& component) {
  return std::visit(
      [](const auto& value) -> std::string_view {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, JointConfig> || std::is_same_v<T, MobileBaseConfig>) {
          return value.name;
        } else {
          return value.common.name;
        }
      },
      component);
}

inline bool replace_component_config(ComponentConfigList& components,
                                     const ComponentConfig& updated) {
  const std::size_t updated_index = updated.index();
  const std::string_view updated_name = component_config_name(updated);
  for (ComponentConfig& component : components) {
    if (component.index() == updated_index && component_config_name(component) == updated_name) {
      component = updated;
      return true;
    }
  }
  return false;
}

inline bool replace_joint_config(ComponentConfigList& components, const JointConfig& updated) {
  return replace_component_config(components, ComponentConfig{updated});
}

}  // namespace mujoco_simulation
