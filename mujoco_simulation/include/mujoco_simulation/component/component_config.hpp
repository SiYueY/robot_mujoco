#pragma once

#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"

namespace mujoco_simulation {

using ComponentConfig = std::variant<JointInfo, ImuInfo, CameraConfig, LidarInfo, MobileBaseInfo>;
using ComponentConfigList = std::vector<ComponentConfig>;

inline std::string component_config_name(const ComponentConfig& component) {
  return std::visit(
      [](const auto& value) -> std::string {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, JointInfo>) {
          return value.joint;
        } else if constexpr (std::is_same_v<T, MobileBaseInfo>) {
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
  const std::string updated_name = component_config_name(updated);
  for (ComponentConfig& component : components) {
    if (component.index() == updated_index && component_config_name(component) == updated_name) {
      component = updated;
      return true;
    }
  }
  return false;
}

inline bool replace_joint_info(ComponentConfigList& components, const JointInfo& updated) {
  return replace_component_config(components, ComponentConfig{updated});
}

}  // namespace mujoco_simulation
