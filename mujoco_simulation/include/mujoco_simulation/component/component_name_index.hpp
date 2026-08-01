#pragma once

#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>

#include "mujoco_simulation/config/simulation_config.hpp"

namespace mujoco_simulation {

// Immutable lookup table published as one shared snapshot.  Simulation never
// mutates an index visible to readers; lifecycle transitions replace it as a
// whole, so convenience name APIs can safely run beside shutdown/reinitialize.
class ComponentNameIndex {
public:
  static ComponentNameIndex build(const ComponentConfigList &components) {
    ComponentNameIndex index;
    for (const ComponentConfig &component : components) {
      std::visit(
          [&index](const auto &info) {
            using Info = std::decay_t<decltype(info)>;
            if constexpr (std::is_same_v<Info, JointInfo>)
              index.joints_.emplace(info.joint_name, info.id);
            else if constexpr (std::is_same_v<Info, ImuInfo>)
              index.imus_.emplace(info.name, info.id);
            else if constexpr (std::is_same_v<Info, CameraConfig>)
              index.cameras_.emplace(info.name, info.id);
            else if constexpr (std::is_same_v<Info, LidarInfo>)
              index.lidars_.emplace(info.name, info.id);
            else
              index.mobile_bases_.emplace(info.mobile_base_name, info.id);
          },
          component);
    }
    return index;
  }

  std::optional<JointId> joint_id(const std::string &name) const {
    return find(joints_, name);
  }
  std::optional<ImuId> imu_id(const std::string &name) const {
    return find(imus_, name);
  }
  std::optional<CameraId> camera_id(const std::string &name) const {
    return find(cameras_, name);
  }
  std::optional<LidarId> lidar_id(const std::string &name) const {
    return find(lidars_, name);
  }
  std::optional<MobileBaseId> mobile_base_id(const std::string &name) const {
    return find(mobile_bases_, name);
  }

private:
  static std::optional<ComponentId>
  find(const std::unordered_map<std::string, ComponentId> &index,
       const std::string &name) {
    const auto found = index.find(name);
    return found == index.end() ? std::nullopt
                                : std::optional<ComponentId>{found->second};
  }

  std::unordered_map<std::string, JointId> joints_;
  std::unordered_map<std::string, ImuId> imus_;
  std::unordered_map<std::string, CameraId> cameras_;
  std::unordered_map<std::string, LidarId> lidars_;
  std::unordered_map<std::string, MobileBaseId> mobile_bases_;
};

} // namespace mujoco_simulation
