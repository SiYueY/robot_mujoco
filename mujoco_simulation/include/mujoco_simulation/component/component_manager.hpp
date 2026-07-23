#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/buffer/camera_buffer.hpp"
#include "mujoco_simulation/component/camera/camera_component.hpp"
#include "mujoco_simulation/component/component_registry.hpp"
#include "mujoco_simulation/component/imu/imu_component.hpp"
#include "mujoco_simulation/component/joint/joint_component.hpp"
#include "mujoco_simulation/component/lidar/lidar_component.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"
#include "mujoco_simulation/config/simulation_config.hpp"
#include "mujoco_simulation/data/robot_command.hpp"
#include "mujoco_simulation/data/robot_state.hpp"

namespace mujoco_simulation {

class ComponentManager {
 public:
  bool init(const mjContext& context, const ComponentConfigList& components);
  void clear();

  bool reset(const mjContext& context);
  bool update(const mjContext& context, CameraRenderer* camera_renderer,
              CameraBuffer* camera_buffer);
  bool write_command(const mjContext& context, const RobotCommand& snapshot);
  bool read_state(const mjContext& context, RobotState& snapshot) const;
  bool read_state(const mjContext& context, JointStates& states) const;
  bool read_state(const mjContext& context, ImuStates& states) const;
  bool read_state(const mjContext& context, LidarStates& states) const;
  bool read_state(const mjContext& context, MobileBaseStates& states) const;

 private:
  bool register_component(const mjContext& context, JointComponent::UniquePtr joint);
  bool register_component(const mjContext& context, CameraComponent::UniquePtr camera);
  bool register_component(const mjContext& context, ImuComponent::UniquePtr imu);
  bool register_component(const mjContext& context, LidarComponent::UniquePtr lidar);
  bool register_component(const mjContext& context, MobileBaseComponent::UniquePtr mobile_base);

  ComponentRegistry component_registry;
};

}  // namespace mujoco_simulation
