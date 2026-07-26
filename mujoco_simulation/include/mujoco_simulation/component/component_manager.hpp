#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/component/camera/camera_component.hpp"
#include "mujoco_simulation/component/component_registry.hpp"
#include "mujoco_simulation/component/imu/imu_component.hpp"
#include "mujoco_simulation/component/joint/joint_component.hpp"
#include "mujoco_simulation/component/lidar/lidar_component.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"
#include "mujoco_simulation/config/simulation_config.hpp"
#include "mujoco_simulation/data/robot_command.hpp"
#include "mujoco_simulation/data/robot_state.hpp"
#include "mujoco_simulation/visibility.hpp"

namespace mujoco_simulation {

class MUJOCO_SIMULATION_PUBLIC ComponentManager {
public:
  bool init(const mjContext &context, const ComponentConfigList &components,
            CameraRenderer &camera_renderer);
  void clear();

  bool reset(const mjContext &context);
  bool update(const mjContext &context);
  bool write_command(const mjContext &context, const RobotCommand &snapshot);
  bool read_state(const mjContext &context, RobotState &snapshot) const;

private:
  bool register_component(const mjContext &context,
                          JointComponent::UniquePtr joint);
  bool register_component(const mjContext &context,
                          CameraComponent::UniquePtr camera,
                          CameraRenderer &camera_renderer);
  bool register_component(const mjContext &context,
                          ImuComponent::UniquePtr imu);
  bool register_component(const mjContext &context,
                          LidarComponent::UniquePtr lidar);
  bool register_component(const mjContext &context,
                          MobileBaseComponent::UniquePtr mobile_base);

  ComponentRegistry component_registry;
  JointStates joints_;
  MobileBaseStates mobile_bases_;
  ImuStates imus_;
  LidarStates lidars_;
  CameraStates cameras_;
};

} // namespace mujoco_simulation
