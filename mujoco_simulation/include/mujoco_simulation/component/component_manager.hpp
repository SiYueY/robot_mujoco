#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mujoco_simulation/component/camera/camera_component.hpp"
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
            ComponentId max_component_id, CameraRenderer &camera_renderer);
  void clear();

  bool reset(const mjContext &context);
  bool update(const mjContext &context);
  bool wait_for_camera_results();
  bool has_cameras() const noexcept;
  void clear_camera_states() noexcept;
  bool write_command(const mjContext &context, const RobotCommand &snapshot);
  bool read_state(const mjContext &context, RobotState &snapshot) const;

private:
  bool consume_camera_results();
  bool submit_due_cameras(const mjContext &context);

  std::vector<JointComponent::UniquePtr> joints_components_;
  std::vector<CameraComponent::UniquePtr> camera_components_;
  std::vector<ImuComponent::UniquePtr> imu_components_;
  std::vector<LidarComponent::UniquePtr> lidar_components_;
  std::vector<MobileBaseComponent::UniquePtr> mobile_base_components_;
  JointStates joints_;
  MobileBaseStates mobile_bases_;
  ImuStates imus_;
  LidarStates lidars_;
  CameraStates cameras_;
  CameraRenderer *camera_renderer_{nullptr};
};

} // namespace mujoco_simulation
