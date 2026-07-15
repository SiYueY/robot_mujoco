#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/buffer/camera_buffer.hpp"
#include "mujoco_simulation/buffer/command_snapshot.hpp"
#include "mujoco_simulation/buffer/state_snapshot.hpp"
#include "mujoco_simulation/component/camera/camera_component.hpp"
#include "mujoco_simulation/component/component_config.hpp"
#include "mujoco_simulation/component/component_registry.hpp"
#include "mujoco_simulation/component/imu/imu_component.hpp"
#include "mujoco_simulation/component/joint/joint_component.hpp"
#include "mujoco_simulation/component/lidar/lidar_component.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"

namespace mujoco_simulation {

class ComponentManager {
 public:
  bool build(const mjModel& model, const ComponentConfigList& components);
  void clear();

  bool reconfigure_component(const mjModel& model, const ComponentConfig& config);

  bool reset_all(const mjModel& model, mjData& data);
  bool update_components(const mjModel& model, const mjData& data, double simulation_time,
                         std::uint64_t step_count, CameraRenderer* camera_renderer,
                         CameraBuffer* camera_buffer);
  bool write_commands(const mjModel& model, mjData& data, const CommandSnapshot& snapshot);
  bool build_state_snapshot(const mjData& data, StateSnapshot& snapshot) const;
  bool read_joint_states(const mjData& data,
                         std::unordered_map<std::string, JointState>& states) const;
  bool read_imu_states(std::unordered_map<std::string, ImuState>& states) const;
  bool read_lidar_states(std::unordered_map<std::string, LidarState>& states) const;
  bool read_mobile_base_states(const mjData& data,
                               std::unordered_map<std::string, MobileBaseState>& states) const;

 private:
  bool register_joint(const mjModel& model, std::unique_ptr<JointComponent> joint);
  bool register_camera(const mjModel& model, std::unique_ptr<CameraComponent> camera);
  bool register_imu(const mjModel& model, std::unique_ptr<ImuComponent> imu);
  bool register_lidar(const mjModel& model, std::unique_ptr<LidarComponent> lidar);
  bool register_mobile_base(const mjModel& model, const MobileBaseInfo& info);
  bool reconfigure_joint(const mjModel& model, const JointInfo& info);

  ComponentRegistry registry_;
};

}  // namespace mujoco_simulation
