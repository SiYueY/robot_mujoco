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
  ResultCode build(const mjModel& model, const ComponentConfigList& components);
  void clear();

  ResultCode reconfigure_component(const mjModel& model, const ComponentConfig& config);

  CommandInterfaceType joint_command_mode(std::string name) const;

  ResultCode reset_all(const mjModel& model, mjData& data);
  ResultCode update_components(const mjModel& model, const mjData& data, double simulation_time,
                               std::uint64_t step_count, CameraRenderer* camera_renderer,
                               CameraBuffer* camera_buffer);
  ResultCode write_commands(const mjModel& model, mjData& data, const CommandSnapshot& snapshot);
  ResultCode build_state_snapshot(const mjData& data, StateSnapshot& snapshot) const;
  ResultCode read_joint_states(const mjData& data,
                               std::unordered_map<std::string, JointState>& states) const;
  ResultCode read_imu_states(std::unordered_map<std::string, ImuState>& states) const;
  ResultCode read_lidar_states(std::unordered_map<std::string, LidarState>& states) const;
  ResultCode read_mobile_base_states(
      const mjData& data, std::unordered_map<std::string, MobileBaseState>& states) const;

 private:
  ResultCode register_joint(const mjModel& model, std::unique_ptr<JointComponent> joint);
  ResultCode register_camera(const mjModel& model, std::unique_ptr<CameraComponent> camera);
  ResultCode register_imu(const mjModel& model, std::unique_ptr<ImuComponent> imu);
  ResultCode register_lidar(const mjModel& model, std::unique_ptr<LidarComponent> lidar);
  ResultCode register_mobile_base(const mjModel& model, const MobileBaseConfig& config);
  ResultCode reconfigure_joint(const mjModel& model, const JointConfig& config);

  ComponentRegistry registry_;
};

}  // namespace mujoco_simulation
