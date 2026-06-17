#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include "mujoco_simulation/buffer/camera_buffer.hpp"
#include "mujoco_simulation/buffer/command_snapshot.hpp"
#include "mujoco_simulation/buffer/simulation_state_snapshot.hpp"
#include "mujoco_simulation/component/camera/camera_component.hpp"
#include "mujoco_simulation/component/component_config.hpp"
#include "mujoco_simulation/component/component_registry.hpp"
#include "mujoco_simulation/component/imu/imu_component.hpp"
#include "mujoco_simulation/component/joint/joint_component.hpp"
#include "mujoco_simulation/component/lidar/lidar_component.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"
#include "mujoco_simulation/scheduler/sensor_scheduler.hpp"

namespace mujoco_simulation {

class ComponentManager {
 public:
  Status build(const mjModel& model, const ComponentConfigList& components);
  void clear();

  Status register_joint(const mjModel& model, std::unique_ptr<JointComponent> joint);
  Status register_camera(const mjModel& model, std::unique_ptr<CameraComponent> camera);
  Status register_imu(const mjModel& model, std::unique_ptr<ImuComponent> imu);
  Status register_lidar(const mjModel& model, std::unique_ptr<LidarComponent> lidar);
  Status register_mobile_base(const mjModel& model, const MobileBaseConfig& config);
  Status reconfigure_component(const mjModel& model, const ComponentConfig& config);
  Status reconfigure_joint(const mjModel& model, const JointConfig& config);
  Status unregister_joint(std::string_view name);
  Status unregister_camera(std::string_view name);
  Status unregister_imu(std::string_view name);
  Status unregister_lidar(std::string_view name);
  Status unregister_mobile_base(std::string_view name);

  bool has_joint(std::string_view name) const;
  bool has_camera(std::string_view name) const;
  bool has_imu(std::string_view name) const;
  bool has_lidar(std::string_view name) const;
  bool has_mobile_base(std::string_view name) const;
  CommandInterfaceType joint_command_mode(std::string_view name) const;
  JointComponent* joint(std::string_view name);
  const JointComponent* joint(std::string_view name) const;
  CameraComponent* camera(std::string_view name);
  const CameraComponent* camera(std::string_view name) const;
  ImuComponent* imu(std::string_view name);
  const ImuComponent* imu(std::string_view name) const;
  LidarComponent* lidar(std::string_view name);
  const LidarComponent* lidar(std::string_view name) const;
  MobileBaseComponent* mobile_base(std::string_view name);
  const MobileBaseComponent* mobile_base(std::string_view name) const;

  Status reset_all(const mjModel& model, mjData& data);
  Status sample_due_sensors(const mjModel& model, const mjData& data, double simulation_time,
                            std::uint64_t step_count, CameraRenderer* camera_renderer,
                            CameraBuffer* camera_buffer);
  Status write_commands(const mjModel& model, mjData& data, const CommandSnapshot& snapshot);
  Status read_joint(const mjData& data, std::string_view name, JointState& state) const;
  Status read_imu(std::string_view name, ImuSample& state) const;
  Status read_lidar(std::string_view name, LidarSample& state) const;
  Status read_mobile_base(const mjData& data, std::string_view name, MobileBaseState& state);
  Status read_states(const mjData& data, SimulationStateSnapshot& snapshot);
  Status read_joint_states(const mjData& data,
                           std::unordered_map<std::string, JointState>& states) const;
  Status read_imu_states(std::unordered_map<std::string, ImuSample>& states) const;
  Status read_lidar_states(std::unordered_map<std::string, LidarSample>& states) const;
  Status read_mobile_base_states(const mjData& data,
                                 std::unordered_map<std::string, MobileBaseState>& states);

 private:
  ComponentRegistry registry_;
  SensorScheduler sensor_scheduler_;
};

}  // namespace mujoco_simulation
