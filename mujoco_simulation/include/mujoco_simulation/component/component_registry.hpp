#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include "mujoco_simulation/component/camera/camera_component.hpp"
#include "mujoco_simulation/component/imu/imu_component.hpp"
#include "mujoco_simulation/component/joint/joint_component.hpp"
#include "mujoco_simulation/component/lidar/lidar_component.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class ComponentRegistry {
 public:
  Status add(std::unique_ptr<SimulationComponent> component);
  Status remove(std::string_view name);
  void clear();

  bool has_joint(std::string_view name) const;
  bool has_camera(std::string_view name) const;
  bool has_imu(std::string_view name) const;
  bool has_lidar(std::string_view name) const;
  bool has_mobile_base(std::string_view name) const;

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

  const std::unordered_map<std::string, JointComponent*>& joints() const noexcept;
  const std::unordered_map<std::string, CameraComponent*>& cameras() const noexcept;
  const std::unordered_map<std::string, ImuComponent*>& imus() const noexcept;
  const std::unordered_map<std::string, LidarComponent*>& lidars() const noexcept;
  const std::unordered_map<std::string, MobileBaseComponent*>& mobile_bases() const noexcept;

  SimulationComponent* find(std::string_view name);
  const SimulationComponent* find(std::string_view name) const;

 private:
  void index_component(SimulationComponent& component, const std::string& component_name);
  void unindex_component(const SimulationComponent& component, std::string_view component_name);

 private:
  std::unordered_map<std::string, std::unique_ptr<SimulationComponent>> components_;
  std::unordered_map<std::string, JointComponent*> joints_;
  std::unordered_map<std::string, CameraComponent*> cameras_;
  std::unordered_map<std::string, ImuComponent*> imus_;
  std::unordered_map<std::string, LidarComponent*> lidars_;
  std::unordered_map<std::string, MobileBaseComponent*> mobile_bases_;
};

}  // namespace mujoco_simulation
