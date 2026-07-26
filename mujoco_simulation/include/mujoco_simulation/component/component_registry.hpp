#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/component/camera/camera_component.hpp"
#include "mujoco_simulation/component/imu/imu_component.hpp"
#include "mujoco_simulation/component/joint/joint_component.hpp"
#include "mujoco_simulation/component/lidar/lidar_component.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"
#include "mujoco_simulation/visibility.hpp"

namespace mujoco_simulation {

class MUJOCO_SIMULATION_PUBLIC ComponentRegistry {
public:
  bool add(std::unique_ptr<SimulationComponent> component);
  void clear();

  bool has_joint(std::string name) const;
  bool has_camera(std::string name) const;
  bool has_imu(std::string name) const;
  bool has_lidar(std::string name) const;
  bool has_mobile_base(std::string name) const;

  JointComponent *joint(std::string name);
  const JointComponent *joint(std::string name) const;
  CameraComponent *camera(std::string name);
  const CameraComponent *camera(std::string name) const;
  ImuComponent *imu(std::string name);
  const ImuComponent *imu(std::string name) const;
  LidarComponent *lidar(std::string name);
  const LidarComponent *lidar(std::string name) const;
  MobileBaseComponent *mobile_base(std::string name);
  const MobileBaseComponent *mobile_base(std::string name) const;

  const std::unordered_map<std::string, JointComponent *> &
  joints() const noexcept;
  const std::unordered_map<std::string, CameraComponent *> &
  cameras() const noexcept;
  const std::unordered_map<std::string, ImuComponent *> &imus() const noexcept;
  const std::unordered_map<std::string, LidarComponent *> &
  lidars() const noexcept;
  const std::unordered_map<std::string, MobileBaseComponent *> &
  mobile_bases() const noexcept;

  SimulationComponent *find(std::string name);
  const SimulationComponent *find(std::string name) const;

private:
  void index_component(SimulationComponent &component,
                       const std::string &component_name);

private:
  std::unordered_map<std::string, std::unique_ptr<SimulationComponent>>
      components_;
  std::unordered_map<std::string, JointComponent *> joints_;
  std::unordered_map<std::string, CameraComponent *> cameras_;
  std::unordered_map<std::string, ImuComponent *> imus_;
  std::unordered_map<std::string, LidarComponent *> lidars_;
  std::unordered_map<std::string, MobileBaseComponent *> mobile_bases_;
};

} // namespace mujoco_simulation
