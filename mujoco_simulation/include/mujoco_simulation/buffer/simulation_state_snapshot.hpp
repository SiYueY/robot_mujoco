#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>

#include "mujoco_simulation/component/imu/imu_sample.hpp"
#include "mujoco_simulation/component/joint/joint_state.hpp"
#include "mujoco_simulation/component/lidar/lidar_sample.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_state.hpp"
#include "mujoco_simulation/result.hpp"

namespace mujoco_simulation {

struct SimulationStateSnapshot {
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  double simulation_time{0.0};
  std::uint64_t step_count{0};
  std::unordered_map<std::string, JointState> joints;
  std::unordered_map<std::string, MobileBaseState> mobile_bases;
  std::unordered_map<std::string, ImuSample> imus;
  std::unordered_map<std::string, LidarSample> lidars;

  Result<JointState> joint_state(std::string_view name) const {
    return lookup(joints, name, "joint state");
  }

  Result<MobileBaseState> mobile_base_state(std::string_view name) const {
    return lookup(mobile_bases, name, "mobile base state");
  }

  Result<ImuSample> imu_sample(std::string_view name) const {
    return lookup(imus, name, "IMU sample");
  }

  Result<LidarSample> lidar_sample(std::string_view name) const {
    return lookup(lidars, name, "lidar sample");
  }

 private:
  template <typename T>
  Result<T> lookup(const std::unordered_map<std::string, T>& values, std::string_view name,
                   std::string_view label) const {
    const auto it = values.find(std::string(name));
    if (it == values.end()) {
      return Status::not_found("MuJoCo " + std::string(label) +
                               " not found in snapshot: " + std::string(name));
    }
    return it->second;
  }
};

}  // namespace mujoco_simulation
