#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"

namespace mujoco_simulation {

struct StateSnapshot {
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  double simulation_time{0.0};
  std::uint64_t step_count{0};
  std::unordered_map<std::string, JointState> joints;
  std::unordered_map<std::string, MobileBaseState> mobile_bases;
  std::unordered_map<std::string, ImuState> imus;
  std::unordered_map<std::string, LidarState> lidars;

  bool joint_state(std::string name, JointState* out) const { return lookup(joints, name, out); }
  bool mobile_base_state(std::string name, MobileBaseState* out) const {
    return lookup(mobile_bases, name, out);
  }
  bool imu_state(std::string name, ImuState* out) const { return lookup(imus, name, out); }
  bool lidar_state(std::string name, LidarState* out) const { return lookup(lidars, name, out); }

 private:
  template <typename T>
  bool lookup(const std::unordered_map<std::string, T>& values, std::string name, T* out) const {
    if (out == nullptr) {
      return false;
    }
    const auto it = values.find(std::string(name));
    if (it == values.end()) {
      return false;
    }
    *out = it->second;
    return true;
  }
};

}  // namespace mujoco_simulation
