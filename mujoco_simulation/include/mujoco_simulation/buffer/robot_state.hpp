#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"

namespace mujoco_simulation {

using JointStates = std::unordered_map<std::string, JointState>;
using MobileBaseStates = std::unordered_map<std::string, MobileBaseState>;
using LidarStates = std::unordered_map<std::string, LidarState>;
using ImuStates = std::unordered_map<std::string, ImuState>;

struct RobotState {
  std::uint64_t sequence{0};
  std::uint64_t timestamp{0};
  double simulation_time{0.0};
  std::uint64_t step{0};

  JointStates joints;
  MobileBaseStates mobile_bases;
  ImuStates imus;
  LidarStates lidars;

  bool joint_state(std::string name, JointState* state) const {
    return lookup(joints, name, state);
  }
  bool mobile_base_state(std::string name, MobileBaseState* state) const {
    return lookup(mobile_bases, name, state);
  }
  bool imu_state(std::string name, ImuState* state) const { return lookup(imus, name, state); }
  bool lidar_state(std::string name, LidarState* state) const {
    return lookup(lidars, name, state);
  }

 private:
  template <typename T>
  static bool lookup(const std::unordered_map<std::string, T>& values, const std::string& name,
                     T* state) {
    if (state == nullptr) {
      return false;
    }
    const auto it = values.find(name);
    if (it == values.end()) {
      return false;
    }
    *state = it->second;
    return true;
  }
};

}  // namespace mujoco_simulation
