#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"

namespace mujoco_simulation {

template <typename State> using StateSnapshot = std::shared_ptr<const State>;

template <typename State>
using StateSnapshotMap = std::unordered_map<std::string, StateSnapshot<State>>;

template <typename State>
using StateSnapshots = std::shared_ptr<const StateSnapshotMap<State>>;

using JointStateMap = StateSnapshotMap<JointState>;
using MobileBaseStateMap = StateSnapshotMap<MobileBaseState>;
using ImuStateMap = StateSnapshotMap<ImuState>;
using LidarStateMap = StateSnapshotMap<LidarState>;
using CameraStateMap = StateSnapshotMap<CameraState>;

using JointStates = StateSnapshots<JointState>;
using MobileBaseStates = StateSnapshots<MobileBaseState>;
using ImuStates = StateSnapshots<ImuState>;
using LidarStates = StateSnapshots<LidarState>;
using CameraStates = StateSnapshots<CameraState>;

struct RobotState {
  std::uint64_t sequence{0};
  std::uint64_t timestamp{0};
  double simulation_time{0.0};
  std::uint64_t step{0};

  JointStates joints;
  MobileBaseStates mobile_bases;
  ImuStates imus;
  LidarStates lidars;
  CameraStates cameras;

  bool read_state(std::string name, JointState &state) const {
    return find(joints, name, state);
  }
  bool read_state(std::string name, MobileBaseState &state) const {
    return find(mobile_bases, name, state);
  }
  bool read_state(std::string name, ImuState &state) const {
    return find(imus, name, state);
  }
  bool read_state(std::string name, LidarState &state) const {
    return find(lidars, name, state);
  }
  bool read_state(std::string name, CameraState &state) const {
    return find(cameras, name, state);
  }

private:
  template <typename State>
  static bool find(const StateSnapshots<State> &states, const std::string &name,
                   State &state) {
    if (states == nullptr) {
      return false;
    }
    const auto it = states->find(name);
    if (it == states->end() || it->second == nullptr) {
      return false;
    }
    state = *it->second;
    return true;
  }
};

} // namespace mujoco_simulation
