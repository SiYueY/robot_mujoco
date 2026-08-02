#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "mujoco_simulation/component/camera.hpp"
#include "mujoco_simulation/component/imu.hpp"
#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/lidar.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"

namespace mujoco_simulation {

template <typename State>
using StateSnapshot = std::shared_ptr<const State>;

template <typename State>
using StateSnapshots = std::shared_ptr<const std::vector<StateSnapshot<State>>>;

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
};

}  // namespace mujoco_simulation
