#pragma once
// Internal state buffering contract.

#include <memory>

#include "mujoco_simulation/data/robot_state.hpp"

namespace mujoco_simulation {

class StateBuffer {
public:
    void write(std::shared_ptr<const RobotState> snapshot);

    std::shared_ptr<const RobotState> read() const;

    bool read_joint_state(JointId id, JointState& out) const;

    bool read_mobile_base_state(MobileBaseId id, MobileBaseState& out) const;

    bool read_imu_state(ImuId id, ImuState& out) const;

    bool read_camera_state(CameraId id, CameraState& out) const;

    bool read_lidar_state(LidarId id, LidarState& out) const;

    void clear();

private:
    std::shared_ptr<const RobotState> current_;
};

}  // namespace mujoco_simulation
