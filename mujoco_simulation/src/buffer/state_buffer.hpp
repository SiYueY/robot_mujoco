#pragma once
// Internal state buffering contract.

#include <cstddef>
#include <memory>
#include <vector>

#include "component/component_index.hpp"
#include "mujoco_simulation/data/robot_state.hpp"

namespace mujoco_simulation {

class StateBuffer {
public:
    bool configure(std::shared_ptr<const ComponentIndex> component_index);

    bool write(std::shared_ptr<const RobotState> snapshot);

    std::shared_ptr<const RobotState> read() const;

    bool read(JointState& state) const;

    bool read(MobileBaseState& state) const;

    bool read(ImuState& state) const;

    bool read(CameraState& state) const;

    bool read(LidarState& state) const;

    void shutdown();

private:
    std::shared_ptr<const ComponentIndex> component_index_;
    std::shared_ptr<const RobotState> state_;
};

}  // namespace mujoco_simulation
