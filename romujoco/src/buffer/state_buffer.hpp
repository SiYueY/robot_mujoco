#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "romujoco/data/robot_state.hpp"

#include "component/component_id_resolver.hpp"

namespace romujoco {

class StateBuffer {
public:
    bool configure(std::shared_ptr<const ComponentIdResolver> id_resolver);
    void shutdown();

    std::shared_ptr<const RobotState> read() const;
    bool read(JointState& state) const;
    bool read(JointStates& states) const;
    bool read(GripperState& state) const;
    bool read(GripperStates& states) const;
    bool read(MobileBaseState& state) const;
    bool read(MobileBaseStates& states) const;
    bool read(ImuState& state) const;
    bool read(ImuStates& states) const;
    bool read(CameraState& state) const;
    bool read(CameraStates& states) const;
    bool read(LaserScanState& state) const;
    bool read(LaserScanStates& states) const;
    bool read(PointCloudState& state) const;
    bool read(PointCloudStates& states) const;

    bool write(std::shared_ptr<const RobotState> state);

private:
    bool initialized_{false};
    std::shared_ptr<const RobotState> state_;
    std::shared_ptr<const ComponentIdResolver> id_resolver_;
};

}  // namespace romujoco
