#pragma once
#include <cstddef>
#include <string>
#include "mujoco_simulation/component/joint.hpp"
namespace ros2_mujoco::hardware {
class Joint {
public:
    bool initialize(std::size_t id, std::string name);
    void update(const mujoco_simulation::JointState& state);
    mujoco_simulation::JointCommand command() const;
    const std::string& name() const { return name_; }
    double& position_state() { return position_; }
    double& velocity_state() { return velocity_; }
    double& effort_state() { return effort_; }
    double& position_command() { return position_command_; }
    double& velocity_command() { return velocity_command_; }
    double& effort_command() { return effort_command_; }
    void set_mode(mujoco_simulation::JointMode mode) { mode_ = mode; }

private:
    std::size_t id_{};
    std::string name_;
    mujoco_simulation::JointMode mode_{mujoco_simulation::JointMode::Position};
    double position_{}, velocity_{}, effort_{}, position_command_{}, velocity_command_{},
        effort_command_{};
};
}  // namespace ros2_mujoco::hardware
