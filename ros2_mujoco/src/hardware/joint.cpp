#include "ros2_mujoco/hardware/joint.hpp"
namespace ros2_mujoco::hardware {
bool Joint::initialize(std::size_t id, std::string name) {
    id_ = id;
    name_ = std::move(name);
    return !name_.empty();
}
void Joint::update(const mujoco_simulation::JointState& s) {
    position_ = s.position;
    velocity_ = s.velocity;
    effort_ = s.effort;
}
mujoco_simulation::JointCommand Joint::command() const {
    mujoco_simulation::JointCommand c;
    c.id = id_;
    c.mode = static_cast<std::uint8_t>(mode_);
    c.position = position_command_;
    c.velocity = velocity_command_;
    c.effort = effort_command_;
    return c;
}
}  // namespace ros2_mujoco::hardware
