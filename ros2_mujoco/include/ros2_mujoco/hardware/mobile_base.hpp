#pragma once
#include <cstddef>
#include "geometry_msgs/msg/twist.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"
namespace ros2_mujoco::hardware {
class MobileBase {
public:
    bool initialize(std::size_t id);
    void set_velocity(const geometry_msgs::msg::Twist& value);
    mujoco_simulation::MobileBaseCommand command() const;
    void update(const mujoco_simulation::MobileBaseState& state) { state_ = state; }
    const mujoco_simulation::MobileBaseState& state() const { return state_; }

private:
    std::size_t id_{};
    geometry_msgs::msg::Twist velocity_;
    mujoco_simulation::MobileBaseState state_;
};
}  // namespace ros2_mujoco::hardware
