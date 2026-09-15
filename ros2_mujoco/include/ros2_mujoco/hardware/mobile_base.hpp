#pragma once
#include <cstddef>
#include "geometry_msgs/msg/twist.hpp"
#include "romujoco/component/mobile_base.hpp"
namespace ros2_mujoco::hardware {
class MobileBase {
public:
    bool initialize(std::size_t id);
    void set_velocity(const geometry_msgs::msg::Twist& value);
    romujoco::MobileBaseCommand command() const;
    void update(const romujoco::MobileBaseState& state) { state_ = state; }
    const romujoco::MobileBaseState& state() const { return state_; }

private:
    std::size_t id_{};
    geometry_msgs::msg::Twist velocity_;
    romujoco::MobileBaseState state_;
};
}  // namespace ros2_mujoco::hardware
