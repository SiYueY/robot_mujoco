#include "ros2_mujoco/hardware/mobile_base.hpp"
namespace ros2_mujoco::hardware {
bool MobileBase::initialize(std::size_t id) {
    id_ = id;
    return true;
}
void MobileBase::set_velocity(const geometry_msgs::msg::Twist& v) { velocity_ = v; }
mujoco_simulation::MobileBaseCommand MobileBase::command() const {
    mujoco_simulation::MobileBaseCommand c;
    c.id = id_;
    c.mode = mujoco_simulation::MobileBaseControlMode::Twist;
    c.base_linear[0] = velocity_.linear.x;
    c.base_linear[1] = velocity_.linear.y;
    c.base_angular[2] = velocity_.angular.z;
    return c;
}
}  // namespace ros2_mujoco::hardware
