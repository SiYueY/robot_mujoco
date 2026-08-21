#pragma once
#include "mujoco_simulation/component/camera.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
namespace ros2_mujoco::hardware {
class Camera {
public:
    sensor_msgs::msg::Image image(const mujoco_simulation::Image& value, double time) const;
    sensor_msgs::msg::CameraInfo info(const mujoco_simulation::CameraState& value) const;
};
}  // namespace ros2_mujoco::hardware
