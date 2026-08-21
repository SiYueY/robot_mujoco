#include "ros2_mujoco/hardware/camera.hpp"

#include <algorithm>

namespace ros2_mujoco::hardware {
sensor_msgs::msg::Image Camera::image(const mujoco_simulation::Image& value, double time) const {
    sensor_msgs::msg::Image message;
    message.header.frame_id = value.frame_id;
    message.header.stamp.sec = static_cast<int32_t>(time);
    message.header.stamp.nanosec = static_cast<uint32_t>((time - message.header.stamp.sec) * 1e9);
    message.height = value.height;
    message.width = value.width;
    message.encoding = value.encoding;
    message.is_bigendian = value.is_bigendian;
    message.step = value.step;
    message.data = value.data;
    return message;
}
sensor_msgs::msg::CameraInfo Camera::info(const mujoco_simulation::CameraState& value) const {
    sensor_msgs::msg::CameraInfo message;
    message.header.frame_id = value.optical_frame_id;
    message.header.stamp.sec = static_cast<int32_t>(value.timestamp / 1000000000ULL);
    message.header.stamp.nanosec = value.timestamp % 1000000000ULL;
    message.height = value.camera_info.height;
    message.width = value.camera_info.width;
    message.distortion_model = value.camera_info.distortion_model;
    message.d = value.camera_info.d;
    std::copy(value.camera_info.k.begin(), value.camera_info.k.end(), message.k.begin());
    std::copy(value.camera_info.r.begin(), value.camera_info.r.end(), message.r.begin());
    std::copy(value.camera_info.p.begin(), value.camera_info.p.end(), message.p.begin());
    return message;
}
}  // namespace ros2_mujoco::hardware
