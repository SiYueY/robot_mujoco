#include "ros2_mujoco/hardware/imu.hpp"

#include <algorithm>

namespace ros2_mujoco::hardware {
sensor_msgs::msg::Imu Imu::data(const mujoco_simulation::ImuState& state) const {
    sensor_msgs::msg::Imu message;
    message.header.frame_id = state.frame_id;
    message.header.stamp.sec = static_cast<int32_t>(state.timestamp);
    message.header.stamp.nanosec =
        static_cast<uint32_t>((state.timestamp - message.header.stamp.sec) * 1e9);
    message.orientation.x = state.orientation[0];
    message.orientation.y = state.orientation[1];
    message.orientation.z = state.orientation[2];
    message.orientation.w = state.orientation[3];
    message.angular_velocity.x = state.angular_velocity[0];
    message.angular_velocity.y = state.angular_velocity[1];
    message.angular_velocity.z = state.angular_velocity[2];
    message.linear_acceleration.x = state.linear_acceleration[0];
    message.linear_acceleration.y = state.linear_acceleration[1];
    message.linear_acceleration.z = state.linear_acceleration[2];
    std::copy(
        state.orientation_covariance.begin(), state.orientation_covariance.end(),
        message.orientation_covariance.begin());
    std::copy(
        state.angular_velocity_covariance.begin(), state.angular_velocity_covariance.end(),
        message.angular_velocity_covariance.begin());
    std::copy(
        state.linear_acceleration_covariance.begin(), state.linear_acceleration_covariance.end(),
        message.linear_acceleration_covariance.begin());
    return message;
}
}  // namespace ros2_mujoco::hardware
