#pragma once

#include "rclcpp/qos.hpp"

namespace robot_mujoco_ros2::qos_profiles {

rclcpp::QoS clock();
rclcpp::QoS sensor_data();

}  // namespace robot_mujoco_ros2::qos_profiles
