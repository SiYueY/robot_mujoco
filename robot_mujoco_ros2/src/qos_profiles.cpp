#include "robot_mujoco_ros2/qos_profiles.hpp"

#include "rmw/types.h"

namespace robot_mujoco_ros2::qos_profiles {

rclcpp::QoS clock() {
  return rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

rclcpp::QoS sensor_data() {
  return rclcpp::QoS(rclcpp::KeepLast(1))
      .best_effort()
      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
}

}  // namespace robot_mujoco_ros2::qos_profiles
