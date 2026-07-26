#pragma once

#include <array>
#include <string>

#include "mujoco_simulation/component/component_id.hpp"
#include "mujoco_simulation/component/mobile_base/mecanum.hpp"

namespace mujoco_simulation {

enum class MobileBaseType {
  Mecanum,
};

enum class MobileBaseControlMode {
  Twist,
  WheelLinear,
  WheelAngular,
};

// 轮
struct WheelInfo {
  std::string wheel_name;    // 轮名称
  std::string actuator_name; // 执行器名称
  double damping{0.0};       // 阻尼系数。
};

//  Mecanum 底盘
using MecanumWheelInfo = std::array<WheelInfo, MecanumWheelCount>;

struct MobileBaseInfo {
  MobileBaseId id{kInvalidComponentId};
  std::string mobile_base_name;
  MobileBaseType type{MobileBaseType::Mecanum};

  std::string base_frame_id{"base_link"};
  std::string odom_frame_id{"odom"};
  std::string base_body_name;

  // Mecanum
  MecanumInfo mecanum_info;
  MecanumWheelInfo mecanum_wheels;

  double update_rate{0.0};
};

// ROS2 Twist:
// https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Twist.msg
struct MobileBaseCommand {
  MobileBaseControlMode mode{MobileBaseControlMode::Twist};
  Vector3d base_linear{};
  Vector3d base_angular{};
  Vector4d
      wheel_linear{}; // m/s: front-left, front-right, rear-left, rear-right
  Vector4d
      wheel_angular{}; // rad/s: front-left, front-right, rear-left, rear-right
};

// ROS2 Odometry:
// https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg
struct MobileBaseState {
  double timestamp{0.0}; // seconds
  // Frame
  std::string odom_frame_id; // 里程计坐标系
  std::string base_frame_id; // 底盘坐标系
  // Pose
  Vector3d pose{};
  Vector36d pose_covariance{};
  // Twist
  Vector3d base_linear{};
  Vector3d base_angular{};
  Vector36d twist_covariance{};
  // Wheel
  Vector4d wheel_linear{};
  Vector4d wheel_angular{};
};

} // namespace mujoco_simulation
