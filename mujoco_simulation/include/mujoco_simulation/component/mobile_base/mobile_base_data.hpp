#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mujoco_simulation/common/math.hpp"

namespace mujoco_simulation {

enum class MobileBaseType {
  None,
  Differential,
  Omnidirectional,
};

enum class OdometrySource {
  WheelIntegration,
  GroundTruthBodyPose,
};

struct MobileBaseConfig {
  std::string name;
  MobileBaseType type{MobileBaseType::None};
  std::string base_frame_id{"base_link"};
  std::string odom_frame_id{"odom"};

  std::string left_wheel_joint;
  std::string right_wheel_joint;
  std::string front_left_joint;
  std::string front_right_joint;
  std::string rear_left_joint;
  std::string rear_right_joint;

  double wheel_radius{0.0};
  double track_width{0.0};
  double wheel_base{0.0};
  OdometrySource odometry_source{OdometrySource::WheelIntegration};
  std::string base_body_name;
};

struct MobileBaseCommand {
  Vector3d linear;
  Vector3d angular;
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  std::uint64_t timestamp_ns{0};
};

struct MobileBaseState {
  std::string base_frame_id;
  std::string odom_frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  Vector3d linear;
  Vector3d angular;
  std::vector<double> wheel_velocities;
  std::uint64_t timestamp_ns{0};
};

}  // namespace mujoco_simulation
