#pragma once

#include <string>
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

}  // namespace mujoco_simulation
