#include "robot_mujoco_ros2/config_builder.hpp"

#include <cmath>

namespace robot_mujoco_ros2 {
namespace {

std::size_t estimate_lidar_sample_count(const LidarData& lidar) {
  if (lidar.config.angle_increment <= 0.0) {
    return 0;
  }
  const double span = lidar.config.angle_max - lidar.config.angle_min;
  if (span < 0.0) {
    return 0;
  }
  return static_cast<std::size_t>(std::floor(span / lidar.config.angle_increment + 0.5)) + 1U;
}

SimulationRosBridgeConfig make_ros_bridge_config(const HardwareConfig& config) {
  SimulationRosBridgeConfig bridge_config;
  bridge_config.imus.reserve(config.imus.size());
  bridge_config.cameras.reserve(config.cameras.size());
  bridge_config.lidars.reserve(config.lidars.size());

  for (const auto& imu : config.imus) {
    bridge_config.imus.push_back({.name = imu.name, .frame_id = imu.frame_id, .topic = imu.topic});
  }
  for (const auto& camera : config.cameras) {
    bridge_config.cameras.push_back({.name = camera.name,
                                     .frame_id = camera.frame_id,
                                     .rgb_topic = camera.rgb_topic,
                                     .depth_topic = camera.depth_topic,
                                     .camera_info_topic = camera.camera_info_topic,
                                     .width = static_cast<std::uint32_t>(camera.config.width),
                                     .height = static_cast<std::uint32_t>(camera.config.height),
                                     .enable_rgb = camera.config.enable_rgb,
                                     .enable_depth = camera.config.enable_depth});
  }
  for (const auto& lidar : config.lidars) {
    bridge_config.lidars.push_back({.name = lidar.name,
                                    .frame_id = lidar.frame_id,
                                    .topic = lidar.topic,
                                    .sample_count = estimate_lidar_sample_count(lidar)});
  }
  return bridge_config;
}

HardwareMappingConfig make_mapping_config(const HardwareConfig& config) {
  HardwareMappingConfig mapping;
  mapping.joint_names.reserve(config.joints.size());
  mapping.imu_names.reserve(config.imus.size());
  mapping.camera_names.reserve(config.cameras.size());
  mapping.lidar_names.reserve(config.lidars.size());
  mapping.mobile_base_names.reserve(config.mobile_bases.size());

  for (const auto& joint : config.joints) {
    mapping.joint_names.push_back(joint.name);
  }
  for (const auto& imu : config.imus) {
    mapping.imu_names.push_back(imu.name);
  }
  for (const auto& camera : config.cameras) {
    mapping.camera_names.push_back(camera.name);
  }
  for (const auto& lidar : config.lidars) {
    mapping.lidar_names.push_back(lidar.name);
  }
  for (const auto& mobile_base : config.mobile_bases) {
    mapping.mobile_base_names.push_back(mobile_base.name);
  }
  return mapping;
}

}  // namespace

bool build_adapter_config(const hardware_interface::HardwareInfo& hardware_info,
                          AdapterConfigBundle* config, std::string& error_message) {
  if (config == nullptr) {
    error_message = "AdapterConfigBundle output pointer must not be null.";
    return false;
  }

  AdapterConfigBundle built;
  if (!parse_hardware_config(hardware_info, &built.runtime_config, error_message)) {
    return false;
  }
  built.ros_interface_config = make_ros_bridge_config(built.runtime_config);
  built.hardware_mapping_config = make_mapping_config(built.runtime_config);
  *config = std::move(built);
  error_message.clear();
  return true;
}

}  // namespace robot_mujoco_ros2
