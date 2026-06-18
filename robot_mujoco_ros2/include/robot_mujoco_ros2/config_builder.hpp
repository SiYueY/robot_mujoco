#pragma once

#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "robot_mujoco_ros2/bridge_config.hpp"
#include "robot_mujoco_ros2/data.hpp"

namespace robot_mujoco_ros2 {

struct HardwareMappingConfig {
  std::vector<std::string> joint_names;
  std::vector<std::string> imu_names;
  std::vector<std::string> camera_names;
  std::vector<std::string> lidar_names;
  std::vector<std::string> mobile_base_names;
};

struct AdapterConfigBundle {
  HardwareConfig runtime_config;
  SimulationRosBridgeConfig ros_interface_config;
  HardwareMappingConfig hardware_mapping_config;
};

bool build_adapter_config(const hardware_interface::HardwareInfo& hardware_info,
                          AdapterConfigBundle* config, std::string& error_message);

}  // namespace robot_mujoco_ros2
