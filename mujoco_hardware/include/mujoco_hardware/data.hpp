#pragma once

#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "mujoco_simulation/component/camera/camera_config.hpp"
#include "mujoco_simulation/component/camera/camera_sample.hpp"
#include "mujoco_simulation/component/imu/imu_config.hpp"
#include "mujoco_simulation/component/imu/imu_sample.hpp"
#include "mujoco_simulation/component/joint/joint_command.hpp"
#include "mujoco_simulation/component/joint/joint_config.hpp"
#include "mujoco_simulation/component/joint/joint_state.hpp"
#include "mujoco_simulation/component/lidar/lidar_config.hpp"
#include "mujoco_simulation/component/lidar/lidar_sample.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_command.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_config.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_state.hpp"
#include "mujoco_simulation/simulation.hpp"

namespace mujoco_hardware {

struct JointData {
  std::string name;
  std::vector<std::string> command_interfaces;
  std::vector<std::string> state_interfaces;
  mujoco_simulation::JointConfig config;
  mujoco_simulation::JointCommand command;
  mujoco_simulation::JointState state;
};

struct ImuData {
  std::string name;
  std::vector<std::string> state_interfaces;
  std::string frame_id;
  std::string topic;
  mujoco_simulation::ImuConfig config;
  mujoco_simulation::ImuSample sample;
};

struct CameraData {
  std::string name;

  // ROS frame used by Image and CameraInfo headers.
  std::string frame_id;

  // ROS topics published for this camera.
  std::string rgb_topic;
  std::string depth_topic;
  std::string camera_info_topic;

  mujoco_simulation::CameraConfig config;
  mujoco_simulation::CameraSample sample;
};

struct LidarData {
  std::string name;
  std::string frame_id;
  std::string topic;
  mujoco_simulation::LidarConfig config;
  mujoco_simulation::LidarSample sample;
};

struct MobileBaseData {
  std::string name;
  mujoco_simulation::MobileBaseConfig config;
  mujoco_simulation::MobileBaseCommand command;
  mujoco_simulation::MobileBaseState state;
};

struct HardwareConfig {
  mujoco_simulation::SimulationConfig simulation;
  std::vector<JointData> joints;
  std::vector<ImuData> imus;
  std::vector<CameraData> cameras;
  std::vector<LidarData> lidars;
  std::vector<MobileBaseData> mobile_bases;
};

bool parse_hardware_config(const hardware_interface::HardwareInfo& hardware_info,
                           HardwareConfig* config, std::string& error_message);

mujoco_simulation::CommandInterfaceType to_joint_control_mode(const std::string& interface_name);
bool is_joint_command_interface(const std::string& interface_name);
bool is_joint_state_interface(const std::string& interface_name);
bool is_imu_state_interface(const std::string& interface_name);

}  // namespace mujoco_hardware
