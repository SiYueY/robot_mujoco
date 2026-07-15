#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/simulation.hpp"
#include "robot_mujoco_ros2/config_builder.hpp"
#include "robot_mujoco_ros2/data.hpp"
#include "robot_mujoco_ros2/publish_channel.hpp"

namespace robot_mujoco_ros2 {
class SimulationRosBridge;
}

namespace robot_mujoco_ros2 {

class MuJoCoHardwareInterface : public hardware_interface::SystemInterface {
 public:
  MuJoCoHardwareInterface();
  ~MuJoCoHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& hardware_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  struct PendingModeSwitch {
    std::map<std::string, std::set<std::string>> next_interfaces;
    bool valid{false};
  };

  enum class SystemState {
    kUnconfigured,
    kInactive,
    kActivating,
    kActive,
    kDeactivating,
    kError,
    kShutdown,
  };

  JointData* find_joint(const std::string& joint_name);
  const JointData* find_joint(const std::string& joint_name) const;
  void initialize_command_buffers();
  mujoco_simulation::ResultCode request_start_status();
  mujoco_simulation::ResultCode request_stop_status();
  mujoco_simulation::ResultCode request_pause_status();
  mujoco_simulation::ResultCode request_resume_status();
  mujoco_simulation::ResultCode request_step_status(uint32_t steps);
  mujoco_simulation::ResultCode request_set_realtime_factor_status(double realtime_factor);
  mujoco_simulation::ResultCode request_keyframe_reset_status(const std::string& keyframe);
  mujoco_simulation::ResultCode request_reset_status();
  mujoco_simulation::ResultCode update_runtime_state();
  mujoco_simulation::ResultCode update_runtime_state_from_snapshot(
      const mujoco_simulation::StateSnapshot& snapshot);
  mujoco_simulation::ResultCode publish_snapshot_to_channel(
      const std::shared_ptr<const mujoco_simulation::StateSnapshot>& snapshot);

  HardwareConfig config_;
  HardwareMappingConfig hardware_mapping_config_;
  std::string sensor_node_name_;
  std::unique_ptr<mujoco_simulation::Simulation> simulation_;
  std::unique_ptr<robot_mujoco_ros2::SimulationRosBridge> ros_bridge_;
  SystemState system_state_{SystemState::kUnconfigured};
  std::vector<mujoco_simulation::ImuState> publish_imu_states_;
  std::vector<mujoco_simulation::LidarState> publish_lidar_states_;
  std::vector<mujoco_simulation::CameraState> publish_camera_states_;
  PublishBundle publish_bundle_;
  std::map<std::string, std::set<std::string>> active_joint_interfaces_;
  PendingModeSwitch pending_mode_switch_;
  std::mutex simulation_control_mutex_;
  bool simulation_started_{false};
};

}  // namespace robot_mujoco_ros2
