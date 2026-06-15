#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mujoco_hardware/data.hpp"
#include "mujoco_simulation/hardware/joint.hpp"
#include "mujoco_simulation/mujoco_simulation.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace mujoco_hardware {

class SensorBridge;

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
    std::map<std::string, mujoco_simulation::CommandInterfaceType> next_modes;
    bool valid{false};
  };

  JointData* find_joint(const std::string& joint_name);
  const JointData* find_joint(const std::string& joint_name) const;
  bool fill_camera_info(const std::string& camera_name, int width, int height,
                        sensor_msgs::msg::CameraInfo* info, std::string& error_message) const;
  void initialize_command_buffers();
  bool update_runtime_state();

  HardwareConfig config_;
  std::string sensor_node_name_;
  std::unique_ptr<mujoco_simulation::MuJoCoSimulation> simulation_;
  std::unique_ptr<SensorBridge> sensor_bridge_;
  std::map<std::string, mujoco_simulation::CommandInterfaceType> active_joint_modes_;
  PendingModeSwitch pending_mode_switch_;
};

}  // namespace mujoco_hardware
