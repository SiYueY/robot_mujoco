#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "mujoco_hardware/data.hpp"
#include "mujoco_simulation/component/joint/joint_config.hpp"
#include "mujoco_simulation/simulation.hpp"

namespace mujoco_simulation_ros {
class SimulationRosBridge;
}

namespace mujoco_hardware {

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
  void initialize_command_buffers();
  mujoco_simulation::Status request_start_status();
  mujoco_simulation::Status request_stop_status();
  mujoco_simulation::Status request_pause_status();
  mujoco_simulation::Status request_resume_status();
  mujoco_simulation::Status request_step_status(uint32_t steps);
  mujoco_simulation::Status request_set_realtime_factor_status(double realtime_factor);
  mujoco_simulation::Status request_keyframe_reset_status(const std::string& keyframe);
  mujoco_simulation::Status request_reset_status();
  mujoco_simulation::Status update_runtime_state();
  mujoco_simulation::Status update_runtime_state_from_snapshot(
      const mujoco_simulation::SimulationStateSnapshot& snapshot);

  HardwareConfig config_;
  std::string sensor_node_name_;
  std::unique_ptr<mujoco_simulation::Simulation> simulation_;
  std::unique_ptr<mujoco_simulation_ros::SimulationRosBridge> ros_bridge_;
  std::map<std::string, mujoco_simulation::CommandInterfaceType> active_joint_modes_;
  PendingModeSwitch pending_mode_switch_;
  std::mutex simulation_control_mutex_;
  bool simulation_started_{false};
};

}  // namespace mujoco_hardware
