#pragma once
#include <memory>
#include <vector>
#include "hardware_interface/system_interface.hpp"
#include "mujoco_simulation/simulation.hpp"
#include "ros2_mujoco/hardware/joint.hpp"
#include "ros2_mujoco/hardware/mobile_base.hpp"
namespace ros2_mujoco {
class MujocoHardware final : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo&) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;
    hardware_interface::return_type write(const rclcpp::Time&, const rclcpp::Duration&) override;

private:
    std::shared_ptr<mujoco_simulation::Simulation> simulation_;
    std::vector<hardware::Joint> joints_;
    std::vector<hardware::MobileBase> mobile_bases_;
    bool active_{};
};
}  // namespace ros2_mujoco
