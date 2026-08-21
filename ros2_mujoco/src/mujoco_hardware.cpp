#include "ros2_mujoco/mujoco_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace ros2_mujoco {
using CallbackReturn = hardware_interface::CallbackReturn;
CallbackReturn MujocoHardware::on_init(const hardware_interface::HardwareInfo& info) {
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
    for (const auto& j : info_.joints) {
        auto it = j.parameters.find("mujoco_id");
        if (it == j.parameters.end()) return CallbackReturn::ERROR;
        hardware::Joint joint;
        if (!joint.initialize(std::stoull(it->second), j.name)) return CallbackReturn::ERROR;
        joints_.push_back(std::move(joint));
    }
    return CallbackReturn::SUCCESS;
}
CallbackReturn MujocoHardware::on_configure(const rclcpp_lifecycle::State&) {
    auto it = info_.hardware_parameters.find("simulation_config");
    if (it == info_.hardware_parameters.end()) return CallbackReturn::ERROR;
    simulation_ = std::make_shared<mujoco_simulation::Simulation>();
    return simulation_->initialize(it->second) && simulation_->start() ? CallbackReturn::SUCCESS
                                                                       : CallbackReturn::ERROR;
}
CallbackReturn MujocoHardware::on_activate(const rclcpp_lifecycle::State&) {
    active_ = true;
    return CallbackReturn::SUCCESS;
}
CallbackReturn MujocoHardware::on_deactivate(const rclcpp_lifecycle::State&) {
    active_ = false;
    return CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> MujocoHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> r;
    for (auto& j : joints_) {
        r.emplace_back(j.name(), hardware_interface::HW_IF_POSITION, &j.position_state());
        r.emplace_back(j.name(), hardware_interface::HW_IF_VELOCITY, &j.velocity_state());
        r.emplace_back(j.name(), hardware_interface::HW_IF_EFFORT, &j.effort_state());
    }
    return r;
}
std::vector<hardware_interface::CommandInterface> MujocoHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> r;
    for (auto& j : joints_) {
        r.emplace_back(j.name(), hardware_interface::HW_IF_POSITION, &j.position_command());
        r.emplace_back(j.name(), hardware_interface::HW_IF_VELOCITY, &j.velocity_command());
        r.emplace_back(j.name(), hardware_interface::HW_IF_EFFORT, &j.effort_command());
    }
    return r;
}
hardware_interface::return_type MujocoHardware::read(const rclcpp::Time&, const rclcpp::Duration&) {
    mujoco_simulation::JointStates s;
    if (!simulation_ || !simulation_->read_state(s) || !s)
        return hardware_interface::return_type::ERROR;
    for (size_t i = 0; i < joints_.size() && i < s->size(); ++i) joints_[i].update(*s->at(i));
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type MujocoHardware::write(
    const rclcpp::Time&, const rclcpp::Duration&) {
    if (!active_) return hardware_interface::return_type::OK;
    mujoco_simulation::JointCommands c;
    for (const auto& j : joints_) c.push_back(j.command());
    return simulation_->write_commands(c) ? hardware_interface::return_type::OK
                                          : hardware_interface::return_type::ERROR;
}
}  // namespace ros2_mujoco
PLUGINLIB_EXPORT_CLASS(ros2_mujoco::MujocoHardware, hardware_interface::SystemInterface)
