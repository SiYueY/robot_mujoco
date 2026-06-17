#include "mujoco_hardware/mujoco_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mujoco_hardware::MuJoCoHardwareInterface,
                       hardware_interface::SystemInterface)
