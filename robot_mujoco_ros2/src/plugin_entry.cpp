#include "pluginlib/class_list_macros.hpp"
#include "robot_mujoco_ros2/mujoco_hardware_interface.hpp"

PLUGINLIB_EXPORT_CLASS(robot_mujoco_ros2::MuJoCoHardwareInterface,
                       hardware_interface::SystemInterface)
