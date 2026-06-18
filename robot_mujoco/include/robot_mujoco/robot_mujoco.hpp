#pragma once

// robot_mujoco — Workspace-level C++ convenience header
//
// Aggregates the key types from both mujoco_simulation and robot_mujoco_ros2
// so that application code only needs a single dependency:
//
//   find_package(robot_mujoco REQUIRED)
//   #include <robot_mujoco/robot_mujoco.hpp>
//
// Re-exports:
//   mujoco_simulation  — Simulation, device types, HardwareManager
//   robot_mujoco_ros2 — MuJoCoHardwareInterface, HardwareConfig, SimulationRosBridge

#include "mujoco_simulation/simulation.hpp"
#include "robot_mujoco_ros2/data.hpp"
#include "robot_mujoco_ros2/mujoco_hardware_interface.hpp"
#include "robot_mujoco_ros2/simulation_ros_bridge.hpp"

namespace robot_mujoco {

// Workspace version identifier.
constexpr int kVersionMajor = 0;
constexpr int kVersionMinor = 1;
constexpr int kVersionPatch = 0;

}  // namespace robot_mujoco
