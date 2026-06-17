#pragma once

// robot_mujoco — Workspace-level C++ convenience header
//
// Aggregates the key types from both mujoco_simulation and mujoco_hardware
// so that application code only needs a single dependency:
//
//   find_package(robot_mujoco REQUIRED)
//   #include <robot_mujoco/robot_mujoco.hpp>
//
// Re-exports:
//   mujoco_simulation  — Simulation, device types, HardwareManager
//   mujoco_hardware    — MuJoCoHardwareInterface, HardwareConfig
//   mujoco_simulation_ros — SimulationRosBridge, bridge config types

#include "mujoco_hardware/data.hpp"
#include "mujoco_hardware/mujoco_hardware_interface.hpp"
#include "mujoco_simulation/simulation.hpp"
#include "mujoco_simulation_ros/simulation_ros_bridge.hpp"

namespace robot_mujoco {

// Workspace version identifier.
constexpr int kVersionMajor = 0;
constexpr int kVersionMinor = 1;
constexpr int kVersionPatch = 0;

}  // namespace robot_mujoco
