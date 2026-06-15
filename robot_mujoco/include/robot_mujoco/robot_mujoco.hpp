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
//   mujoco_simulation  — MuJoCoSimulation, device types, HardwareManager
//   mujoco_hardware    — MuJoCoHardwareInterface, HardwareConfig, SensorBridge

#include "mujoco_hardware/data.hpp"
#include "mujoco_hardware/mujoco_hardware_interface.hpp"
#include "mujoco_hardware/sensor_bridge.hpp"
#include "mujoco_simulation/mujoco_simulation.hpp"

namespace robot_mujoco {

// Workspace version identifier.
constexpr int kVersionMajor = 0;
constexpr int kVersionMinor = 1;
constexpr int kVersionPatch = 0;

}  // namespace robot_mujoco
