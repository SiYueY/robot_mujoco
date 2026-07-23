#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"

namespace mujoco_simulation {

using JointCommands = std::unordered_map<std::string, JointCommand>;
using MobileBaseCommands = std::unordered_map<std::string, MobileBaseCommand>;

struct RobotCommand {
  std::uint64_t sequence{0};
  JointCommands joint_commands;
  MobileBaseCommands mobile_base_commands;
};

}  // namespace mujoco_simulation
