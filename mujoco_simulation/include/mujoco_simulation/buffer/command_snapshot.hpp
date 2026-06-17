#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/component/joint/joint_command.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_command.hpp"

namespace mujoco_simulation {

struct CommandSnapshot {
  std::unordered_map<std::string, JointCommand> joint_commands;
  std::unordered_map<std::string, MobileBaseCommand> mobile_base_commands;
  std::uint64_t sequence{0};
};

}  // namespace mujoco_simulation
