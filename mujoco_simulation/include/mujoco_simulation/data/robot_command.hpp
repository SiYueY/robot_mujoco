#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"

namespace mujoco_simulation {

using JointCommands = std::vector<std::optional<JointCommand>>;
using MobileBaseCommands = std::vector<std::optional<MobileBaseCommand>>;

struct RobotCommand {
  std::uint64_t sequence{0};
  JointCommands joint_commands;
  MobileBaseCommands mobile_base_commands;
};

} // namespace mujoco_simulation
