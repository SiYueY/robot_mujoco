#pragma once

#include <cstddef>
#include <mutex>
#include <string>

#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"
#include "mujoco_simulation/data/robot_command.hpp"
#include "mujoco_simulation/visibility.hpp"

namespace mujoco_simulation {

class MUJOCO_SIMULATION_PUBLIC CommandBuffer {
public:
  bool write_joint_command(JointId id, const JointCommand &command);

  bool write_mobile_base_command(MobileBaseId id,
                                 const MobileBaseCommand &command);

  bool write_command(const RobotCommand &command);

  RobotCommand read() const;

  void clear();

private:
  mutable std::mutex mutex_;
  JointCommands joint_commands_;
  MobileBaseCommands mobile_base_commands_;
  std::uint64_t sequence_{0};
};

} // namespace mujoco_simulation
