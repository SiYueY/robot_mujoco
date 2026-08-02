#include "simulation/simulation_impl.hpp"

#include "common/logging.hpp"

namespace mujoco_simulation {

bool Simulation::Impl::write_command(JointId id, const JointCommand &command) {
  if (!command_buffer_.write<JointCommand>(id, command)) {
    LOG_ERROR << "joint command was rejected for component id " << id << ".";
    return false;
  }
  return true;
}

bool Simulation::Impl::write_command(MobileBaseId id,
                                     const MobileBaseCommand &command) {
  if (!command_buffer_.write<MobileBaseCommand>(id, command)) {
    LOG_ERROR << "mobile base command was rejected for component id " << id
              << ".";
    return false;
  }
  return true;
}

bool Simulation::Impl::write_commands(const JointCommandBatch &commands) {
  if (!command_buffer_.write(CommandBatch<JointCommand>{commands})) {
    LOG_ERROR << "joint command batch was rejected.";
    return false;
  }
  return true;
}

bool Simulation::Impl::write_commands(const MobileBaseCommandBatch &commands) {
  if (!command_buffer_.write(CommandBatch<MobileBaseCommand>{commands})) {
    LOG_ERROR << "mobile base command batch was rejected.";
    return false;
  }
  return true;
}

} // namespace mujoco_simulation
