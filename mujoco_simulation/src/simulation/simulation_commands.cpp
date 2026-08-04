#include "simulation/simulation_impl.hpp"

#include "common/logging.hpp"

namespace mujoco_simulation {

bool Simulation::Impl::write_command(const JointCommand& command) {
    if (!command_buffer_.write(command)) {
        LOG_ERROR << "joint command was rejected for component id " << command.id << ".";
        return false;
    }
    return true;
}

bool Simulation::Impl::write_command(const MobileBaseCommand& command) {
    if (!command_buffer_.write(command)) {
        LOG_ERROR << "mobile base command was rejected for component id " << command.id << ".";
        return false;
    }
    return true;
}

bool Simulation::Impl::write_command(const RobotCommand& command) {
    if (!command_buffer_.write(command)) {
        LOG_ERROR << "atomic robot command was rejected.";
        return false;
    }
    return true;
}

bool Simulation::Impl::write_commands(const JointCommands& commands) {
    if (!command_buffer_.write(commands)) {
        LOG_ERROR << "joint command batch was rejected.";
        return false;
    }
    return true;
}

bool Simulation::Impl::write_commands(const MobileBaseCommands& commands) {
    if (!command_buffer_.write(commands)) {
        LOG_ERROR << "mobile base command batch was rejected.";
        return false;
    }
    return true;
}

}  // namespace mujoco_simulation
