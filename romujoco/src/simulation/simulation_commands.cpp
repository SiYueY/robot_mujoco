#include "simulation/simulation_impl.hpp"

#include "log/logging.hpp"

namespace romujoco {

bool Simulation::Impl::write_command(const JointCommand& command) {
    if (!command_buffer_.write(command)) {
        SIM_ERROR << "failed to write joint command: id = " << command.id
                  << ", command data is invalid.";
        return false;
    }
    return true;
}

bool Simulation::Impl::write_command(const GripperCommand& command) {
    if (!command_buffer_.write(command)) {
        SIM_ERROR << "failed to write gripper command: id = " << command.id
                  << ", command data is invalid.";
        return false;
    }
    return true;
}

bool Simulation::Impl::write_command(const MobileBaseCommand& command) {
    if (!command_buffer_.write(command)) {
        SIM_ERROR << "failed to write mobile base command: id = " << command.id
                  << ", command data is invalid.";
        return false;
    }
    return true;
}

bool Simulation::Impl::write_command(const RobotCommand& command) {
    if (!command_buffer_.write(command)) {
        SIM_ERROR << "failed to write robot command: sequence = " << command.sequence
                  << ", command data is invalid.";
        return false;
    }
    return true;
}

bool Simulation::Impl::write_commands(const JointCommands& commands) {
    if (!command_buffer_.write(commands)) {
        SIM_ERROR << "failed to write joint command batch, command data is invalid.";
        return false;
    }
    return true;
}

bool Simulation::Impl::write_commands(const GripperCommands& commands) {
    if (!command_buffer_.write(commands)) {
        SIM_ERROR << "failed to write gripper command batch, command data is invalid.";
        return false;
    }
    return true;
}

bool Simulation::Impl::write_commands(const MobileBaseCommands& commands) {
    if (!command_buffer_.write(commands)) {
        SIM_ERROR << "failed to write mobile base command batch, command data is invalid.";
        return false;
    }
    return true;
}

}  // namespace romujoco
