#include "mujoco_simulation/simulation.hpp"

#include <utility>

#include "common/macro.hpp"
#include "simulation/simulation_impl.hpp"

namespace mujoco_simulation {

Simulation::Simulation() : impl_(std::make_unique<Impl>()) {}
Simulation::~Simulation() { UNUSED(impl_->shutdown()); }

bool Simulation::initialize(const SimulationConfig& config) { return impl_->initialize(config); }

bool Simulation::initialize(const std::string& config_path) {
    return impl_->initialize(config_path);
}

bool Simulation::shutdown() { return impl_->shutdown(); }
bool Simulation::start() { return impl_->start(); }
bool Simulation::stop() { return impl_->stop(); }
bool Simulation::pause() { return impl_->pause(); }
bool Simulation::resume() { return impl_->resume(); }
bool Simulation::reset() { return impl_->reset(nullptr); }
bool Simulation::reset(std::string keyframe_name) { return impl_->reset(&keyframe_name); }

bool Simulation::write_command(JointId id, const JointCommand& command) {
    return impl_->write_command(id, command);
}
bool Simulation::write_command(MobileBaseId id, const MobileBaseCommand& command) {
    return impl_->write_command(id, command);
}
bool Simulation::write_command(const RobotCommand& command) {
    return impl_->write_command(command);
}
bool Simulation::write_commands(const JointCommands& commands) {
    return impl_->write_commands(commands);
}
bool Simulation::write_commands(const MobileBaseCommands& commands) {
    return impl_->write_commands(commands);
}

bool Simulation::read_state(std::shared_ptr<const RobotState>& out) const {
    return impl_->read_state(out);
}
bool Simulation::read_state(RobotState& out) const { return impl_->read_state(out); }
bool Simulation::read_state(JointId id, JointState& out) const {
    return impl_->read_state(id, out);
}
bool Simulation::read_state(ImuId id, ImuState& out) const { return impl_->read_state(id, out); }
bool Simulation::read_state(CameraId id, CameraState& out) const {
    return impl_->read_state(id, out);
}
bool Simulation::read_state(LidarId id, LidarState& out) const {
    return impl_->read_state(id, out);
}
bool Simulation::read_state(MobileBaseId id, MobileBaseState& out) const {
    return impl_->read_state(id, out);
}
bool Simulation::read_state(JointStates& out) const { return impl_->read_state(out); }
bool Simulation::read_state(ImuStates& out) const { return impl_->read_state(out); }
bool Simulation::read_state(CameraStates& out) const { return impl_->read_state(out); }
bool Simulation::read_state(LidarStates& out) const { return impl_->read_state(out); }
bool Simulation::read_state(MobileBaseStates& out) const { return impl_->read_state(out); }

bool Simulation::step(std::size_t count) { return impl_->step(count); }
uint64_t Simulation::step_count() const { return impl_->step_count(); }
SimulationStatus Simulation::status() const { return impl_->status(); }
double Simulation::time() const { return impl_->time(); }

}  // namespace mujoco_simulation
