#include "simulation/simulation_impl.hpp"

#include <mutex>

namespace mujoco_simulation {

bool Simulation::Impl::read_state(std::shared_ptr<const RobotState>& state) const {
    state = state_buffer_.read();
    return state != nullptr;
}

bool Simulation::Impl::read_state(RobotState& state) const {
    std::shared_ptr<const RobotState> robot_state;
    if (!read_state(robot_state)) return false;
    state = *robot_state;
    return true;
}

bool Simulation::Impl::read_state(JointState& state) const { return state_buffer_.read(state); }

bool Simulation::Impl::read_state(ImuState& state) const { return state_buffer_.read(state); }

bool Simulation::Impl::read_state(CameraState& state) const { return state_buffer_.read(state); }

bool Simulation::Impl::read_state(LidarState& state) const { return state_buffer_.read(state); }

bool Simulation::Impl::read_state(MobileBaseState& state) const {
    return state_buffer_.read(state);
}

bool Simulation::Impl::read_state(JointStates& state) const {
    const auto robot_state = state_buffer_.read();
    if (robot_state == nullptr) return false;
    state = robot_state->joints;
    return state != nullptr;
}
bool Simulation::Impl::read_state(ImuStates& state) const {
    const auto robot_state = state_buffer_.read();
    if (robot_state == nullptr) return false;
    state = robot_state->imus;
    return state != nullptr;
}
bool Simulation::Impl::read_state(CameraStates& state) const {
    const auto robot_state = state_buffer_.read();
    if (robot_state == nullptr) return false;
    state = robot_state->cameras;
    return state != nullptr;
}
bool Simulation::Impl::read_state(LidarStates& state) const {
    const auto robot_state = state_buffer_.read();
    if (robot_state == nullptr) return false;
    state = robot_state->lidars;
    return state != nullptr;
}
bool Simulation::Impl::read_state(MobileBaseStates& state) const {
    const auto robot_state = state_buffer_.read();
    if (robot_state == nullptr) return false;
    state = robot_state->mobile_bases;
    return state != nullptr;
}

bool Simulation::Impl::read_contacts(ContactStates& contacts) const {
    const auto robot_state = state_buffer_.read();
    if (robot_state == nullptr) return false;
    contacts = robot_state->contacts;
    return true;
}

std::uint64_t Simulation::Impl::step_count() const { return step_.load(); }

SimulationStatus Simulation::Impl::status() const {
    if (runtime_failed_.load()) return SimulationStatus::Error;
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (scheduler_ != nullptr) return scheduler_->status();
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    if (runtime_ == nullptr || !runtime_->is_initialized()) return SimulationStatus::Uninitialized;
    return SimulationStatus::Stopped;
}

double Simulation::Impl::time() const {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    if (runtime_ == nullptr || !runtime_->is_initialized()) return 0.0;
    return runtime_->time();
}

}  // namespace mujoco_simulation
