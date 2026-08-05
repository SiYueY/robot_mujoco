#include "buffer/state_buffer.hpp"

#include <algorithm>
#include <limits>

namespace mujoco_simulation {

namespace {

constexpr std::size_t kInvalidStateIndex = std::numeric_limits<std::size_t>::max();

template <typename State>
bool validate_states(const StateSnapshots<State>& states, const std::vector<std::size_t>& indices) {
    if (states == nullptr) return true;
    for (std::size_t index = 0; index < states->size(); ++index) {
        const StateSnapshot<State>& state = (*states)[index];
        if (state == nullptr) return false;
        if (state->id >= indices.size()) return false;
        if (indices[state->id] != index) return false;
    }
    return true;
}

template <typename State>
bool read_state(
    const StateSnapshots<State>& states, const std::vector<std::size_t>& indices, State& state) {
    const std::size_t id = state.id;
    if (states == nullptr) return false;
    if (id >= indices.size()) return false;
    const std::size_t index = indices[id];
    if (index == kInvalidStateIndex) return false;
    if (index >= states->size()) return false;
    if ((*states)[index] == nullptr) return false;
    if ((*states)[index]->id != id) return false;
    state = *(*states)[index];
    return true;
}

}  // namespace

bool StateBuffer::configure(std::shared_ptr<const ComponentIdResolver> id_resolver) {
    if (id_resolver == nullptr) return false;
    if (initialized_) return false;
    id_resolver_ = std::move(id_resolver);
    std::atomic_store_explicit(
        &state_, std::shared_ptr<const RobotState>{}, std::memory_order_release);
    initialized_ = true;
    return true;
}

void StateBuffer::shutdown() {
    std::atomic_store_explicit(
        &state_, std::shared_ptr<const RobotState>{}, std::memory_order_release);
    id_resolver_.reset();
    initialized_ = false;
}

std::shared_ptr<const RobotState> StateBuffer::read() const {
    return std::atomic_load_explicit(&state_, std::memory_order_acquire);
}

bool StateBuffer::read(JointState& state) const {
    if (id_resolver_ == nullptr) return false;
    const auto robot_state = read();
    if (robot_state == nullptr) return false;
    return read_state(robot_state->joints, id_resolver_->joints(), state);
}

bool StateBuffer::read(JointStates& states) const {
    if (id_resolver_ == nullptr) return false;
    const auto robot_state = read();
    if (robot_state == nullptr) return false;
    states = robot_state->joints;
    return true;
}

bool StateBuffer::read(MobileBaseState& state) const {
    if (id_resolver_ == nullptr) return false;
    const auto robot_state = read();
    if (robot_state == nullptr) return false;
    return read_state(robot_state->mobile_bases, id_resolver_->mobile_bases(), state);
}

bool StateBuffer::read(MobileBaseStates& states) const {
    if (id_resolver_ == nullptr) return false;
    const auto robot_state = read();
    if (robot_state == nullptr) return false;
    states = robot_state->mobile_bases;
    return true;
}

bool StateBuffer::read(ImuState& state) const {
    if (id_resolver_ == nullptr) return false;
    const auto robot_state = read();
    if (robot_state == nullptr) return false;
    return read_state(robot_state->imus, id_resolver_->imus(), state);
}

bool StateBuffer::read(ImuStates& states) const {
    if (id_resolver_ == nullptr) return false;
    const auto robot_state = read();
    if (robot_state == nullptr) return false;
    states = robot_state->imus;
    return true;
}

bool StateBuffer::read(CameraState& state) const {
    if (id_resolver_ == nullptr) return false;
    const auto robot_state = read();
    if (robot_state == nullptr) return false;
    return read_state(robot_state->cameras, id_resolver_->cameras(), state);
}

bool StateBuffer::read(CameraStates& states) const {
    if (id_resolver_ == nullptr) return false;
    const auto robot_state = read();
    if (robot_state == nullptr) return false;
    states = robot_state->cameras;
    return true;
}

bool StateBuffer::read(LidarState& state) const {
    if (id_resolver_ == nullptr) return false;
    const auto robot_state = read();
    if (robot_state == nullptr) return false;
    return read_state(robot_state->lidars, id_resolver_->lidars(), state);
}

bool StateBuffer::read(LidarStates& states) const {
    if (id_resolver_ == nullptr) return false;
    const auto robot_state = read();
    if (robot_state == nullptr) return false;
    states = robot_state->lidars;
    return true;
}

bool StateBuffer::write(std::shared_ptr<const RobotState> robot_state) {
    if (id_resolver_ == nullptr) return false;
    if (robot_state == nullptr) return false;
    if (!validate_states(robot_state->joints, id_resolver_->joints())) return false;
    if (!validate_states(robot_state->mobile_bases, id_resolver_->mobile_bases())) return false;
    if (!validate_states(robot_state->imus, id_resolver_->imus())) return false;
    if (!validate_states(robot_state->cameras, id_resolver_->cameras())) return false;
    if (!validate_states(robot_state->lidars, id_resolver_->lidars())) return false;
    std::atomic_store_explicit(&state_, std::move(robot_state), std::memory_order_release);
    return true;
}

}  // namespace mujoco_simulation
