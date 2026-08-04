#include "buffer/state_buffer.hpp"

#include <algorithm>
#include <limits>

namespace mujoco_simulation {

namespace {
constexpr std::size_t kNoStateIndex = std::numeric_limits<std::size_t>::max();

template <typename State>
bool validate_states(const StateSnapshots<State>& states, const std::vector<std::size_t>& indices) {
    if (states == nullptr) return true;
    for (std::size_t index = 0; index < states->size(); ++index) {
        const StateSnapshot<State>& state = (*states)[index];
        if (state == nullptr || state->id >= indices.size() || indices[state->id] != index)
            return false;
    }
    return true;
}

template <typename State>
bool read_by_index(
    const StateSnapshots<State>& states, const std::vector<std::size_t>& indices, State& state) {
    const std::size_t id = state.id;
    if (states == nullptr || id >= indices.size()) return false;
    const std::size_t index = indices[id];
    if (index == kNoStateIndex || index >= states->size() || (*states)[index] == nullptr ||
        (*states)[index]->id != id)
        return false;
    state = *(*states)[index];
    return true;
}
}  // namespace

bool StateBuffer::configure(std::shared_ptr<const ComponentIndex> component_index) {
    if (component_index == nullptr) return false;
    component_index_ = std::move(component_index);
    std::atomic_store_explicit(
        &state_, std::shared_ptr<const RobotState>{}, std::memory_order_release);
    return true;
}

bool StateBuffer::write(std::shared_ptr<const RobotState> snapshot) {
    if (component_index_ == nullptr || snapshot == nullptr ||
        !validate_states(snapshot->joints, component_index_->joints()) ||
        !validate_states(snapshot->mobile_bases, component_index_->mobile_bases()) ||
        !validate_states(snapshot->imus, component_index_->imus()) ||
        !validate_states(snapshot->cameras, component_index_->cameras()) ||
        !validate_states(snapshot->lidars, component_index_->lidars()))
        return false;
    std::atomic_store_explicit(&state_, std::move(snapshot), std::memory_order_release);
    return true;
}

std::shared_ptr<const RobotState> StateBuffer::read() const {
    return std::atomic_load_explicit(&state_, std::memory_order_acquire);
}

bool StateBuffer::read(JointState& state) const {
    const auto snapshot = read();
    return snapshot != nullptr && component_index_ != nullptr &&
           read_by_index(snapshot->joints, component_index_->joints(), state);
}

bool StateBuffer::read(MobileBaseState& state) const {
    const auto snapshot = read();
    return snapshot != nullptr && component_index_ != nullptr &&
           read_by_index(snapshot->mobile_bases, component_index_->mobile_bases(), state);
}

bool StateBuffer::read(ImuState& state) const {
    const auto snapshot = read();
    return snapshot != nullptr && component_index_ != nullptr &&
           read_by_index(snapshot->imus, component_index_->imus(), state);
}

bool StateBuffer::read(CameraState& state) const {
    const auto snapshot = read();
    return snapshot != nullptr && component_index_ != nullptr &&
           read_by_index(snapshot->cameras, component_index_->cameras(), state);
}

bool StateBuffer::read(LidarState& state) const {
    const auto snapshot = read();
    return snapshot != nullptr && component_index_ != nullptr &&
           read_by_index(snapshot->lidars, component_index_->lidars(), state);
}

void StateBuffer::shutdown() {
    std::atomic_store_explicit(
        &state_, std::shared_ptr<const RobotState>{}, std::memory_order_release);
    component_index_.reset();
}

}  // namespace mujoco_simulation
