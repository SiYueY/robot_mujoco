#include "mujoco_simulation/component/simulation_component.hpp"

#include <cmath>

namespace mujoco_simulation {
namespace {

constexpr double kScheduleEpsilon = 1.0e-9;

}  // namespace

SimulationComponent::~SimulationComponent() = default;

bool SimulationComponent::should_update(double simulation_time) {
  if (period_ <= 0.0) {
    return true;
  }

  if (simulation_time + kScheduleEpsilon < next_due_time_) {
    return false;
  }

  std::uint64_t due_slots = 0;
  do {
    next_due_time_ += period_;
    ++due_slots;
  } while (simulation_time + kScheduleEpsilon >= next_due_time_);
  if (due_slots > 1) {
    missed_updates_ += due_slots - 1;
  }
  return true;
}

void SimulationComponent::reset_update_schedule() {
  next_due_time_ = 0.0;
  missed_updates_ = 0;
}

std::uint64_t SimulationComponent::missed_updates() const noexcept { return missed_updates_; }

ResultCode SimulationComponent::set_update_rate(double update_rate, double physics_rate) {
  if (!std::isfinite(update_rate) || !std::isfinite(physics_rate) || update_rate <= 0.0 ||
      physics_rate <= 0.0) {
    return ResultCode::InvalidArgument;
  }
  if (update_rate - physics_rate > kScheduleEpsilon) {
    return ResultCode::InvalidArgument;
  }

  update_rate_ = update_rate;
  period_ = 1.0 / update_rate_;
  reset_update_schedule();
  return ResultCode::Ok;
}

void SimulationComponent::set_update_every_step() noexcept {
  update_rate_ = 0.0;
  period_ = 0.0;
  reset_update_schedule();
}

}  // namespace mujoco_simulation
