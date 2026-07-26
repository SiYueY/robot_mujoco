#include "mujoco_simulation/component/simulation_component.hpp"

#include <cmath>
#include <utility>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/common/math.hpp"

namespace mujoco_simulation {

SimulationComponent::~SimulationComponent() = default;

SimulationComponent::SimulationComponent(std::string name, double update_rate)
    : name_(std::move(name)), update_rate_(update_rate) {}

const std::string &SimulationComponent::name() const noexcept { return name_; }

bool SimulationComponent::configure(const mjContext &context) {
  if (!context.valid()) {
    LOG_ERROR << "component context must provide a model and data.";
    return false;
  }
  if (name_.empty()) {
    LOG_ERROR << "component name must not be empty.";
    return false;
  }
  if (!std::isfinite(update_rate_) || update_rate_ < 0.0) {
    LOG_ERROR << "component '" << name_
              << "' update rate must be finite and non-negative.";
    return false;
  }

  const double physics_rate = 1.0 / context.model->opt.timestep;
  if (!std::isfinite(physics_rate) || physics_rate <= 0.0) {
    LOG_ERROR << "component '" << name_
              << "' requires a finite, positive model timestep.";
    return false;
  }
  if (greater(update_rate_, physics_rate)) {
    LOG_ERROR << "component '" << name_
              << "' update rate must not exceed the physics rate.";
    return false;
  }

  reset_schedule();
  return true;
}

bool SimulationComponent::poll_update(mjTime time) {
  if (update_rate_ <= 0.0) {
    return true;
  }
  const mjTime period = 1.0 / update_rate_;

  if (less(time, next_time_)) {
    return false;
  }

  do {
    next_time_ += period;
  } while (greater(time, next_time_) || equal(time, next_time_));
  return true;
}

bool SimulationComponent::reset_schedule() noexcept {
  next_time_ = 0.0;
  return true;
}

} // namespace mujoco_simulation
