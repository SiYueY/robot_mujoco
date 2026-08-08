#include "component/component.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/compare.hpp"
#include "log/logging.hpp"

namespace mujoco_simulation {

SimulationComponent::~SimulationComponent() = default;

SimulationComponent::SimulationComponent(std::string name, double period)
: name_(std::move(name)), period_(period) {}

const std::string& SimulationComponent::name() const noexcept { return name_; }

bool SimulationComponent::configure(const mjContext& context) {
    if (!context.valid()) {
        SIM_ERROR << "component context must provide a model and data.";
        return false;
    }
    if (name_.empty()) {
        SIM_ERROR << "component name must not be empty.";
        return false;
    }
    if (!std::isfinite(period_) || period_ <= 0.0) {
        SIM_ERROR << "component '" << name_ << "' period must be a finite, positive number.";
        return false;
    }

    const double physics_period = context.model->opt.timestep;
    if (!std::isfinite(physics_period) || physics_period <= 0.0) {
        SIM_ERROR << "component '" << name_ << "' requires a finite, positive model timestep.";
        return false;
    }

    if (math::less(period_, physics_period)) {
        SIM_ERROR << "component '" << name_ << "' period " << period_
                  << " must not be shorter than the physics period " << physics_period << ".";
        return false;
    }

    const double multiple = period_ / physics_period;
    const double rounded_multiple = std::round(multiple);
    const double tolerance = 1e-9 * std::max(1.0, std::abs(multiple));
    if (rounded_multiple < 1.0 || std::abs(multiple - rounded_multiple) > tolerance) {
        SIM_ERROR << "component '" << name_ << "' period " << period_
                  << " must be an integer multiple of the physics period " << physics_period << ".";
        return false;
    }

    SIM_DEBUG << "component '" << name_ << "' is configured with update period " << period_
              << " seconds.";

    if (!reset_schedule()) {
        SIM_ERROR << "failed to reset the update schedule for component '" << name_ << "'.";
        return false;
    }
    return true;
}

bool SimulationComponent::poll_update(mjTime time) {
    if (math::less(time, next_time_)) {
        return false;
    }

    do {
        next_time_ += period_;
    } while (math::greater(time, next_time_) || math::equal(time, next_time_));
    return true;
}

bool SimulationComponent::reset_schedule() noexcept {
    next_time_ = 0.0;
    return true;
}

}  // namespace mujoco_simulation
