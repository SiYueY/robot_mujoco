#include "component/component.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/logging.hpp"
#include "common/compare.hpp"

namespace mujoco_simulation {

SimulationComponent::~SimulationComponent() = default;

SimulationComponent::SimulationComponent(std::string name, double period)
: name_(std::move(name)), period_(period) {}

const std::string& SimulationComponent::name() const noexcept { return name_; }

bool SimulationComponent::configure(const mjContext& context) {
    if (!context.valid()) {
        LOG_ERROR << "component context must provide a model and data.";
        return false;
    }
    if (name_.empty()) {
        LOG_ERROR << "component name must not be empty.";
        return false;
    }
    if (!std::isfinite(period_) || period_ < 0.0) {
        LOG_ERROR << "component '" << name_ << "' period must be finite and non-negative.";
        return false;
    }

    const double physics_period = context.model->opt.timestep;
    if (!std::isfinite(physics_period) || physics_period <= 0.0) {
        LOG_ERROR << "component '" << name_ << "' requires a finite, positive model timestep.";
        return false;
    }

    if (period_ > 0.0 && period_ < physics_period) {
        LOG_ERROR << "component '" << name_ << "' period " << period_
                  << " must not be shorter than the physics period " << physics_period << ".";
        return false;
    }

    if (period_ > 0.0) {
        const double multiple = period_ / physics_period;
        const double rounded_multiple = std::round(multiple);
        const double tolerance = 1e-9 * std::max(1.0, std::abs(multiple));
        if (rounded_multiple < 1.0 || std::abs(multiple - rounded_multiple) > tolerance) {
            LOG_ERROR << "component '" << name_ << "' period " << period_
                      << " must be an integer multiple of the physics period " << physics_period
                      << ".";
            return false;
        }
    }

    if (period_ == 0.0) {
        LOG_DEBUG << "component '" << name_ << "' is configured to update every physics step.";
    } else {
        LOG_DEBUG << "component '" << name_ << "' is configured with period " << period_
                  << " seconds.";
    }

    if (!reset_schedule()) {
        LOG_ERROR << "failed to reset the update schedule for component '" << name_ << "'.";
        return false;
    }
    return true;
}

bool SimulationComponent::poll_update(mjTime time) {
    if (period_ <= 0.0) {
        return true;
    }

    if (less(time, next_time_)) {
        return false;
    }

    do {
        next_time_ += period_;
    } while (greater(time, next_time_) || equal(time, next_time_));
    return true;
}

bool SimulationComponent::reset_schedule() noexcept {
    next_time_ = 0.0;
    return true;
}

}  // namespace mujoco_simulation
