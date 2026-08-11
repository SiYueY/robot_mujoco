#pragma once

#include <cstddef>
#include <limits>
#include <string>

#include "runtime/context.hpp"

namespace mujoco_simulation {

using ComponentId = std::size_t;
inline constexpr ComponentId kInvalidComponentId = std::numeric_limits<ComponentId>::max();

class SimulationComponent {
public:
    SimulationComponent(std::string name, double period);
    virtual ~SimulationComponent();
    const std::string& name() const noexcept;
    virtual bool init(const mjContext& context) = 0;
    virtual bool reset(const mjContext& context) = 0;
    virtual bool advance(const mjContext& context) = 0;
    virtual bool update(const mjContext& context) = 0;

    bool poll_update(mjTime time);
    bool reset_schedule() noexcept;

protected:
    bool configure(const mjContext& context);

private:
    std::string name_;
    double period_{0.0};
    mjTime next_time_{0.0};
};

}  // namespace mujoco_simulation
