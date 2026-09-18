#pragma once

#include <string>

#include "romujoco/component/component_id.hpp"

#include "runtime/simulation_context.hpp"

namespace romujoco {

class SimulationComponent {
public:
    SimulationComponent(std::string name, double period);
    virtual ~SimulationComponent();
    const std::string& name() const noexcept;
    virtual bool init(const SimulationContext& context) = 0;
    virtual bool reset(const SimulationContext& context) = 0;
    virtual bool advance(const SimulationContext& context) = 0;
    virtual bool update(const SimulationContext& context) = 0;

    bool poll_update(SimTime time);
    bool reset_schedule() noexcept;

protected:
    bool configure(const SimulationContext& context);

private:
    std::string name_;
    double period_{0.0};
    SimTime next_time_{0.0};
};

}  // namespace romujoco
