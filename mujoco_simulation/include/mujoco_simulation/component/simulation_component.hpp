#pragma once

#include <string>

#include "mujoco_simulation/mujoco/context.hpp"
#include "mujoco_simulation/visibility.hpp"

namespace mujoco_simulation {

class MUJOCO_SIMULATION_PUBLIC SimulationComponent {
public:
  SimulationComponent(std::string name, double update_rate);
  virtual ~SimulationComponent();
  const std::string &name() const noexcept;
  virtual bool init(const mjContext &context) = 0;
  virtual bool reset(const mjContext &context) = 0;
  virtual bool update(const mjContext &context) = 0;

  bool poll_update(mjTime time);
  bool reset_schedule() noexcept;

protected:
  bool configure(const mjContext &context);

private:
  std::string name_;
  double update_rate_{0.0};
  mjTime next_time_{0.0};
};

} // namespace mujoco_simulation
