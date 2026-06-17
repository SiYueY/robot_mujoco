#pragma once

#include <mujoco/mujoco.h>

#include <string_view>

#include "mujoco_simulation/status.hpp"

namespace mujoco_simulation {

class SimulationComponent {
 public:
  virtual ~SimulationComponent() = default;

  virtual std::string_view name() const noexcept = 0;
  virtual Status bind(const mjModel& model) = 0;
  virtual Status reset(const mjModel& model, mjData& data) = 0;
};

}  // namespace mujoco_simulation
