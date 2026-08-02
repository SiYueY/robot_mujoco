#pragma once

#include "config/simulation_config_validator.hpp"

namespace mujoco_simulation {

bool validate_simulation_config_impl(const SimulationConfig& config, ConfigError* error);

}  // namespace mujoco_simulation
