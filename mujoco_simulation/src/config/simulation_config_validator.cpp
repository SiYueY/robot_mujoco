#include "config/simulation_config_validator.hpp"

namespace mujoco_simulation {

bool SimulationConfigValidator::validate(const SimulationConfig& config, ConfigError* error) {
    return validate_simulation_config_impl(config, error);
}

}  // namespace mujoco_simulation
