#pragma once

#include <string>

#include "config/simulation_config_validator.hpp"

namespace mujoco_simulation {

class SimulationConfigParser {
public:
  bool load_file(const std::string &path, SimulationConfig &config,
                 ConfigError *error = nullptr) const;
};

} // namespace mujoco_simulation
