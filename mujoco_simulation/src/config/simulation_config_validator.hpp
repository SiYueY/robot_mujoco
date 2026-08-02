#pragma once

#include <cstddef>
#include <string>

#include "mujoco_simulation/config/simulation_config.hpp"

namespace mujoco_simulation {

struct ConfigError {
    static constexpr std::size_t kNoComponent = static_cast<std::size_t>(-1);
    int line{0};
    std::size_t component_index{kNoComponent};
    std::string element;
    std::string attribute;
    std::string message;
};

class SimulationConfigValidator {
public:
    static bool validate(const SimulationConfig& config, ConfigError* error = nullptr);
};

}  // namespace mujoco_simulation
