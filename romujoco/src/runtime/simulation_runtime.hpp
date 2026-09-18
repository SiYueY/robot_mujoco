#pragma once

#include <cstddef>
#include <string>

#include <mujoco/mujoco.h>

#include "romujoco/config/simulation_config.hpp"

#include "runtime/simulation_context.hpp"

namespace romujoco {

class Simulation;
class SimulationRuntimeTestPeer;

class SimulationRuntime {
public:
    SimulationRuntime() = default;
    ~SimulationRuntime() = default;

    SimulationRuntime(const SimulationRuntime&) = delete;
    SimulationRuntime& operator=(const SimulationRuntime&) = delete;
    SimulationRuntime(SimulationRuntime&& other) noexcept;
    SimulationRuntime& operator=(SimulationRuntime&& other) noexcept;

    bool init(const ModelConfig& config);
    bool is_initialized() const noexcept;

    bool step();
    bool step(std::size_t count);
    bool forward();
    bool set_timestep(double timestep);
    bool reset();
    bool reset(std::string keyframe_name);

    // 仿真时间
    double time() const noexcept;
    // 时间步长
    double timestep() const noexcept;

private:
    friend class Simulation;
    friend class SimulationRuntimeTestPeer;

    static bool load_model(const std::string& model_path, mjModel*& model);
    bool reset_to_default();
    bool reset_to_keyframe(int keyframe_id);
    const SimulationContext& context() const noexcept;

    bool initialized_{false};
    SimulationContext context_{};
};

}  // namespace romujoco
