#pragma once

#include <cstddef>
#include <string>

#include "mujoco_simulation/reset_options.hpp"
#include "mujoco_simulation/runtime/mujoco_raii.hpp"
#include "mujoco_simulation/status.hpp"

namespace mujoco_simulation {

struct ModelConfig {
  std::string model_path;
  std::string initial_keyframe;
};

class ModelRuntime {
 public:
  ModelRuntime() = default;

  Status load(const ModelConfig& config);
  void unload();

  bool is_loaded() const noexcept;

  Status step();
  Status step(std::size_t count);
  Status forward();
  Status reset(const ResetOptions& options = {});

  const mjModel& model() const;
  mjModel& mutable_model();
  const mjData& data() const;
  mjData& mutable_data();

  double simulation_time() const noexcept;
  double timestep() const noexcept;

  Status copy_data_to(mjData& destination) const;

 private:
  Status validate_loaded_model(const ModelConfig& config) const;

  MjModelPtr model_;
  MjDataPtr data_;
};

}  // namespace mujoco_simulation
