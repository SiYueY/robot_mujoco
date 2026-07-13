#pragma once

#include <mujoco/mujoco.h>

#include <cstddef>
#include <memory>

#include "mujoco_simulation/reset_options.hpp"
#include "mujoco_simulation/result_code.hpp"
#include "mujoco_simulation/simulation_config.hpp"

namespace mujoco_simulation {

class ModelRuntime {
 public:
  ModelRuntime();
  ~ModelRuntime();

  ModelRuntime(const ModelRuntime&) = delete;
  ModelRuntime& operator=(const ModelRuntime&) = delete;
  ModelRuntime(ModelRuntime&&) noexcept;
  ModelRuntime& operator=(ModelRuntime&&) noexcept;

  ResultCode load(const ModelConfig& config);
  void unload();

  bool is_loaded() const noexcept;

  ResultCode step();
  ResultCode step(std::size_t count);
  ResultCode forward();
  ResultCode reset(const ResetOptions& options = {});

  const mjModel& model() const;
  mjModel& mutable_model();
  const mjData& data() const;
  mjData& mutable_data();

  double simulation_time() const noexcept;
  double timestep() const noexcept;

 private:
  ResultCode validate_loaded_model(const ModelConfig& config) const;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_simulation
