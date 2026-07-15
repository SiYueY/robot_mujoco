#pragma once

#include <mujoco/mujoco.h>

#include <cstddef>
#include <memory>
#include <string_view>

#include "mujoco_simulation/reset_options.hpp"
#include "mujoco_simulation/result_code.hpp"
#include "mujoco_simulation/simulation_config.hpp"

namespace mujoco_simulation {

class MuJoCoViewer;
class ModelRuntime;

class ViewerRuntimeHandle {
 public:
  ViewerRuntimeHandle() = default;

  bool valid() const noexcept;

 private:
  friend class ModelRuntime;
  friend class MuJoCoViewer;

  explicit ViewerRuntimeHandle(const ModelRuntime* runtime) noexcept;

  const mjModel* model() const noexcept;
  const mjData* data() const noexcept;

  const ModelRuntime* runtime_{nullptr};
};

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
  ResultCode reset();
  ResultCode reset_to_keyframe_name(std::string_view keyframe_name);
  ResultCode reset_to_keyframe_id(int keyframe_id);

  const mjModel& model() const;
  mjModel& mutable_model();
  const mjData& data() const;
  mjData& mutable_data();

  double simulation_time() const noexcept;
  double timestep() const noexcept;
  ViewerRuntimeHandle viewer_runtime_handle() const noexcept;

 private:
  ResultCode validate_loaded_model(const ModelConfig& config) const;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mujoco_simulation
