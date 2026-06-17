#pragma once

#include <functional>
#include <future>
#include <memory>

#include "mujoco_simulation/reset_options.hpp"
#include "mujoco_simulation/status.hpp"

namespace mujoco_simulation {

struct ResetRequest {
  ResetOptions options;
  std::shared_ptr<std::promise<Status>> completion;
};

struct SchedulerCallbacks {
  std::function<double()> timestep_provider;
  std::function<Status()> write_commands;
  std::function<Status()> step_physics;
  std::function<Status()> read_components;
  std::function<Status()> publish_state_snapshot;
  std::function<Status()> sync_viewer_if_due;
  std::function<Status(const ResetOptions&)> reset_runtime;
};

}  // namespace mujoco_simulation
