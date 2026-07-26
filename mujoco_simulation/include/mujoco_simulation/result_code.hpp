#pragma once

namespace mujoco_simulation {

enum class ResultCode {
  Ok = 0,
  InvalidArgument,
  AlreadyExists,
  InvalidState,
  FailedPrecondition,
  NotFound,
  ModelLoadFailed,
  ModelValidationFailed,
  BindingFailed,
  CommandRejected,
  RenderFailed,
  ThreadFailed,
  Timeout,
  Unimplemented,
  Internal,
};

} // namespace mujoco_simulation
