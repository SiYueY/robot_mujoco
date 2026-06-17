#pragma once

#include <string>
#include <utility>

namespace mujoco_simulation {

enum class StatusCode {
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
  Internal,
};

class Status {
 public:
  Status() = default;

  static Status Ok() { return Status(); }

  static Status invalid_argument(std::string message) {
    return Status(StatusCode::InvalidArgument, std::move(message));
  }

  static Status already_exists(std::string message) {
    return Status(StatusCode::AlreadyExists, std::move(message));
  }

  static Status invalid_state(std::string message) {
    return Status(StatusCode::InvalidState, std::move(message));
  }

  static Status failed_precondition(std::string message) {
    return Status(StatusCode::FailedPrecondition, std::move(message));
  }

  static Status not_found(std::string message) {
    return Status(StatusCode::NotFound, std::move(message));
  }

  static Status model_load_failed(std::string message) {
    return Status(StatusCode::ModelLoadFailed, std::move(message));
  }

  static Status model_validation_failed(std::string message) {
    return Status(StatusCode::ModelValidationFailed, std::move(message));
  }

  static Status binding_failed(std::string message) {
    return Status(StatusCode::BindingFailed, std::move(message));
  }

  static Status command_rejected(std::string message) {
    return Status(StatusCode::CommandRejected, std::move(message));
  }

  static Status render_failed(std::string message) {
    return Status(StatusCode::RenderFailed, std::move(message));
  }

  static Status thread_failed(std::string message) {
    return Status(StatusCode::ThreadFailed, std::move(message));
  }

  static Status timeout(std::string message) {
    return Status(StatusCode::Timeout, std::move(message));
  }

  static Status internal(std::string message) {
    return Status(StatusCode::Internal, std::move(message));
  }

  bool ok() const noexcept { return code_ == StatusCode::Ok; }
  StatusCode code() const noexcept { return code_; }
  const std::string& message() const noexcept { return message_; }

 private:
  Status(StatusCode code, std::string message) : code_(code), message_(std::move(message)) {}

  StatusCode code_{StatusCode::Ok};
  std::string message_;
};

}  // namespace mujoco_simulation
