#pragma once

#include <optional>
#include <utility>

#include "mujoco_simulation/status.hpp"

namespace mujoco_simulation {

template <typename T>
class Result {
 public:
  Result(const T& value) : value_(value) {}
  Result(T&& value) : value_(std::move(value)) {}
  Result(const Status& status) : status_(status) {}
  Result(Status&& status) : status_(std::move(status)) {}

  bool ok() const noexcept { return value_.has_value(); }
  const Status& status() const noexcept { return status_; }
  const T& value() const { return *value_; }
  T& value() { return *value_; }

 private:
  std::optional<T> value_;
  Status status_{Status::internal("Result accessed before initialization.")};
};

}  // namespace mujoco_simulation
