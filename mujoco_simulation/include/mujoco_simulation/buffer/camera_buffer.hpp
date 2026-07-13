#pragma once

#include <mutex>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/component/camera/camera_data.hpp"

namespace mujoco_simulation {

class CameraBuffer {
 public:
  void write(std::string camera_name, CameraState state) {
    std::lock_guard<std::mutex> lock(mutex_);
    states_[std::string(camera_name)] = std::move(state);
  }

  bool read(std::string camera_name, CameraState* out) const {
    if (out == nullptr) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = states_.find(std::string(camera_name));
    if (it == states_.end()) {
      return false;
    }
    *out = it->second;
    return true;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    states_.clear();
  }

 private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, CameraState> states_;
};

}  // namespace mujoco_simulation
