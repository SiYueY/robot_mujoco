#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>

#include "mujoco_simulation/component/camera/camera_sample.hpp"
#include "mujoco_simulation/result.hpp"

namespace mujoco_simulation {

class CameraBuffer {
 public:
  void publish(std::string_view camera_name, std::shared_ptr<const CameraSample> sample) {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_[std::string(camera_name)] = std::move(sample);
  }

  Result<std::shared_ptr<const CameraSample>> read(std::string_view camera_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = samples_.find(std::string(camera_name));
    if (it == samples_.end() || it->second == nullptr) {
      return Status::not_found("Camera sample not found: " + std::string(camera_name));
    }
    return it->second;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
  }

 private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<const CameraSample>> samples_;
};

}  // namespace mujoco_simulation
