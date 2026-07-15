#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/component/camera/camera_data.hpp"

namespace mujoco_simulation {

class CameraBuffer {
 public:
  void write(std::string camera_name, CameraState state);

  bool read(std::string camera_name, CameraState* out) const;

  std::shared_ptr<const CameraState> read_shared(std::string camera_name) const;

  void clear();

 private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<const CameraState>> states_;
};

}  // namespace mujoco_simulation
