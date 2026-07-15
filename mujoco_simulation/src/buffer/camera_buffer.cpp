#include "mujoco_simulation/buffer/camera_buffer.hpp"

namespace mujoco_simulation {

void CameraBuffer::write(std::string camera_name, CameraState state) {
  std::lock_guard<std::mutex> lock(mutex_);
  states_[std::string(camera_name)] = std::make_shared<const CameraState>(std::move(state));
}

bool CameraBuffer::read(std::string camera_name, CameraState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::shared_ptr<const CameraState> state = read_shared(camera_name);
  if (state == nullptr) {
    return false;
  }
  *out = *state;
  return true;
}

std::shared_ptr<const CameraState> CameraBuffer::read_shared(std::string camera_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = states_.find(std::string(camera_name));
  if (it == states_.end()) {
    return nullptr;
  }
  return it->second;
}

void CameraBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  states_.clear();
}

}  // namespace mujoco_simulation
