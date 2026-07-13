#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "mujoco_simulation/buffer/state_snapshot.hpp"

namespace mujoco_simulation {

class StateBuffer {
 public:
  void write(std::shared_ptr<const StateSnapshot> snapshot) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_ = std::move(snapshot);
  }

  std::shared_ptr<const StateSnapshot> read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_;
  }

  bool joint_state(std::string name, JointState* out) const {
    if (out == nullptr) {
      return false;
    }
    const std::shared_ptr<const StateSnapshot> snapshot = read();
    if (snapshot == nullptr) {
      return false;
    }
    return snapshot->joint_state(name, out);
  }

  bool mobile_base_state(std::string name, MobileBaseState* out) const {
    if (out == nullptr) {
      return false;
    }
    const std::shared_ptr<const StateSnapshot> snapshot = read();
    if (snapshot == nullptr) {
      return false;
    }
    return snapshot->mobile_base_state(name, out);
  }

  bool imu_state(std::string name, ImuState* out) const {
    if (out == nullptr) {
      return false;
    }
    const std::shared_ptr<const StateSnapshot> snapshot = read();
    if (snapshot == nullptr) {
      return false;
    }
    return snapshot->imu_state(name, out);
  }

  bool lidar_state(std::string name, LidarState* out) const {
    if (out == nullptr) {
      return false;
    }
    const std::shared_ptr<const StateSnapshot> snapshot = read();
    if (snapshot == nullptr) {
      return false;
    }
    return snapshot->lidar_state(name, out);
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    current_.reset();
  }

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<const StateSnapshot> current_;
};

}  // namespace mujoco_simulation
