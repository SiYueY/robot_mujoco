#pragma once

#include <memory>
#include <mutex>
#include <string_view>

#include "mujoco_simulation/buffer/simulation_state_snapshot.hpp"

namespace mujoco_simulation {

class StateBuffer {
 public:
  void publish(std::shared_ptr<const SimulationStateSnapshot> snapshot) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_ = std::move(snapshot);
  }

  std::shared_ptr<const SimulationStateSnapshot> read() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_;
  }

  Result<JointState> joint_state(std::string_view name) const {
    const std::shared_ptr<const SimulationStateSnapshot> snapshot = read();
    if (snapshot == nullptr) {
      return Status::invalid_state("MuJoCo state snapshot is not available yet.");
    }
    return snapshot->joint_state(name);
  }

  Result<MobileBaseState> mobile_base_state(std::string_view name) const {
    const std::shared_ptr<const SimulationStateSnapshot> snapshot = read();
    if (snapshot == nullptr) {
      return Status::invalid_state("MuJoCo state snapshot is not available yet.");
    }
    return snapshot->mobile_base_state(name);
  }

  Result<ImuSample> imu_sample(std::string_view name) const {
    const std::shared_ptr<const SimulationStateSnapshot> snapshot = read();
    if (snapshot == nullptr) {
      return Status::invalid_state("MuJoCo state snapshot is not available yet.");
    }
    return snapshot->imu_sample(name);
  }

  Result<LidarSample> lidar_sample(std::string_view name) const {
    const std::shared_ptr<const SimulationStateSnapshot> snapshot = read();
    if (snapshot == nullptr) {
      return Status::invalid_state("MuJoCo state snapshot is not available yet.");
    }
    return snapshot->lidar_sample(name);
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    current_.reset();
  }

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<const SimulationStateSnapshot> current_;
};

}  // namespace mujoco_simulation
