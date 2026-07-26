#include "mujoco_simulation/buffer/state_buffer.hpp"

namespace mujoco_simulation {

void StateBuffer::write(std::shared_ptr<const RobotState> snapshot) {
  std::atomic_store_explicit(&current_, std::move(snapshot), std::memory_order_release);
}

std::shared_ptr<const RobotState> StateBuffer::read() const {
  return std::atomic_load_explicit(&current_, std::memory_order_acquire);
}

bool StateBuffer::read_joint_state(std::string name, JointState& out) const {
  const std::shared_ptr<const RobotState> snapshot = read();
  if (snapshot == nullptr) {
    return false;
  }
  return snapshot->read_state(name, out);
}

bool StateBuffer::read_mobile_base_state(std::string name, MobileBaseState& out) const {
  const std::shared_ptr<const RobotState> snapshot = read();
  if (snapshot == nullptr) {
    return false;
  }
  return snapshot->read_state(name, out);
}

bool StateBuffer::read_imu_state(std::string name, ImuState& out) const {
  const std::shared_ptr<const RobotState> snapshot = read();
  if (snapshot == nullptr) {
    return false;
  }
  return snapshot->read_state(name, out);
}

bool StateBuffer::read_camera_state(std::string name, CameraState& out) const {
  const std::shared_ptr<const RobotState> snapshot = read();
  if (snapshot == nullptr) {
    return false;
  }
  return snapshot->read_state(name, out);
}

bool StateBuffer::read_lidar_state(std::string name, LidarState& out) const {
  const std::shared_ptr<const RobotState> snapshot = read();
  if (snapshot == nullptr) {
    return false;
  }
  return snapshot->read_state(name, out);
}

void StateBuffer::clear() {
  std::atomic_store_explicit(&current_, std::shared_ptr<const RobotState>{},
                             std::memory_order_release);
}

}  // namespace mujoco_simulation
