#include "mujoco_simulation/buffer/state_buffer.hpp"

namespace mujoco_simulation {

namespace {
template <typename State>
bool read_by_id(const StateSnapshots<State> &states, ComponentId id,
                State &out) {
  if (states == nullptr || id >= states->size() || (*states)[id] == nullptr) {
    return false;
  }
  out = *(*states)[id];
  return true;
}
} // namespace

void StateBuffer::write(std::shared_ptr<const RobotState> snapshot) {
  std::atomic_store_explicit(&current_, std::move(snapshot),
                             std::memory_order_release);
}

std::shared_ptr<const RobotState> StateBuffer::read() const {
  return std::atomic_load_explicit(&current_, std::memory_order_acquire);
}

bool StateBuffer::read_joint_state(JointId id, JointState &out) const {
  const auto snapshot = read();
  return snapshot != nullptr && read_by_id(snapshot->joints, id, out);
}

bool StateBuffer::read_mobile_base_state(MobileBaseId id,
                                         MobileBaseState &out) const {
  const auto snapshot = read();
  return snapshot != nullptr && read_by_id(snapshot->mobile_bases, id, out);
}

bool StateBuffer::read_imu_state(ImuId id, ImuState &out) const {
  const auto snapshot = read();
  return snapshot != nullptr && read_by_id(snapshot->imus, id, out);
}

bool StateBuffer::read_camera_state(CameraId id, CameraState &out) const {
  const auto snapshot = read();
  return snapshot != nullptr && read_by_id(snapshot->cameras, id, out);
}

bool StateBuffer::read_lidar_state(LidarId id, LidarState &out) const {
  const auto snapshot = read();
  return snapshot != nullptr && read_by_id(snapshot->lidars, id, out);
}

void StateBuffer::clear() {
  std::atomic_store_explicit(&current_, std::shared_ptr<const RobotState>{},
                             std::memory_order_release);
}

} // namespace mujoco_simulation
