#include "simulation/simulation_impl.hpp"

#include <mutex>

namespace mujoco_simulation {

bool Simulation::Impl::read_state(
    std::shared_ptr<const RobotState> &out) const {
  out = state_buffer_.read();
  return out != nullptr;
}

bool Simulation::Impl::read_state(RobotState &out) const {
  std::shared_ptr<const RobotState> snapshot;
  if (!read_state(snapshot))
    return false;
  out = *snapshot;
  return true;
}

bool Simulation::Impl::read_state(JointId id, JointState &out) const {
  return state_buffer_.read_joint_state(id, out);
}
bool Simulation::Impl::read_state(ImuId id, ImuState &out) const {
  return state_buffer_.read_imu_state(id, out);
}
bool Simulation::Impl::read_state(CameraId id, CameraState &out) const {
  return state_buffer_.read_camera_state(id, out);
}
bool Simulation::Impl::read_state(LidarId id, LidarState &out) const {
  return state_buffer_.read_lidar_state(id, out);
}
bool Simulation::Impl::read_state(MobileBaseId id, MobileBaseState &out) const {
  return state_buffer_.read_mobile_base_state(id, out);
}

bool Simulation::Impl::read_state(JointStates &out) const {
  const auto snapshot = state_buffer_.read();
  if (snapshot == nullptr)
    return false;
  out = snapshot->joints;
  return out != nullptr;
}
bool Simulation::Impl::read_state(ImuStates &out) const {
  const auto snapshot = state_buffer_.read();
  if (snapshot == nullptr)
    return false;
  out = snapshot->imus;
  return out != nullptr;
}
bool Simulation::Impl::read_state(CameraStates &out) const {
  const auto snapshot = state_buffer_.read();
  if (snapshot == nullptr)
    return false;
  out = snapshot->cameras;
  return out != nullptr;
}
bool Simulation::Impl::read_state(LidarStates &out) const {
  const auto snapshot = state_buffer_.read();
  if (snapshot == nullptr)
    return false;
  out = snapshot->lidars;
  return out != nullptr;
}
bool Simulation::Impl::read_state(MobileBaseStates &out) const {
  const auto snapshot = state_buffer_.read();
  if (snapshot == nullptr)
    return false;
  out = snapshot->mobile_bases;
  return out != nullptr;
}

std::uint64_t Simulation::Impl::step_count() const { return step_.load(); }

SimulationStatus Simulation::Impl::status() const {
  if (runtime_failed_.load())
    return SimulationStatus::Error;
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ != nullptr)
    return scheduler_->status();
  std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
  if (runtime_ == nullptr || !runtime_->is_initialized())
    return SimulationStatus::Uninitialized;
  return SimulationStatus::Stopped;
}

double Simulation::Impl::time() const {
  std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
  if (runtime_ == nullptr || !runtime_->is_initialized())
    return 0.0;
  return runtime_->time();
}

} // namespace mujoco_simulation
