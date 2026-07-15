#pragma once

#include <memory>
#include <string>

#include "mujoco_simulation/buffer/state_snapshot.hpp"

namespace mujoco_simulation {

class StateBuffer {
 public:
  void write(std::shared_ptr<const StateSnapshot> snapshot);

  std::shared_ptr<const StateSnapshot> read() const;

  bool read_joint_state(std::string name, JointState* out) const;

  bool read_mobile_base_state(std::string name, MobileBaseState* out) const;

  bool read_imu_state(std::string name, ImuState* out) const;

  bool read_lidar_state(std::string name, LidarState* out) const;

  void clear();

 private:
  std::shared_ptr<const StateSnapshot> current_;
};

}  // namespace mujoco_simulation
