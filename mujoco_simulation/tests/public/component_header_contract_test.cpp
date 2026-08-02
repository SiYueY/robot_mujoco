#include <cstddef>
#include <type_traits>

#include "mujoco_simulation/component/camera.hpp"
#include "mujoco_simulation/component/imu.hpp"
#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/lidar.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"
#include "mujoco_simulation/data/robot_command.hpp"

int main() {
  static_assert(std::is_same_v<mujoco_simulation::JointId, std::size_t>);
  static_assert(std::is_same_v<mujoco_simulation::ImuId, std::size_t>);
  static_assert(std::is_same_v<mujoco_simulation::CameraId, std::size_t>);
  static_assert(std::is_same_v<mujoco_simulation::LidarId, std::size_t>);
  static_assert(std::is_same_v<mujoco_simulation::MobileBaseId, std::size_t>);
  static_assert(
      std::is_same_v<decltype(mujoco_simulation::RobotCommand::joints),
                     mujoco_simulation::JointCommandBatch>);
  static_assert(
      std::is_same_v<decltype(mujoco_simulation::RobotCommand::mobile_bases),
                     mujoco_simulation::MobileBaseCommandBatch>);
  static_assert(std::is_same_v<
                mujoco_simulation::JointCommandBatch,
                std::vector<std::optional<mujoco_simulation::JointCommand>>>);
  static_assert(
      std::is_same_v<
          mujoco_simulation::MobileBaseCommandBatch,
          std::vector<std::optional<mujoco_simulation::MobileBaseCommand>>>);
  return 0;
}
