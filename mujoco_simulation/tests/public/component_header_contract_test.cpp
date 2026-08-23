#include <cstddef>
#include <type_traits>
#include <vector>

#include "mujoco_simulation/component/camera.hpp"
#include "mujoco_simulation/component/imu.hpp"
#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/lidar.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"
#include "mujoco_simulation/data/robot_command.hpp"

int main() {
    static_assert(std::is_same_v<mujoco_simulation::JointId, std::size_t>);
    static_assert(std::is_same_v<decltype(mujoco_simulation::JointCommand::mode), std::uint8_t>);
    static_assert(std::is_same_v<decltype(mujoco_simulation::JointState::mode), std::uint8_t>);
    static_assert(std::is_same_v<mujoco_simulation::ImuId, std::size_t>);
    static_assert(std::is_same_v<mujoco_simulation::CameraId, std::size_t>);
    static_assert(std::is_same_v<mujoco_simulation::LidarId, std::size_t>);
    static_assert(std::is_same_v<mujoco_simulation::MobileBaseId, std::size_t>);
    static_assert(
        std::is_same_v<
            decltype(mujoco_simulation::RobotCommand::joints), mujoco_simulation::JointCommands>);
    static_assert(std::is_same_v<
                  decltype(mujoco_simulation::RobotCommand::mobile_bases),
                  mujoco_simulation::MobileBaseCommands>);
    static_assert(std::is_same_v<
                  mujoco_simulation::JointCommands, std::vector<mujoco_simulation::JointCommand>>);
    static_assert(std::is_same_v<
                  mujoco_simulation::MobileBaseCommands,
                  std::vector<mujoco_simulation::MobileBaseCommand>>);
    return 0;
}
