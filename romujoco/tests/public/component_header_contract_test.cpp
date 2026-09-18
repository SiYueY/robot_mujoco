#include <array>
#include <cstddef>
#include <type_traits>
#include <vector>

#include "romujoco/component/camera.hpp"
#include "romujoco/component/gripper.hpp"
#include "romujoco/component/imu.hpp"
#include "romujoco/component/joint.hpp"
#include "romujoco/component/lidar.hpp"
#include "romujoco/component/mobile_base.hpp"
#include "romujoco/data/robot_command.hpp"

int main() {
    static_assert(std::is_same_v<romujoco::JointId, std::size_t>);
    static_assert(std::is_same_v<decltype(romujoco::JointCommand::mode), std::uint8_t>);
    static_assert(std::is_same_v<decltype(romujoco::JointState::mode), std::uint8_t>);
    static_assert(std::is_same_v<romujoco::ImuId, std::size_t>);
    static_assert(std::is_same_v<romujoco::CameraId, std::size_t>);
    static_assert(std::is_same_v<romujoco::LidarId, std::size_t>);
    static_assert(std::is_same_v<romujoco::ComponentId, std::size_t>);
    static_assert(std::is_same_v<romujoco::GripperId, std::size_t>);
    static_assert(romujoco::kGripperFingerCount == 2);
    static_assert(
        std::is_same_v<
            decltype(romujoco::GripperInfo::fingers), std::array<romujoco::GripperFingerInfo, 2>>);
    static_assert(std::is_same_v<decltype(romujoco::GripperCommand::width), double>);
    static_assert(std::is_same_v<decltype(romujoco::GripperState::stalled), bool>);
    static_assert(
        std::is_same_v<decltype(romujoco::RobotCommand::joints), romujoco::JointCommands>);
    static_assert(
        std::is_same_v<decltype(romujoco::RobotCommand::grippers), romujoco::GripperCommands>);
    static_assert(std::is_same_v<
                  decltype(romujoco::RobotCommand::mobile_bases), romujoco::MobileBaseCommands>);
    static_assert(std::is_same_v<romujoco::JointCommands, std::vector<romujoco::JointCommand>>);
    static_assert(std::is_same_v<romujoco::GripperCommands, std::vector<romujoco::GripperCommand>>);
    static_assert(
        std::is_same_v<romujoco::MobileBaseCommands, std::vector<romujoco::MobileBaseCommand>>);
    return 0;
}
