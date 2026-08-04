#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include "mujoco_simulation/common/math.hpp"
namespace mujoco_simulation {

using MobileBaseId = std::size_t;

enum class MecanumWheelIndex : std::size_t {
    FrontLeft = 0,
    FrontRight,
    RearLeft,
    RearRight,
    Count,
};
inline constexpr std::size_t MecanumWheelCount = static_cast<std::size_t>(MecanumWheelIndex::Count);

struct MecanumInfo {
    double wheel_radius{0.0};
    double wheel_base{0.0};
    double track_width{0.0};
};

enum class MobileBaseType { Mecanum };
enum class MobileBaseControlMode { Twist, WheelLinear, WheelAngular };

struct WheelInfo {
    std::string wheel_name;
    std::string actuator_name;
    double damping{0.0};
};
using MecanumWheelInfo = std::array<WheelInfo, MecanumWheelCount>;

struct MobileBaseInfo {
    MobileBaseId id{0};
    std::string mobile_base_name;
    MobileBaseType type{MobileBaseType::Mecanum};
    std::string base_frame_id{"base_link"};
    std::string odom_frame_id{"odom"};
    std::string base_body_name;
    MecanumInfo mecanum_info;
    MecanumWheelInfo mecanum_wheels;
    double period{0.0};
};

struct MobileBaseCommand {
    MobileBaseId id{0};
    MobileBaseControlMode mode{MobileBaseControlMode::Twist};
    Vector3d base_linear{};
    Vector3d base_angular{};
    Vector4d wheel_linear{};
    Vector4d wheel_angular{};
};

using MobileBaseCommands = std::vector<MobileBaseCommand>;

struct MobileBaseState {
    MobileBaseId id{0};
    double timestamp{0.0};
    std::string odom_frame_id;
    std::string base_frame_id;
    Vector3d pose{};
    Vector36d pose_covariance{};
    Vector3d base_linear{};
    Vector3d base_angular{};
    Vector36d twist_covariance{};
    Vector4d wheel_linear{};
    Vector4d wheel_angular{};
};
}  // namespace mujoco_simulation
