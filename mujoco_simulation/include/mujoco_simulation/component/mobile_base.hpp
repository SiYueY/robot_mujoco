#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
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

class MecanumKinematics {
public:
    explicit MecanumKinematics(const MecanumInfo& info)
    : wheel_radius_(info.wheel_radius),
      rotation_coefficient_((info.wheel_base + info.track_width) * 0.5) {
        if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0 ||
            !std::isfinite(info.wheel_base) || info.wheel_base <= 0.0 ||
            !std::isfinite(info.track_width) || info.track_width <= 0.0 ||
            !std::isfinite(rotation_coefficient_) || rotation_coefficient_ <= 0.0)
            throw std::invalid_argument("mecanum dimensions must be finite and positive");
    }
    void inverse(
        const Vector3d& linear, const Vector3d& angular, Vector4d& wheel_angular) const noexcept {
        const double forward = linear[0];
        const double lateral = linear[1];
        const double yaw = rotation_coefficient_ * angular[2];
        wheel_angular = {
            (forward - lateral - yaw) / wheel_radius_, (forward + lateral + yaw) / wheel_radius_,
            (forward + lateral - yaw) / wheel_radius_, (forward - lateral + yaw) / wheel_radius_};
    }
    void forward(
        const Vector4d& wheel_angular, Vector3d& linear, Vector3d& angular) const noexcept {
        const double fl = wheel_angular[static_cast<std::size_t>(MecanumWheelIndex::FrontLeft)];
        const double fr = wheel_angular[static_cast<std::size_t>(MecanumWheelIndex::FrontRight)];
        const double rl = wheel_angular[static_cast<std::size_t>(MecanumWheelIndex::RearLeft)];
        const double rr = wheel_angular[static_cast<std::size_t>(MecanumWheelIndex::RearRight)];
        linear = {
            wheel_radius_ * (fl + fr + rl + rr) * 0.25, wheel_radius_ * (-fl + fr + rl - rr) * 0.25,
            0.0};
        angular = {0.0, 0.0, wheel_radius_ * (-fl + fr - rl + rr) / (4.0 * rotation_coefficient_)};
    }

private:
    double wheel_radius_;
    double rotation_coefficient_;
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
    MobileBaseControlMode mode{MobileBaseControlMode::Twist};
    Vector3d base_linear{};
    Vector3d base_angular{};
    Vector4d wheel_linear{};
    Vector4d wheel_angular{};
};

using MobileBaseCommands = std::vector<MobileBaseCommand>;

struct MobileBaseState {
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
