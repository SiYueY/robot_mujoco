#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "romujoco/component/mobile_base/mecanum.hpp"

namespace romujoco {

class MecanumKinematics {
public:
    explicit MecanumKinematics(const MecanumMobileBaseInfo& info)
    : rotation_coefficient_((info.wheel_base + info.track_width) * 0.5) {
        if (!std::isfinite(info.wheel_base) || info.wheel_base <= 0.0 ||
            !std::isfinite(info.track_width) || info.track_width <= 0.0 ||
            !std::isfinite(rotation_coefficient_) || rotation_coefficient_ <= 0.0)
            throw std::invalid_argument("mecanum dimensions must be finite and positive");
    }

    void inverse(const PlanarTwist& twist, Vector4d& wheel_linear) const noexcept {
        const double forward = twist.linear_x;
        const double lateral = twist.linear_y;
        const double yaw = rotation_coefficient_ * twist.angular_z;
        wheel_linear = {
            forward - lateral - yaw, forward + lateral + yaw, forward + lateral - yaw,
            forward - lateral + yaw};
    }

    void forward(const Vector4d& wheel_linear, Vector3d& linear, Vector3d& angular) const noexcept {
        const double front_left =
            wheel_linear[static_cast<std::size_t>(MecanumWheelIndex::FrontLeft)];
        const double front_right =
            wheel_linear[static_cast<std::size_t>(MecanumWheelIndex::FrontRight)];
        const double rear_left =
            wheel_linear[static_cast<std::size_t>(MecanumWheelIndex::RearLeft)];
        const double rear_right =
            wheel_linear[static_cast<std::size_t>(MecanumWheelIndex::RearRight)];
        linear = {
            (front_left + front_right + rear_left + rear_right) * 0.25,
            (-front_left + front_right + rear_left - rear_right) * 0.25, 0.0};
        angular = {
            0.0, 0.0,
            (-front_left + front_right - rear_left + rear_right) / (4.0 * rotation_coefficient_)};
    }

private:
    // (wheel_base + track_width) / 2, the lever arm of the yaw term.
    double rotation_coefficient_;
};

}  // namespace romujoco
