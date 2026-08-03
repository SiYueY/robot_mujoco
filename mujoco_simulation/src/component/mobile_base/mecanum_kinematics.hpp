#pragma once
// Internal mecanum wheel kinematics.  Only MobileBaseComponent consumes this
// implementation; the public component headers expose the configuration and
// command contracts without this machinery.

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "mujoco_simulation/component/mobile_base.hpp"

namespace mujoco_simulation {

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

}  // namespace mujoco_simulation
