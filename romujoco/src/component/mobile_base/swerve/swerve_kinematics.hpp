#pragma once

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "romujoco/common/math.hpp"
#include "romujoco/component/mobile_base.hpp"

namespace romujoco {

// Placement and wheel radius of one swerve module in the base frame.
struct SwerveModuleGeometry {
    double position_x{0.0};
    double position_y{0.0};
    double radius{0.0};
};

// Steering position and drive velocity requested for one swerve module.
struct SwerveModuleTarget {
    double steering_position{0.0};
    double drive_velocity{0.0};
};

class SwerveKinematics {
public:
    explicit SwerveKinematics(std::vector<SwerveModuleGeometry> geometry)
    : geometry_(std::move(geometry)) {}

    // Converts a planar base twist into per-module targets.  Steering always
    // takes the shortest path; beyond a quarter turn the drive direction is
    // reversed instead.  A module without translation keeps its steering angle.
    bool inverse(
        const PlanarTwist& twist, const std::vector<double>& feedback,
        std::vector<SwerveModuleTarget>& targets) const {
        if (feedback.size() != geometry_.size() || targets.size() != geometry_.size()) return false;
        for (std::size_t index = 0; index < geometry_.size(); ++index) {
            const double module_x = twist.linear_x - twist.angular_z * geometry_[index].position_y;
            const double module_y = twist.linear_y + twist.angular_z * geometry_[index].position_x;
            const double speed = std::hypot(module_x, module_y);
            if (speed < 1e-12) {
                targets[index] = {feedback[index], 0.0};
                continue;
            }
            double steering_angle = std::atan2(module_y, module_x);
            double drive_velocity = speed / geometry_[index].radius;
            const double angle_error = std::remainder(steering_angle - feedback[index], 2.0 * kPi);
            if (std::abs(angle_error) > kPi / 2.0) {
                steering_angle = std::remainder(steering_angle + kPi, 2.0 * kPi);
                drive_velocity = -drive_velocity;
            }
            targets[index] = {steering_angle, drive_velocity};
        }
        return true;
    }

private:
    std::vector<SwerveModuleGeometry> geometry_;
};

}  // namespace romujoco
