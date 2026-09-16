#pragma once
#include <cmath>
#include <vector>
#include "romujoco/component/mobile_base.hpp"
namespace romujoco {
struct SwerveModuleGeometry {
    double x{}, y{}, radius{};
};
struct SwerveModuleTarget {
    double steering_position{}, drive_velocity{};
};
class SwerveKinematics {
public:
    explicit SwerveKinematics(std::vector<SwerveModuleGeometry> g) : geometry_(std::move(g)) {}
    bool inverse(
        const PlanarTwist& v, const std::vector<double>& feedback,
        std::vector<SwerveModuleTarget>& out) const {
        if (feedback.size() != geometry_.size() || out.size() != geometry_.size()) return false;
        for (size_t i = 0; i < geometry_.size(); ++i) {
            double x = v.linear_x - v.angular_z * geometry_[i].y,
                   y = v.linear_y + v.angular_z * geometry_[i].x, s = std::hypot(x, y);
            if (s < 1e-12) {
                out[i] = {feedback[i], 0};
                continue;
            }
            double a = std::atan2(y, x), delta = std::remainder(a - feedback[i], 2 * Pi),
                   drive = s / geometry_[i].radius;
            if (std::abs(delta) > Pi / 2) {
                a = std::remainder(a + Pi, 2 * Pi);
                drive = -drive;
            }
            out[i] = {a, drive};
        }
        return true;
    }

private:
    std::vector<SwerveModuleGeometry> geometry_;
};
}  // namespace romujoco
