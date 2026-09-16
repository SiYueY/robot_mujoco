#pragma once
#include "romujoco/common/math.hpp"
namespace romujoco {
struct Pose3d {
    Vector3d position{};
    Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
};
struct Twist3d {
    Vector3d linear{};
    Vector3d angular{};
};
}  // namespace romujoco
