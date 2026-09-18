#pragma once

#include <limits>

namespace romujoco {

// A plain numeric interval without device semantics.  Joint, gripper and any
// future device share this type so limits are only defined once.
struct Limit {
    double min{-std::numeric_limits<double>::infinity()};
    double max{std::numeric_limits<double>::infinity()};
};

}  // namespace romujoco
