#include <gtest/gtest.h>

#include "mujoco_simulation/common/math.hpp"

namespace mujoco_simulation {
namespace {

TEST(MathTest, ComparesValuesWithTolerance) {
  constexpr double epsilon = kDefaultEpsilon<double>;
  EXPECT_TRUE(equal(1.0, 1.0 + 0.5 * epsilon));
  EXPECT_FALSE(equal(1.0, 1.0 + 2.0 * epsilon));
  EXPECT_TRUE(less(1.0, 1.0 + 2.0 * epsilon));
  EXPECT_FALSE(less(1.0, 1.0 + 0.5 * epsilon));
  EXPECT_TRUE(greater(1.0 + 2.0 * epsilon, 1.0));
}

TEST(MathTest, ComparesIntegralValuesExactly) {
  EXPECT_TRUE(equal(3, 3));
  EXPECT_TRUE(less(2, 3));
  EXPECT_TRUE(greater(3, 2));
}

}  // namespace
}  // namespace mujoco_simulation
