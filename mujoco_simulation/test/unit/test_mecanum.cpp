#include <gtest/gtest.h>

#include <limits>

#include "mujoco_simulation/component/mobile_base/mecanum.hpp"

namespace mujoco_simulation {
namespace {

constexpr double kEpsilon = 1e-12;

TEST(MecanumTest, WheelIndicesFollowTheDocumentedOrder) {
  EXPECT_EQ(to_integer(MecanumWheelIndex::FrontLeft), 0U);
  EXPECT_EQ(to_integer(MecanumWheelIndex::FrontRight), 1U);
  EXPECT_EQ(to_integer(MecanumWheelIndex::RearLeft), 2U);
  EXPECT_EQ(to_integer(MecanumWheelIndex::RearRight), 3U);
  EXPECT_EQ(MecanumWheelCount, 4U);
}

TEST(MecanumTest, InverseMapsPlanarTwistToWheelVelocities) {
  const MecanumKinematics mecanum({.wheel_radius = 0.1, .wheel_base = 0.4, .track_width = 0.6});

  Vector4d wheel_velocities{};
  mecanum.inverse({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, wheel_velocities);
  EXPECT_EQ(wheel_velocities, (Vector4d{10.0, 10.0, 10.0, 10.0}));

  mecanum.inverse({0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}, wheel_velocities);
  EXPECT_EQ(wheel_velocities, (Vector4d{-10.0, 10.0, 10.0, -10.0}));

  mecanum.inverse({0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, wheel_velocities);
  EXPECT_EQ(wheel_velocities, (Vector4d{-5.0, 5.0, -5.0, 5.0}));
}

TEST(MecanumTest, ForwardInvertsWheelVelocities) {
  const MecanumKinematics mecanum({.wheel_radius = 0.1, .wheel_base = 0.4, .track_width = 0.6});
  const Vector3d expected_linear{1.2, -0.3, 0.0};
  const Vector3d expected_angular{0.0, 0.0, 0.5};
  Vector4d wheel_velocities{};
  mecanum.inverse(expected_linear, expected_angular, wheel_velocities);

  Vector3d linear{};
  Vector3d angular{};
  mecanum.forward(wheel_velocities, linear, angular);

  EXPECT_NEAR(linear[0], expected_linear[0], kEpsilon);
  EXPECT_NEAR(linear[1], expected_linear[1], kEpsilon);
  EXPECT_NEAR(linear[2], expected_linear[2], kEpsilon);
  EXPECT_NEAR(angular[0], expected_angular[0], kEpsilon);
  EXPECT_NEAR(angular[1], expected_angular[1], kEpsilon);
  EXPECT_NEAR(angular[2], expected_angular[2], kEpsilon);
}

TEST(MecanumTest, RejectsInvalidGeometry) {
  MecanumInfo info{.wheel_radius = 0.1, .wheel_base = 0.4, .track_width = 0.6};

  info.wheel_radius = 0.0;
  EXPECT_THROW(static_cast<void>(MecanumKinematics{info}), std::invalid_argument);

  info.wheel_radius = 0.1;
  info.wheel_base = -0.4;
  EXPECT_THROW(static_cast<void>(MecanumKinematics{info}), std::invalid_argument);

  info.wheel_base = 0.4;
  info.track_width = std::numeric_limits<double>::infinity();
  EXPECT_THROW(static_cast<void>(MecanumKinematics{info}), std::invalid_argument);
}

}  // namespace
}  // namespace mujoco_simulation
