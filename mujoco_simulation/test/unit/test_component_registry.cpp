#include <gtest/gtest.h>

#include <memory>

#include "mujoco_simulation/component/component_registry.hpp"

namespace mujoco_simulation {
namespace {

TEST(ComponentRegistryTest, TracksTypedIndexesAcrossAddAndRemove) {
  ComponentRegistry registry;

  auto joint = std::make_unique<JointComponent>(JointInfo{.joint = "joint"});
  auto imu =
      std::make_unique<ImuComponent>(ImuInfo{.common = {.name = "imu", .update_rate = 200.0}});
  auto camera = std::make_unique<CameraComponent>(
      CameraConfig{.common = {.name = "camera", .update_rate = 30.0}});
  auto lidar =
      std::make_unique<LidarComponent>(LidarInfo{.common = {.name = "lidar", .update_rate = 10.0}});

  auto mobile_base = std::make_unique<MobileBaseComponent>(
      MobileBaseInfo{.name = "base", .type = MobileBaseType::Differential});

  ASSERT_TRUE(registry.add(std::move(joint)));
  ASSERT_TRUE(registry.add(std::move(imu)));
  ASSERT_TRUE(registry.add(std::move(camera)));
  ASSERT_TRUE(registry.add(std::move(lidar)));
  ASSERT_TRUE(registry.add(std::move(mobile_base)));

  EXPECT_TRUE(registry.has_joint("joint"));
  EXPECT_TRUE(registry.has_imu("imu"));
  EXPECT_TRUE(registry.has_camera("camera"));
  EXPECT_TRUE(registry.has_lidar("lidar"));
  EXPECT_TRUE(registry.has_mobile_base("base"));
  EXPECT_NE(registry.joint("joint"), nullptr);
  EXPECT_NE(registry.imu("imu"), nullptr);
  EXPECT_NE(registry.camera("camera"), nullptr);
  EXPECT_NE(registry.lidar("lidar"), nullptr);
  EXPECT_NE(registry.mobile_base("base"), nullptr);
  EXPECT_EQ(registry.joints().size(), 1u);
  EXPECT_EQ(registry.imus().size(), 1u);
  EXPECT_EQ(registry.cameras().size(), 1u);
  EXPECT_EQ(registry.lidars().size(), 1u);
  EXPECT_EQ(registry.mobile_bases().size(), 1u);

  ASSERT_TRUE(registry.remove("imu"));
  EXPECT_FALSE(registry.has_imu("imu"));
  EXPECT_EQ(registry.imu("imu"), nullptr);
  EXPECT_TRUE(registry.has_joint("joint"));
  EXPECT_TRUE(registry.has_camera("camera"));
  EXPECT_TRUE(registry.has_lidar("lidar"));
  EXPECT_TRUE(registry.has_mobile_base("base"));
}

}  // namespace
}  // namespace mujoco_simulation
