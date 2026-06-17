#include <gtest/gtest.h>

#include <memory>

#include "mujoco_simulation/component/component_registry.hpp"

namespace mujoco_simulation {
namespace {

TEST(ComponentRegistryTest, TracksTypedIndexesAcrossAddAndRemove) {
  ComponentRegistry registry;

  auto joint = std::make_unique<JointComponent>(JointConfig{.name = "joint"});
  auto imu =
      std::make_unique<ImuComponent>(ImuConfig{.common = {.name = "imu", .update_rate = 200.0}});
  auto camera = std::make_unique<CameraComponent>(
      CameraConfig{.common = {.name = "camera", .update_rate = 30.0}});
  auto lidar = std::make_unique<LidarComponent>(
      LidarConfig{.common = {.name = "lidar", .update_rate = 10.0}});

  MobileBaseBinding mobile_base_binding;
  mobile_base_binding.differential.emplace();
  auto mobile_base = std::make_unique<MobileBaseComponent>(
      MobileBaseConfig{.name = "base", .type = MobileBaseType::Differential},
      std::move(mobile_base_binding));

  ASSERT_TRUE(registry.add(std::move(joint)).ok());
  ASSERT_TRUE(registry.add(std::move(imu)).ok());
  ASSERT_TRUE(registry.add(std::move(camera)).ok());
  ASSERT_TRUE(registry.add(std::move(lidar)).ok());
  ASSERT_TRUE(registry.add(std::move(mobile_base)).ok());

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

  ASSERT_TRUE(registry.remove("imu").ok());
  EXPECT_FALSE(registry.has_imu("imu"));
  EXPECT_EQ(registry.imu("imu"), nullptr);
  EXPECT_TRUE(registry.has_joint("joint"));
  EXPECT_TRUE(registry.has_camera("camera"));
  EXPECT_TRUE(registry.has_lidar("lidar"));
  EXPECT_TRUE(registry.has_mobile_base("base"));
}

}  // namespace
}  // namespace mujoco_simulation
