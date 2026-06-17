#include <gtest/gtest.h>

#include <memory>

#include "mujoco_simulation/buffer/state_buffer.hpp"

namespace mujoco_simulation {
namespace {

TEST(StateBufferTest, TypedReadsReturnInvalidStateBeforeFirstPublish) {
  StateBuffer buffer;

  const auto joint = buffer.joint_state("joint");
  EXPECT_FALSE(joint.ok());
  EXPECT_EQ(joint.status().code(), StatusCode::InvalidState);

  const auto imu = buffer.imu_sample("imu");
  EXPECT_FALSE(imu.ok());
  EXPECT_EQ(imu.status().code(), StatusCode::InvalidState);

  const auto lidar = buffer.lidar_sample("lidar");
  EXPECT_FALSE(lidar.ok());
  EXPECT_EQ(lidar.status().code(), StatusCode::InvalidState);

  const auto mobile_base = buffer.mobile_base_state("base");
  EXPECT_FALSE(mobile_base.ok());
  EXPECT_EQ(mobile_base.status().code(), StatusCode::InvalidState);
}

TEST(StateBufferTest, TypedReadsDelegateToSnapshotLookups) {
  StateBuffer buffer;
  auto snapshot = std::make_shared<SimulationStateSnapshot>();
  snapshot->joints.emplace("joint", JointState{.name = "joint", .position = 1.5});
  snapshot->imus.emplace("imu", ImuSample{.frame_id = "imu_link"});
  snapshot->lidars.emplace("lidar", LidarSample{.frame_id = "lidar_link"});
  snapshot->mobile_bases.emplace("base", MobileBaseState{.base_frame_id = "base_link"});
  buffer.publish(snapshot);

  const auto joint = buffer.joint_state("joint");
  ASSERT_TRUE(joint.ok()) << joint.status().message();
  EXPECT_EQ(joint.value().name, "joint");
  EXPECT_DOUBLE_EQ(joint.value().position, 1.5);

  const auto imu = buffer.imu_sample("imu");
  ASSERT_TRUE(imu.ok()) << imu.status().message();
  EXPECT_EQ(imu.value().frame_id, "imu_link");

  const auto lidar = buffer.lidar_sample("lidar");
  ASSERT_TRUE(lidar.ok()) << lidar.status().message();
  EXPECT_EQ(lidar.value().frame_id, "lidar_link");

  const auto mobile_base = buffer.mobile_base_state("base");
  ASSERT_TRUE(mobile_base.ok()) << mobile_base.status().message();
  EXPECT_EQ(mobile_base.value().base_frame_id, "base_link");

  const auto missing_joint = buffer.joint_state("missing_joint");
  EXPECT_FALSE(missing_joint.ok());
  EXPECT_EQ(missing_joint.status().code(), StatusCode::NotFound);
}

}  // namespace
}  // namespace mujoco_simulation
