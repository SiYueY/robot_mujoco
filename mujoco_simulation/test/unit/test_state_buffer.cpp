#include <gtest/gtest.h>

#include <memory>

#include "mujoco_simulation/buffer/state_buffer.hpp"

namespace mujoco_simulation {
namespace {

TEST(StateBufferTest, TypedReadsReturnInvalidStateBeforeFirstPublish) {
  StateBuffer buffer;
  JointState joint{.name = "sentinel"};
  ImuState imu{.frame_id = "sentinel"};
  LidarState lidar{.frame_id = "sentinel"};
  MobileBaseState mobile_base{.base_frame_id = "sentinel"};

  EXPECT_FALSE(buffer.joint_state("joint", &joint));
  EXPECT_EQ(joint.name, "sentinel");
  EXPECT_FALSE(buffer.imu_state("imu", &imu));
  EXPECT_EQ(imu.frame_id, "sentinel");
  EXPECT_FALSE(buffer.lidar_state("lidar", &lidar));
  EXPECT_EQ(lidar.frame_id, "sentinel");
  EXPECT_FALSE(buffer.mobile_base_state("base", &mobile_base));
  EXPECT_EQ(mobile_base.base_frame_id, "sentinel");
}

TEST(StateBufferTest, TypedReadsDelegateToSnapshotLookups) {
  StateBuffer buffer;
  auto snapshot = std::make_shared<StateSnapshot>();
  snapshot->joints.emplace("joint", JointState{.name = "joint", .position = 1.5});
  snapshot->imus.emplace("imu", ImuState{.frame_id = "imu_link"});
  snapshot->lidars.emplace("lidar", LidarState{.frame_id = "lidar_link"});
  snapshot->mobile_bases.emplace("base", MobileBaseState{.base_frame_id = "base_link"});
  buffer.write(snapshot);

  JointState joint;
  ASSERT_TRUE(buffer.joint_state("joint", &joint));
  EXPECT_EQ(joint.name, "joint");
  EXPECT_DOUBLE_EQ(joint.position, 1.5);

  ImuState imu;
  ASSERT_TRUE(buffer.imu_state("imu", &imu));
  EXPECT_EQ(imu.frame_id, "imu_link");

  LidarState lidar;
  ASSERT_TRUE(buffer.lidar_state("lidar", &lidar));
  EXPECT_EQ(lidar.frame_id, "lidar_link");

  MobileBaseState mobile_base;
  ASSERT_TRUE(buffer.mobile_base_state("base", &mobile_base));
  EXPECT_EQ(mobile_base.base_frame_id, "base_link");

  joint.name = "unchanged";
  EXPECT_FALSE(buffer.joint_state("missing_joint", &joint));
  EXPECT_EQ(joint.name, "unchanged");
}

}  // namespace
}  // namespace mujoco_simulation
