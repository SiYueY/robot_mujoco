#include <gtest/gtest.h>

#include <memory>

#include "mujoco_simulation/buffer/state_buffer.hpp"

namespace mujoco_simulation {
namespace {

TEST(StateBufferTest, TypedReadsReturnFalseBeforeFirstWrite) {
  StateBuffer buffer;
  JointState joint{.joint_name = "sentinel"};
  ImuState imu{.frame_id = "sentinel"};
  CameraState camera{.frame_id = "sentinel"};
  LidarState lidar{.frame_id = "sentinel"};
  MobileBaseState mobile_base{.base_frame_id = "sentinel"};

  EXPECT_FALSE(buffer.read_joint_state("joint", joint));
  EXPECT_EQ(joint.joint_name, "sentinel");
  EXPECT_FALSE(buffer.read_imu_state("imu", imu));
  EXPECT_EQ(imu.frame_id, "sentinel");
  EXPECT_FALSE(buffer.read_camera_state("camera", camera));
  EXPECT_EQ(camera.frame_id, "sentinel");
  EXPECT_FALSE(buffer.read_lidar_state("lidar", lidar));
  EXPECT_EQ(lidar.frame_id, "sentinel");
  EXPECT_FALSE(buffer.read_mobile_base_state("base", mobile_base));
  EXPECT_EQ(mobile_base.base_frame_id, "sentinel");
}

TEST(StateBufferTest, TypedReadsDelegateToSnapshotLookups) {
  StateBuffer buffer;
  auto snapshot = std::make_shared<RobotState>();
  auto joints = std::make_shared<JointStateMap>();
  joints->emplace("joint",
                  std::make_shared<JointState>(JointState{.joint_name = "joint", .position = 1.5}));
  snapshot->joints = std::move(joints);
  auto imus = std::make_shared<ImuStateMap>();
  imus->emplace("imu", std::make_shared<ImuState>(ImuState{.frame_id = "imu_link"}));
  snapshot->imus = std::move(imus);
  auto lidars = std::make_shared<LidarStateMap>();
  lidars->emplace("lidar", std::make_shared<LidarState>(LidarState{.frame_id = "lidar_link"}));
  snapshot->lidars = std::move(lidars);
  auto mobile_bases = std::make_shared<MobileBaseStateMap>();
  mobile_bases->emplace(
      "base", std::make_shared<MobileBaseState>(MobileBaseState{.base_frame_id = "base_link"}));
  snapshot->mobile_bases = std::move(mobile_bases);
  auto cameras = std::make_shared<CameraStateMap>();
  cameras->emplace("camera", std::make_shared<CameraState>(CameraState{.frame_id = "camera_link"}));
  snapshot->cameras = std::move(cameras);

  JointState snapshot_joint;
  ASSERT_TRUE(snapshot->read_state("joint", snapshot_joint));
  EXPECT_DOUBLE_EQ(snapshot_joint.position, 1.5);

  ImuState missing_imu{.frame_id = "unchanged"};
  EXPECT_FALSE(snapshot->read_state("missing_imu", missing_imu));
  EXPECT_EQ(missing_imu.frame_id, "unchanged");

  buffer.write(snapshot);

  JointState joint;
  ASSERT_TRUE(buffer.read_joint_state("joint", joint));
  EXPECT_EQ(joint.joint_name, "joint");
  EXPECT_DOUBLE_EQ(joint.position, 1.5);

  ImuState imu;
  ASSERT_TRUE(buffer.read_imu_state("imu", imu));
  EXPECT_EQ(imu.frame_id, "imu_link");

  CameraState camera;
  ASSERT_TRUE(buffer.read_camera_state("camera", camera));
  EXPECT_EQ(camera.frame_id, "camera_link");

  LidarState lidar;
  ASSERT_TRUE(buffer.read_lidar_state("lidar", lidar));
  EXPECT_EQ(lidar.frame_id, "lidar_link");

  MobileBaseState mobile_base;
  ASSERT_TRUE(buffer.read_mobile_base_state("base", mobile_base));
  EXPECT_EQ(mobile_base.base_frame_id, "base_link");

  joint.joint_name = "unchanged";
  EXPECT_FALSE(buffer.read_joint_state("missing_joint", joint));
  EXPECT_EQ(joint.joint_name, "unchanged");
}

TEST(StateBufferTest, RobotStateCopiesShareImmutableComponentSnapshots) {
  auto snapshot = std::make_shared<RobotState>();
  auto lidars = std::make_shared<LidarStateMap>();
  auto lidar = std::make_shared<LidarState>();
  lidar->ranges = {1.0, 2.0};
  lidars->emplace("lidar", lidar);
  snapshot->lidars = lidars;

  RobotState copy = *snapshot;
  ASSERT_NE(copy.lidars, nullptr);
  EXPECT_EQ(copy.lidars, snapshot->lidars);
  ASSERT_NE(copy.lidars->at("lidar"), nullptr);
  EXPECT_EQ(copy.lidars->at("lidar"), lidar);
}

}  // namespace
}  // namespace mujoco_simulation
