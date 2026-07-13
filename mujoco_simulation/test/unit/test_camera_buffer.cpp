#include <gtest/gtest.h>

#include "mujoco_simulation/buffer/camera_buffer.hpp"

namespace mujoco_simulation {
namespace {

TEST(CameraBufferTest, PublishReadAndClearStates) {
  CameraBuffer buffer;

  CameraState state;
  state.sequence = 7;
  state.timestamp_ns = 1234;
  state.frame_id = "camera_link";

  buffer.write("camera", state);

  CameraState read;
  ASSERT_TRUE(buffer.read("camera", &read));
  EXPECT_EQ(read.sequence, 7U);
  EXPECT_EQ(read.timestamp_ns, 1234U);
  EXPECT_EQ(read.frame_id, "camera_link");

  buffer.clear();
  EXPECT_FALSE(buffer.read("camera", &read));
}

}  // namespace
}  // namespace mujoco_simulation
