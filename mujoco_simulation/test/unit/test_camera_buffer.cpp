#include <gtest/gtest.h>

#include "mujoco_simulation/buffer/camera_buffer.hpp"

namespace mujoco_simulation {
namespace {

TEST(CameraBufferTest, PublishReadAndClearSamples) {
  CameraBuffer buffer;

  auto sample = std::make_shared<CameraSample>();
  sample->sequence = 7;
  sample->timestamp_ns = 1234;
  sample->frame_id = "camera_link";

  buffer.publish("camera", sample);

  Result<std::shared_ptr<const CameraSample>> read = buffer.read("camera");
  ASSERT_TRUE(read.ok());
  ASSERT_NE(read.value(), nullptr);
  EXPECT_EQ(read.value()->sequence, 7U);
  EXPECT_EQ(read.value()->timestamp_ns, 1234U);
  EXPECT_EQ(read.value()->frame_id, "camera_link");

  buffer.clear();
  read = buffer.read("camera");
  EXPECT_FALSE(read.ok());
}

}  // namespace
}  // namespace mujoco_simulation
