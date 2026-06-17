#include <gtest/gtest.h>

#include <cstring>

#include "mujoco_simulation/component/camera/camera_component.hpp"

namespace mujoco_simulation {
namespace {

TEST(CameraSampleTest, ConvertsSampleToCompatibleCameraState) {
  CameraSample sample;
  sample.sequence = 5;
  sample.timestamp_ns = 987654321ULL;
  sample.frame_id = "camera_link";
  sample.optical_frame_id = "camera_optical_frame";
  sample.intrinsics.fx = 320.0;
  sample.intrinsics.fy = 321.0;
  sample.intrinsics.cx = 159.5;
  sample.intrinsics.cy = 119.5;
  sample.intrinsics.k = {320.0, 0.0, 159.5, 0.0, 321.0, 119.5, 0.0, 0.0, 1.0};
  sample.intrinsics.p = {320.0, 0.0, 159.5, 0.0, 0.0, 321.0, 119.5, 0.0, 0.0, 0.0, 1.0, 0.0};

  sample.color = ImageBuffer{};
  sample.color->width = 2;
  sample.color->height = 2;
  sample.color->step = 6;
  sample.color->data = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U};

  sample.depth = DepthBuffer{};
  sample.depth->width = 2;
  sample.depth->height = 2;
  sample.depth->data = {1.0F, 2.0F, 3.0F, 4.0F};

  const CameraState state = camera_state_from_sample(sample);
  EXPECT_EQ(state.image.timestamp, sample.timestamp_ns);
  EXPECT_EQ(state.image.frame_id, "camera_optical_frame");
  EXPECT_EQ(state.image.width, 2U);
  EXPECT_EQ(state.image.height, 2U);
  EXPECT_EQ(state.image.step, 6U);
  EXPECT_EQ(state.image.encoding, "rgb8");
  EXPECT_EQ(state.image.data, sample.color->data);

  EXPECT_EQ(state.depth_image.timestamp, sample.timestamp_ns);
  EXPECT_EQ(state.depth_image.frame_id, "camera_optical_frame");
  EXPECT_EQ(state.depth_image.width, 2U);
  EXPECT_EQ(state.depth_image.height, 2U);
  EXPECT_EQ(state.depth_image.step, 2U * sizeof(float));
  EXPECT_EQ(state.depth_image.encoding, "32FC1");
  ASSERT_EQ(state.depth_image.data.size(), 4U * sizeof(float));

  float decoded[4] = {};
  std::memcpy(decoded, state.depth_image.data.data(), state.depth_image.data.size());
  EXPECT_FLOAT_EQ(decoded[0], 1.0F);
  EXPECT_FLOAT_EQ(decoded[3], 4.0F);

  EXPECT_EQ(state.camera_info.width, 2U);
  EXPECT_EQ(state.camera_info.height, 2U);
  EXPECT_EQ(state.camera_info.k, sample.intrinsics.k);
  EXPECT_EQ(state.camera_info.p, sample.intrinsics.p);
}

}  // namespace
}  // namespace mujoco_simulation
