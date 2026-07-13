#include <gtest/gtest.h>

#include <cstring>

#include "mujoco_simulation/component/camera/camera_component.hpp"

namespace mujoco_simulation {
namespace {

TEST(CameraStateTest, StoresCompatibleCameraStatePayload) {
  CameraState state;
  state.sequence = 5;
  state.timestamp_ns = 987654321ULL;
  state.frame_id = "camera_link";
  state.optical_frame_id = "camera_optical_frame";
  state.image.timestamp = state.timestamp_ns;
  state.image.frame_id = "camera_optical_frame";
  state.image.width = 2;
  state.image.height = 2;
  state.image.step = 6;
  state.image.encoding = "rgb8";
  state.image.data = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U};
  state.depth_image.timestamp = state.timestamp_ns;
  state.depth_image.frame_id = "camera_optical_frame";
  state.depth_image.width = 2;
  state.depth_image.height = 2;
  state.depth_image.step = 2U * sizeof(float);
  state.depth_image.encoding = "32FC1";
  state.depth_image.data.resize(4U * sizeof(float));
  const float depth_values[4] = {1.0F, 2.0F, 3.0F, 4.0F};
  std::memcpy(state.depth_image.data.data(), depth_values, state.depth_image.data.size());
  state.camera_info.width = 2;
  state.camera_info.height = 2;
  state.camera_info.k = {320.0, 0.0, 159.5, 0.0, 321.0, 119.5, 0.0, 0.0, 1.0};
  state.camera_info.p = {320.0, 0.0, 159.5, 0.0, 0.0, 321.0, 119.5, 0.0, 0.0, 0.0, 1.0, 0.0};

  EXPECT_EQ(state.image.timestamp, state.timestamp_ns);
  EXPECT_EQ(state.image.frame_id, "camera_optical_frame");
  EXPECT_EQ(state.image.width, 2U);
  EXPECT_EQ(state.image.height, 2U);
  EXPECT_EQ(state.image.step, 6U);
  EXPECT_EQ(state.image.encoding, "rgb8");
  EXPECT_EQ(state.image.data[0], 1U);

  EXPECT_EQ(state.depth_image.timestamp, state.timestamp_ns);
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
  EXPECT_EQ(state.camera_info.k[0], 320.0);
  EXPECT_EQ(state.camera_info.p[5], 321.0);
}

}  // namespace
}  // namespace mujoco_simulation
