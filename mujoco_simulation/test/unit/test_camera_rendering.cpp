#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

#include "mujoco_simulation/simulation.hpp"

namespace mujoco_simulation {
namespace {

#define ASSERT_OK_STATUS(expr)                        \
  do {                                                \
    const Status status__ = (expr);                   \
    ASSERT_TRUE(status__.ok()) << status__.message(); \
  } while (false)

class CameraRenderingTest : public ::testing::Test {
 protected:
  void TearDown() override {
    if (!model_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(model_path_, error);
    }
  }

  std::string write_model(const std::string& xml_contents) {
    const auto temp_dir = std::filesystem::temp_directory_path();
    model_path_ = temp_dir / std::filesystem::path("camera_rendering_test_" +
                                                   std::to_string(::getpid()) + ".xml");
    std::ofstream output(model_path_);
    EXPECT_TRUE(output.is_open());
    output << xml_contents;
    output.close();
    return model_path_.string();
  }

  static float depth_at(const Image& image, std::size_t row, std::size_t column) {
    const std::size_t index = row * image.width + column;
    float value = 0.0F;
    std::memcpy(&value, image.data.data() + index * sizeof(float), sizeof(float));
    return value;
  }

  std::filesystem::path model_path_;
};

TEST_F(CameraRenderingTest, HeadlessCameraRendersColorDepthAndIntrinsics) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="camera_rendering">
  <option timestep="0.01"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <body pos="0 0 0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="1 0 0 1"/>
    </body>
    <body pos="0 0 -0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="0 1 0 1"/>
    </body>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.render_mode = RenderMode::Headless;
  config.components = {ComponentConfig{CameraConfig{
      .common = {.name = "front_camera", .frame_id = "camera_link", .update_rate = 50.0},
      .camera_name = "cam",
      .optical_frame_id = "camera_optical_frame",
      .height = 120,
      .width = 160,
      .enable_rgb = true,
      .enable_depth = true}}};
  ASSERT_OK_STATUS(simulation.initialize(config));

  const auto sample = simulation.camera_sample("front_camera");
  ASSERT_TRUE(sample.ok()) << sample.status().message();
  ASSERT_NE(sample.value(), nullptr);
  const CameraState state = camera_state_from_sample(*sample.value());
  ASSERT_EQ(state.image.width, 160U);
  ASSERT_EQ(state.image.height, 120U);
  ASSERT_EQ(state.image.step, 480U);
  ASSERT_EQ(state.image.encoding, "rgb8");
  ASSERT_EQ(state.image.frame_id, "camera_optical_frame");
  ASSERT_EQ(state.depth_image.width, 160U);
  ASSERT_EQ(state.depth_image.height, 120U);
  ASSERT_EQ(state.depth_image.encoding, "32FC1");
  ASSERT_EQ(state.depth_image.frame_id, "camera_optical_frame");

  EXPECT_EQ(sample.value()->frame_id, "camera_link");
  EXPECT_EQ(sample.value()->optical_frame_id, "camera_optical_frame");
  ASSERT_TRUE(sample.value()->color.has_value());
  ASSERT_TRUE(sample.value()->depth.has_value());
  EXPECT_EQ(sample.value()->color->width, 160U);
  EXPECT_EQ(sample.value()->depth->height, 120U);

  const auto pixel = [&](std::size_t row, std::size_t column, std::size_t channel) -> std::uint8_t {
    return state.image.data[(row * state.image.width + column) * 3U + channel];
  };
  EXPECT_GT(pixel(10, 80, 0), pixel(10, 80, 1));
  EXPECT_GT(pixel(10, 80, 0), pixel(10, 80, 2));
  EXPECT_GT(pixel(105, 80, 1), pixel(105, 80, 0));
  EXPECT_GT(pixel(105, 80, 1), pixel(105, 80, 2));

  const float top_depth = depth_at(state.depth_image, 10, 80);
  const float bottom_depth = depth_at(state.depth_image, 105, 80);
  EXPECT_NEAR(top_depth, 0.98F, 0.05F);
  EXPECT_NEAR(bottom_depth, 0.98F, 0.05F);

  const double aspect = 160.0 / 120.0;
  const double fovy_radians = 45.0 * M_PI / 180.0;
  const double fy = 120.0 / (2.0 * std::tan(fovy_radians / 2.0));
  const double fovx_radians = 2.0 * std::atan(aspect * std::tan(fovy_radians / 2.0));
  const double fx = 160.0 / (2.0 * std::tan(fovx_radians / 2.0));
  EXPECT_NEAR(state.camera_info.k[0], fx, 1.0e-6);
  EXPECT_NEAR(state.camera_info.k[4], fy, 1.0e-6);
  EXPECT_NEAR(state.camera_info.k[2], 79.5, 1.0e-6);
  EXPECT_NEAR(state.camera_info.k[5], 59.5, 1.0e-6);
}

TEST_F(CameraRenderingTest, MultipleCamerasUpdateAtIndependentRatesInHeadlessMode) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="camera_rendering_multi">
  <option timestep="0.01"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <body pos="0 0 0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="1 0 0 1"/>
    </body>
    <body pos="0 0 -0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="0 1 0 1"/>
    </body>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.render_mode = RenderMode::Headless;
  config.components = {
      ComponentConfig{CameraConfig{.common = {.name = "fast_camera", .update_rate = 50.0},
                                   .camera_name = "cam",
                                   .height = 120,
                                   .width = 160,
                                   .enable_rgb = true,
                                   .enable_depth = false}},
      ComponentConfig{CameraConfig{.common = {.name = "slow_camera", .update_rate = 20.0},
                                   .camera_name = "cam",
                                   .height = 120,
                                   .width = 160,
                                   .enable_rgb = true,
                                   .enable_depth = false}},
  };
  ASSERT_OK_STATUS(simulation.initialize(config));

  auto fast_sample = simulation.camera_sample("fast_camera");
  ASSERT_TRUE(fast_sample.ok()) << fast_sample.status().message();
  ASSERT_NE(fast_sample.value(), nullptr);
  auto slow_sample = simulation.camera_sample("slow_camera");
  ASSERT_TRUE(slow_sample.ok()) << slow_sample.status().message();
  ASSERT_NE(slow_sample.value(), nullptr);
  EXPECT_EQ(fast_sample.value()->timestamp_ns, 0U);
  EXPECT_EQ(slow_sample.value()->timestamp_ns, 0U);

  ASSERT_OK_STATUS(simulation.step(4));
  fast_sample = simulation.camera_sample("fast_camera");
  ASSERT_TRUE(fast_sample.ok()) << fast_sample.status().message();
  slow_sample = simulation.camera_sample("slow_camera");
  ASSERT_TRUE(slow_sample.ok()) << slow_sample.status().message();
  EXPECT_EQ(fast_sample.value()->timestamp_ns, 40000000U);
  EXPECT_EQ(slow_sample.value()->timestamp_ns, 0U);

  ASSERT_OK_STATUS(simulation.step(1));
  slow_sample = simulation.camera_sample("slow_camera");
  ASSERT_TRUE(slow_sample.ok()) << slow_sample.status().message();
  EXPECT_EQ(slow_sample.value()->timestamp_ns, 50000000U);
}

TEST_F(CameraRenderingTest, CameraInitializationReportsRenderFailedWhenNoBackendIsAllowed) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="camera_rendering_backend_failure">
  <option timestep="0.01"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.render_mode = RenderMode::Headless;
  config.camera_renderer.allow_glfw_backend = false;
  config.camera_renderer.allow_egl_backend = false;
  config.components = {
      ComponentConfig{CameraConfig{.common = {.name = "front_camera", .update_rate = 50.0},
                                   .camera_name = "cam",
                                   .height = 120,
                                   .width = 160,
                                   .enable_rgb = true,
                                   .enable_depth = false}}};
  const Status status = simulation.initialize(config);
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.code(), StatusCode::RenderFailed);
}

#undef ASSERT_OK_STATUS

}  // namespace
}  // namespace mujoco_simulation
