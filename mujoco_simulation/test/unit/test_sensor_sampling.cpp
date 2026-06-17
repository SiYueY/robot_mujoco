#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include "mujoco_simulation/simulation.hpp"

namespace mujoco_simulation {
namespace {

#define ASSERT_OK_STATUS(expr)                        \
  do {                                                \
    const Status status__ = (expr);                   \
    ASSERT_TRUE(status__.ok()) << status__.message(); \
  } while (false)

class SensorSamplingTest : public ::testing::Test {
 protected:
  void TearDown() override {
    if (!model_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(model_path_, error);
    }
  }

  std::string write_model(const std::string& xml_contents) {
    const auto temp_dir = std::filesystem::temp_directory_path();
    model_path_ = temp_dir / std::filesystem::path("sensor_sampling_test_" +
                                                   std::to_string(::getpid()) + ".xml");
    std::ofstream output(model_path_);
    EXPECT_TRUE(output.is_open());
    output << xml_contents;
    output.close();
    return model_path_.string();
  }

  std::filesystem::path model_path_;
};

TEST_F(SensorSamplingTest, ImuAndLidarRespectConfiguredSamplingRates) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="sensor_sampling">
  <option timestep="0.001"/>
  <worldbody>
    <body name="sensor_body">
      <freejoint/>
      <site name="imu_site" pos="0 0 0"/>
      <site name="front_lidar_site_0" pos="0 0 0" zaxis="1 0 0"/>
      <site name="front_lidar_site_1" pos="0 0 0" zaxis="1 0 0"/>
      <site name="front_lidar_site_2" pos="0 0 0" zaxis="1 0 0"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
    <body name="far_target" pos="3 0 0">
      <geom type="box" size="0.05 0.05 0.05" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_acc" site="imu_site"/>
    <rangefinder name="front_lidar-0" site="front_lidar_site_0"/>
    <rangefinder name="front_lidar-1" site="front_lidar_site_1"/>
    <rangefinder name="front_lidar-2" site="front_lidar_site_2"/>
  </sensor>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.components = {
      ComponentConfig{
          ImuConfig{.common = {.name = "imu", .frame_id = "imu_link", .update_rate = 200.0},
                    .framequat_sensor_name = "imu_quat",
                    .gyro_sensor_name = "imu_gyro",
                    .accelerometer_sensor_name = "imu_acc"}},
      ComponentConfig{LidarConfig{
          .common = {.name = "front_lidar", .frame_id = "front_lidar_link", .update_rate = 10.0},
          .sensor_prefix = "front_lidar",
          .angle_min = -0.2,
          .angle_max = 0.2,
          .angle_increment = 0.2,
          .range_min = 0.1,
          .range_max = 1.0}},
  };
  ASSERT_OK_STATUS(simulation.initialize(config));

  auto imu_sample = simulation.imu_sample("imu");
  ASSERT_TRUE(imu_sample.ok()) << imu_sample.status().message();
  auto lidar_sample = simulation.lidar_sample("front_lidar");
  ASSERT_TRUE(lidar_sample.ok()) << lidar_sample.status().message();
  EXPECT_EQ(imu_sample.value().sequence, 1U);
  EXPECT_EQ(lidar_sample.value().sequence, 1U);

  ASSERT_OK_STATUS(simulation.step(4));
  imu_sample = simulation.imu_sample("imu");
  ASSERT_TRUE(imu_sample.ok()) << imu_sample.status().message();
  lidar_sample = simulation.lidar_sample("front_lidar");
  ASSERT_TRUE(lidar_sample.ok()) << lidar_sample.status().message();
  EXPECT_EQ(imu_sample.value().sequence, 1U);
  EXPECT_EQ(lidar_sample.value().sequence, 1U);

  ASSERT_OK_STATUS(simulation.step(1));
  imu_sample = simulation.imu_sample("imu");
  ASSERT_TRUE(imu_sample.ok()) << imu_sample.status().message();
  EXPECT_EQ(imu_sample.value().sequence, 2U);
  EXPECT_EQ(imu_sample.value().timestamp_ns, 5000000U);
  EXPECT_EQ(imu_sample.value().frame_id, "imu_link");

  ASSERT_OK_STATUS(simulation.step(95));
  imu_sample = simulation.imu_sample("imu");
  ASSERT_TRUE(imu_sample.ok()) << imu_sample.status().message();
  lidar_sample = simulation.lidar_sample("front_lidar");
  ASSERT_TRUE(lidar_sample.ok()) << lidar_sample.status().message();
  EXPECT_EQ(imu_sample.value().sequence, 21U);
  EXPECT_EQ(imu_sample.value().timestamp_ns, 100000000U);
  EXPECT_EQ(lidar_sample.value().sequence, 2U);
  EXPECT_EQ(lidar_sample.value().timestamp_ns, 100000000U);
  EXPECT_DOUBLE_EQ(lidar_sample.value().scan_time, 0.1);
  EXPECT_DOUBLE_EQ(lidar_sample.value().time_increment, 0.0);
  EXPECT_EQ(lidar_sample.value().frame_id, "front_lidar_link");
  ASSERT_EQ(lidar_sample.value().ranges.size(), 3U);
  EXPECT_TRUE(std::isinf(lidar_sample.value().ranges[0]));
  EXPECT_TRUE(std::isinf(lidar_sample.value().ranges[1]));
  EXPECT_TRUE(std::isinf(lidar_sample.value().ranges[2]));
}

TEST_F(SensorSamplingTest, ResetRestartsSensorSamplingWithoutNegativeScanTime) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="sensor_sampling_reset">
  <option timestep="0.001"/>
  <worldbody>
    <body>
      <freejoint/>
      <site name="imu_site" pos="0 0 0"/>
      <site name="front_lidar_site_0" pos="0 0 0" zaxis="1 0 0"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
    <body name="far_target" pos="3 0 0">
      <geom type="box" size="0.05 0.05 0.05" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_acc" site="imu_site"/>
    <rangefinder name="front_lidar-0" site="front_lidar_site_0"/>
  </sensor>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.components = {ComponentConfig{LidarConfig{
      .common = {.name = "front_lidar", .frame_id = "front_lidar_link", .update_rate = 10.0},
      .sensor_prefix = "front_lidar",
      .angle_min = 0.0,
      .angle_max = 0.0,
      .angle_increment = 1.0,
      .range_min = 0.1,
      .range_max = 1.0}}};
  ASSERT_OK_STATUS(simulation.initialize(config));

  ASSERT_OK_STATUS(simulation.step(100));
  ASSERT_OK_STATUS(simulation.reset());

  const auto lidar_sample = simulation.lidar_sample("front_lidar");
  ASSERT_TRUE(lidar_sample.ok()) << lidar_sample.status().message();
  EXPECT_EQ(lidar_sample.value().sequence, 1U);
  EXPECT_EQ(lidar_sample.value().timestamp_ns, 0U);
  EXPECT_GE(lidar_sample.value().scan_time, 0.0);
}

TEST_F(SensorSamplingTest, InvalidSensorBindingsFailInitialization) {
  const std::string model_path = write_model(R"(
<mujoco model="sensor_sampling_invalid">
  <option timestep="0.001"/>
  <worldbody>
    <body>
      <freejoint/>
      <site name="imu_site" pos="0 0 0"/>
      <geom type="sphere" size="0.05" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_acc" site="imu_site"/>
  </sensor>
</mujoco>)");

  Simulation bad_type_simulation;
  SimulationConfig bad_type_config;
  bad_type_config.model.model_path = model_path;
  bad_type_config.components = {
      ComponentConfig{ImuConfig{.common = {.name = "bad_type", .update_rate = 200.0},
                                .framequat_sensor_name = "imu_gyro",
                                .gyro_sensor_name = "imu_gyro",
                                .accelerometer_sensor_name = "imu_acc"}}};
  const Status bad_type_status = bad_type_simulation.initialize(bad_type_config);
  EXPECT_FALSE(bad_type_status.ok());
  EXPECT_FALSE(bad_type_status.message().empty());

  Simulation too_fast_simulation;
  SimulationConfig too_fast_config;
  too_fast_config.model.model_path = model_path;
  too_fast_config.components = {
      ComponentConfig{ImuConfig{.common = {.name = "too_fast", .update_rate = 2000.0},
                                .framequat_sensor_name = "imu_quat",
                                .gyro_sensor_name = "imu_gyro",
                                .accelerometer_sensor_name = "imu_acc"}}};
  const Status too_fast_status = too_fast_simulation.initialize(too_fast_config);
  EXPECT_FALSE(too_fast_status.ok());
  EXPECT_FALSE(too_fast_status.message().empty());
}

#undef ASSERT_OK_STATUS

}  // namespace
}  // namespace mujoco_simulation
