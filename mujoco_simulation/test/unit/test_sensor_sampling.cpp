#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <thread>

#include "mujoco_simulation/simulation.hpp"

namespace mujoco_simulation {
namespace {

#define ASSERT_OK_STATUS(expr)           \
  do {                                   \
    const ResultCode status__ = (expr);  \
    ASSERT_EQ(status__, ResultCode::Ok); \
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

bool wait_for_step_count(Simulation& simulation, std::uint64_t target_step_count,
                         std::chrono::milliseconds timeout = std::chrono::seconds(1)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (simulation.step_count() >= target_step_count) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return simulation.step_count() >= target_step_count;
}

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
      ComponentConfig{ImuInfo{.name = "imu",
                              .frame_id = "imu_link",
                              .framequat_sensor_name = "imu_quat",
                              .gyro_sensor_name = "imu_gyro",
                              .accelerometer_sensor_name = "imu_acc",
                              .update_rate = 200.0}},
      ComponentConfig{LidarInfo{.name = "front_lidar",
                                .frame_id = "front_lidar_link",
                                .update_rate = 10.0,
                                .sensor_prefix = "front_lidar",
                                .angle_min = -0.2,
                                .angle_max = 0.2,
                                .angle_increment = 0.2,
                                .range_min = 0.1,
                                .range_max = 1.0}},
  };
  ASSERT_OK_STATUS(simulation.initialize(config));
  ASSERT_OK_STATUS(simulation.start());

  ImuState imu_state;
  ASSERT_TRUE(simulation.imu_state("imu", &imu_state));
  LidarState lidar_state;
  ASSERT_TRUE(simulation.lidar_state("front_lidar", &lidar_state));
  EXPECT_EQ(imu_state.sequence, 1U);
  EXPECT_EQ(lidar_state.sequence, 1U);

  ASSERT_TRUE(wait_for_step_count(simulation, 4));
  ASSERT_TRUE(simulation.imu_state("imu", &imu_state));
  ASSERT_TRUE(simulation.lidar_state("front_lidar", &lidar_state));
  EXPECT_EQ(imu_state.sequence, 1U);
  EXPECT_EQ(lidar_state.sequence, 1U);

  ASSERT_TRUE(wait_for_step_count(simulation, 5));
  ASSERT_TRUE(simulation.imu_state("imu", &imu_state));
  EXPECT_EQ(imu_state.sequence, 2U);
  EXPECT_DOUBLE_EQ(imu_state.timestamp, 0.005);
  EXPECT_EQ(imu_state.frame_id, "imu_link");

  ASSERT_TRUE(wait_for_step_count(simulation, 100));
  ASSERT_TRUE(simulation.imu_state("imu", &imu_state));
  ASSERT_TRUE(simulation.lidar_state("front_lidar", &lidar_state));
  EXPECT_EQ(imu_state.sequence, 21U);
  EXPECT_NEAR(imu_state.timestamp, 0.1, 1e-9);
  EXPECT_EQ(lidar_state.sequence, 2U);
  EXPECT_NEAR(lidar_state.timestamp, 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(lidar_state.scan_time, 0.1);
  EXPECT_DOUBLE_EQ(lidar_state.time_increment, 0.0);
  EXPECT_EQ(lidar_state.frame_id, "front_lidar_link");
  ASSERT_EQ(lidar_state.ranges.size(), 3U);
  EXPECT_TRUE(std::isinf(lidar_state.ranges[0]));
  EXPECT_TRUE(std::isinf(lidar_state.ranges[1]));
  EXPECT_TRUE(std::isinf(lidar_state.ranges[2]));
  ASSERT_OK_STATUS(simulation.stop());
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
  config.components = {ComponentConfig{LidarInfo{.name = "front_lidar",
                                                 .frame_id = "front_lidar_link",
                                                 .update_rate = 10.0,
                                                 .sensor_prefix = "front_lidar",
                                                 .angle_min = 0.0,
                                                 .angle_max = 0.0,
                                                 .angle_increment = 1.0,
                                                 .range_min = 0.1,
                                                 .range_max = 1.0}}};
  ASSERT_OK_STATUS(simulation.initialize(config));
  ASSERT_OK_STATUS(simulation.start());

  ASSERT_TRUE(wait_for_step_count(simulation, 100));
  ASSERT_OK_STATUS(simulation.stop());
  ASSERT_EQ(simulation.reset(), ResultCode::Unimplemented);

  LidarState lidar_state;
  ASSERT_TRUE(simulation.lidar_state("front_lidar", &lidar_state));
  EXPECT_GT(lidar_state.sequence, 0U);
  EXPECT_GT(lidar_state.timestamp, 0.0);
  EXPECT_GE(lidar_state.scan_time, 0.0);
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
  bad_type_config.components = {ComponentConfig{ImuInfo{.name = "bad_type",
                                                        .framequat_sensor_name = "imu_gyro",
                                                        .gyro_sensor_name = "imu_gyro",
                                                        .accelerometer_sensor_name = "imu_acc",
                                                        .update_rate = 200.0}}};
  const ResultCode bad_type_status = bad_type_simulation.initialize(bad_type_config);
  EXPECT_NE(bad_type_status, ResultCode::Ok);

  Simulation too_fast_simulation;
  SimulationConfig too_fast_config;
  too_fast_config.model.model_path = model_path;
  too_fast_config.components = {ComponentConfig{ImuInfo{.name = "too_fast",
                                                        .framequat_sensor_name = "imu_quat",
                                                        .gyro_sensor_name = "imu_gyro",
                                                        .accelerometer_sensor_name = "imu_acc",
                                                        .update_rate = 2000.0}}};
  const ResultCode too_fast_status = too_fast_simulation.initialize(too_fast_config);
  EXPECT_NE(too_fast_status, ResultCode::Ok);
}

#undef ASSERT_OK_STATUS

}  // namespace
}  // namespace mujoco_simulation
