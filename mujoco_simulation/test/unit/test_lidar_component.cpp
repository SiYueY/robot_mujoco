#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include "mujoco_simulation/component/lidar/lidar_component.hpp"

namespace mujoco_simulation {
namespace {

class LidarComponentTest : public ::testing::Test {
 protected:
  void TearDown() override {
    context_.clear();
    std::error_code error;
    std::filesystem::remove(model_path_, error);
  }

  void load() {
    model_path_ = std::filesystem::temp_directory_path() /
                  ("lidar_component_" + std::to_string(::getpid()) + ".xml");
    std::ofstream output(model_path_);
    output
        << R"(<mujoco><option timestep="0.001"/><worldbody><body><freejoint/><site name="lidar_site_0"/><site name="lidar_site_1"/><geom type="sphere" size=".1"/></body></worldbody><sensor><rangefinder name="lidar-0" site="lidar_site_0"/><rangefinder name="lidar-1" site="lidar_site_1"/></sensor></mujoco>)";
    output.close();

    char error[1024] = {};
    mjModel* model = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(model, nullptr) << error;
    mjData* data = mj_makeData(model);
    ASSERT_NE(data, nullptr);
    mj_forward(model, data);
    context_ = mjContext(model, data);
  }

  LidarInfo valid_info() const {
    return {.name = "lidar",
            .frame_id = "lidar_link",
            .update_rate = 10.0,
            .sensor_prefix = "lidar",
            .angle_min = 0.0,
            .angle_max = 1.0,
            .angle_increment = 1.0,
            .range_min = 0.1,
            .range_max = 5.0};
  }

  const mjContext& context() const { return context_; }

  mjModel* model() const { return const_cast<mjModel*>(context_.model); }
  mjData* data() const { return context_.data; }

  mjContext context_{};
  std::filesystem::path model_path_;
};

TEST_F(LidarComponentTest, InitializesAndSamplesRanges) {
  load();
  LidarComponent lidar(valid_info());
  EXPECT_FALSE(lidar.is_initialized());
  ASSERT_TRUE(lidar.init(context()));
  EXPECT_TRUE(lidar.is_initialized());

  const int first_sensor = mj_name2id(model(), mjOBJ_SENSOR, "lidar-0");
  const int second_sensor = mj_name2id(model(), mjOBJ_SENSOR, "lidar-1");
  data()->sensordata[model()->sensor_adr[first_sensor]] = 1.0;
  data()->sensordata[model()->sensor_adr[second_sensor]] = 6.0;
  data()->time = 0.25;

  ASSERT_TRUE(lidar.update(context()));
  LidarState state;
  ASSERT_TRUE(lidar.read(context(), state));
  EXPECT_EQ(state.sequence, 1U);
  EXPECT_DOUBLE_EQ(state.timestamp, 0.25);
  EXPECT_EQ(state.frame_id, "lidar_link");
  EXPECT_DOUBLE_EQ(state.scan_time, 0.1);
  ASSERT_EQ(state.ranges.size(), 2U);
  EXPECT_DOUBLE_EQ(state.ranges[0], 1.0);
  EXPECT_TRUE(std::isinf(state.ranges[1]));
  EXPECT_EQ(state.intensities, (std::vector<double>{0.0, 0.0}));
}

TEST_F(LidarComponentTest, RejectsLifecycleOperationsBeforeInitialization) {
  load();
  LidarComponent lidar(valid_info());
  LidarState state{.sequence = 42};

  EXPECT_FALSE(lidar.reset(context()));
  EXPECT_FALSE(lidar.update(context()));
  EXPECT_FALSE(lidar.read(context(), state));
  EXPECT_EQ(state.sequence, 42U);
}

TEST_F(LidarComponentTest, FailedReinitializationInvalidatesPreviousBinding) {
  load();
  LidarComponent lidar(valid_info());
  ASSERT_TRUE(lidar.init(context()));

  const int sensor_id = mj_name2id(model(), mjOBJ_SENSOR, "lidar-0");
  model()->sensor_type[sensor_id] = mjSENS_GYRO;
  EXPECT_FALSE(lidar.init(context()));
  EXPECT_FALSE(lidar.is_initialized());
  EXPECT_FALSE(lidar.update(context()));
  LidarState state{.sequence = 42};
  EXPECT_FALSE(lidar.read(context(), state));
  EXPECT_EQ(state.sequence, 42U);
}

TEST_F(LidarComponentTest, RejectsInvalidInfoAndSensorBindings) {
  load();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  LidarInfo invalid_name = valid_info();
  invalid_name.name.clear();
  EXPECT_FALSE(LidarComponent(invalid_name).init(context()));

  LidarInfo invalid_rate = valid_info();
  invalid_rate.update_rate = -1.0;
  EXPECT_FALSE(LidarComponent(invalid_rate).init(context()));

  LidarInfo invalid_angle = valid_info();
  invalid_angle.angle_increment = nan;
  EXPECT_FALSE(LidarComponent(invalid_angle).init(context()));

  LidarInfo invalid_range = valid_info();
  invalid_range.range_max = nan;
  EXPECT_FALSE(LidarComponent(invalid_range).init(context()));

  LidarInfo invalid_span = valid_info();
  invalid_span.angle_max = 1.0;
  invalid_span.angle_increment = 0.3;
  EXPECT_FALSE(LidarComponent(invalid_span).init(context()));

  LidarInfo missing_sensor = valid_info();
  missing_sensor.sensor_prefix = "missing";
  EXPECT_FALSE(LidarComponent(missing_sensor).init(context()));

  const int sensor_id = mj_name2id(model(), mjOBJ_SENSOR, "lidar-0");
  model()->sensor_type[sensor_id] = mjSENS_GYRO;
  EXPECT_FALSE(LidarComponent(valid_info()).init(context()));
  model()->sensor_type[sensor_id] = mjSENS_RANGEFINDER;

  model()->sensor_dim[sensor_id] = 2;
  EXPECT_FALSE(LidarComponent(valid_info()).init(context()));
  model()->sensor_dim[sensor_id] = 1;

  model()->sensor_adr[sensor_id] = model()->nsensordata;
  EXPECT_FALSE(LidarComponent(valid_info()).init(context()));
}

}  // namespace
}  // namespace mujoco_simulation
