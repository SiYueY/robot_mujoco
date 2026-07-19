#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include "mujoco_simulation/component/imu/imu_component.hpp"

namespace mujoco_simulation {
namespace {

class ImuComponentTest : public ::testing::Test {
 protected:
  void TearDown() override {
    context_.clear();
    std::error_code error;
    std::filesystem::remove(model_path_, error);
  }

  void load() {
    model_path_ = std::filesystem::temp_directory_path() /
                  ("imu_component_" + std::to_string(::getpid()) + ".xml");
    std::ofstream output(model_path_);
    output
        << R"(<mujoco><option timestep="0.001"/><worldbody><body><freejoint/><site name="imu_site"/><geom type="sphere" size=".1"/></body></worldbody><sensor><framequat name="imu_quat" objtype="site" objname="imu_site"/><gyro name="imu_gyro" site="imu_site"/><accelerometer name="imu_acc" site="imu_site"/></sensor></mujoco>)";
    output.close();

    char error[1024] = {};
    mjModel* model = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(model, nullptr) << error;
    mjData* data = mj_makeData(model);
    ASSERT_NE(data, nullptr);
    context_ = mjContext(model, data);
    mj_forward(model, data);
  }

  ImuInfo valid_info() const {
    return {.name = "imu",
            .frame_id = "imu_link",
            .framequat_sensor_name = "imu_quat",
            .gyro_sensor_name = "imu_gyro",
            .accelerometer_sensor_name = "imu_acc",
            .orientation_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0},
            .update_rate = 200.0};
  }

  const mjContext& context() const { return context_; }

  mjContext context_{};
  std::filesystem::path model_path_;
};

TEST_F(ImuComponentTest, InitializesAndMapsSensorState) {
  load();
  ImuComponent imu(valid_info());
  EXPECT_FALSE(imu.is_initialized());
  ASSERT_TRUE(imu.init(context()));
  EXPECT_TRUE(imu.is_initialized());

  const mjModel* model = context_.model;
  mjData* data = context_.data;
  const int framequat_address = model->sensor_adr[mj_name2id(model, mjOBJ_SENSOR, "imu_quat")];
  const int gyro_address = model->sensor_adr[mj_name2id(model, mjOBJ_SENSOR, "imu_gyro")];
  const int accelerometer_address = model->sensor_adr[mj_name2id(model, mjOBJ_SENSOR, "imu_acc")];
  data->sensordata[framequat_address] = 4.0;
  data->sensordata[framequat_address + 1] = 1.0;
  data->sensordata[framequat_address + 2] = 2.0;
  data->sensordata[framequat_address + 3] = 3.0;
  data->sensordata[gyro_address] = 5.0;
  data->sensordata[gyro_address + 1] = 6.0;
  data->sensordata[gyro_address + 2] = 7.0;
  data->sensordata[accelerometer_address] = 8.0;
  data->sensordata[accelerometer_address + 1] = 9.0;
  data->sensordata[accelerometer_address + 2] = 10.0;
  data->time = 0.012;

  ASSERT_TRUE(imu.update(context()));
  ImuState state;
  ASSERT_TRUE(imu.read(context(), state));
  EXPECT_EQ(state.sequence, 1U);
  EXPECT_DOUBLE_EQ(state.timestamp, 0.012);
  EXPECT_EQ(state.frame_id, "imu_link");
  EXPECT_EQ(state.orientation, (Vector4d{1.0, 2.0, 3.0, 4.0}));
  EXPECT_EQ(state.angular_velocity, (Vector3d{5.0, 6.0, 7.0}));
  EXPECT_EQ(state.linear_acceleration, (Vector3d{8.0, 9.0, 10.0}));
  EXPECT_EQ(state.orientation_covariance, valid_info().orientation_covariance);
}

TEST_F(ImuComponentTest, RejectsLifecycleOperationsBeforeInitialization) {
  load();
  ImuComponent imu(valid_info());
  ImuState state{.sequence = 42};

  EXPECT_FALSE(imu.reset(context()));
  EXPECT_FALSE(imu.update(context()));
  EXPECT_FALSE(imu.read(context(), state));
  EXPECT_EQ(state.sequence, 42U);
}

TEST_F(ImuComponentTest, FailedReinitializationInvalidatesPreviousBinding) {
  load();
  ImuComponent imu(valid_info());
  ASSERT_TRUE(imu.init(context()));
  ASSERT_TRUE(imu.update(context()));

  mjModel* model = const_cast<mjModel*>(context_.model);
  const int gyro_sensor_id = mj_name2id(model, mjOBJ_SENSOR, "imu_gyro");
  model->sensor_type[gyro_sensor_id] = mjSENS_ACCELEROMETER;
  EXPECT_FALSE(imu.init(context()));
  EXPECT_FALSE(imu.is_initialized());
  EXPECT_FALSE(imu.update(context()));
  ImuState state{.sequence = 42};
  EXPECT_FALSE(imu.read(context(), state));
  EXPECT_EQ(state.sequence, 42U);
}

TEST_F(ImuComponentTest, RejectsInvalidInfoAndSensorBindings) {
  load();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  ImuInfo invalid_name = valid_info();
  invalid_name.name.clear();
  EXPECT_FALSE(ImuComponent(invalid_name).init(context()));

  ImuInfo invalid_rate = valid_info();
  invalid_rate.update_rate = -1.0;
  EXPECT_FALSE(ImuComponent(invalid_rate).init(context()));

  ImuInfo every_step_rate = valid_info();
  every_step_rate.update_rate = 0.0;
  EXPECT_TRUE(ImuComponent(every_step_rate).init(context()));

  ImuInfo invalid_covariance = valid_info();
  invalid_covariance.angular_velocity_covariance[3] = nan;
  EXPECT_FALSE(ImuComponent(invalid_covariance).init(context()));

  ImuInfo missing_sensor = valid_info();
  missing_sensor.gyro_sensor_name = "missing";
  EXPECT_FALSE(ImuComponent(missing_sensor).init(context()));

  ImuInfo invalid_type = valid_info();
  invalid_type.framequat_sensor_name = "imu_gyro";
  EXPECT_FALSE(ImuComponent(invalid_type).init(context()));

  mjModel* model = const_cast<mjModel*>(context_.model);
  const int gyro_sensor_id = mj_name2id(model, mjOBJ_SENSOR, "imu_gyro");
  model->sensor_dim[gyro_sensor_id] = 2;
  EXPECT_FALSE(ImuComponent(valid_info()).init(context()));
}

}  // namespace
}  // namespace mujoco_simulation
