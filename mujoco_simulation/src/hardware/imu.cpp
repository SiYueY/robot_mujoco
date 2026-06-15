#include "mujoco_simulation/hardware/imu.hpp"

#include <algorithm>

namespace mujoco_simulation {
namespace {

bool copy_sensor_vector(const mjData* data, int address, double* dest, int count) {
  if (data == nullptr || address < 0) {
    return false;
  }
  for (int i = 0; i < count; ++i) {
    dest[i] = data->sensordata[address + i];
  }
  return true;
}

}  // namespace

Imu::Imu(const MjContext& context) : context_(context) {}

bool Imu::init(const ImuInfo& data) {
  data_ = data;
  last_error_.clear();

  if (context_.model == nullptr || context_.data == nullptr) {
    last_error_ = "MuJoCo model/data is not available for IMU '" + data.name + "'.";
    return false;
  }

  const int framequat_id =
      mj_name2id(context_.model, mjOBJ_SENSOR, data.framequat_sensor_name.c_str());
  const int gyro_id = mj_name2id(context_.model, mjOBJ_SENSOR, data.gyro_sensor_name.c_str());
  const int accelerometer_id =
      mj_name2id(context_.model, mjOBJ_SENSOR, data.accelerometer_sensor_name.c_str());
  if (framequat_id < 0 || gyro_id < 0 || accelerometer_id < 0) {
    last_error_ = "One or more MuJoCo sensors are missing for IMU '" + data.name + "'.";
    return false;
  }

  framequat_address_ = context_.model->sensor_adr[framequat_id];
  gyro_address_ = context_.model->sensor_adr[gyro_id];
  accelerometer_address_ = context_.model->sensor_adr[accelerometer_id];
  return reset();
}

bool Imu::reset() {
  state_ = {};
  state_.orientation = {0.0, 0.0, 0.0, 1.0};
  return read(state_);
}

bool Imu::write(const ImuCommand&) {
  last_error_.clear();
  return true;
}

bool Imu::read(ImuState& state) {
  last_error_.clear();
  if (context_.data == nullptr || framequat_address_ < 0 || gyro_address_ < 0 ||
      accelerometer_address_ < 0) {
    last_error_ = "IMU '" + data_.name + "' is not initialized.";
    return false;
  }

  state_.orientation[3] = context_.data->sensordata[framequat_address_];
  state_.orientation[0] = context_.data->sensordata[framequat_address_ + 1];
  state_.orientation[1] = context_.data->sensordata[framequat_address_ + 2];
  state_.orientation[2] = context_.data->sensordata[framequat_address_ + 3];
  if (!copy_sensor_vector(context_.data, gyro_address_, state_.angular_velocity.data(), 3) ||
      !copy_sensor_vector(context_.data, accelerometer_address_, state_.linear_acceleration.data(),
                          3)) {
    last_error_ = "Failed to read MuJoCo sensor data for IMU '" + data_.name + "'.";
    return false;
  }

  state = state_;
  return true;
}

}  // namespace mujoco_simulation
