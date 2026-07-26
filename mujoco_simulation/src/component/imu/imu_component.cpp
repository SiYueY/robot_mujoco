#include "mujoco_simulation/component/imu/imu_component.hpp"

#include <cmath>
#include <utility>

#include "common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"

namespace mujoco_simulation {
ImuComponent::ImuComponent(ImuInfo info)
    : SimulationComponent(info.name, info.update_rate), info_(std::move(info)) {
}

bool ImuComponent::init(const mjContext &context) {
  initialized_ = false;
  imu_ = {};
  if (!configure(context)) {
    return false;
  }

  if (info_.framequat_sensor_name.empty()) {
    LOG_ERROR << "imu '" << info_.name
              << "' framequat sensor name must not be empty.";
    return false;
  }
  if (info_.gyro_sensor_name.empty()) {
    LOG_ERROR << "imu '" << info_.name
              << "' gyro sensor name must not be empty.";
    return false;
  }
  if (info_.accelerometer_sensor_name.empty()) {
    LOG_ERROR << "imu '" << info_.name
              << "' accelerometer sensor name must not be empty.";
    return false;
  }
  for (const double value : info_.orientation_covariance) {
    if (!std::isfinite(value)) {
      LOG_ERROR << "imu '" << info_.name
                << "' orientation covariance must be finite.";
      return false;
    }
  }
  for (const double value : info_.angular_velocity_covariance) {
    if (!std::isfinite(value)) {
      LOG_ERROR << "imu '" << info_.name
                << "' angular velocity covariance must be finite.";
      return false;
    }
  }
  for (const double value : info_.linear_acceleration_covariance) {
    if (!std::isfinite(value)) {
      LOG_ERROR << "imu '" << info_.name
                << "' linear acceleration covariance must be finite.";
      return false;
    }
  }

  const mjModel &model = *context.model;
  imu_.framequat_sensor_id =
      mj_name2id(&model, mjOBJ_SENSOR, info_.framequat_sensor_name.c_str());
  if (imu_.framequat_sensor_id < 0) {
    LOG_ERROR << "imu '" << info_.name << "' framequat sensor '"
              << info_.framequat_sensor_name << "' was not found in the model.";
    return false;
  }
  if (model.sensor_type[imu_.framequat_sensor_id] != mjSENS_FRAMEQUAT) {
    LOG_ERROR << "imu '" << info_.name << "' framequat sensor '"
              << info_.framequat_sensor_name << "' has type "
              << model.sensor_type[imu_.framequat_sensor_id] << ", expected "
              << mjSENS_FRAMEQUAT << ".";
    return false;
  }
  if (model.sensor_dim[imu_.framequat_sensor_id] != 4) {
    LOG_ERROR << "imu '" << info_.name << "' framequat sensor '"
              << info_.framequat_sensor_name << "' has dimension "
              << model.sensor_dim[imu_.framequat_sensor_id] << ", expected 4.";
    return false;
  }
  imu_.framequat_address = model.sensor_adr[imu_.framequat_sensor_id];
  if (imu_.framequat_address < 0 ||
      imu_.framequat_address + 4 > model.nsensordata) {
    LOG_ERROR << "imu '" << info_.name << "' framequat sensor '"
              << info_.framequat_sensor_name
              << "' has an out-of-range sensor address.";
    return false;
  }

  imu_.gyro_sensor_id =
      mj_name2id(&model, mjOBJ_SENSOR, info_.gyro_sensor_name.c_str());
  if (imu_.gyro_sensor_id < 0) {
    LOG_ERROR << "imu '" << info_.name << "' gyro sensor '"
              << info_.gyro_sensor_name << "' was not found in the model.";
    return false;
  }
  if (model.sensor_type[imu_.gyro_sensor_id] != mjSENS_GYRO) {
    LOG_ERROR << "imu '" << info_.name << "' gyro sensor '"
              << info_.gyro_sensor_name << "' has type "
              << model.sensor_type[imu_.gyro_sensor_id] << ", expected "
              << mjSENS_GYRO << ".";
    return false;
  }
  if (model.sensor_dim[imu_.gyro_sensor_id] != 3) {
    LOG_ERROR << "imu '" << info_.name << "' gyro sensor '"
              << info_.gyro_sensor_name << "' has dimension "
              << model.sensor_dim[imu_.gyro_sensor_id] << ", expected 3.";
    return false;
  }
  imu_.gyro_address = model.sensor_adr[imu_.gyro_sensor_id];
  if (imu_.gyro_address < 0 || imu_.gyro_address + 3 > model.nsensordata) {
    LOG_ERROR << "imu '" << info_.name << "' gyro sensor '"
              << info_.gyro_sensor_name
              << "' has an out-of-range sensor address.";
    return false;
  }

  imu_.accelerometer_sensor_id =
      mj_name2id(&model, mjOBJ_SENSOR, info_.accelerometer_sensor_name.c_str());
  if (imu_.accelerometer_sensor_id < 0) {
    LOG_ERROR << "imu '" << info_.name << "' accelerometer sensor '"
              << info_.accelerometer_sensor_name
              << "' was not found in the model.";
    return false;
  }
  if (model.sensor_type[imu_.accelerometer_sensor_id] != mjSENS_ACCELEROMETER) {
    LOG_ERROR << "imu '" << info_.name << "' accelerometer sensor '"
              << info_.accelerometer_sensor_name << "' has type "
              << model.sensor_type[imu_.accelerometer_sensor_id]
              << ", expected " << mjSENS_ACCELEROMETER << ".";
    return false;
  }
  if (model.sensor_dim[imu_.accelerometer_sensor_id] != 3) {
    LOG_ERROR << "imu '" << info_.name << "' accelerometer sensor '"
              << info_.accelerometer_sensor_name << "' has dimension "
              << model.sensor_dim[imu_.accelerometer_sensor_id]
              << ", expected 3.";
    return false;
  }
  imu_.accelerometer_address = model.sensor_adr[imu_.accelerometer_sensor_id];
  if (imu_.accelerometer_address < 0 ||
      imu_.accelerometer_address + 3 > model.nsensordata) {
    LOG_ERROR << "imu '" << info_.name << "' accelerometer sensor '"
              << info_.accelerometer_sensor_name
              << "' has an out-of-range sensor address.";
    return false;
  }

  sequence_ = 0;
  auto state = std::make_shared<ImuState>();
  state->frame_id = info_.frame_id;
  state->orientation = {0.0, 0.0, 0.0, 1.0};
  state->orientation_covariance = info_.orientation_covariance;
  state->angular_velocity_covariance = info_.angular_velocity_covariance;
  state->linear_acceleration_covariance = info_.linear_acceleration_covariance;
  state_ = std::move(state);
  initialized_ = true;
  return true;
}

bool ImuComponent::reset(const mjContext &context) {
  UNUSED(context);
  if (!initialized_) {
    LOG_ERROR << "imu '" << info_.name << "' is not initialized.";
    return false;
  }

  sequence_ = 0;
  auto state = std::make_shared<ImuState>();
  state->frame_id = info_.frame_id;
  state->orientation = {0.0, 0.0, 0.0, 1.0};
  state->orientation_covariance = info_.orientation_covariance;
  state->angular_velocity_covariance = info_.angular_velocity_covariance;
  state->linear_acceleration_covariance = info_.linear_acceleration_covariance;
  state_ = std::move(state);
  return true;
}

bool ImuComponent::update(const mjContext &context) {
  if (!initialized_) {
    LOG_ERROR << "imu '" << info_.name << "' is not initialized.";
    return false;
  }

  auto state = std::make_shared<ImuState>(*state_);
  state->sequence = ++sequence_;
  state->timestamp = context.data->time;
  const double *sensor_data = context.data->sensordata;
  state->orientation[0] = sensor_data[imu_.framequat_address + 1];
  state->orientation[1] = sensor_data[imu_.framequat_address + 2];
  state->orientation[2] = sensor_data[imu_.framequat_address + 3];
  state->orientation[3] = sensor_data[imu_.framequat_address];
  for (int index = 0; index < 3; ++index) {
    state->angular_velocity[index] = sensor_data[imu_.gyro_address + index];
    state->linear_acceleration[index] =
        sensor_data[imu_.accelerometer_address + index];
  }
  state_ = std::move(state);

  return true;
}

bool ImuComponent::read_state(std::shared_ptr<const ImuState> &state) const {
  if (!initialized_) {
    LOG_ERROR << "imu '" << info_.name << "' is not initialized.";
    return false;
  }

  state = state_;
  return state != nullptr;
}

bool ImuComponent::read(const mjContext &context, ImuState &state) const {
  UNUSED(context);
  std::shared_ptr<const ImuState> snapshot;
  if (!read_state(snapshot)) {
    return false;
  }
  state = *snapshot;
  return true;
}

} // namespace mujoco_simulation
