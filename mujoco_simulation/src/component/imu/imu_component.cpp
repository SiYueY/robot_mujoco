#include "mujoco_simulation/component/imu/imu_component.hpp"

#include <utility>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/component/update_context.hpp"

namespace mujoco_simulation {
namespace {

bool copy_sensor_vector(const mjData& data, int address, double* dest, int count) {
  if (address < 0 || dest == nullptr || count <= 0) {
    return false;
  }
  for (int index = 0; index < count; ++index) {
    dest[index] = data.sensordata[address + index];
  }
  return true;
}

}  // namespace

ImuComponent::ImuComponent(ImuInfo info) : info_(std::move(info)) {}

std::string ImuComponent::name() const noexcept { return info_.common.name; }

bool ImuComponent::validate_sensor_binding(const mjModel& model, std::string component_name,
                                           std::string sensor_name, int expected_type,
                                           int expected_dim, int* sensor_id, int* sensor_address) {
  (void)component_name;
  if (sensor_id == nullptr || sensor_address == nullptr) {
    return log_component_error("ImuComponent::validate_sensor_binding",
                               "sensor output pointers must not be null.");
  }
  if (sensor_name.empty()) {
    return log_component_error("ImuComponent::validate_sensor_binding",
                               "sensor name must not be empty.");
  }

  const int id = mj_name2id(&model, mjOBJ_SENSOR, std::string(sensor_name).c_str());
  if (id < 0) {
    return log_component_error("ImuComponent::validate_sensor_binding",
                               "sensor was not found in model.");
  }
  if (model.sensor_type[id] != expected_type) {
    return log_component_error("ImuComponent::validate_sensor_binding",
                               "sensor type does not match expected type.");
  }
  if (model.sensor_dim[id] != expected_dim) {
    return log_component_error("ImuComponent::validate_sensor_binding",
                               "sensor dimension does not match expected dimension.");
  }

  const int address = model.sensor_adr[id];
  if (address < 0 || address + expected_dim > model.nsensordata) {
    return log_component_error("ImuComponent::validate_sensor_binding",
                               "sensor address is out of range.");
  }

  *sensor_id = id;
  *sensor_address = address;
  return true;
}

bool ImuComponent::bind(const mjModel& model) {
  if (info_.common.name.empty()) {
    return log_component_error("ImuComponent::bind", "imu component name must not be empty.");
  }
  if (model.opt.timestep <= 0.0) {
    return log_component_error("ImuComponent::bind", "model timestep must be positive.");
  }

  if (!validate_sensor_binding(model, info_.common.name, info_.framequat_sensor_name,
                               mjSENS_FRAMEQUAT, 4, &framequat_sensor_id_, &framequat_address_)) {
    return false;
  }
  if (!validate_sensor_binding(model, info_.common.name, info_.gyro_sensor_name, mjSENS_GYRO, 3,
                               &gyro_sensor_id_, &gyro_address_)) {
    return false;
  }
  if (!validate_sensor_binding(model, info_.common.name, info_.accelerometer_sensor_name,
                               mjSENS_ACCELEROMETER, 3, &accelerometer_sensor_id_,
                               &accelerometer_address_)) {
    return false;
  }
  if (!set_update_rate(info_.common.update_rate, 1.0 / model.opt.timestep)) {
    return false;
  }

  sample_sequence_ = 0;
  state_ = {};
  state_.frame_id = info_.common.frame_id;
  state_.orientation = {0.0, 0.0, 0.0, 1.0};
  state_.orientation_covariance = info_.orientation_covariance;
  state_.angular_velocity_covariance = info_.angular_velocity_covariance;
  state_.linear_acceleration_covariance = info_.linear_acceleration_covariance;
  return true;
}

bool ImuComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  (void)data;
  sample_sequence_ = 0;
  state_ = {};
  state_.frame_id = info_.common.frame_id;
  state_.orientation = {0.0, 0.0, 0.0, 1.0};
  state_.orientation_covariance = info_.orientation_covariance;
  state_.angular_velocity_covariance = info_.angular_velocity_covariance;
  state_.linear_acceleration_covariance = info_.linear_acceleration_covariance;
  return true;
}

bool ImuComponent::update(const UpdateContext& context) {
  (void)context.model;
  (void)context.step_count;
  if (framequat_address_ < 0 || gyro_address_ < 0 || accelerometer_address_ < 0) {
    return log_component_error("ImuComponent::update", "imu sensors must be bound before update.");
  }

  state_.sequence = ++sample_sequence_;
  state_.timestamp_ns = context.simulation_time <= 0.0
                            ? 0
                            : static_cast<std::uint64_t>(context.simulation_time * 1.0e9);
  state_.frame_id = info_.common.frame_id;
  state_.orientation_covariance = info_.orientation_covariance;
  state_.angular_velocity_covariance = info_.angular_velocity_covariance;
  state_.linear_acceleration_covariance = info_.linear_acceleration_covariance;
  state_.orientation[3] = context.data.sensordata[framequat_address_];
  state_.orientation[0] = context.data.sensordata[framequat_address_ + 1];
  state_.orientation[1] = context.data.sensordata[framequat_address_ + 2];
  state_.orientation[2] = context.data.sensordata[framequat_address_ + 3];

  if (!copy_sensor_vector(context.data, gyro_address_, state_.angular_velocity.data(), 3) ||
      !copy_sensor_vector(context.data, accelerometer_address_, state_.linear_acceleration.data(),
                          3)) {
    return log_component_error("ImuComponent::update", "failed to copy imu sensor data.");
  }
  return true;
}

bool ImuComponent::read(ImuState& state) const {
  state = state_;
  return true;
}

}  // namespace mujoco_simulation
