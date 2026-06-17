#include "mujoco_simulation/component/imu/imu_component.hpp"

#include <utility>

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

ImuComponent::ImuComponent(ImuConfig info) : info_(std::move(info)) {}

std::string_view ImuComponent::name() const noexcept { return info_.common.name; }

Status ImuComponent::validate_sensor_binding(const mjModel& model, std::string_view component_name,
                                             std::string_view sensor_name, int expected_type,
                                             int expected_dim, int* sensor_id,
                                             int* sensor_address) {
  if (sensor_id == nullptr || sensor_address == nullptr) {
    return Status::invalid_argument("IMU binding output pointers must not be null.");
  }
  if (sensor_name.empty()) {
    return Status::invalid_argument("IMU sensor name must not be empty for '" +
                                    std::string(component_name) + "'.");
  }

  const int id = mj_name2id(&model, mjOBJ_SENSOR, std::string(sensor_name).c_str());
  if (id < 0) {
    return Status::binding_failed("IMU '" + std::string(component_name) +
                                  "' could not bind to MuJoCo sensor '" + std::string(sensor_name) +
                                  "'.");
  }
  if (model.sensor_type[id] != expected_type) {
    return Status::model_validation_failed("IMU '" + std::string(component_name) +
                                           "' expected MuJoCo sensor '" + std::string(sensor_name) +
                                           "' to have a different sensor type.");
  }
  if (model.sensor_dim[id] != expected_dim) {
    return Status::model_validation_failed("IMU '" + std::string(component_name) +
                                           "' expected MuJoCo sensor '" + std::string(sensor_name) +
                                           "' to expose a different sensor dimension.");
  }

  const int address = model.sensor_adr[id];
  if (address < 0 || address + expected_dim > model.nsensordata) {
    return Status::model_validation_failed(
        "IMU '" + std::string(component_name) + "' is bound to MuJoCo sensor '" +
        std::string(sensor_name) + "' with an invalid sensordata address.");
  }

  *sensor_id = id;
  *sensor_address = address;
  return Status::Ok();
}

Status ImuComponent::bind(const mjModel& model) {
  if (info_.common.name.empty()) {
    return Status::invalid_argument("IMU name must not be empty.");
  }

  Status status = validate_sensor_binding(model, info_.common.name, info_.framequat_sensor_name,
                                          mjSENS_FRAMEQUAT, 4, &binding_.framequat_sensor_id,
                                          &binding_.framequat_address);
  if (!status.ok()) {
    return status;
  }
  status = validate_sensor_binding(model, info_.common.name, info_.gyro_sensor_name, mjSENS_GYRO, 3,
                                   &binding_.gyro_sensor_id, &binding_.gyro_address);
  if (!status.ok()) {
    return status;
  }
  status = validate_sensor_binding(model, info_.common.name, info_.accelerometer_sensor_name,
                                   mjSENS_ACCELEROMETER, 3, &binding_.accelerometer_sensor_id,
                                   &binding_.accelerometer_address);
  if (!status.ok()) {
    return status;
  }

  sample_sequence_ = 0;
  state_ = {};
  state_.frame_id = info_.common.frame_id;
  state_.orientation = {0.0, 0.0, 0.0, 1.0};
  state_.orientation_covariance = info_.orientation_covariance;
  state_.angular_velocity_covariance = info_.angular_velocity_covariance;
  state_.linear_acceleration_covariance = info_.linear_acceleration_covariance;
  return Status::Ok();
}

Status ImuComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  (void)data;
  sample_sequence_ = 0;
  state_ = {};
  state_.frame_id = info_.common.frame_id;
  state_.orientation = {0.0, 0.0, 0.0, 1.0};
  state_.orientation_covariance = info_.orientation_covariance;
  state_.angular_velocity_covariance = info_.angular_velocity_covariance;
  state_.linear_acceleration_covariance = info_.linear_acceleration_covariance;
  return Status::Ok();
}

double ImuComponent::update_rate() const noexcept { return info_.common.update_rate; }

Status ImuComponent::sample(const SensorSampleContext& context) {
  (void)context.model;
  (void)context.step_count;
  if (binding_.framequat_address < 0 || binding_.gyro_address < 0 ||
      binding_.accelerometer_address < 0) {
    return Status::failed_precondition("IMU component is not bound: " + info_.common.name);
  }

  state_.sequence = ++sample_sequence_;
  state_.timestamp_ns = context.simulation_time <= 0.0
                            ? 0
                            : static_cast<std::uint64_t>(context.simulation_time * 1.0e9);
  state_.frame_id = info_.common.frame_id;
  state_.orientation_covariance = info_.orientation_covariance;
  state_.angular_velocity_covariance = info_.angular_velocity_covariance;
  state_.linear_acceleration_covariance = info_.linear_acceleration_covariance;
  state_.orientation[3] = context.data.sensordata[binding_.framequat_address];
  state_.orientation[0] = context.data.sensordata[binding_.framequat_address + 1];
  state_.orientation[1] = context.data.sensordata[binding_.framequat_address + 2];
  state_.orientation[2] = context.data.sensordata[binding_.framequat_address + 3];

  if (!copy_sensor_vector(context.data, binding_.gyro_address, state_.angular_velocity.data(), 3) ||
      !copy_sensor_vector(context.data, binding_.accelerometer_address,
                          state_.linear_acceleration.data(), 3)) {
    return Status::internal("Failed to sample IMU sensor data: " + info_.common.name);
  }
  return Status::Ok();
}

Status ImuComponent::read(ImuSample& state) const {
  state = state_;
  return Status::Ok();
}

}  // namespace mujoco_simulation
