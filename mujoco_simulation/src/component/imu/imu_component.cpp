#include "mujoco_simulation/component/imu/imu_component.hpp"

#include <utility>

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

ImuComponent::ImuComponent(ImuConfig info) : info_(std::move(info)) {}

std::string ImuComponent::name() const noexcept { return info_.common.name; }

ResultCode ImuComponent::validate_sensor_binding(const mjModel& model, std::string component_name,
                                                 std::string sensor_name, int expected_type,
                                                 int expected_dim, int* sensor_id,
                                                 int* sensor_address) {
  (void)component_name;
  if (sensor_id == nullptr || sensor_address == nullptr) {
    return ResultCode::InvalidArgument;
  }
  if (sensor_name.empty()) {
    return ResultCode::InvalidArgument;
  }

  const int id = mj_name2id(&model, mjOBJ_SENSOR, std::string(sensor_name).c_str());
  if (id < 0) {
    return ResultCode::BindingFailed;
  }
  if (model.sensor_type[id] != expected_type) {
    return ResultCode::ModelValidationFailed;
  }
  if (model.sensor_dim[id] != expected_dim) {
    return ResultCode::ModelValidationFailed;
  }

  const int address = model.sensor_adr[id];
  if (address < 0 || address + expected_dim > model.nsensordata) {
    return ResultCode::ModelValidationFailed;
  }

  *sensor_id = id;
  *sensor_address = address;
  return ResultCode::Ok;
}

ResultCode ImuComponent::bind(const mjModel& model) {
  if (info_.common.name.empty()) {
    return ResultCode::InvalidArgument;
  }
  if (model.opt.timestep <= 0.0) {
    return ResultCode::InvalidArgument;
  }

  ResultCode status = validate_sensor_binding(model, info_.common.name, info_.framequat_sensor_name,
                                              mjSENS_FRAMEQUAT, 4, &binding_.framequat_sensor_id,
                                              &binding_.framequat_address);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = validate_sensor_binding(model, info_.common.name, info_.gyro_sensor_name, mjSENS_GYRO, 3,
                                   &binding_.gyro_sensor_id, &binding_.gyro_address);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = validate_sensor_binding(model, info_.common.name, info_.accelerometer_sensor_name,
                                   mjSENS_ACCELEROMETER, 3, &binding_.accelerometer_sensor_id,
                                   &binding_.accelerometer_address);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = set_update_rate(info_.common.update_rate, 1.0 / model.opt.timestep);
  if (status != ResultCode::Ok) {
    return status;
  }

  sample_sequence_ = 0;
  state_ = {};
  state_.frame_id = info_.common.frame_id;
  state_.orientation = {0.0, 0.0, 0.0, 1.0};
  state_.orientation_covariance = info_.orientation_covariance;
  state_.angular_velocity_covariance = info_.angular_velocity_covariance;
  state_.linear_acceleration_covariance = info_.linear_acceleration_covariance;
  return ResultCode::Ok;
}

ResultCode ImuComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  (void)data;
  sample_sequence_ = 0;
  state_ = {};
  state_.frame_id = info_.common.frame_id;
  state_.orientation = {0.0, 0.0, 0.0, 1.0};
  state_.orientation_covariance = info_.orientation_covariance;
  state_.angular_velocity_covariance = info_.angular_velocity_covariance;
  state_.linear_acceleration_covariance = info_.linear_acceleration_covariance;
  return ResultCode::Ok;
}

ResultCode ImuComponent::update(const UpdateContext& context) {
  (void)context.model;
  (void)context.step_count;
  if (binding_.framequat_address < 0 || binding_.gyro_address < 0 ||
      binding_.accelerometer_address < 0) {
    return ResultCode::FailedPrecondition;
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
    return ResultCode::Internal;
  }
  return ResultCode::Ok;
}

ResultCode ImuComponent::read(ImuState& state) const {
  state = state_;
  return ResultCode::Ok;
}

}  // namespace mujoco_simulation
