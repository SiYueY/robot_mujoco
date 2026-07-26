#include "mujoco_simulation/component/lidar/lidar_component.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"
#include "mujoco_simulation/common/math.hpp"

namespace mujoco_simulation {

LidarComponent::LidarComponent(LidarInfo info)
    : SimulationComponent(info.name, info.update_rate), info_(std::move(info)) {
}

bool LidarComponent::init(const mjContext &context) {
  initialized_ = false;
  beam_addresses_.clear();
  if (!configure(context)) {
    return false;
  }

  const mjModel &model = *context.model;
  if (info_.sensor_prefix.empty()) {
    LOG_ERROR << "lidar '" << info_.name
              << "' sensor prefix must not be empty.";
    return false;
  }
  if (!std::isfinite(info_.angle_min) || !std::isfinite(info_.angle_max) ||
      !std::isfinite(info_.angle_increment)) {
    LOG_ERROR << "lidar '" << info_.name
              << "' angular configuration must be finite.";
    return false;
  }
  if (!std::isfinite(info_.range_min) || !std::isfinite(info_.range_max)) {
    LOG_ERROR << "lidar '" << info_.name
              << "' range configuration must be finite.";
    return false;
  }
  if (info_.angle_increment <= 0.0) {
    LOG_ERROR << "lidar '" << info_.name
              << "' angle increment must be positive.";
    return false;
  }
  if (info_.angle_max < info_.angle_min) {
    LOG_ERROR << "lidar '" << info_.name
              << "' angle maximum is less than angle minimum.";
    return false;
  }
  if (info_.range_max < info_.range_min) {
    LOG_ERROR << "lidar '" << info_.name
              << "' range maximum is less than range minimum.";
    return false;
  }

  const double span =
      (info_.angle_max - info_.angle_min) / info_.angle_increment;
  const double rounded_span = std::round(span);
  if (!equal(span, rounded_span) ||
      rounded_span > static_cast<double>(std::numeric_limits<int>::max() - 1)) {
    LOG_ERROR << "lidar '" << info_.name
              << "' angle span must be an integral number of increments.";
    return false;
  }
  const int beam_count = static_cast<int>(rounded_span) + 1;

  beam_addresses_.resize(static_cast<std::size_t>(beam_count), -1);
  for (int beam_index = 0; beam_index < beam_count; ++beam_index) {
    const std::string sensor_name =
        info_.sensor_prefix + "-" + std::to_string(beam_index);
    const int sensor_id = mj_name2id(&model, mjOBJ_SENSOR, sensor_name.c_str());
    if (sensor_id < 0) {
      LOG_ERROR << "lidar '" << info_.name << "' beam sensor '" << sensor_name
                << "' was not found in the model.";
      return false;
    }
    if (model.sensor_type[sensor_id] != mjSENS_RANGEFINDER) {
      LOG_ERROR << "lidar '" << info_.name << "' beam sensor '" << sensor_name
                << "' has type " << model.sensor_type[sensor_id]
                << ", expected " << mjSENS_RANGEFINDER << ".";
      return false;
    }
    if (model.sensor_dim[sensor_id] != 1) {
      LOG_ERROR << "lidar '" << info_.name << "' beam sensor '" << sensor_name
                << "' has dimension " << model.sensor_dim[sensor_id]
                << ", expected 1.";
      return false;
    }
    const int address = model.sensor_adr[sensor_id];
    if (address < 0 || address >= model.nsensordata) {
      LOG_ERROR << "lidar '" << info_.name << "' beam sensor '" << sensor_name
                << "' has an out-of-range sensor address.";
      return false;
    }
    beam_addresses_[static_cast<std::size_t>(beam_index)] = address;
  }

  sequence_ = 0;
  auto state = std::make_shared<LidarState>();
  state->frame_id = info_.frame_id;
  state->angle_min = info_.angle_min;
  state->angle_max = info_.angle_max;
  state->angle_increment = info_.angle_increment;
  state->range_min = info_.range_min;
  state->range_max = info_.range_max;
  state->ranges.assign(beam_addresses_.size(),
                       std::numeric_limits<double>::infinity());
  state->intensities.assign(beam_addresses_.size(), 0.0);
  state_ = std::move(state);
  initialized_ = true;
  return true;
}

bool LidarComponent::reset(const mjContext &context) {
  UNUSED(context);
  if (!initialized_) {
    LOG_ERROR << "lidar '" << info_.name << "' is not initialized.";
    return false;
  }

  sequence_ = 0;
  auto state = std::make_shared<LidarState>();
  state->frame_id = info_.frame_id;
  state->angle_min = info_.angle_min;
  state->angle_max = info_.angle_max;
  state->angle_increment = info_.angle_increment;
  state->range_min = info_.range_min;
  state->range_max = info_.range_max;
  state->ranges.assign(beam_addresses_.size(),
                       std::numeric_limits<double>::infinity());
  state->intensities.assign(beam_addresses_.size(), 0.0);
  state_ = std::move(state);
  return true;
}

bool LidarComponent::update(const mjContext &context) {
  if (!initialized_) {
    LOG_ERROR << "lidar '" << info_.name << "' is not initialized.";
    return false;
  }

  auto state = std::make_shared<LidarState>();
  state->sequence = ++sequence_;
  state->timestamp = context.data->time;
  state->frame_id = info_.frame_id;
  state->angle_min = info_.angle_min;
  state->angle_max = info_.angle_max;
  state->angle_increment = info_.angle_increment;
  state->range_min = info_.range_min;
  state->range_max = info_.range_max;
  state->scan_time = info_.update_rate > 0.0 ? 1.0 / info_.update_rate
                                             : context.model->opt.timestep;
  state->time_increment = 0.0;
  state->ranges.resize(beam_addresses_.size());
  state->intensities.resize(beam_addresses_.size());

  for (std::size_t beam_index = 0; beam_index < beam_addresses_.size();
       ++beam_index) {
    const double range = context.data->sensordata[beam_addresses_[beam_index]];
    state->ranges[beam_index] =
        (!std::isfinite(range) || range < info_.range_min ||
         range > info_.range_max)
            ? std::numeric_limits<double>::infinity()
            : range;
    state->intensities[beam_index] = 0.0;
  }
  state_ = std::move(state);

  return true;
}

bool LidarComponent::read_state(
    std::shared_ptr<const LidarState> &state) const {
  if (!initialized_) {
    LOG_ERROR << "lidar '" << info_.name << "' is not initialized.";
    return false;
  }
  state = state_;
  return state != nullptr;
}

bool LidarComponent::read(const mjContext &context, LidarState &state) const {
  UNUSED(context);
  std::shared_ptr<const LidarState> snapshot;
  if (!read_state(snapshot)) {
    return false;
  }
  state = *snapshot;
  return true;
}

} // namespace mujoco_simulation
