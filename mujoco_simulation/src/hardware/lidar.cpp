#include "mujoco_simulation/hardware/lidar.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace mujoco_simulation {
namespace {

int parse_beam_index(const std::string& sensor_name, const std::string& prefix) {
  const std::string expected_prefix = prefix + "-";
  if (sensor_name.rfind(expected_prefix, 0) != 0) {
    return -1;
  }

  const std::string suffix = sensor_name.substr(expected_prefix.size());
  if (suffix.empty() || !std::all_of(suffix.begin(), suffix.end(),
                                     [](unsigned char c) { return std::isdigit(c) != 0; })) {
    return -1;
  }
  return std::stoi(suffix);
}

}  // namespace

Lidar::Lidar(const MjContext& context) : context_(context) {}

bool Lidar::init(const LidarInfo& data) {
  data_ = data;
  state_ = {};
  sensor_addresses_.clear();
  last_read_time_ = 0.0;
  last_error_.clear();

  if (context_.model == nullptr || context_.data == nullptr) {
    return set_error("MuJoCo model/data is not available for lidar '" + data.name + "'.");
  }
  if (data.sensor_prefix.empty()) {
    return set_error("Lidar '" + data.name + "' requires a non-empty sensor_prefix.");
  }
  if (data.angle_increment <= 0.0 || data.angle_max < data.angle_min) {
    return set_error("Lidar '" + data.name + "' has invalid angular configuration.");
  }

  const double span = (data.angle_max - data.angle_min) / data.angle_increment;
  const int beam_count = static_cast<int>(std::llround(span)) + 1;
  if (beam_count <= 0) {
    return set_error("Lidar '" + data.name + "' computed an invalid beam count.");
  }

  sensor_addresses_.assign(static_cast<std::size_t>(beam_count), -1);
  for (int sensor_id = 0; sensor_id < context_.model->nsensor; ++sensor_id) {
    if (context_.model->sensor_type[sensor_id] != mjSENS_RANGEFINDER) {
      continue;
    }
    const char* sensor_name = mj_id2name(context_.model, mjOBJ_SENSOR, sensor_id);
    if (sensor_name == nullptr) {
      continue;
    }

    const int beam_index = parse_beam_index(sensor_name, data.sensor_prefix);
    if (beam_index < 0 || beam_index >= beam_count) {
      continue;
    }
    sensor_addresses_[static_cast<std::size_t>(beam_index)] = context_.model->sensor_adr[sensor_id];
  }

  if (std::find(sensor_addresses_.begin(), sensor_addresses_.end(), -1) !=
      sensor_addresses_.end()) {
    return set_error("Lidar '" + data.name + "' is missing one or more rangefinder beams.");
  }

  state_.laser_scan.frame_id = data.frame_name;
  state_.laser_scan.angle_min = data.angle_min;
  state_.laser_scan.angle_max = data.angle_max;
  state_.laser_scan.angle_increment = data.angle_increment;
  state_.laser_scan.range_min = data.range_min;
  state_.laser_scan.range_max = data.range_max;
  state_.laser_scan.time_increment = 0.0;
  state_.laser_scan.scan_time = 0.0;
  state_.laser_scan.ranges.assign(sensor_addresses_.size(), -1.0);
  state_.laser_scan.intensities.assign(sensor_addresses_.size(), 0.0);
  return true;
}

bool Lidar::reset() {
  last_error_.clear();
  last_read_time_ = 0.0;
  std::fill(state_.laser_scan.ranges.begin(), state_.laser_scan.ranges.end(), -1.0);
  std::fill(state_.laser_scan.intensities.begin(), state_.laser_scan.intensities.end(), 0.0);
  state_.laser_scan.time_increment = 0.0;
  state_.laser_scan.scan_time = 0.0;
  return true;
}

bool Lidar::write(const LidarCommand&) {
  last_error_.clear();
  return true;
}

bool Lidar::read(LidarState& state) {
  last_error_.clear();
  if (context_.data == nullptr || sensor_addresses_.empty()) {
    return set_error("Lidar '" + data_.name + "' is not initialized.");
  }

  const double current_time = context_.data->time;
  const double scan_time = last_read_time_ == 0.0 ? 0.0 : current_time - last_read_time_;
  last_read_time_ = current_time;
  state_.laser_scan.scan_time = scan_time;
  state_.laser_scan.time_increment =
      sensor_addresses_.empty() ? 0.0 : scan_time / static_cast<double>(sensor_addresses_.size());

  for (std::size_t i = 0; i < sensor_addresses_.size(); ++i) {
    const int address = sensor_addresses_[i];
    const double range = context_.data->sensordata[address];
    state_.laser_scan.ranges[i] =
        (range < data_.range_min || range > data_.range_max) ? -1.0 : range;
    state_.laser_scan.intensities[i] = 0.0;
  }

  state = state_;
  return true;
}

bool Lidar::set_error(const std::string& message) {
  last_error_ = message;
  return false;
}

}  // namespace mujoco_simulation
