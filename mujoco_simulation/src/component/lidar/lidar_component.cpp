#include "mujoco_simulation/component/lidar/lidar_component.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <utility>

namespace mujoco_simulation {
namespace {

int parse_beam_index(const std::string& sensor_name, const std::string& prefix) {
  const std::string expected_prefix = prefix + "-";
  if (sensor_name.rfind(expected_prefix, 0) != 0) {
    return -1;
  }

  const std::string suffix = sensor_name.substr(expected_prefix.size());
  if (suffix.empty() || !std::all_of(suffix.begin(), suffix.end(), [](unsigned char value) {
        return std::isdigit(value) != 0;
      })) {
    return -1;
  }
  return std::stoi(suffix);
}

}  // namespace

LidarComponent::LidarComponent(LidarConfig info) : info_(std::move(info)) {}

std::string_view LidarComponent::name() const noexcept { return info_.common.name; }

Status LidarComponent::bind(const mjModel& model) {
  if (info_.common.name.empty()) {
    return Status::invalid_argument("Lidar name must not be empty.");
  }
  if (info_.sensor_prefix.empty()) {
    return Status::invalid_argument("Lidar '" + info_.common.name +
                                    "' requires a non-empty sensor_prefix.");
  }
  if (info_.angle_increment <= 0.0 || info_.angle_max < info_.angle_min) {
    return Status::invalid_argument("Lidar '" + info_.common.name +
                                    "' has invalid angular configuration.");
  }
  if (info_.range_max < info_.range_min) {
    return Status::invalid_argument("Lidar '" + info_.common.name +
                                    "' has invalid range configuration.");
  }

  const double span = (info_.angle_max - info_.angle_min) / info_.angle_increment;
  const int beam_count = static_cast<int>(std::llround(span)) + 1;
  if (beam_count <= 0) {
    return Status::invalid_argument("Lidar '" + info_.common.name +
                                    "' computed an invalid beam count.");
  }

  binding_.beams.assign(static_cast<std::size_t>(beam_count), {});
  for (int beam_index = 0; beam_index < beam_count; ++beam_index) {
    binding_.beams[static_cast<std::size_t>(beam_index)].beam_index =
        static_cast<std::size_t>(beam_index);
    binding_.beams[static_cast<std::size_t>(beam_index)].sensor_id = -1;
    binding_.beams[static_cast<std::size_t>(beam_index)].sensor_address = -1;
  }

  for (int sensor_id = 0; sensor_id < model.nsensor; ++sensor_id) {
    if (model.sensor_type[sensor_id] != mjSENS_RANGEFINDER) {
      continue;
    }
    if (model.sensor_dim[sensor_id] != 1) {
      continue;
    }

    const char* sensor_name = mj_id2name(&model, mjOBJ_SENSOR, sensor_id);
    if (sensor_name == nullptr) {
      continue;
    }

    const int beam_index = parse_beam_index(sensor_name, info_.sensor_prefix);
    if (beam_index < 0 || beam_index >= beam_count) {
      continue;
    }

    LidarBeamBinding& beam = binding_.beams[static_cast<std::size_t>(beam_index)];
    if (beam.sensor_id >= 0) {
      return Status::model_validation_failed("Lidar '" + info_.common.name +
                                             "' found a duplicate MuJoCo rangefinder beam '" +
                                             std::string(sensor_name) + "'.");
    }

    const int address = model.sensor_adr[sensor_id];
    if (address < 0 || address >= model.nsensordata) {
      return Status::model_validation_failed(
          "Lidar '" + info_.common.name + "' found MuJoCo rangefinder sensor '" +
          std::string(sensor_name) + "' with an invalid sensordata address.");
    }
    beam.sensor_id = sensor_id;
    beam.sensor_address = address;
  }

  for (const LidarBeamBinding& beam : binding_.beams) {
    if (beam.sensor_id < 0 || beam.sensor_address < 0) {
      return Status::model_validation_failed("Lidar '" + info_.common.name +
                                             "' is missing one or more MuJoCo rangefinder beams.");
    }
  }

  return set_defaults();
}

Status LidarComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  (void)data;
  sample_sequence_ = 0;
  return set_defaults();
}

double LidarComponent::update_rate() const noexcept { return info_.common.update_rate; }

Status LidarComponent::sample(const SensorSampleContext& context) {
  (void)context.model;
  (void)context.step_count;
  if (binding_.beams.empty()) {
    return Status::failed_precondition("Lidar component is not bound: " + info_.common.name);
  }

  state_.sequence = ++sample_sequence_;
  state_.timestamp_ns = context.simulation_time <= 0.0
                            ? 0
                            : static_cast<std::uint64_t>(context.simulation_time * 1.0e9);
  state_.frame_id = info_.common.frame_id;
  state_.scan_time = info_.common.update_rate > 0.0 ? 1.0 / info_.common.update_rate : 0.0;
  state_.time_increment = 0.0;

  for (const LidarBeamBinding& beam : binding_.beams) {
    const double range = context.data.sensordata[beam.sensor_address];
    state_.ranges[beam.beam_index] =
        (!std::isfinite(range) || range < info_.range_min || range > info_.range_max)
            ? std::numeric_limits<double>::infinity()
            : range;
    state_.intensities[beam.beam_index] = 0.0;
  }

  return Status::Ok();
}

Status LidarComponent::read(LidarSample& state) const {
  state = state_;
  return Status::Ok();
}

Status LidarComponent::set_defaults() {
  state_ = {};
  state_.frame_id = info_.common.frame_id;
  state_.angle_min = info_.angle_min;
  state_.angle_max = info_.angle_max;
  state_.angle_increment = info_.angle_increment;
  state_.range_min = info_.range_min;
  state_.range_max = info_.range_max;
  state_.time_increment = 0.0;
  state_.scan_time = 0.0;
  state_.ranges.assign(binding_.beams.size(), std::numeric_limits<double>::infinity());
  state_.intensities.assign(binding_.beams.size(), 0.0);
  return Status::Ok();
}

}  // namespace mujoco_simulation
