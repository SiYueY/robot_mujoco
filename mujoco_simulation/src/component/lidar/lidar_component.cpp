#include "mujoco_simulation/component/lidar/lidar_component.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <utility>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/component/update_context.hpp"

namespace mujoco_simulation {
namespace {

struct LidarBeamBinding {
  std::size_t beam_index{0};
  int sensor_id{-1};
  int sensor_address{-1};
};

struct LidarBinding {
  std::vector<LidarBeamBinding> beams;
};

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

struct LidarComponent::Impl {
  LidarBinding binding;
};

LidarComponent::LidarComponent(LidarInfo info)
    : impl_(std::make_unique<Impl>()), info_(std::move(info)) {}

LidarComponent::~LidarComponent() = default;

LidarComponent::LidarComponent(LidarComponent&&) noexcept = default;

LidarComponent& LidarComponent::operator=(LidarComponent&&) noexcept = default;

std::string LidarComponent::name() const noexcept { return info_.common.name; }

bool LidarComponent::bind(const mjModel& model) {
  if (info_.common.name.empty()) {
    LOG_ERROR << "LidarComponent::bind"
              << ": "
              << "lidar component name must not be empty.";
    return false;
  }
  if (model.opt.timestep <= 0.0) {
    LOG_ERROR << "LidarComponent::bind"
              << ": "
              << "model timestep must be positive.";
    return false;
  }
  if (info_.sensor_prefix.empty()) {
    LOG_ERROR << "LidarComponent::bind"
              << ": "
              << "sensor prefix must not be empty.";
    return false;
  }
  if (info_.angle_increment <= 0.0 || info_.angle_max < info_.angle_min) {
    LOG_ERROR << "LidarComponent::bind"
              << ": "
              << "lidar angles are invalid.";
    return false;
  }
  if (info_.range_max < info_.range_min) {
    LOG_ERROR << "LidarComponent::bind"
              << ": "
              << "lidar ranges are invalid.";
    return false;
  }

  const double span = (info_.angle_max - info_.angle_min) / info_.angle_increment;
  const int beam_count = static_cast<int>(std::llround(span)) + 1;
  if (beam_count <= 0) {
    LOG_ERROR << "LidarComponent::bind"
              << ": "
              << "computed beam count must be positive.";
    return false;
  }

  impl_->binding.beams.assign(static_cast<std::size_t>(beam_count), {});
  for (int beam_index = 0; beam_index < beam_count; ++beam_index) {
    impl_->binding.beams[static_cast<std::size_t>(beam_index)].beam_index =
        static_cast<std::size_t>(beam_index);
    impl_->binding.beams[static_cast<std::size_t>(beam_index)].sensor_id = -1;
    impl_->binding.beams[static_cast<std::size_t>(beam_index)].sensor_address = -1;
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

    LidarBeamBinding& beam = impl_->binding.beams[static_cast<std::size_t>(beam_index)];
    if (beam.sensor_id >= 0) {
      LOG_ERROR << "LidarComponent::bind"
                << ": "
                << "duplicate lidar beam sensor binding.";
      return false;
    }

    const int address = model.sensor_adr[sensor_id];
    if (address < 0 || address >= model.nsensordata) {
      LOG_ERROR << "LidarComponent::bind"
                << ": "
                << "lidar sensor address is out of range.";
      return false;
    }
    beam.sensor_id = sensor_id;
    beam.sensor_address = address;
  }

  for (const LidarBeamBinding& beam : impl_->binding.beams) {
    if (beam.sensor_id < 0 || beam.sensor_address < 0) {
      LOG_ERROR << "LidarComponent::bind"
                << ": "
                << "missing lidar beam sensor binding.";
      return false;
    }
  }

  if (!set_update_rate(info_.common.update_rate, 1.0 / model.opt.timestep)) {
    return false;
  }

  return set_defaults();
}

bool LidarComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  (void)data;
  sample_sequence_ = 0;
  return set_defaults();
}

bool LidarComponent::update(const UpdateContext& context) {
  (void)context.model;
  (void)context.step_count;
  if (impl_->binding.beams.empty()) {
    LOG_ERROR << "LidarComponent::update"
              << ": "
              << "lidar must be bound before update.";
    return false;
  }

  state_.sequence = ++sample_sequence_;
  state_.timestamp_ns = context.simulation_time <= 0.0
                            ? 0
                            : static_cast<std::uint64_t>(context.simulation_time * 1.0e9);
  state_.frame_id = info_.common.frame_id;
  state_.scan_time = info_.common.update_rate > 0.0 ? 1.0 / info_.common.update_rate : 0.0;
  state_.time_increment = 0.0;

  for (const LidarBeamBinding& beam : impl_->binding.beams) {
    const double range = context.data.sensordata[beam.sensor_address];
    state_.ranges[beam.beam_index] =
        (!std::isfinite(range) || range < info_.range_min || range > info_.range_max)
            ? std::numeric_limits<double>::infinity()
            : range;
    state_.intensities[beam.beam_index] = 0.0;
  }

  return true;
}

bool LidarComponent::read(LidarState& state) const {
  state = state_;
  return true;
}

bool LidarComponent::set_defaults() {
  state_ = {};
  state_.frame_id = info_.common.frame_id;
  state_.angle_min = info_.angle_min;
  state_.angle_max = info_.angle_max;
  state_.angle_increment = info_.angle_increment;
  state_.range_min = info_.range_min;
  state_.range_max = info_.range_max;
  state_.time_increment = 0.0;
  state_.scan_time = 0.0;
  state_.ranges.assign(impl_->binding.beams.size(), std::numeric_limits<double>::infinity());
  state_.intensities.assign(impl_->binding.beams.size(), 0.0);
  return true;
}

}  // namespace mujoco_simulation
