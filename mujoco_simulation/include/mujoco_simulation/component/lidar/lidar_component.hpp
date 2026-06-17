#pragma once

#include <string_view>

#include "mujoco_simulation/component/lidar/lidar_binding.hpp"
#include "mujoco_simulation/component/lidar/lidar_config.hpp"
#include "mujoco_simulation/component/lidar/lidar_sample.hpp"
#include "mujoco_simulation/component/sensor_component.hpp"

namespace mujoco_simulation {

class LidarComponent : public SensorComponent {
 public:
  explicit LidarComponent(LidarConfig info);

  std::string_view name() const noexcept override;
  Status bind(const mjModel& model) override;
  Status reset(const mjModel& model, mjData& data) override;
  double update_rate() const noexcept override;
  Status sample(const SensorSampleContext& context) override;

  const LidarConfig& info() const noexcept { return info_; }
  Status read(LidarSample& state) const;

 private:
  Status set_defaults();

  LidarConfig info_;
  LidarSample state_{};
  LidarBinding binding_{};
  std::uint64_t sample_sequence_{0};
};

}  // namespace mujoco_simulation
