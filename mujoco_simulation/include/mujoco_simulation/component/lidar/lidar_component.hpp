#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

struct LidarBeamBinding {
  std::size_t beam_index{0};
  int sensor_id{-1};
  int sensor_address{-1};
};

struct LidarBinding {
  std::vector<LidarBeamBinding> beams;
};

class LidarComponent : public SimulationComponent {
 public:
  explicit LidarComponent(LidarConfig info);

  std::string name() const noexcept override;
  ResultCode bind(const mjModel& model) override;
  ResultCode reset(const mjModel& model, mjData& data) override;
  ResultCode update(const UpdateContext& context) override;

  const LidarConfig& info() const noexcept { return info_; }
  ResultCode read(LidarState& state) const;

 private:
  ResultCode set_defaults();

  LidarConfig info_;
  LidarState state_{};
  LidarBinding binding_{};
  std::uint64_t sample_sequence_{0};
};

}  // namespace mujoco_simulation
