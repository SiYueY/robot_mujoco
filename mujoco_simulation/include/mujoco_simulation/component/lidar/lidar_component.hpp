#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class LidarComponent : public SimulationComponent {
 public:
  explicit LidarComponent(LidarInfo info);
  ~LidarComponent();

  LidarComponent(const LidarComponent&) = delete;
  LidarComponent& operator=(const LidarComponent&) = delete;
  LidarComponent(LidarComponent&&) noexcept;
  LidarComponent& operator=(LidarComponent&&) noexcept;

  std::string name() const noexcept override;
  bool bind(const mjModel& model) override;
  bool reset(const mjModel& model, mjData& data) override;
  bool update(const UpdateContext& context) override;

  const LidarInfo& info() const noexcept { return info_; }
  bool read(LidarState& state) const;

 private:
  bool set_defaults();

  struct Impl;
  std::unique_ptr<Impl> impl_;
  LidarInfo info_;
  LidarState state_{};
  std::uint64_t sample_sequence_{0};
};

}  // namespace mujoco_simulation
