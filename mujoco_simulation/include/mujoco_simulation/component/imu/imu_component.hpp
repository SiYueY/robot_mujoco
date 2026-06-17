#pragma once

#include <string>
#include <string_view>

#include "mujoco_simulation/component/imu/imu_binding.hpp"
#include "mujoco_simulation/component/imu/imu_config.hpp"
#include "mujoco_simulation/component/imu/imu_sample.hpp"
#include "mujoco_simulation/component/sensor_component.hpp"

namespace mujoco_simulation {

class ImuComponent : public SensorComponent {
 public:
  explicit ImuComponent(ImuConfig info);

  std::string_view name() const noexcept override;
  Status bind(const mjModel& model) override;
  Status reset(const mjModel& model, mjData& data) override;
  double update_rate() const noexcept override;
  Status sample(const SensorSampleContext& context) override;

  const ImuConfig& info() const noexcept { return info_; }
  Status read(ImuSample& state) const;

 private:
  static Status validate_sensor_binding(const mjModel& model, std::string_view component_name,
                                        std::string_view sensor_name, int expected_type,
                                        int expected_dim, int* sensor_id, int* sensor_address);

  ImuConfig info_;
  ImuSample state_{};
  ImuBinding binding_{};
  std::uint64_t sample_sequence_{0};
};

}  // namespace mujoco_simulation
