#pragma once

#include <string>

#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

struct ImuBinding {
  int framequat_sensor_id{-1};
  int framequat_address{-1};
  int gyro_sensor_id{-1};
  int gyro_address{-1};
  int accelerometer_sensor_id{-1};
  int accelerometer_address{-1};
};

class ImuComponent : public SimulationComponent {
 public:
  explicit ImuComponent(ImuConfig info);

  std::string name() const noexcept override;
  ResultCode bind(const mjModel& model) override;
  ResultCode reset(const mjModel& model, mjData& data) override;
  ResultCode update(const UpdateContext& context) override;

  const ImuConfig& info() const noexcept { return info_; }
  ResultCode read(ImuState& state) const;

 private:
  static ResultCode validate_sensor_binding(const mjModel& model, std::string component_name,
                                            std::string sensor_name, int expected_type,
                                            int expected_dim, int* sensor_id, int* sensor_address);

  ImuConfig info_;
  ImuState state_{};
  ImuBinding binding_{};
  std::uint64_t sample_sequence_{0};
};

}  // namespace mujoco_simulation
