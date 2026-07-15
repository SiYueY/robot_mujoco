#pragma once

#include <string>

#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class ImuComponent : public SimulationComponent {
 public:
  explicit ImuComponent(ImuInfo info);

  std::string name() const noexcept override;
  bool bind(const mjModel& model) override;
  bool reset(const mjModel& model, mjData& data) override;
  bool update(const UpdateContext& context) override;

  const ImuInfo& info() const noexcept { return info_; }
  bool read(ImuState& state) const;

 private:
  static bool validate_sensor_binding(const mjModel& model, std::string component_name,
                                      std::string sensor_name, int expected_type, int expected_dim,
                                      int* sensor_id, int* sensor_address);

  ImuInfo info_;
  ImuState state_{};
  int framequat_sensor_id_{-1};
  int framequat_address_{-1};
  int gyro_sensor_id_{-1};
  int gyro_address_{-1};
  int accelerometer_sensor_id_{-1};
  int accelerometer_address_{-1};
  std::uint64_t sample_sequence_{0};
};

}  // namespace mujoco_simulation
