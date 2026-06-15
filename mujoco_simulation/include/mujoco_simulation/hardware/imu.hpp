#pragma once

#include <mujoco/mujoco.h>

#include <string>

#include "mujoco_simulation/hardware/data.hpp"
#include "mujoco_simulation/hardware/hardware_interface.hpp"
#include "mujoco_simulation/hardware/mj_context.hpp"

namespace mujoco_simulation {

struct ImuInfo {
  std::string name;

  std::string framequat_sensor_name;
  std::string gyro_sensor_name;
  std::string accelerometer_sensor_name;
};

struct ImuCommand {};

// https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg
struct ImuState {
  Quaterniond orientation{0.0, 0.0, 0.0, 1.0};
  Vector9d orientation_covariance{};
  Vector3d angular_velocity{0.0, 0.0, 0.0};
  Vector9d angular_velocity_covariance{};
  Vector3d linear_acceleration{0.0, 0.0, 0.0};
  Vector9d linear_acceleration_covariance{};
};

class Imu : public HardwareInterface<ImuInfo, ImuCommand, ImuState> {
 public:
  explicit Imu(const MjContext& context);
  ~Imu() override = default;

  bool init(const ImuInfo& data) override;
  bool reset() override;

  bool write(const ImuCommand& command) override;
  bool read(ImuState& state) override;

  const std::string& last_error() const override { return last_error_; }

 private:
  MjContext context_{};

  int framequat_address_{-1};
  int gyro_address_{-1};
  int accelerometer_address_{-1};

  ImuInfo data_;
  ImuState state_;
  std::string last_error_;
};

}  // namespace mujoco_simulation
