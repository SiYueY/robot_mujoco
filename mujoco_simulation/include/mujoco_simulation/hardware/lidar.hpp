#pragma once

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

#include "mujoco_simulation/hardware/hardware_interface.hpp"
#include "mujoco_simulation/hardware/mj_context.hpp"

namespace mujoco_simulation {

struct LidarInfo {
  std::string name;
  std::string frame_name;
  std::string sensor_prefix;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

struct LidarCommand {};

// https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg
struct LaserScan {
  std::string frame_id;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double time_increment{0.0};
  double scan_time{0.0};
  double range_min{0.0};
  double range_max{0.0};
  std::vector<double> ranges;
  std::vector<double> intensities;
};

// https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/PointCloud.msg
// struct PointCloud {
//   std_msgs / Header header
//   uint32_t height;
//   uint32_t width;
//   std::vector<PointField> fields;
//   bool is_bigendian;
//   uint32_t point_step;
//   uint32_t row_step;
//   std::vector<uint8_t> data;
//   bool is_dense;
// };
struct LidarState {
  LaserScan laser_scan;
  // PointCloud point_cloud;
};

class Lidar : public HardwareInterface<LidarInfo, LidarCommand, LidarState> {
 public:
  explicit Lidar(const MjContext& context);
  ~Lidar() override = default;

  bool init(const LidarInfo& data) override;
  bool reset() override;
  bool write(const LidarCommand& command) override;
  bool read(LidarState& state) override;

  const LidarInfo& data() const { return data_; }
  const std::string& last_error() const override { return last_error_; }

 private:
  bool set_error(const std::string& message);

  MjContext context_{};

  LidarInfo data_;
  LidarState state_;
  std::vector<int> sensor_addresses_;
  double last_read_time_{0.0};
  std::string last_error_;
};

}  // namespace mujoco_simulation
