#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/result_code.hpp"
#include "mujoco_simulation/simulation.hpp"
#include "rclcpp/time.hpp"
#include "realtime_tools/lock_free_queue.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace robot_mujoco_ros2 {

struct PublishImuState {
  std::size_t publisher_index{0};
  mujoco_simulation::ImuState state;
};

struct PublishLidarState {
  std::size_t publisher_index{0};
  mujoco_simulation::LidarState state;
};

struct PublishCameraState {
  std::size_t publisher_index{0};
  const mujoco_simulation::CameraState* state{nullptr};
};

struct PublishBundle {
  rclcpp::Time sim_time{0, 0, RCL_ROS_TIME};
  std::vector<PublishImuState> imus;
  std::vector<PublishLidarState> lidars;
  std::vector<PublishCameraState> cameras;
};

struct SmallPublishSnapshot {
  rclcpp::Time sim_time{0, 0, RCL_ROS_TIME};
  std::vector<PublishImuState> imus;
  std::vector<PublishLidarState> lidars;
};

struct CameraFrameConfig {
  std::uint32_t width{0};
  std::uint32_t height{0};
  bool enable_rgb{false};
  bool enable_depth{false};
};

struct CameraFrame {
  std::size_t publisher_index{0};
  std::uint64_t sequence{0};
  rclcpp::Time acquisition_stamp{0, 0, RCL_ROS_TIME};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t rgb_step{0};
  std::uint32_t depth_step{0};
  bool has_rgb{false};
  bool has_depth{false};
  sensor_msgs::msg::CameraInfo camera_info;
  std::vector<std::uint8_t> rgb_data;
  std::vector<std::uint8_t> depth_data;
};

struct PublishChannelConfig {
  std::size_t imu_count{0};
  std::size_t lidar_count{0};
  std::vector<std::size_t> lidar_sample_counts;
  std::vector<CameraFrameConfig> camera_frames;
  std::size_t camera_queue_capacity{0};
};

class PublishChannel {
 public:
  PublishChannel();
  explicit PublishChannel(PublishChannelConfig config);

  void reset(PublishChannelConfig config);
  [[nodiscard]] mujoco_simulation::ResultCode publish_bundle(const PublishBundle& bundle);
  void update_clock(const rclcpp::Time& sim_time);

  [[nodiscard]] bool consume_latest_small_snapshot(SmallPublishSnapshot* snapshot);
  [[nodiscard]] bool consume_latest_clock(rclcpp::Time* sim_time);
  [[nodiscard]] bool pop_camera_frame(CameraFrame* frame);

  [[nodiscard]] std::size_t camera_queue_capacity() const;
  [[nodiscard]] std::size_t camera_slot_count() const;

 private:
  struct SmallSnapshotSlot {
    SmallPublishSnapshot snapshot;
    std::uint64_t sequence{0};
  };

  bool initialize_small_snapshot_slot(SmallSnapshotSlot* slot);
  bool copy_lidar_state(const PublishLidarState& source, PublishLidarState* target);
  bool copy_camera_state(std::size_t publisher_index, const mujoco_simulation::CameraState& state,
                         CameraFrame* frame);

  PublishChannelConfig config_;
  std::vector<SmallSnapshotSlot> small_snapshot_slots_;
  std::atomic<std::size_t> active_small_snapshot_slot_{0};
  std::atomic<std::uint64_t> latest_small_snapshot_sequence_{0};
  std::atomic<std::uint64_t> consumed_small_snapshot_sequence_{0};
  std::atomic<std::int64_t> latest_clock_ns_{0};
  std::atomic<std::uint64_t> latest_clock_sequence_{0};
  std::atomic<std::uint64_t> consumed_clock_sequence_{0};
  std::vector<CameraFrame> camera_slots_;
  std::unique_ptr<realtime_tools::LockFreeSPSCQueue<std::size_t>> camera_ready_slots_;
  std::unique_ptr<realtime_tools::LockFreeSPSCQueue<std::size_t>> camera_free_slots_;
};

}  // namespace robot_mujoco_ros2
