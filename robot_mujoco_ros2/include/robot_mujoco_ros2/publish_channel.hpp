#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "mujoco_simulation/component/camera/camera_sample.hpp"
#include "mujoco_simulation/component/imu/imu_sample.hpp"
#include "mujoco_simulation/component/lidar/lidar_sample.hpp"
#include "mujoco_simulation/status.hpp"
#include "rclcpp/time.hpp"
#include "realtime_tools/lock_free_queue.hpp"

namespace robot_mujoco_ros2 {

struct PublishImuSample {
  std::size_t publisher_index{0};
  mujoco_simulation::ImuSample sample;
};

struct PublishLidarSample {
  std::size_t publisher_index{0};
  mujoco_simulation::LidarSample sample;
};

struct PublishCameraSample {
  std::size_t publisher_index{0};
  const mujoco_simulation::CameraSample* sample{nullptr};
};

struct PublishBundle {
  rclcpp::Time sim_time{0, 0, RCL_ROS_TIME};
  std::vector<PublishImuSample> imus;
  std::vector<PublishLidarSample> lidars;
  std::vector<PublishCameraSample> cameras;
};

struct SmallPublishSnapshot {
  rclcpp::Time sim_time{0, 0, RCL_ROS_TIME};
  std::vector<PublishImuSample> imus;
  std::vector<PublishLidarSample> lidars;
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
  mujoco_simulation::CameraIntrinsics intrinsics{};
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
  [[nodiscard]] mujoco_simulation::Status publish_bundle(const PublishBundle& bundle);
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
  bool copy_lidar_sample(const PublishLidarSample& source, PublishLidarSample* target);
  bool copy_camera_sample(std::size_t publisher_index,
                          const mujoco_simulation::CameraSample& sample, CameraFrame* frame);

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
