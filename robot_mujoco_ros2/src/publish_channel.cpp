#include "robot_mujoco_ros2/publish_channel.hpp"

#include <algorithm>
#include <cstring>
#include <utility>

namespace robot_mujoco_ros2 {
namespace {

constexpr std::size_t kSnapshotSlotCount = 2;

std::size_t default_camera_queue_capacity(std::size_t camera_count) {
  return std::max<std::size_t>(camera_count * 4U, 4U);
}

}  // namespace

PublishChannel::PublishChannel()
    : camera_ready_slots_(std::make_unique<realtime_tools::LockFreeSPSCQueue<std::size_t>>(1U)),
      camera_free_slots_(std::make_unique<realtime_tools::LockFreeSPSCQueue<std::size_t>>(1U)) {}

PublishChannel::PublishChannel(PublishChannelConfig config) : PublishChannel() {
  reset(std::move(config));
}

void PublishChannel::reset(PublishChannelConfig config) {
  config_ = std::move(config);
  if (config_.camera_queue_capacity == 0) {
    config_.camera_queue_capacity = default_camera_queue_capacity(config_.camera_frames.size());
  }

  small_snapshot_slots_.assign(kSnapshotSlotCount, {});
  for (auto& slot : small_snapshot_slots_) {
    initialize_small_snapshot_slot(&slot);
  }
  active_small_snapshot_slot_.store(0);
  latest_small_snapshot_sequence_.store(0);
  consumed_small_snapshot_sequence_.store(0);
  latest_clock_ns_.store(0);
  latest_clock_sequence_.store(0);
  consumed_clock_sequence_.store(0);

  const std::size_t slot_count = std::max<std::size_t>(config_.camera_queue_capacity, 1U);
  camera_slots_.clear();
  camera_slots_.resize(slot_count);
  for (auto& slot : camera_slots_) {
    slot.publisher_index = 0;
  }

  camera_ready_slots_ =
      std::make_unique<realtime_tools::LockFreeSPSCQueue<std::size_t>>(slot_count + 1U);
  camera_free_slots_ =
      std::make_unique<realtime_tools::LockFreeSPSCQueue<std::size_t>>(slot_count + 1U);
  for (std::size_t i = 0; i < camera_slots_.size(); ++i) {
    (void)camera_free_slots_->push(i);
  }

  for (auto& slot : camera_slots_) {
    for (const auto& camera : config_.camera_frames) {
      const std::size_t rgb_bytes =
          camera.enable_rgb ? static_cast<std::size_t>(camera.width) * camera.height * 3U : 0U;
      const std::size_t depth_bytes = camera.enable_depth ? static_cast<std::size_t>(camera.width) *
                                                                camera.height * sizeof(float)
                                                          : 0U;
      slot.rgb_data.resize(std::max(slot.rgb_data.size(), rgb_bytes));
      slot.depth_data.resize(std::max(slot.depth_data.size(), depth_bytes));
    }
  }
}

bool PublishChannel::initialize_small_snapshot_slot(SmallSnapshotSlot* slot) {
  if (slot == nullptr) {
    return false;
  }
  slot->snapshot.sim_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  slot->snapshot.imus.resize(config_.imu_count);
  slot->snapshot.lidars.resize(config_.lidar_count);
  for (std::size_t i = 0; i < slot->snapshot.imus.size(); ++i) {
    slot->snapshot.imus[i].publisher_index = i;
  }
  for (std::size_t i = 0; i < slot->snapshot.lidars.size(); ++i) {
    slot->snapshot.lidars[i].publisher_index = i;
    const std::size_t range_count =
        i < config_.lidar_sample_counts.size() ? config_.lidar_sample_counts[i] : 0U;
    slot->snapshot.lidars[i].sample.ranges.resize(range_count, 0.0);
    slot->snapshot.lidars[i].sample.intensities.resize(range_count, 0.0);
  }
  slot->sequence = 0;
  return true;
}

bool PublishChannel::copy_lidar_sample(const PublishLidarSample& source,
                                       PublishLidarSample* target) {
  if (target == nullptr) {
    return false;
  }
  target->publisher_index = source.publisher_index;
  target->sample.sequence = source.sample.sequence;
  target->sample.timestamp_ns = source.sample.timestamp_ns;
  target->sample.frame_id = source.sample.frame_id;
  target->sample.angle_min = source.sample.angle_min;
  target->sample.angle_max = source.sample.angle_max;
  target->sample.angle_increment = source.sample.angle_increment;
  target->sample.time_increment = source.sample.time_increment;
  target->sample.scan_time = source.sample.scan_time;
  target->sample.range_min = source.sample.range_min;
  target->sample.range_max = source.sample.range_max;

  if (target->sample.ranges.size() < source.sample.ranges.size() ||
      target->sample.intensities.size() < source.sample.ranges.size()) {
    return false;
  }

  std::copy(source.sample.ranges.begin(), source.sample.ranges.end(),
            target->sample.ranges.begin());
  if (source.sample.intensities.empty()) {
    std::fill(target->sample.intensities.begin(),
              target->sample.intensities.begin() +
                  static_cast<std::ptrdiff_t>(source.sample.ranges.size()),
              0.0);
  } else {
    if (target->sample.intensities.size() < source.sample.intensities.size()) {
      return false;
    }
    std::copy(source.sample.intensities.begin(), source.sample.intensities.end(),
              target->sample.intensities.begin());
  }
  return true;
}

bool PublishChannel::copy_camera_sample(std::size_t publisher_index,
                                        const mujoco_simulation::CameraSample& sample,
                                        CameraFrame* frame) {
  if (frame == nullptr || publisher_index >= config_.camera_frames.size()) {
    return false;
  }

  const auto& config = config_.camera_frames[publisher_index];
  frame->publisher_index = publisher_index;
  frame->sequence = sample.sequence;
  frame->acquisition_stamp =
      sample.timestamp_ns == 0
          ? rclcpp::Time(0, 0, RCL_ROS_TIME)
          : rclcpp::Time(static_cast<int64_t>(sample.timestamp_ns), RCL_ROS_TIME);
  frame->intrinsics = sample.intrinsics;
  frame->has_rgb = config.enable_rgb && sample.color.has_value();
  frame->has_depth = config.enable_depth && sample.depth.has_value();
  frame->width = frame->has_rgb ? sample.color->width
                                : (frame->has_depth ? sample.depth->width : config.width);
  frame->height = frame->has_rgb ? sample.color->height
                                 : (frame->has_depth ? sample.depth->height : config.height);
  frame->rgb_step = frame->has_rgb ? sample.color->step : frame->width * 3U;
  frame->depth_step = frame->width * static_cast<std::uint32_t>(sizeof(float));

  if (frame->has_rgb) {
    const std::size_t rgb_bytes = sample.color->data.size();
    if (frame->rgb_data.size() < rgb_bytes) {
      return false;
    }
    std::memcpy(frame->rgb_data.data(), sample.color->data.data(), rgb_bytes);
  }
  if (frame->has_depth) {
    const std::size_t depth_bytes = sample.depth->data.size() * sizeof(float);
    if (frame->depth_data.size() < depth_bytes) {
      return false;
    }
    std::memcpy(frame->depth_data.data(), sample.depth->data.data(), depth_bytes);
  }
  return true;
}

mujoco_simulation::Status PublishChannel::publish_bundle(const PublishBundle& bundle) {
  if (bundle.imus.size() != config_.imu_count || bundle.lidars.size() != config_.lidar_count ||
      bundle.cameras.size() != config_.camera_frames.size()) {
    return mujoco_simulation::Status::invalid_argument(
        "Publish bundle does not match configured sensor counts.");
  }

  const std::size_t next_slot = (active_small_snapshot_slot_.load(std::memory_order_relaxed) + 1U) %
                                small_snapshot_slots_.size();
  SmallSnapshotSlot& snapshot_slot = small_snapshot_slots_[next_slot];
  snapshot_slot.snapshot.sim_time = bundle.sim_time;
  for (std::size_t i = 0; i < bundle.imus.size(); ++i) {
    snapshot_slot.snapshot.imus[i] = bundle.imus[i];
  }
  for (std::size_t i = 0; i < bundle.lidars.size(); ++i) {
    if (!copy_lidar_sample(bundle.lidars[i], &snapshot_slot.snapshot.lidars[i])) {
      return mujoco_simulation::Status::failed_precondition(
          "Lidar publish channel buffer is smaller than the incoming sample.");
    }
  }

  for (const auto& camera : bundle.cameras) {
    if (camera.sample == nullptr) {
      continue;
    }
    std::size_t slot_index = 0;
    if (!camera_free_slots_->pop(slot_index)) {
      if (!camera_ready_slots_->pop(slot_index)) {
        return mujoco_simulation::Status::failed_precondition(
            "Camera publish channel could not reclaim a frame slot.");
      }
    }
    CameraFrame& frame = camera_slots_[slot_index];
    if (!copy_camera_sample(camera.publisher_index, *camera.sample, &frame)) {
      (void)camera_free_slots_->push(slot_index);
      return mujoco_simulation::Status::failed_precondition(
          "Camera publish channel buffer is smaller than the incoming sample.");
    }
    if (!camera_ready_slots_->push(slot_index)) {
      std::size_t discarded_slot = 0;
      if (!camera_ready_slots_->pop(discarded_slot) || !camera_ready_slots_->push(slot_index)) {
        (void)camera_free_slots_->push(slot_index);
        return mujoco_simulation::Status::failed_precondition(
            "Camera publish channel queue push failed.");
      }
      (void)camera_free_slots_->push(discarded_slot);
    }
  }

  latest_clock_ns_.store(bundle.sim_time.nanoseconds(), std::memory_order_release);
  latest_clock_sequence_.fetch_add(1U, std::memory_order_acq_rel);
  snapshot_slot.sequence = latest_small_snapshot_sequence_.load(std::memory_order_relaxed) + 1U;
  active_small_snapshot_slot_.store(next_slot, std::memory_order_release);
  latest_small_snapshot_sequence_.store(snapshot_slot.sequence, std::memory_order_release);
  return mujoco_simulation::Status::Ok();
}

void PublishChannel::update_clock(const rclcpp::Time& sim_time) {
  latest_clock_ns_.store(sim_time.nanoseconds(), std::memory_order_release);
  latest_clock_sequence_.fetch_add(1U, std::memory_order_acq_rel);
}

bool PublishChannel::consume_latest_small_snapshot(SmallPublishSnapshot* snapshot) {
  if (snapshot == nullptr) {
    return false;
  }
  const std::uint64_t latest = latest_small_snapshot_sequence_.load(std::memory_order_acquire);
  const std::uint64_t consumed = consumed_small_snapshot_sequence_.load(std::memory_order_acquire);
  if (latest == 0 || latest == consumed) {
    return false;
  }
  const std::size_t slot = active_small_snapshot_slot_.load(std::memory_order_acquire);
  *snapshot = small_snapshot_slots_[slot].snapshot;
  consumed_small_snapshot_sequence_.store(latest, std::memory_order_release);
  return true;
}

bool PublishChannel::consume_latest_clock(rclcpp::Time* sim_time) {
  if (sim_time == nullptr) {
    return false;
  }
  const std::uint64_t latest = latest_clock_sequence_.load(std::memory_order_acquire);
  const std::uint64_t consumed = consumed_clock_sequence_.load(std::memory_order_acquire);
  if (latest == 0 || latest == consumed) {
    return false;
  }
  *sim_time = rclcpp::Time(latest_clock_ns_.load(std::memory_order_acquire), RCL_ROS_TIME);
  consumed_clock_sequence_.store(latest, std::memory_order_release);
  return true;
}

bool PublishChannel::pop_camera_frame(CameraFrame* frame) {
  if (frame == nullptr) {
    return false;
  }
  std::size_t slot_index = 0;
  if (!camera_ready_slots_->pop(slot_index)) {
    return false;
  }
  *frame = camera_slots_[slot_index];
  (void)camera_free_slots_->push(slot_index);
  return true;
}

std::size_t PublishChannel::camera_queue_capacity() const { return config_.camera_queue_capacity; }

std::size_t PublishChannel::camera_slot_count() const { return camera_slots_.size(); }

}  // namespace robot_mujoco_ros2
