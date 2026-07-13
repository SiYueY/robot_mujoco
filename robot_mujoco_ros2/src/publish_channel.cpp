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
    slot->snapshot.lidars[i].state.ranges.resize(range_count, 0.0);
    slot->snapshot.lidars[i].state.intensities.resize(range_count, 0.0);
  }
  slot->sequence = 0;
  return true;
}

bool PublishChannel::copy_lidar_state(const PublishLidarState& source, PublishLidarState* target) {
  if (target == nullptr) {
    return false;
  }
  target->publisher_index = source.publisher_index;
  target->state.sequence = source.state.sequence;
  target->state.timestamp_ns = source.state.timestamp_ns;
  target->state.frame_id = source.state.frame_id;
  target->state.angle_min = source.state.angle_min;
  target->state.angle_max = source.state.angle_max;
  target->state.angle_increment = source.state.angle_increment;
  target->state.time_increment = source.state.time_increment;
  target->state.scan_time = source.state.scan_time;
  target->state.range_min = source.state.range_min;
  target->state.range_max = source.state.range_max;

  if (target->state.ranges.size() < source.state.ranges.size() ||
      target->state.intensities.size() < source.state.ranges.size()) {
    return false;
  }

  std::copy(source.state.ranges.begin(), source.state.ranges.end(), target->state.ranges.begin());
  if (source.state.intensities.empty()) {
    std::fill(
        target->state.intensities.begin(),
        target->state.intensities.begin() + static_cast<std::ptrdiff_t>(source.state.ranges.size()),
        0.0);
  } else {
    if (target->state.intensities.size() < source.state.intensities.size()) {
      return false;
    }
    std::copy(source.state.intensities.begin(), source.state.intensities.end(),
              target->state.intensities.begin());
  }
  return true;
}

bool PublishChannel::copy_camera_state(std::size_t publisher_index,
                                       const mujoco_simulation::CameraState& state,
                                       CameraFrame* frame) {
  if (frame == nullptr || publisher_index >= config_.camera_frames.size()) {
    return false;
  }

  const auto& config = config_.camera_frames[publisher_index];
  frame->publisher_index = publisher_index;
  frame->sequence = state.sequence;
  frame->acquisition_stamp =
      state.timestamp_ns == 0
          ? rclcpp::Time(0, 0, RCL_ROS_TIME)
          : rclcpp::Time(static_cast<int64_t>(state.timestamp_ns), RCL_ROS_TIME);
  frame->camera_info.width = state.camera_info.width;
  frame->camera_info.height = state.camera_info.height;
  frame->camera_info.distortion_model = state.camera_info.distortion_model;
  frame->camera_info.d = state.camera_info.d;
  frame->camera_info.k = state.camera_info.k;
  frame->camera_info.r = state.camera_info.r;
  frame->camera_info.p = state.camera_info.p;
  frame->camera_info.binning_x = state.camera_info.binning_x;
  frame->camera_info.binning_y = state.camera_info.binning_y;
  frame->has_rgb = config.enable_rgb && !state.image.data.empty();
  frame->has_depth = config.enable_depth && !state.depth_image.data.empty();
  frame->width = frame->has_rgb ? state.image.width
                                : (frame->has_depth ? state.depth_image.width : config.width);
  frame->height = frame->has_rgb ? state.image.height
                                 : (frame->has_depth ? state.depth_image.height : config.height);
  frame->rgb_step = frame->has_rgb ? state.image.step : frame->width * 3U;
  frame->depth_step = frame->width * static_cast<std::uint32_t>(sizeof(float));

  if (frame->has_rgb) {
    const std::size_t rgb_bytes = state.image.data.size();
    if (frame->rgb_data.size() < rgb_bytes) {
      return false;
    }
    std::memcpy(frame->rgb_data.data(), state.image.data.data(), rgb_bytes);
  }
  if (frame->has_depth) {
    const std::size_t depth_bytes = state.depth_image.data.size();
    if (frame->depth_data.size() < depth_bytes) {
      return false;
    }
    std::memcpy(frame->depth_data.data(), state.depth_image.data.data(), depth_bytes);
  }
  return true;
}

mujoco_simulation::ResultCode PublishChannel::publish_bundle(const PublishBundle& bundle) {
  if (bundle.imus.size() != config_.imu_count || bundle.lidars.size() != config_.lidar_count ||
      bundle.cameras.size() != config_.camera_frames.size()) {
    return mujoco_simulation::ResultCode::InvalidArgument;
  }

  const std::size_t next_slot = (active_small_snapshot_slot_.load(std::memory_order_relaxed) + 1U) %
                                small_snapshot_slots_.size();
  SmallSnapshotSlot& snapshot_slot = small_snapshot_slots_[next_slot];
  snapshot_slot.snapshot.sim_time = bundle.sim_time;
  for (std::size_t i = 0; i < bundle.imus.size(); ++i) {
    snapshot_slot.snapshot.imus[i] = bundle.imus[i];
  }
  for (std::size_t i = 0; i < bundle.lidars.size(); ++i) {
    if (!copy_lidar_state(bundle.lidars[i], &snapshot_slot.snapshot.lidars[i])) {
      return mujoco_simulation::ResultCode::FailedPrecondition;
    }
  }

  for (const auto& camera : bundle.cameras) {
    if (camera.state == nullptr) {
      continue;
    }
    std::size_t slot_index = 0;
    if (!camera_free_slots_->pop(slot_index)) {
      if (!camera_ready_slots_->pop(slot_index)) {
        return mujoco_simulation::ResultCode::FailedPrecondition;
      }
    }
    CameraFrame& frame = camera_slots_[slot_index];
    if (!copy_camera_state(camera.publisher_index, *camera.state, &frame)) {
      (void)camera_free_slots_->push(slot_index);
      return mujoco_simulation::ResultCode::FailedPrecondition;
    }
    if (!camera_ready_slots_->push(slot_index)) {
      std::size_t discarded_slot = 0;
      if (!camera_ready_slots_->pop(discarded_slot) || !camera_ready_slots_->push(slot_index)) {
        (void)camera_free_slots_->push(slot_index);
        return mujoco_simulation::ResultCode::FailedPrecondition;
      }
      (void)camera_free_slots_->push(discarded_slot);
    }
  }

  latest_clock_ns_.store(bundle.sim_time.nanoseconds(), std::memory_order_release);
  latest_clock_sequence_.fetch_add(1U, std::memory_order_acq_rel);
  snapshot_slot.sequence = latest_small_snapshot_sequence_.load(std::memory_order_relaxed) + 1U;
  active_small_snapshot_slot_.store(next_slot, std::memory_order_release);
  latest_small_snapshot_sequence_.store(snapshot_slot.sequence, std::memory_order_release);
  return mujoco_simulation::ResultCode::Ok;
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
