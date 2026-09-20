#include "component/lidar/lidar_component.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include "common/macro.hpp"
#include "log/logging.hpp"

namespace romujoco {
namespace {
constexpr std::uint32_t kPointStep = 12U;
constexpr std::uint32_t kFloat32Size = 4U;

std::uint64_t timestamp_ns(double simulation_time) {
    constexpr double kNanosecondsPerSecond = 1.0e9;
    const double timestamp = simulation_time * kNanosecondsPerSecond;
    if (!std::isfinite(timestamp) || timestamp <= 0.0) return 0U;
    if (timestamp >= static_cast<double>(std::numeric_limits<std::uint64_t>::max()))
        return std::numeric_limits<std::uint64_t>::max();
    return static_cast<std::uint64_t>(timestamp);
}

void write_float32_le(std::vector<std::uint8_t>& data, std::size_t offset, float value) {
    static_assert(sizeof(float) == kFloat32Size, "PointCloud2 requires float32");
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    data[offset] = static_cast<std::uint8_t>(bits & 0xFFU);
    data[offset + 1U] = static_cast<std::uint8_t>((bits >> 8U) & 0xFFU);
    data[offset + 2U] = static_cast<std::uint8_t>((bits >> 16U) & 0xFFU);
    data[offset + 3U] = static_cast<std::uint8_t>((bits >> 24U) & 0xFFU);
}
}  // namespace

LidarComponent::LidarComponent(LidarInfo info)
: SimulationComponent(info.name, info.period), info_(std::move(info)) {}

bool LidarComponent::init(const SimulationContext& context) {
    initialized_ = false;
    local_directions_.clear();
    world_directions_.clear();
    distances_.clear();
    geom_groups_.clear();
    laser_scan_state_.reset();
    point_cloud_state_.reset();
    if (!configure(context)) return false;
    const mjModel& model = *context.model;
    site_id_ = mj_name2id(&model, mjOBJ_SITE, info_.site_name.c_str());
    if (site_id_ < 0) {
        SIM_ERROR << "lidar '" << info_.name << "' site '" << info_.site_name
                  << "' was not found in the MuJoCo model.";
        return false;
    }
    body_id_ = model.site_bodyid[site_id_];
    const std::size_t ray_count =
        info_.channels.size() * static_cast<std::size_t>(info_.azimuth_samples);
    local_directions_.resize(ray_count * 3U);
    world_directions_.resize(ray_count * 3U);
    distances_.resize(ray_count);
    for (std::size_t channel_index = 0; channel_index < info_.channels.size(); ++channel_index) {
        const LidarChannel& channel = info_.channels[channel_index];
        const double cos_elevation = std::cos(channel.elevation);
        for (std::uint32_t azimuth_index = 0; azimuth_index < info_.azimuth_samples;
             ++azimuth_index) {
            const std::size_t index = channel_index * info_.azimuth_samples + azimuth_index;
            const double azimuth = info_.azimuth_start +
                                   static_cast<double>(azimuth_index) * info_.azimuth_increment +
                                   channel.azimuth_offset;
            local_directions_[index * 3U] = cos_elevation * std::cos(azimuth);
            local_directions_[index * 3U + 1U] = cos_elevation * std::sin(azimuth);
            local_directions_[index * 3U + 2U] = std::sin(channel.elevation);
        }
    }
    if (info_.geom_group_mask != 0U) {
        geom_groups_.resize(mjNGROUP);
        for (int group = 0; group < mjNGROUP; ++group)
            geom_groups_[static_cast<std::size_t>(group)] =
                (info_.geom_group_mask & (1U << static_cast<unsigned>(group))) != 0U;
    }
    sequence_ = 0;
    initialized_ = true;
    return reset(context);
}

bool LidarComponent::reset(const SimulationContext& context) {
    UNUSED(context);
    if (!initialized_) return false;
    sequence_ = 0;
    laser_scan_state_.reset();
    point_cloud_state_.reset();
    return true;
}
bool LidarComponent::advance(const SimulationContext& context) {
    UNUSED(context);
    return true;
}

bool LidarComponent::update(const SimulationContext& context) {
    if (!initialized_) {
        SIM_ERROR << "lidar '" << info_.name << "' is not initialized.";
        return false;
    }
    const std::size_t ray_count = distances_.size();
    for (std::size_t index = 0; index < ray_count; ++index)
        mju_mulMatVec3(
            world_directions_.data() + index * 3U, context.data->site_xmat + site_id_ * 9,
            local_directions_.data() + index * 3U);
    mj_multiRay(
        context.model, context.data, context.data->site_xpos + site_id_ * 3,
        world_directions_.data(), geom_groups_.empty() ? nullptr : geom_groups_.data(), 1,
        info_.exclude_parent_body ? body_id_ : -1, nullptr, distances_.data(), nullptr,
        static_cast<int>(ray_count), info_.range_max);
    const std::uint64_t timestamp = timestamp_ns(context.data->time);
    if (info_.output == LidarOutput::LaserScan) {
        auto state = std::make_shared<LaserScanState>();
        state->id = info_.id;
        state->sequence = ++sequence_;
        LaserScan& scan = state->scan;
        scan.timestamp = timestamp;
        scan.frame_id = info_.frame_id;
        scan.angle_min =
            static_cast<float>(info_.azimuth_start + info_.channels.front().azimuth_offset);
        scan.angle_increment = static_cast<float>(info_.azimuth_increment);
        scan.angle_max =
            scan.angle_min + static_cast<float>(info_.azimuth_samples - 1U) * scan.angle_increment;
        scan.time_increment = 0.0F;
        scan.scan_time = static_cast<float>(info_.period);
        scan.range_min = static_cast<float>(info_.range_min);
        scan.range_max = static_cast<float>(info_.range_max);
        scan.ranges.resize(ray_count);
        for (std::size_t index = 0; index < ray_count; ++index) {
            const mjtNum distance = distances_[index];
            scan.ranges[index] =
                !std::isfinite(distance) || distance < info_.range_min || distance > info_.range_max
                    ? std::numeric_limits<float>::infinity()
                    : static_cast<float>(distance);
        }
        scan.intensities.clear();
        laser_scan_state_ = std::move(state);
        point_cloud_state_.reset();
        return true;
    }
    auto state = std::make_shared<PointCloudState>();
    state->id = info_.id;
    state->sequence = ++sequence_;
    PointCloud2& cloud = state->cloud;
    cloud.timestamp = timestamp;
    cloud.frame_id = info_.frame_id;
    cloud.height = static_cast<std::uint32_t>(info_.channels.size());
    cloud.width = info_.azimuth_samples;
    cloud.fields = {
        {"x", 0U, PointFieldType::Float32, 1U},
        {"y", 4U, PointFieldType::Float32, 1U},
        {"z", 8U, PointFieldType::Float32, 1U}};
    cloud.is_bigendian = false;
    cloud.point_step = kPointStep;
    cloud.row_step = static_cast<std::uint32_t>(
        static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.point_step));
    cloud.data.resize(ray_count * static_cast<std::size_t>(cloud.point_step));
    cloud.is_dense = true;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (std::size_t index = 0; index < ray_count; ++index) {
        const mjtNum distance = distances_[index];
        const bool valid =
            std::isfinite(distance) && distance >= info_.range_min && distance <= info_.range_max;
        const std::size_t offset = index * kPointStep;
        if (!valid) cloud.is_dense = false;
        write_float32_le(
            cloud.data, offset,
            valid ? static_cast<float>(distance * local_directions_[index * 3U]) : nan);
        write_float32_le(
            cloud.data, offset + 4U,
            valid ? static_cast<float>(distance * local_directions_[index * 3U + 1U]) : nan);
        write_float32_le(
            cloud.data, offset + 8U,
            valid ? static_cast<float>(distance * local_directions_[index * 3U + 2U]) : nan);
    }
    point_cloud_state_ = std::move(state);
    laser_scan_state_.reset();
    return true;
}

bool LidarComponent::read_laser_scan_state(std::shared_ptr<const LaserScanState>& state) const {
    state = laser_scan_state_;
    return initialized_ && state != nullptr;
}
bool LidarComponent::read_point_cloud_state(std::shared_ptr<const PointCloudState>& state) const {
    state = point_cloud_state_;
    return initialized_ && state != nullptr;
}
}  // namespace romujoco
