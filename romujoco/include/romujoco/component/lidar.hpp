#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace romujoco {

using LidarId = std::size_t;

enum class LidarOutput {
    LaserScan,
    PointCloud2,
};

struct LidarChannel {
    double elevation{0.0};
    double azimuth_offset{0.0};
};

struct LidarInfo {
    LidarId id{0};
    std::string name;
    std::string frame_id;
    std::string site_name;
    LidarOutput output{LidarOutput::LaserScan};
    double period{1.0 / 10.0};
    double azimuth_start{0.0};
    double azimuth_increment{0.0};
    std::uint32_t azimuth_samples{0};
    std::vector<LidarChannel> channels;
    double range_min{0.0};
    double range_max{0.0};
    std::uint32_t geom_group_mask{0};  // Zero includes every MuJoCo geom group.
    bool exclude_parent_body{true};
};

struct LaserScan {
    std::uint64_t timestamp{0};
    std::string frame_id;
    float angle_min{0.0F};
    float angle_max{0.0F};
    float angle_increment{0.0F};
    float time_increment{0.0F};
    float scan_time{0.0F};
    float range_min{0.0F};
    float range_max{0.0F};
    std::vector<float> ranges;
    std::vector<float> intensities;
};

struct LaserScanState {
    LidarId id{0};
    std::uint64_t sequence{0};
    LaserScan scan;
};

enum class PointFieldType : std::uint8_t {
    Int8 = 1,
    UInt8 = 2,
    Int16 = 3,
    UInt16 = 4,
    Int32 = 5,
    UInt32 = 6,
    Float32 = 7,
    Float64 = 8,
};

struct PointField {
    std::string name;
    std::uint32_t offset{0};
    PointFieldType datatype{PointFieldType::Float32};
    std::uint32_t count{1};
};

struct PointCloud2 {
    std::uint64_t timestamp{0};
    std::string frame_id;
    std::uint32_t height{0};
    std::uint32_t width{0};
    std::vector<PointField> fields;
    bool is_bigendian{false};
    std::uint32_t point_step{0};
    std::uint32_t row_step{0};
    std::vector<std::uint8_t> data;
    bool is_dense{false};
};

struct PointCloudState {
    LidarId id{0};
    std::uint64_t sequence{0};
    PointCloud2 cloud;
};
}  // namespace romujoco
