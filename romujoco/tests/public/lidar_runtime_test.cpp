#include <cmath>
#include <cstring>
#include <iostream>

#include "romujoco/simulation.hpp"
#include "test_support.hpp"

namespace {
bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

float read_float32_le(const std::vector<std::uint8_t>& data, std::size_t offset) {
    const std::uint32_t bits = static_cast<std::uint32_t>(data[offset]) |
                               (static_cast<std::uint32_t>(data[offset + 1U]) << 8U) |
                               (static_cast<std::uint32_t>(data[offset + 2U]) << 16U) |
                               (static_cast<std::uint32_t>(data[offset + 3U]) << 24U);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}
}  // namespace

int main() {
    romujoco_test::TemporaryFile model("lidar_runtime_model.xml");
    romujoco_test::TemporaryFile config("lidar_runtime_config.xml");
    const char* model_xml = R"(<mujoco><worldbody>
      <body name="lidar_body"><site name="lidar_site"/>
        <site name="rotated_site" euler="0 0 90"/><geom type="box" pos=".5 0 0" size=".1 .1 .1"/>
      </body>
      <geom type="box" pos="3 0 0" size=".1 .1 .1" group="1"/>
      <geom type="box" pos="0 4 0" size=".1 .1 .1" group="2"/>
    </worldbody></mujoco>)";
    const char* config_xml = R"(<robot_mujoco><mujoco><mjcf>lidar_runtime_model.xml</mjcf></mujoco>
      <simulation><physics period=".001"/><viewer period=".02" enabled="false"/></simulation>
      <robot><lidar id="2" name="lidar" frame_id="lidar_frame" site="lidar_site"
      output="laser_scan" period=".001" range_min=".1" range_max="10">
      <scan azimuth_start="0" azimuth_increment="1.57079632679489661923" azimuth_samples="2"/>
      <channel elevation="0"/><raycast exclude_parent_body="true"/></lidar>
      <lidar id="3" name="cloud" frame_id="lidar_frame" site="lidar_site"
      output="point_cloud2" period=".001" range_min=".1" range_max="10">
      <scan azimuth_start="0" azimuth_increment="1.57079632679489661923" azimuth_samples="2"/>
      <channel elevation="0"/><channel elevation="1.57079632679489661923"/>
      <raycast exclude_parent_body="true"/></lidar>
      <lidar id="4" name="rotated" frame_id="rotated_frame" site="rotated_site"
      output="laser_scan" period=".001" range_min=".1" range_max="10">
      <scan azimuth_start="0" azimuth_increment="1" azimuth_samples="1"/>
      <channel elevation="0"/><raycast exclude_parent_body="true"/></lidar>
      <lidar id="5" name="filtered" frame_id="lidar_frame" site="lidar_site"
      output="laser_scan" period=".001" range_min=".1" range_max="10">
      <scan azimuth_start="0" azimuth_increment="1.57079632679489661923" azimuth_samples="2"/>
      <channel elevation="0"/><raycast geom_groups="1" exclude_parent_body="true"/></lidar>
      <lidar id="6" name="negative" frame_id="lidar_frame" site="lidar_site" output="laser_scan" period=".001" range_min=".1" range_max="10"><scan azimuth_start="3.14159265358979323846" azimuth_increment="-1.57079632679489661923" azimuth_samples="3"/><channel elevation="0"/><raycast exclude_parent_body="true"/></lidar>
      <lidar id="7" name="offset" frame_id="lidar_frame" site="lidar_site" output="laser_scan" period=".001" range_min=".1" range_max="10"><scan azimuth_start="0" azimuth_increment="1" azimuth_samples="1"/><channel elevation="0" azimuth_offset="1.57079632679489661923"/><raycast exclude_parent_body="true"/></lidar>
      <lidar id="8" name="full" frame_id="lidar_frame" site="lidar_site" output="laser_scan" period=".001" range_min=".1" range_max="10"><scan azimuth_start="0" azimuth_increment="1.57079632679489661923" azimuth_samples="4"/><channel elevation="0"/><raycast exclude_parent_body="true"/></lidar>
      </robot></robot_mujoco>)";
    if (!check(model.write(model_xml), "failed to write model") ||
        !check(config.write(config_xml), "failed to write config"))
        return 1;
    romujoco::Simulation simulation;
    if (!check(
            simulation.initialize(config.path().string()),
            "lidar simulation initialization failed"))
        return 1;
    romujoco::LaserScanState state;
    state.id = 2;
    romujoco::PointCloudState cloud;
    cloud.id = 3;
    romujoco::LaserScanState rotated;
    rotated.id = 4;
    romujoco::LaserScanState filtered;
    filtered.id = 5;
    romujoco::LaserScanState negative;
    negative.id = 6;
    romujoco::LaserScanState offset;
    offset.id = 7;
    romujoco::LaserScanState full;
    full.id = 8;
    const bool passed =
        check(simulation.read_state(state), "laser scan was not published") &&
        check(
            state.sequence == 1U && state.scan.timestamp == 0U,
            "initial scan metadata is incorrect") &&
        check(
            state.scan.ranges.size() == 2U && state.scan.intensities.empty(),
            "LaserScan shape is incorrect") &&
        check(
            std::abs(state.scan.ranges[0] - 2.9F) < 1.0e-4F &&
                std::abs(state.scan.ranges[1] - 3.9F) < 1.0e-4F,
            "raycast distances or parent-body exclusion are incorrect") &&
        check(
            simulation.read_state(rotated) && rotated.scan.ranges.size() == 1U &&
                std::abs(rotated.scan.ranges[0] - 3.9F) < 1.0e-4F,
            "site rotation was not applied to ray directions") &&
        check(
            simulation.read_state(filtered) && filtered.scan.ranges.size() == 2U &&
                std::abs(filtered.scan.ranges[0] - 2.9F) < 1.0e-4F &&
                std::isinf(filtered.scan.ranges[1]),
            "geom group filtering is incorrect") &&
        check(
            simulation.read_state(negative) && negative.scan.ranges.size() == 3U &&
                std::isinf(negative.scan.ranges[0]) &&
                std::abs(negative.scan.ranges[1] - 3.9F) < 1.0e-4F &&
                std::abs(negative.scan.ranges[2] - 2.9F) < 1.0e-4F,
            "negative azimuth increment is incorrect") &&
        check(
            simulation.read_state(offset) && offset.scan.ranges.size() == 1U &&
                std::abs(offset.scan.ranges[0] - 3.9F) < 1.0e-4F,
            "channel azimuth offset is incorrect") &&
        check(
            simulation.read_state(full) && full.scan.ranges.size() == 4U &&
                std::abs(full.scan.angle_max - 4.71238898F) < 1.0e-4F,
            "360-degree scan duplicated its first ray") &&
        check(
            simulation.read_state(cloud) && cloud.sequence == 1U && cloud.cloud.height == 2U &&
                cloud.cloud.width == 2U && cloud.cloud.point_step == 12U &&
                cloud.cloud.row_step == 24U && cloud.cloud.data.size() == 48U &&
                !cloud.cloud.is_dense,
            "organized PointCloud2 metadata is incorrect") &&
        check(
            std::abs(read_float32_le(cloud.cloud.data, 0U) - 2.9F) < 1.0e-4F &&
                std::abs(read_float32_le(cloud.cloud.data, 16U) - 3.9F) < 1.0e-4F &&
                std::isnan(read_float32_le(cloud.cloud.data, 24U)) &&
                std::isnan(read_float32_le(cloud.cloud.data, 36U)),
            "PointCloud2 local XYZ or invalid-point encoding is incorrect") &&
        check(simulation.step(), "simulation step failed") &&
        check(
            simulation.read_state(state) && state.sequence == 2U && simulation.read_state(cloud) &&
                cloud.sequence == 2U && simulation.read_state(rotated) && rotated.sequence == 2U &&
                simulation.read_state(filtered) && filtered.sequence == 2U &&
                simulation.read_state(negative) && negative.sequence == 2U &&
                simulation.read_state(offset) && offset.sequence == 2U &&
                simulation.read_state(full) && full.sequence == 2U,
            "scan sequence did not advance") &&
        check(simulation.shutdown(), "simulation shutdown failed");
    return passed ? 0 : 1;
}
