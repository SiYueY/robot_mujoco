#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

#include "config/simulation_config_parser.hpp"
#include "config/simulation_config_validator.hpp"

namespace {
bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

romujoco::LidarInfo valid_lidar() {
    romujoco::LidarInfo lidar;
    lidar.id = 0;
    lidar.name = "lidar";
    lidar.frame_id = "lidar_frame";
    lidar.site_name = "lidar_site";
    lidar.period = 0.1;
    lidar.azimuth_increment = 0.1;
    lidar.azimuth_samples = 10;
    lidar.channels = {{0.0, 0.0}};
    lidar.range_max = 10.0;
    return lidar;
}

bool validates(const romujoco::LidarInfo& lidar) {
    romujoco::SimulationConfig config;
    config.model.model_path = "model.xml";
    config.components.emplace_back(lidar);
    return romujoco::SimulationConfigValidator::validate(config);
}
}  // namespace

int main() {
    const romujoco::LidarInfo valid = valid_lidar();
    if (!check(validates(valid), "valid laser lidar was rejected")) return 1;

    auto invalid = valid;
    invalid.site_name.clear();
    if (!check(!validates(invalid), "empty site was accepted")) return 1;
    invalid = valid;
    invalid.azimuth_increment = 0.0;
    if (!check(!validates(invalid), "zero azimuth increment was accepted")) return 1;
    invalid = valid;
    invalid.azimuth_samples = 0;
    if (!check(!validates(invalid), "zero azimuth samples was accepted")) return 1;
    invalid = valid;
    invalid.channels.clear();
    if (!check(!validates(invalid), "empty channels were accepted")) return 1;
    invalid = valid;
    invalid.channels = {{std::numeric_limits<double>::infinity(), 0.0}};
    if (!check(!validates(invalid), "non-finite elevation was accepted")) return 1;
    invalid = valid;
    invalid.channels = {{2.0, 0.0}};
    if (!check(!validates(invalid), "out-of-range elevation was accepted")) return 1;
    invalid = valid;
    invalid.channels.push_back({0.0, 0.0});
    if (!check(!validates(invalid), "multi-channel laser scan was accepted")) return 1;
    invalid = valid;
    invalid.channels = {{0.1, 0.0}};
    if (!check(!validates(invalid), "non-planar laser scan was accepted")) return 1;
    invalid = valid;
    invalid.geom_group_mask = 1U << 6U;
    if (!check(!validates(invalid), "unsupported geom group was accepted")) return 1;
    invalid = valid;
    invalid.output = static_cast<romujoco::LidarOutput>(99);
    if (!check(!validates(invalid), "invalid output enum was accepted")) return 1;
    invalid = valid;
    invalid.output = romujoco::LidarOutput::PointCloud2;
    invalid.azimuth_samples = std::numeric_limits<std::uint32_t>::max() / 12U + 1U;
    if (!check(!validates(invalid), "overflowing PointCloud2 row step was accepted")) return 1;

    const auto path = std::filesystem::temp_directory_path() / "romujoco_lidar_config_test.xml";
    const char* xml =
        R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><lidar id="0" name="lidar" frame_id="lidar_frame" site="lidar_site" output="point_cloud2" period=".1" range_min="0" range_max="10"><scan azimuth_start="0" azimuth_increment=".1" azimuth_samples="2"/><channel elevation="0"/><raycast geom_groups="1,3" exclude_parent_body="false"/></lidar></robot></robot_mujoco>)";
    std::ofstream stream(path);
    stream << xml;
    stream.close();
    romujoco::SimulationConfig parsed;
    romujoco::SimulationConfigParser parser;
    const bool parsed_ok = parser.load_file(path.string(), parsed);
    std::filesystem::remove(path);
    if (!check(parsed_ok, "raycast child XML was rejected")) return 1;
    const auto& lidar = std::get<romujoco::LidarInfo>(parsed.components.front());
    return check(
               lidar.geom_group_mask == ((1U << 1U) | (1U << 3U)) && !lidar.exclude_parent_body,
               "raycast XML was parsed incorrectly")
               ? 0
               : 1;
}
