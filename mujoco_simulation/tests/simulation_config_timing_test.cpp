#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <variant>

#include "mujoco_simulation/config/simulation_config.hpp"

namespace {

bool check(bool condition, const char *message) {
  if (!condition) {
    std::cerr << message << '\n';
  }
  return condition;
}

bool write_file(const std::filesystem::path &path, const char *content) {
  std::ofstream stream(path);
  stream << content;
  return stream.good();
}

} // namespace

int main() {
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
                                     "mujoco_simulation_timing_config_test.xml";
  const auto cleanup = [&path] { std::filesystem::remove(path); };

  const char *valid = R"(
<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation>
    <physics period="0.001"/>
    <viewer period="0.02"/>
  </simulation>
</robot_mujoco>)";
  if (!check(write_file(path, valid), "failed to write valid XML")) {
    cleanup();
    return 1;
  }

  mujoco_simulation::SimulationConfig config;
  mujoco_simulation::SimulationConfigParser parser;
  if (!check(parser.load_file(path.string(), config),
             "valid timing XML was rejected") ||
      !check(std::abs(config.scheduler.physics_period - 0.001) < 1e-12,
             "physics period was not parsed") ||
      !check(std::abs(config.scheduler.viewer_period - 0.02) < 1e-12,
             "viewer period was not parsed")) {
    cleanup();
    return 1;
  }

  const char *invalid = R"(
<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation><physics period="0.001"/></simulation>
</robot_mujoco>)";
  if (!check(write_file(path, invalid), "failed to write invalid XML") ||
      !check(!parser.load_file(path.string(), config),
             "missing viewer period was accepted")) {
    cleanup();
    return 1;
  }

  const char *component_periods = R"(
<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation>
    <physics period="0.001"/>
    <viewer period="0.02"/>
  </simulation>
  <robot>
    <joint id="0" name="joint" period="0.002"/>
    <imu id="1" name="imu" frame_id="imu" framequat_sensor="quat"
         gyro_sensor="gyro" accelerometer_sensor="accel" period="0.003"/>
    <camera id="2" name="camera" frame_id="camera" camera_name="camera"
            optical_frame_id="optical" width="16" height="12" period="0.004"/>
    <lidar id="3" name="lidar" frame_id="lidar" sensor_prefix="lidar"
           angle_min="0" angle_max="1" angle_increment="1" range_min="0"
           range_max="1" period="0.005"/>
    <mobile_base id="4" name="base" base_body="base" wheel_radius="0.1"
                 wheel_base="0.2" track_width="0.2" period="0.006">
      <wheel name="front_left" actuator="front_left"/>
      <wheel name="front_right" actuator="front_right"/>
      <wheel name="rear_left" actuator="rear_left"/>
      <wheel name="rear_right" actuator="rear_right"/>
    </mobile_base>
  </robot>
</robot_mujoco>)";
  if (!check(write_file(path, component_periods),
             "failed to write component-period XML") ||
      !check(parser.load_file(path.string(), config),
             "component periods were rejected") ||
      !check(config.components.size() == 5U,
             "component period XML did not parse every component") ||
      !check(
          std::abs(std::get<mujoco_simulation::JointInfo>(config.components[0])
                       .period -
                   0.002) < 1e-12,
          "joint period was not parsed") ||
      !check(std::abs(std::get<mujoco_simulation::ImuInfo>(config.components[1])
                          .period -
                      0.003) < 1e-12,
             "IMU period was not parsed") ||
      !check(std::abs(
                 std::get<mujoco_simulation::CameraConfig>(config.components[2])
                     .period -
                 0.004) < 1e-12,
             "camera period was not parsed") ||
      !check(
          std::abs(std::get<mujoco_simulation::LidarInfo>(config.components[3])
                       .period -
                   0.005) < 1e-12,
          "lidar period was not parsed") ||
      !check(std::abs(std::get<mujoco_simulation::MobileBaseInfo>(
                          config.components[4])
                          .period -
                      0.006) < 1e-12,
             "mobile-base period was not parsed")) {
    cleanup();
    return 1;
  }

  const char *legacy_rate = R"(
<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation>
    <physics period="0.001"/>
    <viewer period="0.02"/>
  </simulation>
  <robot><joint id="0" name="joint" update_rate="1000"/></robot>
</robot_mujoco>)";
  if (!check(write_file(path, legacy_rate), "failed to write legacy XML") ||
      !check(!parser.load_file(path.string(), config),
             "legacy update_rate attribute was accepted")) {
    cleanup();
    return 1;
  }

  cleanup();
  return 0;
}
