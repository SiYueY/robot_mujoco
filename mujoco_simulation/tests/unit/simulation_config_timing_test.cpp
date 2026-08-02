#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <variant>

#include "config/simulation_config_parser.hpp"
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
    <viewer period="0.02" enabled="false"/>
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
             "viewer period was not parsed") ||
      !check(!config.viewer_enabled, "viewer enabled was not parsed")) {
    cleanup();
    return 1;
  }

  const char *invalid = R"(
<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation><physics period="0.001"/></simulation>
</robot_mujoco>)";
  mujoco_simulation::ConfigError timing_error;
  if (!check(write_file(path, invalid), "failed to write invalid XML") ||
      !check(!parser.load_file(path.string(), config, &timing_error),
             "missing viewer period was accepted") ||
      !check(timing_error.element == "simulation" &&
                 timing_error.attribute == "viewer" &&
                 !timing_error.message.empty(),
             "missing viewer period had no precise diagnostic")) {
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

  const char *invalid_viewer_enabled = R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02" enabled="maybe"/>
</simulation></robot_mujoco>)";
  mujoco_simulation::ConfigError invalid_viewer_error;
  if (!check(write_file(path, invalid_viewer_enabled),
             "failed to write invalid viewer XML") ||
      !check(!parser.load_file(path.string(), config, &invalid_viewer_error) &&
                 invalid_viewer_error.element == "viewer" &&
                 invalid_viewer_error.attribute == "enabled",
             "invalid viewer enabled attribute was accepted")) {
    cleanup();
    return 1;
  }

  struct InvalidConfig {
    const char *name;
    const char *content;
  };
  const std::array<InvalidConfig, 10> invalid_configs = {{
      {"trailing joint numeric text", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><position><stiffness>100abc</stiffness></position></joint>
</robot></robot_mujoco>)"},
      {"infinite joint numeric text", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><limit><effort><max>inf</max></effort></limit></joint>
</robot></robot_mujoco>)"},
      {"negative stiffness", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><position><stiffness>-1</stiffness></position></joint>
</robot></robot_mujoco>)"},
      {"negative damping", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><velocity><damping>-1</damping></velocity></joint>
</robot></robot_mujoco>)"},
      {"signed maximum component id", R"(
<robot_mujoco max_component_id="+1"><mujoco><mjcf>model.xml</mjcf></mujoco>
<simulation><physics period="0.001"/><viewer period="0.02"/></simulation></robot_mujoco>)"},
      {"negative maximum component id", R"(
<robot_mujoco max_component_id="-1"><mujoco><mjcf>model.xml</mjcf></mujoco>
<simulation><physics period="0.001"/><viewer period="0.02"/></simulation></robot_mujoco>)"},
      {"oversized maximum component id", R"(
<robot_mujoco max_component_id="65536"><mujoco><mjcf>model.xml</mjcf></mujoco>
<simulation><physics period="0.001"/><viewer period="0.02"/></simulation></robot_mujoco>)"},
      {"oversized component id", R"(
<robot_mujoco max_component_id="65535"><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="65536" name="joint"/></robot></robot_mujoco>)"},
      {"invalid camera rgb bool", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<camera id="0" name="camera" frame_id="camera" camera_name="camera" optical_frame_id="optical"
width="16" height="12" enable_rgb="abc"/></robot></robot_mujoco>)"},
      {"invalid camera depth bool", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<camera id="0" name="camera" frame_id="camera" camera_name="camera" optical_frame_id="optical"
width="16" height="12" enable_depth="abc"/></robot></robot_mujoco>)"},
  }};
  for (const InvalidConfig &invalid_config : invalid_configs) {
    if (!check(write_file(path, invalid_config.content), invalid_config.name) ||
        !check(!parser.load_file(path.string(), config), invalid_config.name)) {
      cleanup();
      return 1;
    }
  }

  const std::array<InvalidConfig, 11> semantic_invalid_configs = {{
      {"zero camera width",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><camera id="0" name="c" frame_id="f" camera_name="c" optical_frame_id="o" width="0" height="1"/></robot></robot_mujoco>)"},
      {"negative camera height",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><camera id="0" name="c" frame_id="f" camera_name="c" optical_frame_id="o" width="1" height="-1"/></robot></robot_mujoco>)"},
      {"oversized camera width",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><camera id="0" name="c" frame_id="f" camera_name="c" optical_frame_id="o" width="8193" height="1"/></robot></robot_mujoco>)"},
      {"camera output too large",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><camera id="0" name="c" frame_id="f" camera_name="c" optical_frame_id="o" width="8192" height="8192" enable_rgb="true" enable_depth="true"/></robot></robot_mujoco>)"},
      {"zero lidar increment",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><lidar id="0" name="l" frame_id="f" sensor_prefix="s" angle_min="0" angle_max="1" angle_increment="0" range_min="0" range_max="1"/></robot></robot_mujoco>)"},
      {"reversed lidar angle",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><lidar id="0" name="l" frame_id="f" sensor_prefix="s" angle_min="1" angle_max="0" angle_increment="1" range_min="0" range_max="1"/></robot></robot_mujoco>)"},
      {"negative lidar range",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><lidar id="0" name="l" frame_id="f" sensor_prefix="s" angle_min="0" angle_max="1" angle_increment="1" range_min="-1" range_max="1"/></robot></robot_mujoco>)"},
      {"equal lidar range",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><lidar id="0" name="l" frame_id="f" sensor_prefix="s" angle_min="0" angle_max="1" angle_increment="1" range_min="1" range_max="1"/></robot></robot_mujoco>)"},
      {"non-positive base geometry",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><mobile_base id="0" name="b" base_body="b" wheel_radius="0" wheel_base="1" track_width="1"><wheel name="a" actuator="a"/><wheel name="b" actuator="b"/><wheel name="c" actuator="c"/><wheel name="d" actuator="d"/></mobile_base></robot></robot_mujoco>)"},
      {"duplicate wheel name",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><mobile_base id="0" name="b" base_body="b" wheel_radius="1" wheel_base="1" track_width="1"><wheel name="a" actuator="a"/><wheel name="a" actuator="b"/><wheel name="c" actuator="c"/><wheel name="d" actuator="d"/></mobile_base></robot></robot_mujoco>)"},
      {"duplicate wheel actuator",
       R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><mobile_base id="0" name="b" base_body="b" wheel_radius="1" wheel_base="1" track_width="1"><wheel name="a" actuator="x"/><wheel name="b" actuator="x"/><wheel name="c" actuator="c"/><wheel name="d" actuator="d"/></mobile_base></robot></robot_mujoco>)"},
  }};
  for (const InvalidConfig &invalid_config : semantic_invalid_configs) {
    mujoco_simulation::ConfigError error;
    if (!check(write_file(path, invalid_config.content), invalid_config.name) ||
        !check(!parser.load_file(path.string(), config, &error),
               invalid_config.name) ||
        !check(
            error.line > 0 && !error.element.empty() &&
                !error.attribute.empty() && !error.message.empty(),
            "semantic parse failure did not provide a complete diagnostic")) {
      cleanup();
      return 1;
    }
  }

  const char *camera_boundary =
      R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><camera id="0" name="c" frame_id="f" camera_name="c" optical_frame_id="o" width="8192" height="8192" enable_rgb="true" enable_depth="false"/></robot></robot_mujoco>)";
  if (!check(write_file(path, camera_boundary),
             "failed to write camera boundary XML") ||
      !check(parser.load_file(path.string(), config),
             "valid maximum-dimension RGB camera was rejected")) {
    cleanup();
    return 1;
  }

  const char *duplicate_joint = R"(<robot_mujoco>
<mujoco><mjcf>model.xml</mjcf></mujoco>
<simulation><physics period=".001"/><viewer period=".02"/></simulation>
<robot>
  <joint id="0" name="first"/>
  <joint id="0" name="second"/>
</robot>
</robot_mujoco>)";
  mujoco_simulation::ConfigError duplicate_error;
  if (!check(write_file(path, duplicate_joint),
             "failed to write duplicate XML") ||
      !check(!parser.load_file(path.string(), config, &duplicate_error) &&
                 duplicate_error.line == 6 &&
                 duplicate_error.element == "joint" &&
                 duplicate_error.attribute == "id",
             "duplicate component did not retain its XML source location")) {
    cleanup();
    return 1;
  }

  mujoco_simulation::SimulationConfig direct_config;
  direct_config.model.model_path = "model.xml";
  mujoco_simulation::CameraConfig invalid_camera;
  invalid_camera.width = 0;
  invalid_camera.height = 1;
  direct_config.components.emplace_back(invalid_camera);
  mujoco_simulation::ConfigError validation_error;
  if (!check(!mujoco_simulation::SimulationConfigValidator::validate(
                 direct_config, &validation_error) &&
                 validation_error.attribute == "width",
             "validator accepted an invalid direct C++ camera configuration")) {
    cleanup();
    return 1;
  }

  mujoco_simulation::SimulationConfig invalid_period_config;
  invalid_period_config.model.model_path = "model.xml";
  mujoco_simulation::JointInfo invalid_period_joint;
  invalid_period_joint.id = 0;
  invalid_period_joint.joint_name = "joint";
  invalid_period_joint.actuator_name = "actuator";
  invalid_period_joint.period = -0.001;
  invalid_period_config.components.emplace_back(invalid_period_joint);
  if (!check(!mujoco_simulation::SimulationConfigValidator::validate(
                 invalid_period_config, &validation_error) &&
                 validation_error.attribute == "period",
             "validator accepted a negative direct C++ component period")) {
    cleanup();
    return 1;
  }

  mujoco_simulation::SimulationConfig duplicate_config;
  duplicate_config.model.model_path = "model.xml";
  invalid_period_joint.period = 0.0;
  duplicate_config.components.emplace_back(invalid_period_joint);
  duplicate_config.components.emplace_back(invalid_period_joint);
  if (!check(!mujoco_simulation::SimulationConfigValidator::validate(
                 duplicate_config, &validation_error) &&
                 validation_error.attribute == "id",
             "validator accepted duplicate direct C++ component IDs")) {
    cleanup();
    return 1;
  }

  mujoco_simulation::SimulationConfig missing_model_config;
  if (!check(!mujoco_simulation::SimulationConfigValidator::validate(
                 missing_model_config, &validation_error) &&
                 validation_error.attribute == "model_path",
             "validator accepted an empty direct C++ model path")) {
    cleanup();
    return 1;
  }

  mujoco_simulation::SimulationConfig missing_camera_name_config;
  missing_camera_name_config.model.model_path = "model.xml";
  mujoco_simulation::CameraConfig missing_camera_name;
  missing_camera_name.id = 0;
  missing_camera_name.name = "camera";
  missing_camera_name.width = 1;
  missing_camera_name.height = 1;
  missing_camera_name.frame_id.clear();
  missing_camera_name.camera_name = "camera";
  missing_camera_name.optical_frame_id = "camera_optical";
  missing_camera_name_config.components.emplace_back(missing_camera_name);
  if (!check(!mujoco_simulation::SimulationConfigValidator::validate(
                 missing_camera_name_config, &validation_error) &&
                 validation_error.attribute == "frame_id" &&
                 !validation_error.message.empty(),
             "validator omitted a camera name diagnostic")) {
    cleanup();
    return 1;
  }

  mujoco_simulation::SimulationConfig invalid_renderer_config;
  invalid_renderer_config.model.model_path = "model.xml";
  invalid_renderer_config.camera_renderer.completed_ticket_history = 0U;
  if (!check(!mujoco_simulation::SimulationConfigValidator::validate(
                 invalid_renderer_config, &validation_error) &&
                 validation_error.attribute == "completed_ticket_history",
             "validator accepted zero camera ticket history")) {
    cleanup();
    return 1;
  }

  mujoco_simulation::SimulationConfig no_backend_config;
  no_backend_config.model.model_path = "model.xml";
  no_backend_config.camera_renderer.allow_glfw_backend = false;
  no_backend_config.camera_renderer.allow_egl_backend = false;
  if (!check(!mujoco_simulation::SimulationConfigValidator::validate(
                 no_backend_config, &validation_error) &&
                 validation_error.attribute == "camera_renderer",
             "validator accepted a camera renderer without a backend")) {
    cleanup();
    return 1;
  }

  mujoco_simulation::SimulationConfig missing_lidar_name_config;
  missing_lidar_name_config.model.model_path = "model.xml";
  mujoco_simulation::LidarInfo missing_lidar_name;
  missing_lidar_name.id = 0;
  missing_lidar_name.name = "lidar";
  missing_lidar_name.sensor_prefix = "scan";
  missing_lidar_name.angle_max = 1.0;
  missing_lidar_name.angle_increment = 1.0;
  missing_lidar_name.range_max = 1.0;
  missing_lidar_name_config.components.emplace_back(missing_lidar_name);
  if (!check(!mujoco_simulation::SimulationConfigValidator::validate(
                 missing_lidar_name_config, &validation_error) &&
                 validation_error.attribute == "frame_id" &&
                 !validation_error.message.empty(),
             "validator omitted a lidar name diagnostic")) {
    cleanup();
    return 1;
  }

  mujoco_simulation::SimulationConfig missing_base_name_config;
  missing_base_name_config.model.model_path = "model.xml";
  mujoco_simulation::MobileBaseInfo missing_base_name;
  missing_base_name.id = 0;
  missing_base_name.mobile_base_name = "base";
  missing_base_name.mecanum_info.wheel_radius = 1.0;
  missing_base_name.mecanum_info.wheel_base = 1.0;
  missing_base_name.mecanum_info.track_width = 1.0;
  for (std::size_t index = 0; index < missing_base_name.mecanum_wheels.size();
       ++index) {
    missing_base_name.mecanum_wheels[index].wheel_name =
        "wheel" + std::to_string(index);
    missing_base_name.mecanum_wheels[index].actuator_name =
        "actuator" + std::to_string(index);
  }
  missing_base_name_config.components.emplace_back(missing_base_name);
  if (!check(!mujoco_simulation::SimulationConfigValidator::validate(
                 missing_base_name_config, &validation_error) &&
                 validation_error.attribute == "base_body" &&
                 !validation_error.message.empty(),
             "validator omitted a mobile-base name diagnostic")) {
    cleanup();
    return 1;
  }

  cleanup();
  return 0;
}
