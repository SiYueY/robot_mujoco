#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <variant>

#include "config/simulation_config_parser.hpp"
#include "mujoco_simulation/config/simulation_config.hpp"

namespace {

bool check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
    }
    return condition;
}

bool write_file(const std::filesystem::path& path, const char* content) {
    std::ofstream stream(path);
    stream << content;
    return stream.good();
}

}  // namespace

int main() {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "mujoco_simulation_timing_config_test.xml";
    const auto cleanup = [&path] { std::filesystem::remove(path); };

    const char* valid = R"(
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
    if (!check(parser.load_file(path.string(), config), "valid timing XML was rejected") ||
        !check(
            std::abs(config.scheduler.physics_period - 0.001) < 1e-12,
            "physics period was not parsed") ||
        !check(
            std::abs(config.scheduler.viewer_period - 0.02) < 1e-12,
            "viewer period was not parsed") ||
        !check(!config.viewer_enabled, "viewer enabled was not parsed")) {
        cleanup();
        return 1;
    }

    const char* invalid = R"(
<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation><physics period="0.001"/></simulation>
</robot_mujoco>)";
    if (!check(write_file(path, invalid), "failed to write invalid XML") ||
        !check(!parser.load_file(path.string(), config), "missing viewer period was accepted")) {
        cleanup();
        return 1;
    }

    const char* component_periods = R"(
<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation>
    <physics period="0.001"/>
    <viewer period="0.02"/>
  </simulation>
  <robot>
    <joint id="0" name="joint" period="0.002" mode="effort"><control><effort/></control></joint>
    <imu id="1" name="imu" frame_id="imu" framequat_sensor="quat"
         gyro_sensor="gyro" accelerometer_sensor="accel" period="0.003"/>
    <camera id="2" name="camera" frame_id="camera" camera_name="camera"
            optical_frame_id="optical" width="16" height="12" period="0.004"/>
    <lidar id="3" name="lidar" frame_id="lidar" sensor_prefix="lidar"
           angle_min="0" angle_max="1" angle_increment="1" range_min="0"
           range_max="1" period="0.005"/>
    <mobile_base id="4" name="base" base_body="base" base_joint="base_free"
                 wheel_base="0.2" track_width="0.2" period="0.006">
      <wheel index="front_left" name="front_left" radius="0.1" direction="1" speed_response="0"/>
      <wheel index="front_right" name="front_right" radius="0.1" direction="1" speed_response="0"/>
      <wheel index="rear_left" name="rear_left" radius="0.1" direction="1" speed_response="0"/>
      <wheel index="rear_right" name="rear_right" radius="0.1" direction="1" speed_response="0"/>
    </mobile_base>
  </robot>
</robot_mujoco>)";
    if (!check(write_file(path, component_periods), "failed to write component-period XML") ||
        !check(parser.load_file(path.string(), config), "component periods were rejected") ||
        !check(
            config.components.size() == 5U, "component period XML did not parse every component") ||
        !check(
            std::abs(std::get<mujoco_simulation::JointInfo>(config.components[0]).period - 0.002) <
                1e-12,
            "joint period was not parsed") ||
        !check(
            std::abs(std::get<mujoco_simulation::ImuInfo>(config.components[1]).period - 0.003) <
                1e-12,
            "IMU period was not parsed") ||
        !check(
            std::abs(
                std::get<mujoco_simulation::CameraConfig>(config.components[2]).period - 0.004) <
                1e-12,
            "camera period was not parsed") ||
        !check(
            std::abs(std::get<mujoco_simulation::LidarInfo>(config.components[3]).period - 0.005) <
                1e-12,
            "lidar period was not parsed") ||
        !check(
            std::abs(
                std::get<mujoco_simulation::MobileBaseInfo>(config.components[4]).period - 0.006) <
                1e-12,
            "mobile-base period was not parsed")) {
        cleanup();
        return 1;
    }

    const char* defaulted_period = R"(
<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation>
    <physics period="0.001"/>
    <viewer period="0.02"/>
  </simulation>
  <robot><joint id="0" name="joint" mode="effort"><control><effort/></control></joint></robot>
</robot_mujoco>)";
    if (!check(write_file(path, defaulted_period), "failed to write default-period XML") ||
        !check(parser.load_file(path.string(), config), "default-period XML was rejected") ||
        !check(
            std::abs(std::get<mujoco_simulation::JointInfo>(config.components[0]).period - 0.001) <
                1e-12,
            "missing joint period was not defaulted to the physics period")) {
        cleanup();
        return 1;
    }

    const char* attribute_limits = R"(
<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation><physics period="0.001"/><viewer period="0.02"/></simulation>
  <robot>
    <joint id="0" name="joint" mode="position"><control>
      <position stiffness="300" damping="10"/>
      <velocity damping="20"/>
    </control><limit>
      <position min="-2.8" max="2.8"/>
      <velocity min="-5" max="5"/>
      <effort min="-100" max="100"/>
    </limit></joint>
  </robot>
</robot_mujoco>)";
    if (!check(write_file(path, attribute_limits), "failed to write attribute-limit XML") ||
        !check(parser.load_file(path.string(), config), "attribute limits were rejected")) {
        cleanup();
        return 1;
    }
    const auto& attribute_limit_joint =
        std::get<mujoco_simulation::JointInfo>(config.components[0]);
    if (!check(
            attribute_limit_joint.position_stiffness == 300.0 &&
                attribute_limit_joint.position_damping == 10.0 &&
                attribute_limit_joint.velocity_damping == 20.0 &&
                attribute_limit_joint.position_limits.min == -2.8 &&
                attribute_limit_joint.position_limits.max == 2.8 &&
                attribute_limit_joint.velocity_limits.min == -5.0 &&
                attribute_limit_joint.velocity_limits.max == 5.0 &&
                attribute_limit_joint.effort_limits.min == -100.0 &&
                attribute_limit_joint.effort_limits.max == 100.0,
            "attribute limits were not parsed")) {
        cleanup();
        return 1;
    }

    const char* defaulted_control = R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint" mode="position"><control><position stiffness="0" damping="0"/><velocity damping="0"/></control></joint>
</robot></robot_mujoco>)";
    if (!check(write_file(path, defaulted_control), "failed to write default-control XML") ||
        !check(parser.load_file(path.string(), config), "default-control XML was rejected")) {
        cleanup();
        return 1;
    }
    const auto& defaulted_control_joint =
        std::get<mujoco_simulation::JointInfo>(config.components[0]);
    if (!check(
            defaulted_control_joint.position_stiffness == 0.0 &&
                defaulted_control_joint.position_damping == 0.0 &&
                defaulted_control_joint.velocity_damping == 0.0,
            "missing control attributes were not defaulted to zero")) {
        cleanup();
        return 1;
    }

    const char* one_sided_limit = R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint" mode="effort"><control><effort/></control><limit><position min="-2.8"/><velocity max="5"/></limit></joint>
</robot></robot_mujoco>)";
    if (!check(write_file(path, one_sided_limit), "failed to write one-sided limit XML") ||
        !check(parser.load_file(path.string(), config), "one-sided limits were rejected")) {
        cleanup();
        return 1;
    }
    const auto& one_sided_joint = std::get<mujoco_simulation::JointInfo>(config.components[0]);
    if (!check(
            one_sided_joint.position_limits.min == -2.8 &&
                std::isinf(one_sided_joint.position_limits.max) &&
                one_sided_joint.position_limits.max > 0.0 &&
                std::isinf(one_sided_joint.velocity_limits.min) &&
                one_sided_joint.velocity_limits.min < 0.0 &&
                one_sided_joint.velocity_limits.max == 5.0,
            "one-sided limits did not preserve unbounded sides")) {
        cleanup();
        return 1;
    }

    const char* legacy_rate = R"(
<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation>
    <physics period="0.001"/>
    <viewer period="0.02"/>
  </simulation>
  <robot><joint id="0" name="joint" update_rate="1000"/></robot>
</robot_mujoco>)";
    if (!check(write_file(path, legacy_rate), "failed to write legacy XML") ||
        !check(
            !parser.load_file(path.string(), config),
            "legacy update_rate attribute was accepted")) {
        cleanup();
        return 1;
    }

    const char* invalid_viewer_enabled = R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02" enabled="maybe"/>
</simulation></robot_mujoco>)";
    if (!check(write_file(path, invalid_viewer_enabled), "failed to write invalid viewer XML") ||
        !check(
            !parser.load_file(path.string(), config),
            "invalid viewer enabled attribute was accepted")) {
        cleanup();
        return 1;
    }

    struct InvalidConfig {
        const char* name;
        const char* content;
    };
    const std::array<InvalidConfig, 12> invalid_configs = {{
        {"trailing joint numeric text", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><control><position stiffness="100abc"/></control></joint>
</robot></robot_mujoco>)"},
        {"infinite joint numeric text", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><limit><effort max="inf"/></limit></joint>
</robot></robot_mujoco>)"},
        {"infinite control attribute", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><control><position stiffness="inf"/></control></joint>
</robot></robot_mujoco>)"},
        {"legacy joint limit child elements", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><limit><position><min>-1</min></position></limit></joint>
</robot></robot_mujoco>)"},
        {"reversed joint limit attributes", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><limit><position min="1" max="0"/></limit></joint>
</robot></robot_mujoco>)"},
        {"legacy direct joint control", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><position><stiffness>-1</stiffness></position></joint>
</robot></robot_mujoco>)"},
        {"negative stiffness", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><control><position stiffness="-1"/></control></joint>
</robot></robot_mujoco>)"},
        {"negative damping", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><control><velocity damping="-1"/></control></joint>
</robot></robot_mujoco>)"},
        {"unknown control mode", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="0" name="joint"><control><effort/></control></joint>
</robot></robot_mujoco>)"},
        {"oversized component id", R"(
<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation>
<physics period="0.001"/><viewer period="0.02"/></simulation><robot>
<joint id="256" name="joint"/></robot></robot_mujoco>)"},
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
    for (const InvalidConfig& invalid_config : invalid_configs) {
        if (!check(write_file(path, invalid_config.content), invalid_config.name) ||
            !check(!parser.load_file(path.string(), config), invalid_config.name)) {
            cleanup();
            return 1;
        }
    }

    const std::array<InvalidConfig, 13> semantic_invalid_configs =
        {
            {
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
                {"legacy base wheel radius",
                 R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><mobile_base id="0" name="b" base_body="b" base_joint="j" wheel_radius="1" wheel_base="1" track_width="1"><wheel index="front_left" name="a" radius="1" direction="1" speed_response="0"/><wheel index="front_right" name="b" radius="1" direction="1" speed_response="0"/><wheel index="rear_left" name="c" radius="1" direction="1" speed_response="0"/><wheel index="rear_right" name="d" radius="1" direction="1" speed_response="0"/></mobile_base></robot></robot_mujoco>)"},
                {"non-positive wheel radius",
                 R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><mobile_base id="0" name="b" base_body="b" base_joint="j" wheel_base="1" track_width="1"><wheel index="front_left" name="a" radius="0" direction="1" speed_response="0"/><wheel index="front_right" name="b" radius="1" direction="1" speed_response="0"/><wheel index="rear_left" name="c" radius="1" direction="1" speed_response="0"/><wheel index="rear_right" name="d" radius="1" direction="1" speed_response="0"/></mobile_base></robot></robot_mujoco>)"},
                {"duplicate wheel name",
                 R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><mobile_base id="0" name="b" base_body="b" base_joint="j" wheel_base="1" track_width="1"><wheel index="front_left" name="a" radius="1" direction="1" speed_response="0"/><wheel index="front_right" name="a" radius="1" direction="1" speed_response="0"/><wheel index="rear_left" name="c" radius="1" direction="1" speed_response="0"/><wheel index="rear_right" name="d" radius="1" direction="1" speed_response="0"/></mobile_base></robot></robot_mujoco>)"},
                {"invalid wheel direction",
                 R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><mobile_base id="0" name="b" base_body="b" base_joint="j" wheel_base="1" track_width="1"><wheel index="front_left" name="a" radius="1" direction="0" speed_response="0"/><wheel index="front_right" name="b" radius="1" direction="1" speed_response="0"/><wheel index="rear_left" name="c" radius="1" direction="1" speed_response="0"/><wheel index="rear_right" name="d" radius="1" direction="1" speed_response="0"/></mobile_base></robot></robot_mujoco>)"},
                {"negative wheel response",
                 R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><mobile_base id="0" name="b" base_body="b" base_joint="j" wheel_base="1" track_width="1"><wheel index="front_left" name="a" radius="1" direction="1" speed_response="-1"/><wheel index="front_right" name="b" radius="1" direction="1" speed_response="0"/><wheel index="rear_left" name="c" radius="1" direction="1" speed_response="0"/><wheel index="rear_right" name="d" radius="1" direction="1" speed_response="0"/></mobile_base></robot></robot_mujoco>)"},
            }};
    for (const InvalidConfig& invalid_config : semantic_invalid_configs) {
        if (!check(write_file(path, invalid_config.content), invalid_config.name) ||
            !check(!parser.load_file(path.string(), config), invalid_config.name)) {
            cleanup();
            return 1;
        }
    }

    const char* camera_boundary =
        R"(<robot_mujoco><mujoco><mjcf>model.xml</mjcf></mujoco><simulation><physics period=".001"/><viewer period=".02"/></simulation><robot><camera id="0" name="c" frame_id="f" camera_name="c" optical_frame_id="o" width="8192" height="8192" enable_rgb="true" enable_depth="false"/></robot></robot_mujoco>)";
    if (!check(write_file(path, camera_boundary), "failed to write camera boundary XML") ||
        !check(
            parser.load_file(path.string(), config),
            "valid maximum-dimension RGB camera was rejected")) {
        cleanup();
        return 1;
    }

    const char* duplicate_joint = R"(<robot_mujoco>
<mujoco><mjcf>model.xml</mjcf></mujoco>
<simulation><physics period=".001"/><viewer period=".02"/></simulation>
<robot>
  <joint id="0" name="first"/>
  <joint id="0" name="second"/>
</robot>
</robot_mujoco>)";
    if (!check(write_file(path, duplicate_joint), "failed to write duplicate XML") ||
        !check(!parser.load_file(path.string(), config), "duplicate component was accepted")) {
        cleanup();
        return 1;
    }

    mujoco_simulation::SimulationConfig direct_config;
    direct_config.model.model_path = "model.xml";
    mujoco_simulation::CameraConfig invalid_camera;
    invalid_camera.width = 0;
    invalid_camera.height = 1;
    direct_config.components.emplace_back(invalid_camera);
    if (!check(
            !mujoco_simulation::SimulationConfigValidator::validate(direct_config),
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
    if (!check(
            !mujoco_simulation::SimulationConfigValidator::validate(invalid_period_config),
            "validator accepted a negative direct C++ component period")) {
        cleanup();
        return 1;
    }

    mujoco_simulation::SimulationConfig duplicate_config;
    duplicate_config.model.model_path = "model.xml";
    invalid_period_joint.period = 0.001;
    duplicate_config.components.emplace_back(invalid_period_joint);
    duplicate_config.components.emplace_back(invalid_period_joint);
    if (!check(
            !mujoco_simulation::SimulationConfigValidator::validate(duplicate_config),
            "validator accepted duplicate direct C++ component IDs")) {
        cleanup();
        return 1;
    }

    mujoco_simulation::SimulationConfig missing_model_config;
    if (!check(
            !mujoco_simulation::SimulationConfigValidator::validate(missing_model_config),
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
    if (!check(
            !mujoco_simulation::SimulationConfigValidator::validate(missing_camera_name_config),
            "validator omitted a camera name diagnostic")) {
        cleanup();
        return 1;
    }

    mujoco_simulation::SimulationConfig invalid_renderer_config;
    invalid_renderer_config.model.model_path = "model.xml";
    invalid_renderer_config.camera_renderer.completed_ticket_history = 0U;
    if (!check(
            !mujoco_simulation::SimulationConfigValidator::validate(invalid_renderer_config),
            "validator accepted zero camera ticket history")) {
        cleanup();
        return 1;
    }

    mujoco_simulation::SimulationConfig no_backend_config;
    no_backend_config.model.model_path = "model.xml";
    no_backend_config.camera_renderer.allow_glfw_backend = false;
    no_backend_config.camera_renderer.allow_egl_backend = false;
    if (!check(
            !mujoco_simulation::SimulationConfigValidator::validate(no_backend_config),
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
    if (!check(
            !mujoco_simulation::SimulationConfigValidator::validate(missing_lidar_name_config),
            "validator omitted a lidar name diagnostic")) {
        cleanup();
        return 1;
    }

    mujoco_simulation::SimulationConfig missing_base_name_config;
    missing_base_name_config.model.model_path = "model.xml";
    mujoco_simulation::MobileBaseInfo missing_base_name;
    missing_base_name.id = 0;
    missing_base_name.mobile_base_name = "base";
    missing_base_name.base_joint_name = "base_free";
    missing_base_name.period = 0.001;
    missing_base_name.mecanum_info.wheel_base = 1.0;
    missing_base_name.mecanum_info.track_width = 1.0;
    for (std::size_t index = 0; index < missing_base_name.mecanum_wheels.size(); ++index) {
        missing_base_name.mecanum_wheels[index].wheel_name = "wheel" + std::to_string(index);
        missing_base_name.mecanum_wheels[index].radius = 1.0;
    }
    missing_base_name_config.components.emplace_back(missing_base_name);
    if (!check(
            !mujoco_simulation::SimulationConfigValidator::validate(missing_base_name_config),
            "validator omitted a mobile-base name diagnostic")) {
        cleanup();
        return 1;
    }

    cleanup();
    return 0;
}
