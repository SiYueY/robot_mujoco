#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <variant>

#include "romujoco/config/simulation_config.hpp"

#include "buffer/command_buffer.hpp"
#include "component/component_id_resolver.hpp"
#include "config/simulation_config_parser.hpp"
#include "test_support.hpp"

namespace {

bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

const char* kConfigPrefix = R"(<robot_mujoco>
  <mujoco><mjcf>model.xml</mjcf></mujoco>
  <simulation>
    <physics period="0.001"/>
    <viewer period="0.02" enabled="false"/>
  </simulation>
  <robot>
)";

const char* kConfigSuffix = R"(
  </robot>
</robot_mujoco>)";

std::string make_xml(const std::string& robot_section) {
    return std::string(kConfigPrefix) + robot_section + kConfigSuffix;
}

bool parse(const std::string& xml, romujoco::SimulationConfig& out) {
    romujoco_test::TemporaryFile file("romujoco_gripper_config_test.xml");
    if (!file.write(xml)) {
        std::cerr << "failed to write configuration\n";
        return false;
    }
    romujoco::SimulationConfigParser parser;
    return parser.load_file(file.path().string(), out);
}

romujoco::GripperInfo make_gripper(std::size_t id = 4, std::string name = "gripper") {
    romujoco::GripperInfo info;
    info.id = id;
    info.name = std::move(name);
    info.period = 0.001;
    info.fingers[0] = {"finger1", "finger1_motor"};
    info.fingers[1] = {"finger2", ""};
    info.control.stiffness = 200.0;
    info.control.damping = 8.0;
    info.width_limits = {0.0, 0.08};
    info.velocity_limits = {0.0, 0.5};
    info.effort_limits = {0.0, 10.0};
    return info;
}

romujoco::JointInfo make_joint(std::size_t id, std::string joint, std::string actuator) {
    romujoco::JointInfo info;
    info.id = id;
    info.joint_name = std::move(joint);
    info.actuator_name = std::move(actuator);
    info.period = 0.001;
    info.default_mode = romujoco::JointMode::Effort;
    info.allowed_modes = {romujoco::JointMode::Effort};
    return info;
}

romujoco::SimulationConfig make_config() {
    romujoco::SimulationConfig config;
    config.model.model_path = "model.xml";
    config.scheduler.physics_period = 0.001;
    config.scheduler.viewer_period = 0.02;
    config.viewer_enabled = false;
    config.components.push_back(make_gripper());
    return config;
}

bool test_xml_parsing() {
    bool success = true;
    romujoco::SimulationConfig parsed;
    const std::string valid = make_xml(R"(
    <gripper id="4" name="left_gripper" period="0.002">
      <finger joint="left_finger1" actuator="left_motor"/>
      <finger joint="left_finger2"/>
      <control stiffness="200" damping="8"/>
      <limit>
        <width min="0" max="0.08"/>
        <velocity min="0" max="0.5"/>
        <effort min="0" max="20"/>
      </limit>
      <stall width_tolerance="0.001" velocity_threshold="0.01"
             effort_ratio="0.9" timeout="0.25"/>
    </gripper>)");
    if (!check(parse(valid, parsed), "valid gripper configuration was rejected") ||
        !check(parsed.components.size() == 1U, "gripper component was not parsed"))
        return false;
    const auto* info = std::get_if<romujoco::GripperInfo>(&parsed.components.front());
    success = success && check(info != nullptr, "parsed component is not a gripper") &&
              check(info->id == 4U && info->name == "left_gripper", "gripper identity lost") &&
              check(std::abs(info->period - 0.002) < 1.0e-12, "gripper period lost") &&
              check(
                  info->fingers[0].joint_name == "left_finger1" &&
                      info->fingers[0].actuator_name == "left_motor" &&
                      info->fingers[1].joint_name == "left_finger2" &&
                      info->fingers[1].actuator_name.empty(),
                  "gripper fingers were not parsed") &&
              check(
                  std::abs(info->control.stiffness - 200.0) < 1.0e-12 &&
                      std::abs(info->control.damping - 8.0) < 1.0e-12,
                  "gripper control gains were not parsed") &&
              check(
                  std::abs(info->width_limits.max - 0.08) < 1.0e-12 &&
                      std::abs(info->velocity_limits.max - 0.5) < 1.0e-12 &&
                      std::abs(info->effort_limits.max - 20.0) < 1.0e-12,
                  "gripper limits were not parsed") &&
              check(
                  std::abs(info->stall.width_tolerance - 0.001) < 1.0e-12 &&
                      std::abs(info->stall.velocity_threshold - 0.01) < 1.0e-12 &&
                      std::abs(info->stall.effort_ratio - 0.9) < 1.0e-12 &&
                      std::abs(info->stall.timeout - 0.25) < 1.0e-12,
                  "gripper stall parameters were not parsed");

    // A missing period defers to the physics period.
    romujoco::SimulationConfig defaulted;
    const std::string without_period = make_xml(R"(
    <gripper id="0" name="gripper">
      <finger joint="finger1"/>
      <finger joint="finger2" actuator="motor"/>
    </gripper>)");
    success = success && check(parse(without_period, defaulted), "default period was rejected");
    if (success) {
        const auto* info = std::get_if<romujoco::GripperInfo>(&defaulted.components.front());
        success = success && check(
                                 info != nullptr && std::abs(info->period - 0.001) < 1.0e-12,
                                 "missing period was not resolved to the physics period");
    }

    const std::string invalid_documents[] = {
        make_xml(R"(<gripper id="0" name="gripper"><finger joint="finger1"/></gripper>)"),
        make_xml(R"(<gripper id="0" name="gripper">
          <finger joint="finger1"/><finger joint="finger2"/><finger joint="finger3"/>
        </gripper>)"),
        make_xml(R"(<gripper id="0" name="gripper">
          <finger/><finger joint="finger2"/>
        </gripper>)"),
        make_xml(R"(<gripper id="0" name="gripper">
          <finger joint="finger1"/><finger joint="finger2"/><module name="m"/>
        </gripper>)"),
        make_xml(R"(<gripper id="0">
          <finger joint="finger1"/><finger joint="finger2"/>
        </gripper>)"),
        make_xml(R"(<gripper name="gripper">
          <finger joint="finger1"/><finger joint="finger2"/>
        </gripper>)"),
        make_xml(R"(<gripper id="0" name="gripper">
          <finger joint="finger1"><nested/></finger><finger joint="finger2"/>
        </gripper>)"),
        make_xml(R"(<gripper id="0" name="gripper">
          <finger joint="finger1"/><finger joint="finger2"/>
          <stall effort_ratio="not-a-number"/>
        </gripper>)"),
        make_xml(R"(<gripper id="0" name="gripper">
          <finger joint="finger1"/><finger joint="finger2"/>
          <limit><width min="0" max="0.08" unknown="1"/></limit>
        </gripper>)"),
        make_xml(R"(<gripper id="0" name="gripper">
          <finger joint="finger1"/><finger joint="finger2"/>
          <control stiffness="200"><nested/></control>
        </gripper>)"),
    };
    for (const std::string& xml : invalid_documents) {
        romujoco::SimulationConfig output;
        success = success && check(!parse(xml, output), "invalid gripper XML was accepted");
    }
    return success;
}

bool test_validation() {
    bool success = check(
        romujoco::SimulationConfigValidator::validate(make_config()),
        "valid gripper configuration failed validation");

    const auto expect_invalid = [&success](const char* message, const auto& mutate) {
        romujoco::SimulationConfig config = make_config();
        mutate(config);
        success = success && check(!romujoco::SimulationConfigValidator::validate(config), message);
    };

    expect_invalid("duplicate gripper ID was accepted", [](romujoco::SimulationConfig& c) {
        c.components.push_back(make_gripper(4, "other"));
    });
    expect_invalid("duplicate gripper name was accepted", [](romujoco::SimulationConfig& c) {
        c.components.push_back(make_gripper(5, "gripper"));
    });
    expect_invalid(
        "gripper with a duplicate finger joint was accepted", [](romujoco::SimulationConfig& c) {
            auto& info = std::get<romujoco::GripperInfo>(c.components.front());
            info.fingers[1].joint_name = info.fingers[0].joint_name;
        });
    expect_invalid("gripper without an actuator was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.fingers[0].actuator_name.clear();
    });
    expect_invalid("empty finger joint name was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.fingers[1].joint_name.clear();
    });
    expect_invalid("negative width limit was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.width_limits.min = -0.01;
    });
    expect_invalid("inverted width limits were accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.width_limits = {0.08, 0.0};
    });
    expect_invalid("non-zero velocity minimum was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.velocity_limits.min = 0.1;
    });
    expect_invalid("non-positive velocity maximum was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.velocity_limits.max = 0.0;
    });
    expect_invalid("non-zero effort minimum was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.effort_limits.min = 1.0;
    });
    expect_invalid("non-positive effort maximum was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.effort_limits.max = 0.0;
    });
    expect_invalid("zero stall effort ratio was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.stall.effort_ratio = 0.0;
    });
    expect_invalid("stall effort ratio above one was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.stall.effort_ratio = 1.01;
    });
    expect_invalid("negative stall timeout was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.stall.timeout = -1.0;
    });
    expect_invalid("negative stiffness was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.control.stiffness = -1.0;
    });
    expect_invalid("zero gripper period was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.period = 0.0;
    });
    expect_invalid("empty gripper name was accepted", [](romujoco::SimulationConfig& c) {
        auto& info = std::get<romujoco::GripperInfo>(c.components.front());
        info.name.clear();
    });
    return success;
}

bool test_resource_ownership() {
    bool success = true;
    const auto expect_invalid = [&success](const char* message, auto mutate) {
        romujoco::SimulationConfig config = make_config();
        mutate(config);
        success = success && check(!romujoco::SimulationConfigValidator::validate(config), message);
    };

    expect_invalid("gripper finger joint was accepted as a joint component", [](auto& c) {
        c.components.push_back(make_joint(0, "finger1", "other_motor"));
    });
    expect_invalid("gripper actuator was accepted as a joint actuator", [](auto& c) {
        c.components.push_back(make_joint(0, "other_joint", "finger1_motor"));
    });
    expect_invalid("two grippers sharing an actuator were accepted", [](auto& c) {
        auto other = make_gripper(5, "other");
        other.fingers[0] = {"other_finger1", "finger1_motor"};
        other.fingers[1] = {"other_finger2", ""};
        c.components.push_back(std::move(other));
    });
    expect_invalid("gripper finger joint was accepted as a swerve module joint", [](auto& c) {
        romujoco::SwerveMobileBaseInfo base;
        base.common.id = 1;
        base.common.name = "base";
        base.common.base_body_name = "base_link";
        base.common.period = 0.001;
        base.common.execution_mode = romujoco::MobileBaseExecutionMode::Dynamic;
        romujoco::SwerveModuleInfo front;
        front.name = "front";
        front.position_x = 0.3;
        front.position_y = -0.2;
        front.wheel_radius = 0.05;
        front.steering_joint_name = "finger1";
        front.steering_actuator_name = "steering_motor";
        front.drive_joint_name = "drive_joint";
        front.drive_actuator_name = "drive_motor";
        romujoco::SwerveModuleInfo rear = front;
        rear.name = "rear";
        rear.position_x = -0.3;
        rear.steering_joint_name = "rear_steering";
        rear.drive_joint_name = "rear_drive";
        rear.steering_actuator_name = "rear_steering_motor";
        rear.drive_actuator_name = "rear_drive_motor";
        base.modules = {front, rear};
        c.components.push_back(std::move(base));
    });

    romujoco::SimulationConfig shared = make_config();
    shared.components.push_back(make_joint(0, "arm_joint", "arm_motor"));
    success = success && check(
                             romujoco::SimulationConfigValidator::validate(shared),
                             "independent joint and gripper resources were rejected");
    return success;
}

bool test_command_buffer() {
    romujoco::SimulationConfig config = make_config();
    config.components.insert(config.components.begin(), make_joint(0, "joint", "joint_motor"));
    auto resolver = romujoco::ComponentIdResolver::create(config.components);
    bool success = check(resolver != nullptr, "component id resolver failed");
    romujoco::CommandBuffer buffer;
    success = success && check(buffer.configure(resolver, config.components), "configure failed");

    romujoco::GripperCommand command;
    command.id = 4;
    command.width = 0.05;
    command.velocity = 0.2;
    command.effort = 1.0;
    success = success && check(buffer.write(command), "valid gripper command was rejected");
    auto snapshot = buffer.read();
    success = success && check(
                             snapshot != nullptr && snapshot->grippers.size() == 1U &&
                                 snapshot->grippers[0].id == 4U &&
                                 std::abs(snapshot->grippers[0].width - 0.05) < 1.0e-12,
                             "gripper command was not published");

    romujoco::JointCommand joint;
    joint.id = 0;
    joint.mode = static_cast<std::uint8_t>(romujoco::JointMode::Effort);
    joint.effort = 2.0;
    success = success && check(buffer.write(joint), "joint command was rejected");
    snapshot = buffer.read();
    success = success && check(
                             snapshot != nullptr && snapshot->grippers.size() == 1U &&
                                 std::abs(snapshot->grippers[0].width - 0.05) < 1.0e-12,
                             "joint update did not preserve the gripper command");

    romujoco::GripperCommand duplicate_id = command;
    romujoco::GripperCommand other = command;
    other.id = 4;
    success = success && check(
                             !buffer.write(romujoco::GripperCommands{duplicate_id, other}),
                             "duplicate gripper id was accepted");
    romujoco::GripperCommand unknown = command;
    unknown.id = 9;
    success = success && check(!buffer.write(unknown), "unknown gripper id was accepted");
    romujoco::GripperCommand negative = command;
    negative.velocity = -0.1;
    success = success && check(!buffer.write(negative), "negative velocity was accepted");
    romujoco::GripperCommand infinite = command;
    infinite.effort = std::numeric_limits<double>::infinity();
    success = success && check(!buffer.write(infinite), "non-finite effort was accepted");

    romujoco::RobotCommand robot;
    romujoco::GripperCommand batch = command;
    batch.width = 0.02;
    robot.grippers.push_back(batch);
    success = success && check(buffer.write(robot), "robot command was rejected");
    snapshot = buffer.read();
    success = success &&
              check(
                  snapshot != nullptr && std::abs(snapshot->grippers[0].width - 0.02) < 1.0e-12 &&
                      std::abs(snapshot->joints[0].effort - 2.0) < 1.0e-12,
                  "robot command did not update the gripper snapshot");
    buffer.shutdown();
    return success;
}

}  // namespace

int main() {
    bool success = test_xml_parsing();
    success = success && test_validation();
    success = success && test_resource_ownership();
    success = success && test_command_buffer();
    return success ? 0 : 1;
}
