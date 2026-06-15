#pragma once

#include <mujoco/mujoco.h>

#include <string>

#include "mujoco_simulation/hardware/hardware_interface.hpp"
#include "mujoco_simulation/hardware/mj_context.hpp"

namespace mujoco_simulation {

enum class JointType {
  Unknown,
  Hinge,
  Slide,
  Ball,
  Free,
};

enum class ActuatorType {
  Unknown,
  Passive,
  Motor,
  Position,
  Velocity,
  Custom,
};

enum class CommandInterfaceType {
  None,
  Position,
  Velocity,
  Effort,
};

struct JointInfo {
  std::string name;
  std::string actuator_name;
  CommandInterfaceType command_mode{CommandInterfaceType::None};
};

struct JointCommand {
  std::string name;
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double effort{0.0};
};

struct JointState {
  std::string name;
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

class Joint : public HardwareInterface<JointInfo, JointCommand, JointState> {
 public:
  explicit Joint(const MjContext& context);
  ~Joint() override = default;

  bool init(const JointInfo& data) override;
  bool reset() override;
  bool write(const JointCommand& command) override;
  bool read(JointState& state) override;

  const JointInfo& data() const { return data_; }
  JointType joint_type() const { return joint_type_; }
  ActuatorType actuator_type() const { return actuator_type_; }
  const std::string& last_error() const override { return last_error_; }

 private:
  bool set_error(const std::string& message);
  int find_actuator_id() const;
  JointType parse_joint_type(int mujoco_joint_type) const;
  ActuatorType parse_actuator_type(int actuator_id) const;
  bool validate_command_mode() const;

  MjContext context_{};

  int joint_id_{-1};
  int qpos_address_{-1};
  int dof_address_{-1};
  int actuator_id_{-1};
  JointType joint_type_{JointType::Unknown};
  ActuatorType actuator_type_{ActuatorType::Unknown};
  CommandInterfaceType command_mode_{CommandInterfaceType::None};

  JointInfo data_;
  JointCommand command_;
  JointState state_;
  std::string last_error_;
};

}  // namespace mujoco_simulation
