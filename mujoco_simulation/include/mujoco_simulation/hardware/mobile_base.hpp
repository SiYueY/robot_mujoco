#pragma once

#include <string>
#include <vector>

#include "mujoco_simulation/hardware/data.hpp"
#include "mujoco_simulation/hardware/hardware_interface.hpp"
#include "mujoco_simulation/hardware/joint.hpp"
#include "mujoco_simulation/hardware/mj_context.hpp"

namespace mujoco_simulation {

enum class MobileBaseType {
  None,
  Differential,
  Omnidirectional,
};

struct MobileBaseInfo {
  std::string name;
  MobileBaseType type{MobileBaseType::None};
  std::string base_frame_id{"base_link"};
  std::string odom_frame_id{"odom"};
  std::vector<std::string> traction_joint_names;
  std::vector<std::string> passive_joint_names;
  double wheel_radius{0.0};
  double track_width{0.0};
  double wheel_base{0.0};
};

// https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/Twist.msg
struct MobileBaseCommand {
  Vector3d linear;
  Vector3d angular;
};

struct MobileBaseState {
  std::string base_frame_id;
  std::string odom_frame_id;
  Vector3d linear;
  Vector3d angular;
  std::vector<double> wheel_velocities;
};

class MobileBase : public HardwareInterface<MobileBaseInfo, MobileBaseCommand, MobileBaseState> {
 public:
  MobileBase(const MjContext& context, const std::vector<Joint*>& traction_joints);
  ~MobileBase() override = default;

  bool init(const MobileBaseInfo& data) override;
  bool reset() override;
  bool write(const MobileBaseCommand& command) override;
  bool read(MobileBaseState& state) override;

  const MobileBaseInfo& data() const { return data_; }
  const std::string& last_error() const override { return last_error_; }

 private:
  bool set_error(const std::string& message);
  bool validate() const;
  bool write_differential(const MobileBaseCommand& command);
  bool write_omnidirectional(const MobileBaseCommand& command);
  bool read_differential(MobileBaseState& state);
  bool read_omnidirectional(MobileBaseState& state);

  MjContext context_{};
  std::vector<Joint*> traction_joints_;
  MobileBaseInfo data_;
  MobileBaseCommand command_;
  MobileBaseState state_;
  std::string last_error_;
};

}  // namespace mujoco_simulation
