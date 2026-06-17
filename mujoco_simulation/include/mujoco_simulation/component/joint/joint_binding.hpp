#pragma once

namespace mujoco_simulation {

struct JointBinding {
  int joint_id{-1};
  int qpos_address{-1};
  int dof_address{-1};
  int joint_type{-1};
  int actuator_id{-1};
  int actuator_type{-1};
  bool has_actuator{false};
};

}  // namespace mujoco_simulation
