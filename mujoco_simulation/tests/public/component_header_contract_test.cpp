#include <type_traits>

#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"

int main() {
  static_assert(std::is_same_v<
                mujoco_simulation::JointCommandBatch,
                std::vector<std::optional<mujoco_simulation::JointCommand>>>);
  static_assert(
      std::is_same_v<
          mujoco_simulation::MobileBaseCommandBatch,
          std::vector<std::optional<mujoco_simulation::MobileBaseCommand>>>);
  return 0;
}
