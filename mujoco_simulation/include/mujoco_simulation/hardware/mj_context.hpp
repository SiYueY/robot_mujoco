#pragma once

#include <mujoco/mujoco.h>

namespace mujoco_simulation {

struct MjContext {
  const mjModel* model{nullptr};
  mjData* data{nullptr};
  mjvScene* scene{nullptr};
  mjrContext* render{nullptr};
};

}  // namespace mujoco_simulation
