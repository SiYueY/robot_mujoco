#pragma once

#include <mujoco/mujoco.h>

#include <memory>

namespace mujoco_simulation {

struct MjModelDeleter {
  void operator()(mjModel* model) const noexcept {
    if (model != nullptr) {
      mj_deleteModel(model);
    }
  }
};

struct MjDataDeleter {
  void operator()(mjData* data) const noexcept {
    if (data != nullptr) {
      mj_deleteData(data);
    }
  }
};

using MjModelPtr = std::unique_ptr<mjModel, MjModelDeleter>;
using MjDataPtr = std::unique_ptr<mjData, MjDataDeleter>;

}  // namespace mujoco_simulation
