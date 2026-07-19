#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
namespace mujoco_simulation {

using mjTime = double;
using mjStep = std::uint64_t;

struct mjContext {
  constexpr mjContext() = default;
  constexpr mjContext(mjModel* model, mjData* data) noexcept : model(model), data(data) {}

  ~mjContext() { clear(); }

  mjContext(const mjContext&) = delete;
  mjContext& operator=(const mjContext&) = delete;

  mjContext(mjContext&& other) noexcept : model(other.model), data(other.data) {
    other.model = nullptr;
    other.data = nullptr;
  }

  mjContext& operator=(mjContext&& other) noexcept {
    if (this != &other) {
      clear();
      model = other.model;
      data = other.data;
      other.model = nullptr;
      other.data = nullptr;
    }
    return *this;
  }

  void clear() noexcept {
    if (data != nullptr) {
      mj_deleteData(data);
    }
    if (model != nullptr) {
      mj_deleteModel(const_cast<mjModel*>(model));
    }
    model = nullptr;
    data = nullptr;
  }

  const mjModel* model{nullptr};
  mjData* data{nullptr};

  bool valid() const noexcept { return model != nullptr && data != nullptr; }
};

struct mjWheel {
  int wheel_id{-1};
  int actuator_id{-1};
  int dof_address{-1};
};

struct mjJoint {
  int joint_id{-1};
  int actuator_id{-1};
  int qpos_address{-1};
  int dof_address{-1};
};

struct mjImu {
  int framequat_sensor_id{-1};
  int framequat_address{-1};
  int gyro_sensor_id{-1};
  int gyro_address{-1};
  int accelerometer_sensor_id{-1};
  int accelerometer_address{-1};
};

}  // namespace mujoco_simulation
