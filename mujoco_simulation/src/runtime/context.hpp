#pragma once
// Internal MuJoCo context wrapper.

#include <mujoco/mujoco.h>

#include "runtime/type.hpp"

namespace mujoco_simulation {

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

}  // namespace mujoco_simulation
