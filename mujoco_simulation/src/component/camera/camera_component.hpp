#pragma once

#include <memory>

#include <mujoco/mujoco.h>

#include "mujoco_simulation/component/camera.hpp"

#include "component/camera/camera_render_service.hpp"
#include "component/component.hpp"

namespace mujoco_simulation {

class CameraComponent : public SimulationComponent {
public:
    explicit CameraComponent(CameraConfig config);

    bool init(const mjContext& context) override;
    bool reset(const mjContext& context) override;
    bool advance(const mjContext& context) override;
    bool update(const mjContext& context) override;
    CameraRenderTask make_render_task(std::uint64_t timestamp);
    bool apply_render_result(const CameraRenderTaskResult& result);
    void clear_render_state() noexcept;
    bool read_state(std::shared_ptr<const CameraState>& state) const;

    const CameraConfig& config() const noexcept { return config_; }
    const CameraConfig& info() const noexcept { return config_; }

public:
    using SharedPtr = std::shared_ptr<CameraComponent>;
    using UniquePtr = std::unique_ptr<CameraComponent>;
    using WeakPtr = std::weak_ptr<CameraComponent>;

private:
    CameraConfig config_;
    int camera_id_{-1};
    std::uint64_t sample_sequence_{0};
    std::uint64_t last_applied_sequence_{0};
    std::shared_ptr<const CameraState> state_;
};

}  // namespace mujoco_simulation
