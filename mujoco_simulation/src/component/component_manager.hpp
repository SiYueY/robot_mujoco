#pragma once
// Internal component orchestration contract.

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <typeindex>
#include <vector>

#include "component/camera/camera_component.hpp"
#include "component/camera/camera_render_service.hpp"
#include "component/component.hpp"
#include "component/imu/imu_component.hpp"
#include "component/joint/joint_component.hpp"
#include "component/lidar/lidar_component.hpp"
#include "component/mobile_base/mobile_base_component.hpp"
#include "data/command_snapshot.hpp"
#include "mujoco_simulation/config/simulation_config.hpp"
#include "mujoco_simulation/data/robot_state.hpp"

namespace mujoco_simulation {

class ComponentManager {
public:
    bool init(
        const mjContext& context, const ComponentConfigList& components,
        ComponentId max_component_id, CameraRenderService& camera_render_service);
    void clear();

    bool reset(const mjContext& context);
    bool update(const mjContext& context);
    bool wait_for_camera_results();
    bool has_cameras() const noexcept;
    void clear_camera_states() noexcept;
    bool apply_commands(const mjContext& context, const CommandSnapshot& snapshot);
    bool read_state(const mjContext& context, RobotState& snapshot) const;

private:
    using CommandApplier = std::function<bool(const mjContext&, const CommandSnapshot&)>;

    template <typename Command>
    void register_command_applier(
        std::function<bool(const mjContext&, const std::vector<Command>&)> applier) {
        command_appliers_.emplace_back(
            [applier = std::move(applier)](
                const mjContext& context, const CommandSnapshot& snapshot) {
                const auto* commands = snapshot.channel<Command>();
                return commands == nullptr || applier(context, *commands);
            });
    }
    bool apply_joint_commands(const mjContext& context, const std::vector<JointCommand>& commands);
    bool apply_mobile_base_commands(
        const mjContext& context, const std::vector<MobileBaseCommand>& commands);
    bool consume_camera_results();
    bool submit_due_cameras(const mjContext& context);

    std::vector<JointComponent::UniquePtr> joints_components_;
    std::vector<CameraComponent::UniquePtr> camera_components_;
    std::vector<ImuComponent::UniquePtr> imu_components_;
    std::vector<LidarComponent::UniquePtr> lidar_components_;
    std::vector<MobileBaseComponent::UniquePtr> mobile_base_components_;
    std::size_t joint_component_count_{0};
    std::size_t camera_component_count_{0};
    std::size_t imu_component_count_{0};
    std::size_t lidar_component_count_{0};
    std::size_t mobile_base_component_count_{0};
    JointStates joints_;
    MobileBaseStates mobile_bases_;
    ImuStates imus_;
    LidarStates lidars_;
    CameraStates cameras_;
    CameraRenderService* camera_render_service_{nullptr};
    std::optional<CameraRenderTicket> active_camera_ticket_;
    std::optional<CameraRenderTicket> pending_camera_ticket_;
    std::uint64_t camera_request_sequence_{0};
    std::uint64_t camera_generation_{1};
    std::uint64_t simulation_step_{0};
    std::vector<CommandApplier> command_appliers_;
};

}  // namespace mujoco_simulation
