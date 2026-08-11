#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mujoco_simulation/config/simulation_config.hpp"
#include "mujoco_simulation/data/robot_command.hpp"
#include "mujoco_simulation/data/robot_state.hpp"

#include "component/camera/camera_component.hpp"
#include "component/camera/camera_render_service.hpp"
#include "component/component.hpp"
#include "component/imu/imu_component.hpp"
#include "component/joint/joint_component.hpp"
#include "component/lidar/lidar_component.hpp"
#include "component/mobile_base/mobile_base_component.hpp"

namespace mujoco_simulation {

class ComponentManager {
public:
    bool init(
        const mjContext& context, const ComponentConfigList& components,
        CameraRenderService& camera_render_service);
    void clear();

    bool reset(const mjContext& context);
    bool advance(const mjContext& context);
    bool update(const mjContext& context);
    bool write_command(const mjContext& context, const RobotCommand& command);
    bool read_state(const mjContext& context, RobotState& snapshot) const;
    bool wait_for_camera_results();
    bool has_cameras() const noexcept;
    void clear_camera_states() noexcept;

private:
    bool write_joint_commands(const mjContext& context, const std::vector<JointCommand>& commands);
    bool write_mobile_base_commands(
        const mjContext& context, const std::vector<MobileBaseCommand>& commands);
    bool consume_camera_results();
    bool submit_due_cameras(const mjContext& context);

    std::vector<JointComponent::UniquePtr> joints_components_;
    std::vector<CameraComponent::UniquePtr> camera_components_;
    std::vector<ImuComponent::UniquePtr> imu_components_;
    std::vector<LidarComponent::UniquePtr> lidar_components_;
    std::vector<MobileBaseComponent::UniquePtr> mobile_base_components_;
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
};

}  // namespace mujoco_simulation
