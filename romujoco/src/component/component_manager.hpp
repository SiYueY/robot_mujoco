#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "romujoco/config/simulation_config.hpp"
#include "romujoco/data/robot_command.hpp"
#include "romujoco/data/robot_state.hpp"

#include "component/camera/camera_component.hpp"
#include "component/camera/camera_render_service.hpp"
#include "component/component.hpp"
#include "component/gripper/gripper_component.hpp"
#include "component/imu/imu_component.hpp"
#include "component/joint/joint_component.hpp"
#include "component/lidar/lidar_component.hpp"
#include "component/mobile_base/mobile_base_component.hpp"

namespace romujoco {

class ComponentManager {
public:
    bool init(
        const SimulationContext& context, const ComponentConfigList& components,
        CameraRenderService& camera_render_service);
    void clear();

    bool reset(const SimulationContext& context);
    bool reset(const SimulationContext& context, RobotCommand& commands);
    bool advance(const SimulationContext& context);
    bool update(const SimulationContext& context);
    bool write_command(const SimulationContext& context, const RobotCommand& command);
    bool read_state(const SimulationContext& context, RobotState& snapshot) const;
    bool wait_for_camera_results();
    bool has_cameras() const noexcept;
    void clear_camera_states() noexcept;

private:
    bool write_joint_commands(
        const SimulationContext& context, const std::vector<JointCommand>& commands);
    bool write_gripper_commands(
        const SimulationContext& context, const std::vector<GripperCommand>& commands);
    bool write_mobile_base_commands(
        const SimulationContext& context, const std::vector<MobileBaseCommand>& commands);
    bool consume_camera_results();
    bool submit_due_cameras(const SimulationContext& context);

    std::vector<JointComponent::UniquePtr> joints_components_;
    std::vector<GripperComponent::UniquePtr> gripper_components_;
    std::vector<std::uint64_t> joint_command_stamps_;
    std::uint64_t joint_command_epoch_{0};
    std::vector<CameraComponent::UniquePtr> camera_components_;
    std::vector<ImuComponent::UniquePtr> imu_components_;
    std::vector<LidarComponent::UniquePtr> lidar_components_;
    std::vector<MobileBaseComponent::UniquePtr> mobile_base_components_;
    JointStates joints_;
    GripperStates grippers_;
    MobileBaseStates mobile_bases_;
    ImuStates imus_;
    LaserScanStates laser_scans_;
    PointCloudStates point_clouds_;
    CameraStates cameras_;
    CameraRenderService* camera_render_service_{nullptr};
    std::optional<CameraRenderTicket> active_camera_ticket_;
    std::optional<CameraRenderTicket> pending_camera_ticket_;
    std::uint64_t camera_request_sequence_{0};
    std::uint64_t camera_generation_{1};
    std::uint64_t simulation_step_{0};
};

}  // namespace romujoco
