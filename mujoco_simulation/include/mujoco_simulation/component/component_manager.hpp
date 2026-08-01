#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <typeindex>
#include <vector>

#include "mujoco_simulation/component/camera/camera_component.hpp"
#include "mujoco_simulation/component/imu/imu_component.hpp"
#include "mujoco_simulation/component/joint/joint_component.hpp"
#include "mujoco_simulation/component/lidar/lidar_component.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"
#include "mujoco_simulation/config/simulation_config.hpp"
#include "mujoco_simulation/data/command_snapshot.hpp"
#include "mujoco_simulation/data/robot_state.hpp"
#include "mujoco_simulation/visibility.hpp"

namespace mujoco_simulation {

class MUJOCO_SIMULATION_PUBLIC ComponentManager {
public:
  bool init(const mjContext &context, const ComponentConfigList &components,
            ComponentId max_component_id, CameraRenderer &camera_renderer);
  void clear();

  bool reset(const mjContext &context);
  bool update(const mjContext &context);
  bool wait_for_camera_results();
  bool has_cameras() const noexcept;
  void clear_camera_states() noexcept;
  bool apply_commands(const mjContext &context,
                      const CommandSnapshot &snapshot);
  bool read_state(const mjContext &context, RobotState &snapshot) const;

private:
  using CommandApplier =
      std::function<bool(const mjContext &, const CommandSnapshot &)>;

  template <typename Command>
  void register_command_applier(
      std::function<bool(const mjContext &,
                         const std::vector<std::optional<Command>> &)>
          applier) {
    command_appliers_.emplace_back(
        [applier = std::move(applier)](const mjContext &context,
                                       const CommandSnapshot &snapshot) {
          const auto *commands = snapshot.channel<Command>();
          return commands == nullptr || applier(context, *commands);
        });
  }
  bool apply_joint_commands(
      const mjContext &context,
      const std::vector<std::optional<JointCommand>> &commands);
  bool apply_mobile_base_commands(
      const mjContext &context,
      const std::vector<std::optional<MobileBaseCommand>> &commands);
  bool consume_camera_results();
  bool submit_due_cameras(const mjContext &context);

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
  CameraRenderer *camera_renderer_{nullptr};
  std::optional<CameraRenderTicket> pending_camera_ticket_;
  std::vector<CommandApplier> command_appliers_;
};

} // namespace mujoco_simulation
