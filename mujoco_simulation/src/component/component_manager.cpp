#include "mujoco_simulation/component/component_manager.hpp"

#include <utility>
#include <vector>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"

namespace mujoco_simulation {
bool ComponentManager::init(const mjContext& context, const ComponentConfigList& components) {
  if (!context.valid()) {
    LOG_ERROR << "component manager requires a valid MuJoCo context.";
    return false;
  }
  clear();

  for (const ComponentConfig& component : components) {
    if (const auto* joint = std::get_if<JointInfo>(&component)) {
      if (!register_component(context, std::make_unique<JointComponent>(*joint))) {
        clear();
        return false;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* imu = std::get_if<ImuInfo>(&component)) {
      if (!register_component(context, std::make_unique<ImuComponent>(*imu))) {
        clear();
        return false;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* lidar = std::get_if<LidarInfo>(&component)) {
      if (!register_component(context, std::make_unique<LidarComponent>(*lidar))) {
        clear();
        return false;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* mobile_base = std::get_if<MobileBaseInfo>(&component)) {
      auto mobile_base_component = std::make_unique<MobileBaseComponent>(*mobile_base);
      if (!register_component(context, std::move(mobile_base_component))) {
        clear();
        return false;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* camera = std::get_if<CameraConfig>(&component)) {
      if (!register_component(context, std::make_unique<CameraComponent>(*camera))) {
        clear();
        return false;
      }
    }
  }

  return true;
}

void ComponentManager::clear() { component_registry.clear(); }

bool ComponentManager::register_component(const mjContext& context,
                                          JointComponent::UniquePtr joint) {
  if (joint == nullptr) {
    LOG_ERROR << "joint must not be null.";
    return false;
  }
  if (!joint->init(context)) {
    return false;
  }
  return component_registry.add(std::move(joint));
}

bool ComponentManager::register_component(const mjContext& context,
                                          CameraComponent::UniquePtr camera) {
  if (camera == nullptr) {
    LOG_ERROR << "camera must not be null.";
    return false;
  }
  if (!camera->init(context)) {
    return false;
  }
  return component_registry.add(std::move(camera));
}

bool ComponentManager::register_component(const mjContext& context, ImuComponent::UniquePtr imu) {
  if (imu == nullptr) {
    LOG_ERROR << "imu must not be null.";
    return false;
  }
  if (!imu->init(context)) {
    return false;
  }
  return component_registry.add(std::move(imu));
}

bool ComponentManager::register_component(const mjContext& context,
                                          LidarComponent::UniquePtr lidar) {
  if (lidar == nullptr) {
    LOG_ERROR << "lidar must not be null.";
    return false;
  }
  if (!lidar->init(context)) {
    return false;
  }
  return component_registry.add(std::move(lidar));
}

bool ComponentManager::register_component(const mjContext& context,
                                          MobileBaseComponent::UniquePtr mobile_base) {
  if (mobile_base == nullptr) {
    LOG_ERROR << "mobile base must not be null.";
    return false;
  }
  if (!mobile_base->init(context)) {
    return false;
  }
  return component_registry.add(std::move(mobile_base));
}

bool ComponentManager::reset(const mjContext& context) {
  if (!context.valid()) {
    LOG_ERROR << "component manager requires a valid MuJoCo context.";
    return false;
  }
  for (const auto& [name, joint_component] : component_registry.joints()) {
    UNUSED(name);
    if (!joint_component->reset(context)) {
      return false;
    }
  }
  for (const auto& [name, camera_component] : component_registry.cameras()) {
    UNUSED(name);
    if (!camera_component->reset(context)) {
      return false;
    }
  }
  for (const auto& [name, imu_component] : component_registry.imus()) {
    UNUSED(name);
    if (!imu_component->reset(context)) {
      return false;
    }
  }
  for (const auto& [name, lidar_component] : component_registry.lidars()) {
    UNUSED(name);
    if (!lidar_component->reset(context)) {
      return false;
    }
  }
  for (const auto& [name, mobile_base_component] : component_registry.mobile_bases()) {
    UNUSED(name);
    if (!mobile_base_component->reset(context)) {
      return false;
    }
  }
  for (const auto& [name, joint_component] : component_registry.joints()) {
    UNUSED(name);
    UNUSED(joint_component->reset_schedule());
  }
  for (const auto& [name, camera_component] : component_registry.cameras()) {
    UNUSED(name);
    UNUSED(camera_component->reset_schedule());
  }
  for (const auto& [name, imu_component] : component_registry.imus()) {
    UNUSED(name);
    UNUSED(imu_component->reset_schedule());
  }
  for (const auto& [name, lidar_component] : component_registry.lidars()) {
    UNUSED(name);
    UNUSED(lidar_component->reset_schedule());
  }
  for (const auto& [name, mobile_base_component] : component_registry.mobile_bases()) {
    UNUSED(name);
    UNUSED(mobile_base_component->reset_schedule());
  }
  return true;
}

bool ComponentManager::update(const mjContext& context, CameraRenderer* camera_renderer,
                              CameraBuffer* camera_buffer) {
  if (!context.valid()) {
    LOG_ERROR << "component manager requires a valid MuJoCo context.";
    return false;
  }
  std::vector<JointComponent*> due_joints;
  std::vector<MobileBaseComponent*> due_mobile_bases;
  std::vector<ImuComponent*> due_imus;
  std::vector<LidarComponent*> due_lidars;
  std::vector<CameraComponent*> due_cameras;

  due_joints.reserve(component_registry.joints().size());
  due_mobile_bases.reserve(component_registry.mobile_bases().size());
  due_imus.reserve(component_registry.imus().size());
  due_lidars.reserve(component_registry.lidars().size());
  due_cameras.reserve(component_registry.cameras().size());

  for (const auto& [name, joint_component] : component_registry.joints()) {
    UNUSED(name);
    if (joint_component->poll_update(context.data->time)) {
      due_joints.push_back(joint_component);
    }
  }
  for (const auto& [name, mobile_base_component] : component_registry.mobile_bases()) {
    UNUSED(name);
    if (mobile_base_component->poll_update(context.data->time)) {
      due_mobile_bases.push_back(mobile_base_component);
    }
  }
  for (const auto& [name, imu_component] : component_registry.imus()) {
    UNUSED(name);
    if (imu_component->poll_update(context.data->time)) {
      due_imus.push_back(imu_component);
    }
  }
  for (const auto& [name, lidar_component] : component_registry.lidars()) {
    UNUSED(name);
    if (lidar_component->poll_update(context.data->time)) {
      due_lidars.push_back(lidar_component);
    }
  }
  for (const auto& [name, camera_component] : component_registry.cameras()) {
    UNUSED(name);
    if (camera_component->poll_update(context.data->time)) {
      due_cameras.push_back(camera_component);
    }
  }

  if (!due_cameras.empty()) {
    if (camera_renderer == nullptr || camera_buffer == nullptr) {
      LOG_ERROR << "camera update requires camera_renderer and camera_buffer.";
      return false;
    }
    if (!camera_renderer->copy_simulation_data(*context.model, *context.data)) {
      LOG_ERROR << "failed to copy simulation data for camera rendering.";
      return false;
    }
  }

  for (JointComponent* joint_component : due_joints) {
    if (!joint_component->update(context)) {
      return false;
    }
  }
  for (MobileBaseComponent* mobile_base_component : due_mobile_bases) {
    if (!mobile_base_component->update(context)) {
      return false;
    }
  }
  for (ImuComponent* imu_component : due_imus) {
    if (!imu_component->update(context)) {
      return false;
    }
  }
  for (LidarComponent* lidar_component : due_lidars) {
    if (!lidar_component->update(context)) {
      return false;
    }
  }
  for (CameraComponent* camera_component : due_cameras) {
    if (!camera_component->configure_rendering(camera_renderer, camera_buffer)) {
      return false;
    }
    if (!camera_component->update(context)) {
      return false;
    }
  }
  return true;
}

bool ComponentManager::write_command(const mjContext& context, const RobotCommand& snapshot) {
  if (!context.valid()) {
    LOG_ERROR << "component manager requires a valid MuJoCo context.";
    return false;
  }
  for (const auto& [joint_name, command] : snapshot.joint_commands) {
    JointComponent* joint_component = component_registry.joint(joint_name);
    if (joint_component == nullptr) {
      LOG_ERROR << "joint command target was not found.";
      return false;
    }
    if (!joint_component->write(context, command)) {
      return false;
    }
  }
  for (const auto& [mobile_base_name, command] : snapshot.mobile_base_commands) {
    MobileBaseComponent* mobile_base_component = component_registry.mobile_base(mobile_base_name);
    if (mobile_base_component == nullptr) {
      LOG_ERROR << "mobile base command target was not found.";
      return false;
    }
    if (!mobile_base_component->write(context, command)) {
      return false;
    }
  }
  return true;
}

bool ComponentManager::read_state(const mjContext& context, RobotState& snapshot) const {
  if (!context.valid()) {
    LOG_ERROR << "component manager requires a valid MuJoCo context.";
    return false;
  }
  if (!read_state(context, snapshot.joints)) {
    return false;
  }
  if (!read_state(context, snapshot.mobile_bases)) {
    return false;
  }
  if (!read_state(context, snapshot.imus)) {
    return false;
  }
  return read_state(context, snapshot.lidars);
}

bool ComponentManager::read_state(const mjContext& context, JointStates& states) const {
  if (!context.valid()) {
    LOG_ERROR << "component manager requires a valid MuJoCo context.";
    return false;
  }
  JointStates snapshot;
  snapshot.reserve(component_registry.joints().size());
  for (const auto& [name, joint_component] : component_registry.joints()) {
    JointState state;
    if (!joint_component->read(context, state)) {
      return false;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return true;
}

bool ComponentManager::read_state(const mjContext& context, ImuStates& states) const {
  if (!context.valid()) {
    LOG_ERROR << "component manager requires a valid MuJoCo context.";
    return false;
  }
  ImuStates snapshot;
  snapshot.reserve(component_registry.imus().size());
  for (const auto& [name, imu_component] : component_registry.imus()) {
    ImuState state;
    if (!imu_component->read(context, state)) {
      return false;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return true;
}

bool ComponentManager::read_state(const mjContext& context, LidarStates& states) const {
  if (!context.valid()) {
    LOG_ERROR << "component manager requires a valid MuJoCo context.";
    return false;
  }
  LidarStates snapshot;
  snapshot.reserve(component_registry.lidars().size());
  for (const auto& [name, lidar_component] : component_registry.lidars()) {
    LidarState state;
    if (!lidar_component->read(context, state)) {
      return false;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return true;
}

bool ComponentManager::read_state(const mjContext& context, MobileBaseStates& states) const {
  if (!context.valid()) {
    LOG_ERROR << "component manager requires a valid MuJoCo context.";
    return false;
  }
  MobileBaseStates snapshot;
  snapshot.reserve(component_registry.mobile_bases().size());
  for (const auto& [name, mobile_base_component] : component_registry.mobile_bases()) {
    MobileBaseState state;
    if (!mobile_base_component->read(context, state)) {
      return false;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return true;
}

}  // namespace mujoco_simulation
