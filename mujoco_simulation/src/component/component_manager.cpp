#include "mujoco_simulation/component/component_manager.hpp"

#include <string_view>
#include <utility>
#include <vector>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/component/update_context.hpp"

namespace mujoco_simulation {
namespace {

std::vector<std::string> traction_joint_names(const MobileBaseInfo& info) {
  if (info.type == MobileBaseType::Differential) {
    return {info.left_wheel_joint, info.right_wheel_joint};
  }
  if (info.type == MobileBaseType::Omnidirectional) {
    return {info.front_left_joint, info.front_right_joint, info.rear_left_joint,
            info.rear_right_joint};
  }
  return {};
}

bool validate_mobile_base_joint_binding(const JointComponent& joint_component) {
  if (joint_component.joint_id() < 0 || joint_component.dof_address() < 0) {
    return log_component_error("ComponentManager::validate_mobile_base_joint_binding",
                               "wheel joint is not bound.");
  }
  return true;
}

JointComponent* joint(ComponentRegistry& registry, std::string_view name) {
  return registry.joint(std::string(name));
}

const JointComponent* joint(const ComponentRegistry& registry, std::string_view name) {
  return registry.joint(std::string(name));
}

MobileBaseComponent* mobile_base(ComponentRegistry& registry, std::string_view name) {
  return registry.mobile_base(std::string(name));
}

}  // namespace

bool ComponentManager::build(const mjModel& model, const ComponentConfigList& components) {
  clear();

  for (const ComponentConfig& component : components) {
    if (const auto* joint = std::get_if<JointInfo>(&component)) {
      if (!register_joint(model, std::make_unique<JointComponent>(*joint))) {
        clear();
        return false;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* imu = std::get_if<ImuInfo>(&component)) {
      if (!register_imu(model, std::make_unique<ImuComponent>(*imu))) {
        clear();
        return false;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* lidar = std::get_if<LidarInfo>(&component)) {
      if (!register_lidar(model, std::make_unique<LidarComponent>(*lidar))) {
        clear();
        return false;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* mobile_base = std::get_if<MobileBaseInfo>(&component)) {
      if (!register_mobile_base(model, *mobile_base)) {
        clear();
        return false;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* camera = std::get_if<CameraConfig>(&component)) {
      if (!register_camera(model, std::make_unique<CameraComponent>(*camera))) {
        clear();
        return false;
      }
    }
  }

  return true;
}

void ComponentManager::clear() { registry_.clear(); }

bool ComponentManager::register_joint(const mjModel& model, std::unique_ptr<JointComponent> joint) {
  if (joint == nullptr) {
    return log_component_error("ComponentManager::register_joint", "joint must not be null.");
  }
  if (!joint->bind(model)) {
    return false;
  }
  return registry_.add(std::move(joint));
}

bool ComponentManager::register_camera(const mjModel& model,
                                       std::unique_ptr<CameraComponent> camera) {
  if (camera == nullptr) {
    return log_component_error("ComponentManager::register_camera", "camera must not be null.");
  }
  if (!camera->bind(model)) {
    return false;
  }
  return registry_.add(std::move(camera));
}

bool ComponentManager::register_imu(const mjModel& model, std::unique_ptr<ImuComponent> imu) {
  if (imu == nullptr) {
    return log_component_error("ComponentManager::register_imu", "imu must not be null.");
  }
  if (!imu->bind(model)) {
    return false;
  }
  return registry_.add(std::move(imu));
}

bool ComponentManager::register_lidar(const mjModel& model, std::unique_ptr<LidarComponent> lidar) {
  if (lidar == nullptr) {
    return log_component_error("ComponentManager::register_lidar", "lidar must not be null.");
  }
  if (!lidar->bind(model)) {
    return false;
  }
  return registry_.add(std::move(lidar));
}

bool ComponentManager::register_mobile_base(const mjModel& model, const MobileBaseInfo& info) {
  if (info.name.empty()) {
    return log_component_error("ComponentManager::register_mobile_base",
                               "mobile base name must not be empty.");
  }
  if (registry_.has_mobile_base(info.name)) {
    return log_component_error("ComponentManager::register_mobile_base",
                               "mobile base name already exists.");
  }

  const std::vector<std::string> joint_names = traction_joint_names(info);
  std::vector<const JointComponent*> wheel_components;
  wheel_components.reserve(joint_names.size());
  for (const std::string& joint_name : joint_names) {
    if (joint_name.empty()) {
      return log_component_error("ComponentManager::register_mobile_base",
                                 "traction joint name must not be empty.");
    }
    JointComponent* joint_component = joint(registry_, joint_name);
    if (joint_component == nullptr) {
      return log_component_error("ComponentManager::register_mobile_base",
                                 "traction joint was not found.");
    }
    if (!validate_mobile_base_joint_binding(*joint_component)) {
      return false;
    }
    wheel_components.push_back(joint_component);
  }

  auto mobile_base = std::make_unique<MobileBaseComponent>(info);
  if (info.type == MobileBaseType::Differential) {
    if (wheel_components.size() != 2U) {
      return log_component_error("ComponentManager::register_mobile_base",
                                 "differential drive requires two wheel components.");
    }
    if (!mobile_base->configure_differential_drive(*wheel_components[0], *wheel_components[1])) {
      return false;
    }
  } else if (info.type == MobileBaseType::Omnidirectional) {
    if (wheel_components.size() != 4U) {
      return log_component_error("ComponentManager::register_mobile_base",
                                 "omnidirectional drive requires four wheel components.");
    }
    if (!mobile_base->configure_omnidirectional_drive(*wheel_components[0], *wheel_components[1],
                                                      *wheel_components[2], *wheel_components[3])) {
      return false;
    }
  }
  if (!mobile_base->bind(model)) {
    return false;
  }
  return registry_.add(std::move(mobile_base));
}

bool ComponentManager::reconfigure_joint(const mjModel& model, const JointInfo& info) {
  if (info.joint.empty()) {
    return log_component_error("ComponentManager::reconfigure_joint",
                               "joint name must not be empty.");
  }

  const JointComponent* existing = joint(registry_, info.joint);
  if (existing == nullptr) {
    return log_component_error("ComponentManager::reconfigure_joint", "joint was not found.");
  }

  const JointInfo previous_info = existing->info();
  auto replacement = std::make_unique<JointComponent>(info);
  if (!replacement->bind(model)) {
    return false;
  }

  if (!registry_.remove(info.joint)) {
    return false;
  }

  if (registry_.add(std::move(replacement))) {
    return true;
  }

  auto restore = std::make_unique<JointComponent>(previous_info);
  if (restore->bind(model)) {
    if (registry_.add(std::move(restore))) {
      return log_component_error(
          "ComponentManager::reconfigure_joint",
          "failed to add replacement joint; restored previous configuration.");
    }
    return log_component_error("ComponentManager::reconfigure_joint",
                               "failed to restore previous joint after replacement add failure.");
  }

  return log_component_error("ComponentManager::reconfigure_joint",
                             "failed to restore previous joint after replacement add failure.");
}

bool ComponentManager::reconfigure_component(const mjModel& model, const ComponentConfig& config) {
  if (const auto* joint_info = std::get_if<JointInfo>(&config)) {
    return reconfigure_joint(model, *joint_info);
  }
  return log_component_error("ComponentManager::reconfigure_component",
                             "only joint reconfiguration is supported.");
}

bool ComponentManager::reset_all(const mjModel& model, mjData& data) {
  for (const auto& [name, joint_component] : registry_.joints()) {
    (void)name;
    if (!joint_component->reset(model, data)) {
      return false;
    }
  }
  for (const auto& [name, camera_component] : registry_.cameras()) {
    (void)name;
    if (!camera_component->reset(model, data)) {
      return false;
    }
  }
  for (const auto& [name, imu_component] : registry_.imus()) {
    (void)name;
    if (!imu_component->reset(model, data)) {
      return false;
    }
  }
  for (const auto& [name, lidar_component] : registry_.lidars()) {
    (void)name;
    if (!lidar_component->reset(model, data)) {
      return false;
    }
  }
  for (const auto& [name, mobile_base_component] : registry_.mobile_bases()) {
    (void)name;
    if (!mobile_base_component->reset(model, data)) {
      return false;
    }
  }
  for (const auto& [name, joint_component] : registry_.joints()) {
    (void)name;
    joint_component->reset_update_schedule();
  }
  for (const auto& [name, camera_component] : registry_.cameras()) {
    (void)name;
    camera_component->reset_update_schedule();
  }
  for (const auto& [name, imu_component] : registry_.imus()) {
    (void)name;
    imu_component->reset_update_schedule();
  }
  for (const auto& [name, lidar_component] : registry_.lidars()) {
    (void)name;
    lidar_component->reset_update_schedule();
  }
  for (const auto& [name, mobile_base_component] : registry_.mobile_bases()) {
    (void)name;
    mobile_base_component->reset_update_schedule();
  }
  return true;
}

bool ComponentManager::update_components(const mjModel& model, const mjData& data,
                                         double simulation_time, std::uint64_t step_count,
                                         CameraRenderer* camera_renderer,
                                         CameraBuffer* camera_buffer) {
  std::vector<JointComponent*> due_joints;
  std::vector<MobileBaseComponent*> due_mobile_bases;
  std::vector<ImuComponent*> due_imus;
  std::vector<LidarComponent*> due_lidars;
  std::vector<CameraComponent*> due_cameras;

  due_joints.reserve(registry_.joints().size());
  due_mobile_bases.reserve(registry_.mobile_bases().size());
  due_imus.reserve(registry_.imus().size());
  due_lidars.reserve(registry_.lidars().size());
  due_cameras.reserve(registry_.cameras().size());

  for (const auto& [name, joint_component] : registry_.joints()) {
    (void)name;
    if (joint_component->should_update(simulation_time)) {
      due_joints.push_back(joint_component);
    }
  }
  for (const auto& [name, mobile_base_component] : registry_.mobile_bases()) {
    (void)name;
    if (mobile_base_component->should_update(simulation_time)) {
      due_mobile_bases.push_back(mobile_base_component);
    }
  }
  for (const auto& [name, imu_component] : registry_.imus()) {
    (void)name;
    if (imu_component->should_update(simulation_time)) {
      due_imus.push_back(imu_component);
    }
  }
  for (const auto& [name, lidar_component] : registry_.lidars()) {
    (void)name;
    if (lidar_component->should_update(simulation_time)) {
      due_lidars.push_back(lidar_component);
    }
  }
  for (const auto& [name, camera_component] : registry_.cameras()) {
    (void)name;
    if (camera_component->should_update(simulation_time)) {
      due_cameras.push_back(camera_component);
    }
  }

  if (!due_cameras.empty()) {
    if (camera_renderer == nullptr || camera_buffer == nullptr) {
      return log_component_error("ComponentManager::update_components",
                                 "camera update requires camera_renderer and camera_buffer.");
    }
    if (!camera_renderer->copy_simulation_data(model, data)) {
      return log_component_error("ComponentManager::update_components",
                                 "failed to copy simulation data for camera rendering.");
    }
  }

  const UpdateContext context{model,           data,         simulation_time, step_count,
                              camera_renderer, camera_buffer};

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
    if (!camera_component->update(context)) {
      return false;
    }
  }
  return true;
}

bool ComponentManager::write_commands(const mjModel& model, mjData& data,
                                      const CommandSnapshot& snapshot) {
  for (const auto& [joint_name, command] : snapshot.joint_commands) {
    JointComponent* joint_component = joint(registry_, joint_name);
    if (joint_component == nullptr) {
      return log_component_error("ComponentManager::write_commands",
                                 "joint command target was not found.");
    }
    if (!joint_component->write(model, data, command)) {
      return false;
    }
  }
  for (const auto& [mobile_base_name, command] : snapshot.mobile_base_commands) {
    MobileBaseComponent* mobile_base_component = mobile_base(registry_, mobile_base_name);
    if (mobile_base_component == nullptr) {
      return log_component_error("ComponentManager::write_commands",
                                 "mobile base command target was not found.");
    }
    if (!mobile_base_component->write(model, data, command)) {
      return false;
    }
  }
  return true;
}

bool ComponentManager::build_state_snapshot(const mjData& data, StateSnapshot& snapshot) const {
  if (!read_joint_states(data, snapshot.joints)) {
    return false;
  }
  if (!read_mobile_base_states(data, snapshot.mobile_bases)) {
    return false;
  }
  if (!read_imu_states(snapshot.imus)) {
    return false;
  }
  return read_lidar_states(snapshot.lidars);
}

bool ComponentManager::read_joint_states(
    const mjData& data, std::unordered_map<std::string, JointState>& states) const {
  std::unordered_map<std::string, JointState> snapshot;
  snapshot.reserve(registry_.joints().size());
  for (const auto& [name, joint_component] : registry_.joints()) {
    JointState state;
    if (!joint_component->read(data, state)) {
      return false;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return true;
}

bool ComponentManager::read_imu_states(std::unordered_map<std::string, ImuState>& states) const {
  std::unordered_map<std::string, ImuState> snapshot;
  snapshot.reserve(registry_.imus().size());
  for (const auto& [name, imu_component] : registry_.imus()) {
    ImuState state;
    if (!imu_component->read(state)) {
      return false;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return true;
}

bool ComponentManager::read_lidar_states(
    std::unordered_map<std::string, LidarState>& states) const {
  std::unordered_map<std::string, LidarState> snapshot;
  snapshot.reserve(registry_.lidars().size());
  for (const auto& [name, lidar_component] : registry_.lidars()) {
    LidarState state;
    if (!lidar_component->read(state)) {
      return false;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return true;
}

bool ComponentManager::read_mobile_base_states(
    const mjData& data, std::unordered_map<std::string, MobileBaseState>& states) const {
  std::unordered_map<std::string, MobileBaseState> snapshot;
  snapshot.reserve(registry_.mobile_bases().size());
  for (const auto& [name, mobile_base_component] : registry_.mobile_bases()) {
    MobileBaseState state;
    if (!mobile_base_component->read(data, state)) {
      return false;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return true;
}

}  // namespace mujoco_simulation
