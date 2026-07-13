#include "mujoco_simulation/component/component_manager.hpp"

#include <utility>
#include <vector>

#include "mujoco_simulation/component/update_context.hpp"

namespace mujoco_simulation {
namespace {

std::vector<std::string> traction_joint_names(const MobileBaseConfig& config) {
  if (config.type == MobileBaseType::Differential) {
    return {config.left_wheel_joint, config.right_wheel_joint};
  }
  if (config.type == MobileBaseType::Omnidirectional) {
    return {config.front_left_joint, config.front_right_joint, config.rear_left_joint,
            config.rear_right_joint};
  }
  return {};
}

ResultCode resolve_wheel_binding(const JointComponent& joint_component,
                                 MobileBaseWheelBinding& wheel) {
  const JointBinding& binding = joint_component.binding();
  if (binding.joint_id < 0 || binding.dof_address < 0) {
    return ResultCode::BindingFailed;
  }

  wheel.joint_name = std::string(joint_component.name());
  wheel.joint = binding;
  return ResultCode::Ok;
}

}  // namespace

ResultCode ComponentManager::build(const mjModel& model, const ComponentConfigList& components) {
  clear();

  for (const ComponentConfig& component : components) {
    if (const auto* joint = std::get_if<JointConfig>(&component)) {
      ResultCode status = register_joint(model, std::make_unique<JointComponent>(*joint));
      if (status != ResultCode::Ok) {
        clear();
        return status;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* imu = std::get_if<ImuConfig>(&component)) {
      ResultCode status = register_imu(model, std::make_unique<ImuComponent>(*imu));
      if (status != ResultCode::Ok) {
        clear();
        return status;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* lidar = std::get_if<LidarConfig>(&component)) {
      ResultCode status = register_lidar(model, std::make_unique<LidarComponent>(*lidar));
      if (status != ResultCode::Ok) {
        clear();
        return status;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* mobile_base = std::get_if<MobileBaseConfig>(&component)) {
      ResultCode status = register_mobile_base(model, *mobile_base);
      if (status != ResultCode::Ok) {
        clear();
        return status;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* camera = std::get_if<CameraConfig>(&component)) {
      ResultCode status = register_camera(model, std::make_unique<CameraComponent>(*camera));
      if (status != ResultCode::Ok) {
        clear();
        return status;
      }
    }
  }

  return ResultCode::Ok;
}

void ComponentManager::clear() { registry_.clear(); }

ResultCode ComponentManager::register_joint(const mjModel& model,
                                            std::unique_ptr<JointComponent> joint) {
  if (joint == nullptr) {
    return ResultCode::InvalidArgument;
  }

  ResultCode status = joint->bind(model);
  if (status != ResultCode::Ok) {
    return status;
  }

  JointComponent* joint_ptr = joint.get();
  status = registry_.add(std::move(joint));
  if (status != ResultCode::Ok) {
    return status;
  }
  (void)joint_ptr;
  return ResultCode::Ok;
}

ResultCode ComponentManager::register_camera(const mjModel& model,
                                             std::unique_ptr<CameraComponent> camera) {
  if (camera == nullptr) {
    return ResultCode::InvalidArgument;
  }

  ResultCode status = camera->bind(model);
  if (status != ResultCode::Ok) {
    return status;
  }

  status = registry_.add(std::move(camera));
  return ResultCode::Ok;
}

ResultCode ComponentManager::register_imu(const mjModel& model, std::unique_ptr<ImuComponent> imu) {
  if (imu == nullptr) {
    return ResultCode::InvalidArgument;
  }

  ResultCode status = imu->bind(model);
  if (status != ResultCode::Ok) {
    return status;
  }

  status = registry_.add(std::move(imu));
  return ResultCode::Ok;
}

ResultCode ComponentManager::register_lidar(const mjModel& model,
                                            std::unique_ptr<LidarComponent> lidar) {
  if (lidar == nullptr) {
    return ResultCode::InvalidArgument;
  }

  ResultCode status = lidar->bind(model);
  if (status != ResultCode::Ok) {
    return status;
  }

  status = registry_.add(std::move(lidar));
  return ResultCode::Ok;
}

ResultCode ComponentManager::register_mobile_base(const mjModel& model,
                                                  const MobileBaseConfig& config) {
  if (config.name.empty()) {
    return ResultCode::InvalidArgument;
  }
  if (has_mobile_base(config.name)) {
    return ResultCode::AlreadyExists;
  }

  const std::vector<std::string> joint_names = traction_joint_names(config);
  MobileBaseBinding binding;
  if (config.type == MobileBaseType::Differential) {
    binding.differential.emplace();
  } else if (config.type == MobileBaseType::Omnidirectional) {
    binding.omnidirectional.emplace();
  }
  for (const std::string& joint_name : joint_names) {
    if (joint_name.empty()) {
      return ResultCode::InvalidArgument;
    }
    JointComponent* joint_component = joint(joint_name);
    if (joint_component == nullptr) {
      return ResultCode::NotFound;
    }
    MobileBaseWheelBinding* wheel = nullptr;
    if (binding.differential.has_value()) {
      if (joint_name == config.left_wheel_joint) {
        wheel = &binding.differential->left_wheel;
      } else if (joint_name == config.right_wheel_joint) {
        wheel = &binding.differential->right_wheel;
      }
    } else if (binding.omnidirectional.has_value()) {
      if (joint_name == config.front_left_joint) {
        wheel = &binding.omnidirectional->front_left;
      } else if (joint_name == config.front_right_joint) {
        wheel = &binding.omnidirectional->front_right;
      } else if (joint_name == config.rear_left_joint) {
        wheel = &binding.omnidirectional->rear_left;
      } else if (joint_name == config.rear_right_joint) {
        wheel = &binding.omnidirectional->rear_right;
      }
    }
    if (wheel == nullptr) {
      return ResultCode::Internal;
    }
    ResultCode status = resolve_wheel_binding(*joint_component, *wheel);
    if (status != ResultCode::Ok) {
      return status;
    }
  }

  auto mobile_base = std::make_unique<MobileBaseComponent>(config, std::move(binding));
  ResultCode status = mobile_base->bind(model);
  if (status != ResultCode::Ok) {
    return status;
  }

  MobileBaseComponent* mobile_base_ptr = mobile_base.get();
  status = registry_.add(std::move(mobile_base));
  if (status != ResultCode::Ok) {
    return status;
  }
  (void)mobile_base_ptr;
  return ResultCode::Ok;
}

ResultCode ComponentManager::reconfigure_joint(const mjModel& model, const JointConfig& config) {
  if (config.name.empty()) {
    return ResultCode::InvalidArgument;
  }

  const JointComponent* existing = joint(config.name);
  if (existing == nullptr) {
    return ResultCode::NotFound;
  }

  const JointConfig previous_config = existing->config();
  auto replacement = std::make_unique<JointComponent>(config);
  ResultCode status = replacement->bind(model);
  if (status != ResultCode::Ok) {
    return status;
  }

  status = registry_.remove(config.name);
  if (status != ResultCode::Ok) {
    return status;
  }

  status = registry_.add(std::move(replacement));
  if (status == ResultCode::Ok) {
    return ResultCode::Ok;
  }

  auto restore = std::make_unique<JointComponent>(previous_config);
  const ResultCode restore_bind_status = restore->bind(model);
  if (restore_bind_status == ResultCode::Ok) {
    const ResultCode restore_add_status = registry_.add(std::move(restore));
    if (restore_add_status == ResultCode::Ok) {
      return status;
    }
    return ResultCode::Internal;
  }

  return ResultCode::Internal;
}

ResultCode ComponentManager::reconfigure_component(const mjModel& model,
                                                   const ComponentConfig& config) {
  if (const auto* joint_config = std::get_if<JointConfig>(&config)) {
    return reconfigure_joint(model, *joint_config);
  }
  return ResultCode::FailedPrecondition;
}

ResultCode ComponentManager::unregister_joint(std::string name) {
  if (!registry_.has_joint(name)) {
    return ResultCode::NotFound;
  }
  return registry_.remove(name);
}

ResultCode ComponentManager::unregister_camera(std::string name) {
  if (!registry_.has_camera(name)) {
    return ResultCode::NotFound;
  }
  return registry_.remove(name);
}

ResultCode ComponentManager::unregister_imu(std::string name) {
  if (!registry_.has_imu(name)) {
    return ResultCode::NotFound;
  }
  return registry_.remove(name);
}

ResultCode ComponentManager::unregister_lidar(std::string name) {
  if (!registry_.has_lidar(name)) {
    return ResultCode::NotFound;
  }
  return registry_.remove(name);
}

ResultCode ComponentManager::unregister_mobile_base(std::string name) {
  if (!registry_.has_mobile_base(name)) {
    return ResultCode::NotFound;
  }
  return registry_.remove(name);
}

bool ComponentManager::has_joint(std::string name) const { return registry_.has_joint(name); }

bool ComponentManager::has_camera(std::string name) const { return registry_.has_camera(name); }

bool ComponentManager::has_imu(std::string name) const { return registry_.has_imu(name); }

bool ComponentManager::has_lidar(std::string name) const { return registry_.has_lidar(name); }

bool ComponentManager::has_mobile_base(std::string name) const {
  return registry_.has_mobile_base(name);
}

CommandInterfaceType ComponentManager::joint_command_mode(std::string name) const {
  const JointComponent* component = joint(name);
  return component == nullptr ? CommandInterfaceType::None : component->config().command_mode;
}

JointComponent* ComponentManager::joint(std::string name) { return registry_.joint(name); }

const JointComponent* ComponentManager::joint(std::string name) const {
  return registry_.joint(name);
}

CameraComponent* ComponentManager::camera(std::string name) { return registry_.camera(name); }

const CameraComponent* ComponentManager::camera(std::string name) const {
  return registry_.camera(name);
}

ImuComponent* ComponentManager::imu(std::string name) { return registry_.imu(name); }

const ImuComponent* ComponentManager::imu(std::string name) const { return registry_.imu(name); }

LidarComponent* ComponentManager::lidar(std::string name) { return registry_.lidar(name); }

const LidarComponent* ComponentManager::lidar(std::string name) const {
  return registry_.lidar(name);
}

MobileBaseComponent* ComponentManager::mobile_base(std::string name) {
  return registry_.mobile_base(name);
}

const MobileBaseComponent* ComponentManager::mobile_base(std::string name) const {
  return registry_.mobile_base(name);
}

ResultCode ComponentManager::reset_all(const mjModel& model, mjData& data) {
  for (const auto& [name, joint_component] : registry_.joints()) {
    (void)name;
    ResultCode status = joint_component->reset(model, data);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  for (const auto& [name, camera_component] : registry_.cameras()) {
    (void)name;
    ResultCode status = camera_component->reset(model, data);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  for (const auto& [name, imu_component] : registry_.imus()) {
    (void)name;
    ResultCode status = imu_component->reset(model, data);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  for (const auto& [name, lidar_component] : registry_.lidars()) {
    (void)name;
    ResultCode status = lidar_component->reset(model, data);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  for (const auto& [name, mobile_base_component] : registry_.mobile_bases()) {
    (void)name;
    ResultCode status = mobile_base_component->reset(model, data);
    if (status != ResultCode::Ok) {
      return status;
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
  return ResultCode::Ok;
}

ResultCode ComponentManager::update_components(const mjModel& model, const mjData& data,
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
      return ResultCode::FailedPrecondition;
    }
    ResultCode status = camera_renderer->copy_simulation_data(model, data);
    if (status != ResultCode::Ok) {
      return status;
    }
  }

  const UpdateContext context{model,           data,         simulation_time, step_count,
                              camera_renderer, camera_buffer};

  for (JointComponent* joint_component : due_joints) {
    ResultCode status = joint_component->update(context);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  for (MobileBaseComponent* mobile_base_component : due_mobile_bases) {
    ResultCode status = mobile_base_component->update(context);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  for (ImuComponent* imu_component : due_imus) {
    ResultCode status = imu_component->update(context);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  for (LidarComponent* lidar_component : due_lidars) {
    ResultCode status = lidar_component->update(context);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  for (CameraComponent* camera_component : due_cameras) {
    ResultCode status = camera_component->update(context);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  return ResultCode::Ok;
}

ResultCode ComponentManager::write_commands(const mjModel& model, mjData& data,
                                            const CommandSnapshot& snapshot) {
  for (const auto& [joint_name, command] : snapshot.joint_commands) {
    JointComponent* joint_component = joint(joint_name);
    if (joint_component == nullptr) {
      return ResultCode::NotFound;
    }
    ResultCode status = joint_component->write(model, data, command);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  for (const auto& [mobile_base_name, command] : snapshot.mobile_base_commands) {
    MobileBaseComponent* mobile_base_component = mobile_base(mobile_base_name);
    if (mobile_base_component == nullptr) {
      return ResultCode::NotFound;
    }
    ResultCode status = mobile_base_component->write(model, data, command);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  return ResultCode::Ok;
}

ResultCode ComponentManager::read_joint(const mjData& data, std::string name,
                                        JointState& state) const {
  const JointComponent* joint_component = joint(name);
  if (joint_component == nullptr) {
    return ResultCode::NotFound;
  }
  return joint_component->read(data, state);
}

ResultCode ComponentManager::read_imu(std::string name, ImuState& state) const {
  const ImuComponent* imu_component = imu(name);
  if (imu_component == nullptr) {
    return ResultCode::NotFound;
  }
  return imu_component->read(state);
}

ResultCode ComponentManager::read_lidar(std::string name, LidarState& state) const {
  const LidarComponent* lidar_component = lidar(name);
  if (lidar_component == nullptr) {
    return ResultCode::NotFound;
  }
  return lidar_component->read(state);
}

ResultCode ComponentManager::read_mobile_base(const mjData& data, std::string name,
                                              MobileBaseState& state) {
  MobileBaseComponent* mobile_base_component = mobile_base(name);
  if (mobile_base_component == nullptr) {
    return ResultCode::NotFound;
  }
  return mobile_base_component->read(data, state);
}

ResultCode ComponentManager::read_states(const mjData& data, StateSnapshot& snapshot) {
  ResultCode status = read_joint_states(data, snapshot.joints);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = read_mobile_base_states(data, snapshot.mobile_bases);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = read_imu_states(snapshot.imus);
  if (status != ResultCode::Ok) {
    return status;
  }
  return read_lidar_states(snapshot.lidars);
}

ResultCode ComponentManager::read_joint_states(
    const mjData& data, std::unordered_map<std::string, JointState>& states) const {
  std::unordered_map<std::string, JointState> snapshot;
  snapshot.reserve(registry_.joints().size());
  for (const auto& [name, joint_component] : registry_.joints()) {
    JointState state;
    ResultCode status = joint_component->read(data, state);
    if (status != ResultCode::Ok) {
      return status;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return ResultCode::Ok;
}

ResultCode ComponentManager::read_imu_states(
    std::unordered_map<std::string, ImuState>& states) const {
  std::unordered_map<std::string, ImuState> snapshot;
  snapshot.reserve(registry_.imus().size());
  for (const auto& [name, imu_component] : registry_.imus()) {
    ImuState state;
    ResultCode status = imu_component->read(state);
    if (status != ResultCode::Ok) {
      return status;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return ResultCode::Ok;
}

ResultCode ComponentManager::read_lidar_states(
    std::unordered_map<std::string, LidarState>& states) const {
  std::unordered_map<std::string, LidarState> snapshot;
  snapshot.reserve(registry_.lidars().size());
  for (const auto& [name, lidar_component] : registry_.lidars()) {
    LidarState state;
    ResultCode status = lidar_component->read(state);
    if (status != ResultCode::Ok) {
      return status;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return ResultCode::Ok;
}

ResultCode ComponentManager::read_mobile_base_states(
    const mjData& data, std::unordered_map<std::string, MobileBaseState>& states) {
  std::unordered_map<std::string, MobileBaseState> snapshot;
  snapshot.reserve(registry_.mobile_bases().size());
  for (const auto& [name, mobile_base_component] : registry_.mobile_bases()) {
    MobileBaseState state;
    ResultCode status = mobile_base_component->read(data, state);
    if (status != ResultCode::Ok) {
      return status;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return ResultCode::Ok;
}

}  // namespace mujoco_simulation
