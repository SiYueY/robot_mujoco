#include "mujoco_simulation/component/component_manager.hpp"

#include <utility>

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

Status resolve_wheel_binding(const JointComponent& joint_component, MobileBaseWheelBinding& wheel) {
  const JointBinding& binding = joint_component.binding();
  if (binding.joint_id < 0 || binding.dof_address < 0) {
    return Status::binding_failed("MobileBase wheel '" + std::string(joint_component.name()) +
                                  "' has an invalid MuJoCo joint binding.");
  }

  wheel.joint_name = std::string(joint_component.name());
  wheel.joint = binding;
  return Status::Ok();
}

}  // namespace

Status ComponentManager::build(const mjModel& model, const ComponentConfigList& components) {
  clear();

  for (const ComponentConfig& component : components) {
    if (const auto* joint = std::get_if<JointConfig>(&component)) {
      Status status = register_joint(model, std::make_unique<JointComponent>(*joint));
      if (!status.ok()) {
        clear();
        return status;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* imu = std::get_if<ImuConfig>(&component)) {
      Status status = register_imu(model, std::make_unique<ImuComponent>(*imu));
      if (!status.ok()) {
        clear();
        return status;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* lidar = std::get_if<LidarConfig>(&component)) {
      Status status = register_lidar(model, std::make_unique<LidarComponent>(*lidar));
      if (!status.ok()) {
        clear();
        return status;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* mobile_base = std::get_if<MobileBaseConfig>(&component)) {
      Status status = register_mobile_base(model, *mobile_base);
      if (!status.ok()) {
        clear();
        return status;
      }
    }
  }
  for (const ComponentConfig& component : components) {
    if (const auto* camera = std::get_if<CameraConfig>(&component)) {
      Status status = register_camera(model, std::make_unique<CameraComponent>(*camera));
      if (!status.ok()) {
        clear();
        return status;
      }
    }
  }

  return Status::Ok();
}

void ComponentManager::clear() {
  sensor_scheduler_.reset();
  registry_.clear();
}

Status ComponentManager::register_joint(const mjModel& model,
                                        std::unique_ptr<JointComponent> joint) {
  if (joint == nullptr) {
    return Status::invalid_argument("JointComponent must not be null.");
  }

  Status status = joint->bind(model);
  if (!status.ok()) {
    return status;
  }

  JointComponent* joint_ptr = joint.get();
  status = registry_.add(std::move(joint));
  if (!status.ok()) {
    return status;
  }
  (void)joint_ptr;
  return Status::Ok();
}

Status ComponentManager::register_camera(const mjModel& model,
                                         std::unique_ptr<CameraComponent> camera) {
  if (camera == nullptr) {
    return Status::invalid_argument("CameraComponent must not be null.");
  }
  if (model.opt.timestep <= 0.0) {
    return Status::invalid_argument(
        "MuJoCo physics timestep must be positive for camera registration.");
  }

  Status status = camera->bind(model);
  if (!status.ok()) {
    return status;
  }
  status = sensor_scheduler_.register_sensor(camera->name(), camera->update_rate(),
                                             1.0 / model.opt.timestep);
  if (!status.ok()) {
    return status;
  }

  CameraComponent* camera_ptr = camera.get();
  status = registry_.add(std::move(camera));
  if (!status.ok()) {
    (void)sensor_scheduler_.unregister_sensor(camera_ptr->name());
    return status;
  }
  (void)camera_ptr;
  return Status::Ok();
}

Status ComponentManager::register_imu(const mjModel& model, std::unique_ptr<ImuComponent> imu) {
  if (imu == nullptr) {
    return Status::invalid_argument("ImuComponent must not be null.");
  }
  if (model.opt.timestep <= 0.0) {
    return Status::invalid_argument(
        "MuJoCo physics timestep must be positive for IMU registration.");
  }

  Status status = imu->bind(model);
  if (!status.ok()) {
    return status;
  }
  status =
      sensor_scheduler_.register_sensor(imu->name(), imu->update_rate(), 1.0 / model.opt.timestep);
  if (!status.ok()) {
    return status;
  }

  ImuComponent* imu_ptr = imu.get();
  status = registry_.add(std::move(imu));
  if (!status.ok()) {
    (void)sensor_scheduler_.unregister_sensor(imu_ptr->name());
    return status;
  }
  (void)imu_ptr;
  return Status::Ok();
}

Status ComponentManager::register_lidar(const mjModel& model,
                                        std::unique_ptr<LidarComponent> lidar) {
  if (lidar == nullptr) {
    return Status::invalid_argument("LidarComponent must not be null.");
  }
  if (model.opt.timestep <= 0.0) {
    return Status::invalid_argument(
        "MuJoCo physics timestep must be positive for lidar registration.");
  }

  Status status = lidar->bind(model);
  if (!status.ok()) {
    return status;
  }
  status = sensor_scheduler_.register_sensor(lidar->name(), lidar->update_rate(),
                                             1.0 / model.opt.timestep);
  if (!status.ok()) {
    return status;
  }

  LidarComponent* lidar_ptr = lidar.get();
  status = registry_.add(std::move(lidar));
  if (!status.ok()) {
    (void)sensor_scheduler_.unregister_sensor(lidar_ptr->name());
    return status;
  }
  (void)lidar_ptr;
  return Status::Ok();
}

Status ComponentManager::register_mobile_base(const mjModel& model,
                                              const MobileBaseConfig& config) {
  if (config.name.empty()) {
    return Status::invalid_argument("MobileBase name must not be empty.");
  }
  if (has_mobile_base(config.name)) {
    return Status::already_exists("MobileBase already registered: " + config.name);
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
      return Status::invalid_argument("MobileBase '" + config.name +
                                      "' requires all traction joint names to be configured.");
    }
    JointComponent* joint_component = joint(joint_name);
    if (joint_component == nullptr) {
      return Status::not_found("MobileBase traction joint not found: " + joint_name);
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
      return Status::internal("MobileBase '" + config.name + "' could not map traction joint '" +
                              joint_name + "' to a semantic wheel binding.");
    }
    Status status = resolve_wheel_binding(*joint_component, *wheel);
    if (!status.ok()) {
      return status;
    }
  }

  auto mobile_base = std::make_unique<MobileBaseComponent>(config, std::move(binding));
  Status status = mobile_base->bind(model);
  if (!status.ok()) {
    return status;
  }

  MobileBaseComponent* mobile_base_ptr = mobile_base.get();
  status = registry_.add(std::move(mobile_base));
  if (!status.ok()) {
    return status;
  }
  (void)mobile_base_ptr;
  return Status::Ok();
}

Status ComponentManager::reconfigure_joint(const mjModel& model, const JointConfig& config) {
  if (config.name.empty()) {
    return Status::invalid_argument("JointComponent name must not be empty.");
  }

  const JointComponent* existing = joint(config.name);
  if (existing == nullptr) {
    return Status::not_found("JointComponent not found: " + config.name);
  }

  const JointConfig previous_config = existing->config();
  auto replacement = std::make_unique<JointComponent>(config);
  Status status = replacement->bind(model);
  if (!status.ok()) {
    return status;
  }

  status = registry_.remove(config.name);
  if (!status.ok()) {
    return status;
  }

  status = registry_.add(std::move(replacement));
  if (status.ok()) {
    return Status::Ok();
  }

  auto restore = std::make_unique<JointComponent>(previous_config);
  const Status restore_bind_status = restore->bind(model);
  if (restore_bind_status.ok()) {
    const Status restore_add_status = registry_.add(std::move(restore));
    if (restore_add_status.ok()) {
      return status;
    }
    return Status::internal("Failed to restore JointComponent '" + config.name +
                            "' after reconfiguration failure: " + restore_add_status.message());
  }

  return Status::internal("Failed to restore JointComponent '" + config.name +
                          "' after reconfiguration failure: " + restore_bind_status.message());
}

Status ComponentManager::reconfigure_component(const mjModel& model,
                                               const ComponentConfig& config) {
  if (const auto* joint_config = std::get_if<JointConfig>(&config)) {
    return reconfigure_joint(model, *joint_config);
  }
  return Status::failed_precondition("Runtime reconfiguration is only supported for joints: " +
                                     std::string(component_config_name(config)));
}

Status ComponentManager::unregister_joint(std::string_view name) {
  if (!registry_.has_joint(name)) {
    return Status::not_found("JointComponent not found: " + std::string(name));
  }
  return registry_.remove(name);
}

Status ComponentManager::unregister_camera(std::string_view name) {
  if (!registry_.has_camera(name)) {
    return Status::not_found("CameraComponent not found: " + std::string(name));
  }
  const Status scheduler_status = sensor_scheduler_.unregister_sensor(name);
  if (!scheduler_status.ok()) {
    return scheduler_status;
  }
  return registry_.remove(name);
}

Status ComponentManager::unregister_imu(std::string_view name) {
  if (!registry_.has_imu(name)) {
    return Status::not_found("ImuComponent not found: " + std::string(name));
  }
  const Status scheduler_status = sensor_scheduler_.unregister_sensor(name);
  if (!scheduler_status.ok()) {
    return scheduler_status;
  }
  return registry_.remove(name);
}

Status ComponentManager::unregister_lidar(std::string_view name) {
  if (!registry_.has_lidar(name)) {
    return Status::not_found("LidarComponent not found: " + std::string(name));
  }
  const Status scheduler_status = sensor_scheduler_.unregister_sensor(name);
  if (!scheduler_status.ok()) {
    return scheduler_status;
  }
  return registry_.remove(name);
}

Status ComponentManager::unregister_mobile_base(std::string_view name) {
  if (!registry_.has_mobile_base(name)) {
    return Status::not_found("MobileBase not found: " + std::string(name));
  }
  return registry_.remove(name);
}

bool ComponentManager::has_joint(std::string_view name) const { return registry_.has_joint(name); }

bool ComponentManager::has_camera(std::string_view name) const {
  return registry_.has_camera(name);
}

bool ComponentManager::has_imu(std::string_view name) const { return registry_.has_imu(name); }

bool ComponentManager::has_lidar(std::string_view name) const { return registry_.has_lidar(name); }

bool ComponentManager::has_mobile_base(std::string_view name) const {
  return registry_.has_mobile_base(name);
}

CommandInterfaceType ComponentManager::joint_command_mode(std::string_view name) const {
  const JointComponent* component = joint(name);
  return component == nullptr ? CommandInterfaceType::None : component->config().command_mode;
}

JointComponent* ComponentManager::joint(std::string_view name) { return registry_.joint(name); }

const JointComponent* ComponentManager::joint(std::string_view name) const {
  return registry_.joint(name);
}

CameraComponent* ComponentManager::camera(std::string_view name) { return registry_.camera(name); }

const CameraComponent* ComponentManager::camera(std::string_view name) const {
  return registry_.camera(name);
}

ImuComponent* ComponentManager::imu(std::string_view name) { return registry_.imu(name); }

const ImuComponent* ComponentManager::imu(std::string_view name) const {
  return registry_.imu(name);
}

LidarComponent* ComponentManager::lidar(std::string_view name) { return registry_.lidar(name); }

const LidarComponent* ComponentManager::lidar(std::string_view name) const {
  return registry_.lidar(name);
}

MobileBaseComponent* ComponentManager::mobile_base(std::string_view name) {
  return registry_.mobile_base(name);
}

const MobileBaseComponent* ComponentManager::mobile_base(std::string_view name) const {
  return registry_.mobile_base(name);
}

Status ComponentManager::reset_all(const mjModel& model, mjData& data) {
  for (const auto& [name, joint_component] : registry_.joints()) {
    (void)name;
    Status status = joint_component->reset(model, data);
    if (!status.ok()) {
      return status;
    }
  }
  for (const auto& [name, camera_component] : registry_.cameras()) {
    (void)name;
    Status status = camera_component->reset(model, data);
    if (!status.ok()) {
      return status;
    }
  }
  for (const auto& [name, imu_component] : registry_.imus()) {
    (void)name;
    Status status = imu_component->reset(model, data);
    if (!status.ok()) {
      return status;
    }
  }
  for (const auto& [name, lidar_component] : registry_.lidars()) {
    (void)name;
    Status status = lidar_component->reset(model, data);
    if (!status.ok()) {
      return status;
    }
  }
  for (const auto& [name, mobile_base_component] : registry_.mobile_bases()) {
    (void)name;
    Status status = mobile_base_component->reset(model, data);
    if (!status.ok()) {
      return status;
    }
  }
  sensor_scheduler_.reset();
  return Status::Ok();
}

Status ComponentManager::sample_due_sensors(const mjModel& model, const mjData& data,
                                            double simulation_time, std::uint64_t step_count,
                                            CameraRenderer* camera_renderer,
                                            CameraBuffer* camera_buffer) {
  bool any_due_camera = false;
  for (const auto& [name, camera_component] : registry_.cameras()) {
    (void)camera_component;
    if (sensor_scheduler_.is_due(name, simulation_time)) {
      any_due_camera = true;
      break;
    }
  }
  if (any_due_camera) {
    if (camera_renderer == nullptr || camera_buffer == nullptr) {
      return Status::failed_precondition("Camera runtime resources are not initialized.");
    }
    Status status = camera_renderer->copy_simulation_data(model, data);
    if (!status.ok()) {
      return status;
    }
  }

  const SensorSampleContext context{model,           data,         simulation_time, step_count,
                                    camera_renderer, camera_buffer};

  for (const auto& [name, imu_component] : registry_.imus()) {
    if (!sensor_scheduler_.is_due(name, simulation_time)) {
      continue;
    }
    Status status = imu_component->sample(context);
    if (!status.ok()) {
      return status;
    }
    status = sensor_scheduler_.mark_sampled(name, simulation_time);
    if (!status.ok()) {
      return status;
    }
  }
  for (const auto& [name, lidar_component] : registry_.lidars()) {
    if (!sensor_scheduler_.is_due(name, simulation_time)) {
      continue;
    }
    Status status = lidar_component->sample(context);
    if (!status.ok()) {
      return status;
    }
    status = sensor_scheduler_.mark_sampled(name, simulation_time);
    if (!status.ok()) {
      return status;
    }
  }
  for (const auto& [name, camera_component] : registry_.cameras()) {
    if (!sensor_scheduler_.is_due(name, simulation_time)) {
      continue;
    }
    Status status = camera_component->sample(context);
    if (!status.ok()) {
      return status;
    }
    status = sensor_scheduler_.mark_sampled(name, simulation_time);
    if (!status.ok()) {
      return status;
    }
  }
  return Status::Ok();
}

Status ComponentManager::write_commands(const mjModel& model, mjData& data,
                                        const CommandSnapshot& snapshot) {
  for (const auto& [joint_name, command] : snapshot.joint_commands) {
    JointComponent* joint_component = joint(joint_name);
    if (joint_component == nullptr) {
      return Status::not_found("JointComponent not found: " + joint_name);
    }
    Status status = joint_component->write(model, data, command);
    if (!status.ok()) {
      return status;
    }
  }
  for (const auto& [mobile_base_name, command] : snapshot.mobile_base_commands) {
    MobileBaseComponent* mobile_base_component = mobile_base(mobile_base_name);
    if (mobile_base_component == nullptr) {
      return Status::not_found("MobileBase not found: " + mobile_base_name);
    }
    Status status = mobile_base_component->write(model, data, command);
    if (!status.ok()) {
      return status;
    }
  }
  return Status::Ok();
}

Status ComponentManager::read_joint(const mjData& data, std::string_view name,
                                    JointState& state) const {
  const JointComponent* joint_component = joint(name);
  if (joint_component == nullptr) {
    return Status::not_found("JointComponent not found: " + std::string(name));
  }
  return joint_component->read(data, state);
}

Status ComponentManager::read_imu(std::string_view name, ImuSample& state) const {
  const ImuComponent* imu_component = imu(name);
  if (imu_component == nullptr) {
    return Status::not_found("ImuComponent not found: " + std::string(name));
  }
  return imu_component->read(state);
}

Status ComponentManager::read_lidar(std::string_view name, LidarSample& state) const {
  const LidarComponent* lidar_component = lidar(name);
  if (lidar_component == nullptr) {
    return Status::not_found("LidarComponent not found: " + std::string(name));
  }
  return lidar_component->read(state);
}

Status ComponentManager::read_mobile_base(const mjData& data, std::string_view name,
                                          MobileBaseState& state) {
  MobileBaseComponent* mobile_base_component = mobile_base(name);
  if (mobile_base_component == nullptr) {
    return Status::not_found("MobileBase not found: " + std::string(name));
  }
  return mobile_base_component->read(data, state);
}

Status ComponentManager::read_states(const mjData& data, SimulationStateSnapshot& snapshot) {
  Status status = read_joint_states(data, snapshot.joints);
  if (!status.ok()) {
    return status;
  }
  status = read_mobile_base_states(data, snapshot.mobile_bases);
  if (!status.ok()) {
    return status;
  }
  status = read_imu_states(snapshot.imus);
  if (!status.ok()) {
    return status;
  }
  return read_lidar_states(snapshot.lidars);
}

Status ComponentManager::read_joint_states(
    const mjData& data, std::unordered_map<std::string, JointState>& states) const {
  std::unordered_map<std::string, JointState> snapshot;
  snapshot.reserve(registry_.joints().size());
  for (const auto& [name, joint_component] : registry_.joints()) {
    JointState state;
    Status status = joint_component->read(data, state);
    if (!status.ok()) {
      return status;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return Status::Ok();
}

Status ComponentManager::read_imu_states(std::unordered_map<std::string, ImuSample>& states) const {
  std::unordered_map<std::string, ImuSample> snapshot;
  snapshot.reserve(registry_.imus().size());
  for (const auto& [name, imu_component] : registry_.imus()) {
    ImuSample state;
    Status status = imu_component->read(state);
    if (!status.ok()) {
      return status;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return Status::Ok();
}

Status ComponentManager::read_lidar_states(
    std::unordered_map<std::string, LidarSample>& states) const {
  std::unordered_map<std::string, LidarSample> snapshot;
  snapshot.reserve(registry_.lidars().size());
  for (const auto& [name, lidar_component] : registry_.lidars()) {
    LidarSample state;
    Status status = lidar_component->read(state);
    if (!status.ok()) {
      return status;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return Status::Ok();
}

Status ComponentManager::read_mobile_base_states(
    const mjData& data, std::unordered_map<std::string, MobileBaseState>& states) {
  std::unordered_map<std::string, MobileBaseState> snapshot;
  snapshot.reserve(registry_.mobile_bases().size());
  for (const auto& [name, mobile_base_component] : registry_.mobile_bases()) {
    MobileBaseState state;
    Status status = mobile_base_component->read(data, state);
    if (!status.ok()) {
      return status;
    }
    snapshot.emplace(name, std::move(state));
  }
  states = std::move(snapshot);
  return Status::Ok();
}

}  // namespace mujoco_simulation
