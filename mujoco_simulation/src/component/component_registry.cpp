#include "mujoco_simulation/component/component_registry.hpp"

namespace mujoco_simulation {

Status ComponentRegistry::add(std::unique_ptr<SimulationComponent> component) {
  if (component == nullptr) {
    return Status::invalid_argument("SimulationComponent must not be null.");
  }

  const std::string component_name(component->name());
  if (component_name.empty()) {
    return Status::invalid_argument("SimulationComponent name must not be empty.");
  }
  if (components_.find(component_name) != components_.end()) {
    return Status::already_exists("SimulationComponent already registered: " + component_name);
  }

  index_component(*component, component_name);
  components_.emplace(component_name, std::move(component));
  return Status::Ok();
}

Status ComponentRegistry::remove(std::string_view name) {
  const auto it = components_.find(std::string(name));
  if (it == components_.end()) {
    return Status::not_found("SimulationComponent not found: " + std::string(name));
  }

  unindex_component(*it->second, name);
  components_.erase(it);
  return Status::Ok();
}

void ComponentRegistry::clear() {
  components_.clear();
  joints_.clear();
  cameras_.clear();
  imus_.clear();
  lidars_.clear();
  mobile_bases_.clear();
}

bool ComponentRegistry::has_joint(std::string_view name) const {
  return joints_.find(std::string(name)) != joints_.end();
}

bool ComponentRegistry::has_camera(std::string_view name) const {
  return cameras_.find(std::string(name)) != cameras_.end();
}

bool ComponentRegistry::has_imu(std::string_view name) const {
  return imus_.find(std::string(name)) != imus_.end();
}

bool ComponentRegistry::has_lidar(std::string_view name) const {
  return lidars_.find(std::string(name)) != lidars_.end();
}

bool ComponentRegistry::has_mobile_base(std::string_view name) const {
  return mobile_bases_.find(std::string(name)) != mobile_bases_.end();
}

JointComponent* ComponentRegistry::joint(std::string_view name) {
  const auto it = joints_.find(std::string(name));
  return it == joints_.end() ? nullptr : it->second;
}

const JointComponent* ComponentRegistry::joint(std::string_view name) const {
  const auto it = joints_.find(std::string(name));
  return it == joints_.end() ? nullptr : it->second;
}

CameraComponent* ComponentRegistry::camera(std::string_view name) {
  const auto it = cameras_.find(std::string(name));
  return it == cameras_.end() ? nullptr : it->second;
}

const CameraComponent* ComponentRegistry::camera(std::string_view name) const {
  const auto it = cameras_.find(std::string(name));
  return it == cameras_.end() ? nullptr : it->second;
}

ImuComponent* ComponentRegistry::imu(std::string_view name) {
  const auto it = imus_.find(std::string(name));
  return it == imus_.end() ? nullptr : it->second;
}

const ImuComponent* ComponentRegistry::imu(std::string_view name) const {
  const auto it = imus_.find(std::string(name));
  return it == imus_.end() ? nullptr : it->second;
}

LidarComponent* ComponentRegistry::lidar(std::string_view name) {
  const auto it = lidars_.find(std::string(name));
  return it == lidars_.end() ? nullptr : it->second;
}

const LidarComponent* ComponentRegistry::lidar(std::string_view name) const {
  const auto it = lidars_.find(std::string(name));
  return it == lidars_.end() ? nullptr : it->second;
}

MobileBaseComponent* ComponentRegistry::mobile_base(std::string_view name) {
  const auto it = mobile_bases_.find(std::string(name));
  return it == mobile_bases_.end() ? nullptr : it->second;
}

const MobileBaseComponent* ComponentRegistry::mobile_base(std::string_view name) const {
  const auto it = mobile_bases_.find(std::string(name));
  return it == mobile_bases_.end() ? nullptr : it->second;
}

const std::unordered_map<std::string, JointComponent*>& ComponentRegistry::joints() const noexcept {
  return joints_;
}

const std::unordered_map<std::string, CameraComponent*>& ComponentRegistry::cameras()
    const noexcept {
  return cameras_;
}

const std::unordered_map<std::string, ImuComponent*>& ComponentRegistry::imus() const noexcept {
  return imus_;
}

const std::unordered_map<std::string, LidarComponent*>& ComponentRegistry::lidars() const noexcept {
  return lidars_;
}

const std::unordered_map<std::string, MobileBaseComponent*>& ComponentRegistry::mobile_bases()
    const noexcept {
  return mobile_bases_;
}

SimulationComponent* ComponentRegistry::find(std::string_view name) {
  const auto it = components_.find(std::string(name));
  return it == components_.end() ? nullptr : it->second.get();
}

const SimulationComponent* ComponentRegistry::find(std::string_view name) const {
  const auto it = components_.find(std::string(name));
  return it == components_.end() ? nullptr : it->second.get();
}

void ComponentRegistry::index_component(SimulationComponent& component,
                                        const std::string& component_name) {
  if (auto* joint = dynamic_cast<JointComponent*>(&component)) {
    joints_.emplace(component_name, joint);
    return;
  }
  if (auto* camera = dynamic_cast<CameraComponent*>(&component)) {
    cameras_.emplace(component_name, camera);
    return;
  }
  if (auto* imu = dynamic_cast<ImuComponent*>(&component)) {
    imus_.emplace(component_name, imu);
    return;
  }
  if (auto* lidar = dynamic_cast<LidarComponent*>(&component)) {
    lidars_.emplace(component_name, lidar);
    return;
  }
  if (auto* mobile_base = dynamic_cast<MobileBaseComponent*>(&component)) {
    mobile_bases_.emplace(component_name, mobile_base);
  }
}

void ComponentRegistry::unindex_component(const SimulationComponent& component,
                                          std::string_view component_name) {
  const std::string key(component_name);
  if (dynamic_cast<const JointComponent*>(&component) != nullptr) {
    joints_.erase(key);
    return;
  }
  if (dynamic_cast<const CameraComponent*>(&component) != nullptr) {
    cameras_.erase(key);
    return;
  }
  if (dynamic_cast<const ImuComponent*>(&component) != nullptr) {
    imus_.erase(key);
    return;
  }
  if (dynamic_cast<const LidarComponent*>(&component) != nullptr) {
    lidars_.erase(key);
    return;
  }
  if (dynamic_cast<const MobileBaseComponent*>(&component) != nullptr) {
    mobile_bases_.erase(key);
  }
}

}  // namespace mujoco_simulation
