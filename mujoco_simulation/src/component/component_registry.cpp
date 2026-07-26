#include "mujoco_simulation/component/component_registry.hpp"

#include "mujoco_simulation/common/logging.hpp"

namespace mujoco_simulation {

bool ComponentRegistry::add(std::unique_ptr<SimulationComponent> component) {
  if (component == nullptr) {
    LOG_ERROR << "component must not be null.";
    return false;
  }

  const std::string component_name(component->name());
  if (component_name.empty()) {
    LOG_ERROR << "component name must not be empty.";
    return false;
  }
  if (components_.find(component_name) != components_.end()) {
    LOG_ERROR << "component name already exists.";
    return false;
  }

  index_component(*component, component_name);
  components_.emplace(component_name, std::move(component));
  return true;
}

void ComponentRegistry::clear() {
  components_.clear();
  joints_.clear();
  cameras_.clear();
  imus_.clear();
  lidars_.clear();
  mobile_bases_.clear();
}

bool ComponentRegistry::has_joint(std::string name) const {
  return joints_.find(std::string(name)) != joints_.end();
}

bool ComponentRegistry::has_camera(std::string name) const {
  return cameras_.find(std::string(name)) != cameras_.end();
}

bool ComponentRegistry::has_imu(std::string name) const {
  return imus_.find(std::string(name)) != imus_.end();
}

bool ComponentRegistry::has_lidar(std::string name) const {
  return lidars_.find(std::string(name)) != lidars_.end();
}

bool ComponentRegistry::has_mobile_base(std::string name) const {
  return mobile_bases_.find(std::string(name)) != mobile_bases_.end();
}

JointComponent *ComponentRegistry::joint(std::string name) {
  const auto it = joints_.find(std::string(name));
  return it == joints_.end() ? nullptr : it->second;
}

const JointComponent *ComponentRegistry::joint(std::string name) const {
  const auto it = joints_.find(std::string(name));
  return it == joints_.end() ? nullptr : it->second;
}

CameraComponent *ComponentRegistry::camera(std::string name) {
  const auto it = cameras_.find(std::string(name));
  return it == cameras_.end() ? nullptr : it->second;
}

const CameraComponent *ComponentRegistry::camera(std::string name) const {
  const auto it = cameras_.find(std::string(name));
  return it == cameras_.end() ? nullptr : it->second;
}

ImuComponent *ComponentRegistry::imu(std::string name) {
  const auto it = imus_.find(std::string(name));
  return it == imus_.end() ? nullptr : it->second;
}

const ImuComponent *ComponentRegistry::imu(std::string name) const {
  const auto it = imus_.find(std::string(name));
  return it == imus_.end() ? nullptr : it->second;
}

LidarComponent *ComponentRegistry::lidar(std::string name) {
  const auto it = lidars_.find(std::string(name));
  return it == lidars_.end() ? nullptr : it->second;
}

const LidarComponent *ComponentRegistry::lidar(std::string name) const {
  const auto it = lidars_.find(std::string(name));
  return it == lidars_.end() ? nullptr : it->second;
}

MobileBaseComponent *ComponentRegistry::mobile_base(std::string name) {
  const auto it = mobile_bases_.find(std::string(name));
  return it == mobile_bases_.end() ? nullptr : it->second;
}

const MobileBaseComponent *
ComponentRegistry::mobile_base(std::string name) const {
  const auto it = mobile_bases_.find(std::string(name));
  return it == mobile_bases_.end() ? nullptr : it->second;
}

const std::unordered_map<std::string, JointComponent *> &
ComponentRegistry::joints() const noexcept {
  return joints_;
}

const std::unordered_map<std::string, CameraComponent *> &
ComponentRegistry::cameras() const noexcept {
  return cameras_;
}

const std::unordered_map<std::string, ImuComponent *> &
ComponentRegistry::imus() const noexcept {
  return imus_;
}

const std::unordered_map<std::string, LidarComponent *> &
ComponentRegistry::lidars() const noexcept {
  return lidars_;
}

const std::unordered_map<std::string, MobileBaseComponent *> &
ComponentRegistry::mobile_bases() const noexcept {
  return mobile_bases_;
}

SimulationComponent *ComponentRegistry::find(std::string name) {
  const auto it = components_.find(std::string(name));
  return it == components_.end() ? nullptr : it->second.get();
}

const SimulationComponent *ComponentRegistry::find(std::string name) const {
  const auto it = components_.find(std::string(name));
  return it == components_.end() ? nullptr : it->second.get();
}

void ComponentRegistry::index_component(SimulationComponent &component,
                                        const std::string &component_name) {
  if (auto *joint = dynamic_cast<JointComponent *>(&component)) {
    joints_.emplace(component_name, joint);
    return;
  }
  if (auto *camera = dynamic_cast<CameraComponent *>(&component)) {
    cameras_.emplace(component_name, camera);
    return;
  }
  if (auto *imu = dynamic_cast<ImuComponent *>(&component)) {
    imus_.emplace(component_name, imu);
    return;
  }
  if (auto *lidar = dynamic_cast<LidarComponent *>(&component)) {
    lidars_.emplace(component_name, lidar);
    return;
  }
  if (auto *mobile_base = dynamic_cast<MobileBaseComponent *>(&component)) {
    mobile_bases_.emplace(component_name, mobile_base);
  }
}

} // namespace mujoco_simulation
