#include "mujoco_simulation/hardware/hardware_manager.hpp"

#include <utility>

namespace mujoco_simulation {
namespace {

template <typename DeviceMap, typename State>
bool read_named_device(const std::string& name, DeviceMap& devices, std::string& last_error,
                       State& state) {
  const auto it = devices.find(name);
  if (it == devices.end()) {
    last_error = "Hardware device not found: " + name;
    return false;
  }
  if (!it->second->read(state)) {
    last_error = it->second->last_error();
    return false;
  }
  last_error.clear();
  return true;
}

template <typename DeviceMap>
bool unregister_named_device(const std::string& name, DeviceMap& devices, std::string& last_error,
                             const char* device_type) {
  const auto it = devices.find(name);
  if (it == devices.end()) {
    last_error = std::string(device_type) + " not found: " + name;
    return false;
  }
  devices.erase(it);
  last_error.clear();
  return true;
}

}  // namespace

HardwareManager::HardwareManager(const mjModel* model, mjData* data, mjvScene* scene,
                                 mjrContext* render_context)
    : context_{model, data, scene, render_context} {}

void HardwareManager::set_render_resources(mjvScene* scene, mjrContext* render_context) {
  context_.scene = scene;
  context_.render = render_context;
  last_error_.clear();
}

bool HardwareManager::register_joint(const JointInfo& data) {
  if (data.name.empty()) {
    return set_error("Joint name must not be empty.");
  }
  if (joint_infos_.find(data.name) != joint_infos_.end()) {
    return set_error("Joint already registered: " + data.name);
  }

  auto device = std::make_unique<Joint>(context_);
  if (!device->init(data)) {
    return set_error(device->last_error());
  }
  joint_infos_[data.name] = data;
  joints_[data.name] = std::move(device);
  last_error_.clear();
  return true;
}

bool HardwareManager::has_joint(const std::string& name) const {
  return joint_infos_.find(name) != joint_infos_.end();
}

bool HardwareManager::configure_joint_command_mode(const std::string& name,
                                                   CommandInterfaceType command_mode) {
  const auto it = joint_infos_.find(name);
  if (it == joint_infos_.end()) {
    return set_error("Joint is not registered: " + name);
  }

  JointInfo updated = it->second;
  updated.command_mode = command_mode;

  if (!unregister_joint(name)) {
    return false;
  }

  if (!register_joint(updated)) {
    return false;
  }

  last_error_.clear();
  return true;
}

bool HardwareManager::register_imu(const ImuInfo& data) {
  if (data.name.empty()) {
    return set_error("IMU name must not be empty.");
  }
  if (imu_infos_.find(data.name) != imu_infos_.end()) {
    return set_error("IMU already registered: " + data.name);
  }

  auto device = std::make_unique<Imu>(context_);
  if (!device->init(data)) {
    return set_error(device->last_error());
  }
  imu_infos_[data.name] = data;
  imus_[data.name] = std::move(device);
  last_error_.clear();
  return true;
}

bool HardwareManager::has_imu(const std::string& name) const {
  return imu_infos_.find(name) != imu_infos_.end();
}

bool HardwareManager::register_camera(const CameraSpec& data) {
  if (data.name.empty()) {
    return set_error("Camera name must not be empty.");
  }
  if (camera_specs_.find(data.name) != camera_specs_.end()) {
    return set_error("Camera already registered: " + data.name);
  }

  camera_specs_[data.name] = data;
  if (context_.scene == nullptr || context_.render == nullptr) {
    pending_camera_specs_[data.name] = data;
    last_error_.clear();
    return true;
  }
  return register_camera_device(data);
}

bool HardwareManager::register_lidar(const LidarInfo& data) {
  if (data.name.empty()) {
    return set_error("Lidar name must not be empty.");
  }
  if (lidar_infos_.find(data.name) != lidar_infos_.end()) {
    return set_error("Lidar already registered: " + data.name);
  }

  auto device = std::make_unique<Lidar>(context_);
  if (!device->init(data)) {
    return set_error(device->last_error());
  }
  lidar_infos_[data.name] = data;
  lidars_[data.name] = std::move(device);
  last_error_.clear();
  return true;
}

bool HardwareManager::has_lidar(const std::string& name) const {
  return lidar_infos_.find(name) != lidar_infos_.end();
}

bool HardwareManager::register_mobile_base(const MobileBaseInfo& data) {
  if (data.name.empty()) {
    return set_error("Mobile base name must not be empty.");
  }
  if (mobile_bases_.find(data.name) != mobile_bases_.end()) {
    return set_error("Mobile base already registered: " + data.name);
  }

  std::vector<Joint*> traction_joints;
  traction_joints.reserve(data.traction_joint_names.size());
  for (const auto& joint_name : data.traction_joint_names) {
    const auto it = joints_.find(joint_name);
    if (it == joints_.end()) {
      return set_error("Mobile base traction joint not found: " + joint_name);
    }
    traction_joints.push_back(it->second.get());
  }

  auto device = std::make_unique<MobileBase>(context_, traction_joints);
  if (!device->init(data)) {
    return set_error(device->last_error());
  }
  mobile_bases_[data.name] = std::move(device);
  last_error_.clear();
  return true;
}

bool HardwareManager::unregister_joint(const std::string& name) {
  if (!unregister_named_device(name, joints_, last_error_, "Joint")) {
    return false;
  }
  joint_infos_.erase(name);
  last_error_.clear();
  return true;
}

bool HardwareManager::unregister_imu(const std::string& name) {
  if (!unregister_named_device(name, imus_, last_error_, "IMU")) {
    return false;
  }
  imu_infos_.erase(name);
  last_error_.clear();
  return true;
}

bool HardwareManager::unregister_camera(const std::string& name) {
  if (!has_camera(name)) {
    return set_error("Camera not found: " + name);
  }
  if (has_registered_camera(name) &&
      !unregister_named_device(name, cameras_, last_error_, "Camera")) {
    return false;
  }
  camera_specs_.erase(name);
  pending_camera_specs_.erase(name);
  last_error_.clear();
  return true;
}

bool HardwareManager::has_camera(const std::string& name) const {
  return camera_specs_.find(name) != camera_specs_.end();
}

bool HardwareManager::has_registered_camera(const std::string& name) const {
  return cameras_.find(name) != cameras_.end();
}

bool HardwareManager::ensure_camera_registered(const std::string& name) {
  const auto it = camera_specs_.find(name);
  if (it == camera_specs_.end()) {
    return set_error("Camera is not registered: " + name);
  }
  if (has_registered_camera(name)) {
    last_error_.clear();
    return true;
  }
  if (context_.scene == nullptr || context_.render == nullptr) {
    return set_error("MuJoCo rendering state is not available for camera '" + name + "'.");
  }
  return register_camera_device(it->second);
}

bool HardwareManager::register_pending_cameras() {
  if (context_.scene == nullptr || context_.render == nullptr) {
    return set_error("MuJoCo rendering state is not available for pending cameras.");
  }

  std::vector<std::string> pending_names;
  pending_names.reserve(pending_camera_specs_.size());
  for (const auto& [name, spec] : pending_camera_specs_) {
    (void)spec;
    pending_names.push_back(name);
  }

  for (const auto& name : pending_names) {
    if (!ensure_camera_registered(name)) {
      return false;
    }
  }

  last_error_.clear();
  return true;
}

bool HardwareManager::unregister_lidar(const std::string& name) {
  if (!unregister_named_device(name, lidars_, last_error_, "Lidar")) {
    return false;
  }
  lidar_infos_.erase(name);
  last_error_.clear();
  return true;
}

bool HardwareManager::unregister_mobile_base(const std::string& name) {
  return unregister_named_device(name, mobile_bases_, last_error_, "Mobile base");
}

bool HardwareManager::reset_all() {
  return reset_device_map(joints_) && reset_device_map(imus_) && reset_device_map(cameras_) &&
         reset_device_map(lidars_) && reset_device_map(mobile_bases_);
}

bool HardwareManager::write_joint(const std::string& name, const JointCommand& command) {
  const auto it = joints_.find(name);
  if (it == joints_.end()) {
    return set_error("Joint not found: " + name);
  }
  if (!it->second->write(command)) {
    return set_error(it->second->last_error());
  }
  last_error_.clear();
  return true;
}

bool HardwareManager::read_joint(const std::string& name, JointState& state) {
  return read_named_device(name, joints_, last_error_, state);
}

bool HardwareManager::read_imu(const std::string& name, ImuState& state) {
  return read_named_device(name, imus_, last_error_, state);
}

bool HardwareManager::read_camera(const std::string& name, CameraState& state) {
  return read_named_device(name, cameras_, last_error_, state);
}

bool HardwareManager::read_lidar(const std::string& name, LidarState& state) {
  return read_named_device(name, lidars_, last_error_, state);
}

bool HardwareManager::write_mobile_base(const std::string& name, const MobileBaseCommand& command) {
  const auto it = mobile_bases_.find(name);
  if (it == mobile_bases_.end()) {
    return set_error("Mobile base not found: " + name);
  }
  if (!it->second->write(command)) {
    return set_error(it->second->last_error());
  }
  last_error_.clear();
  return true;
}

bool HardwareManager::read_mobile_base(const std::string& name, MobileBaseState& state) {
  return read_named_device(name, mobile_bases_, last_error_, state);
}

std::unordered_map<std::string, JointState> HardwareManager::read_joint_states() {
  return read_all_from<std::unordered_map<std::string, std::unique_ptr<Joint>>, JointState>(
      joints_);
}

std::unordered_map<std::string, LidarState> HardwareManager::read_lidar_states() {
  return read_all_from<std::unordered_map<std::string, std::unique_ptr<Lidar>>, LidarState>(
      lidars_);
}

std::unordered_map<std::string, MobileBaseState> HardwareManager::read_mobile_base_states() {
  return read_all_from<std::unordered_map<std::string, std::unique_ptr<MobileBase>>,
                       MobileBaseState>(mobile_bases_);
}

bool HardwareManager::set_error(const std::string& message) {
  last_error_ = message;
  return false;
}

bool HardwareManager::register_camera_device(const CameraSpec& data) {
  if (has_registered_camera(data.name)) {
    return set_error("Camera already registered: " + data.name);
  }

  auto device = std::make_unique<Camera>(context_);
  if (!device->init(data)) {
    return set_error(device->last_error());
  }
  pending_camera_specs_.erase(data.name);
  cameras_[data.name] = std::move(device);
  last_error_.clear();
  return true;
}

template <typename DeviceMap>
bool HardwareManager::reset_device_map(DeviceMap& devices) {
  for (auto& [name, device] : devices) {
    if (!device->reset()) {
      return set_error(device->last_error().empty() ? ("Failed to reset device: " + name)
                                                    : device->last_error());
    }
  }
  last_error_.clear();
  return true;
}

template <typename DeviceMap, typename State>
std::unordered_map<std::string, State> HardwareManager::read_all_from(DeviceMap& devices) {
  std::unordered_map<std::string, State> result;
  for (auto& [name, device] : devices) {
    State state;
    if (!device->read(state)) {
      set_error(device->last_error().empty() ? ("Failed to read device: " + name)
                                             : device->last_error());
      return {};
    }
    result.emplace(name, std::move(state));
  }
  last_error_.clear();
  return result;
}

template bool HardwareManager::reset_device_map(
    std::unordered_map<std::string, std::unique_ptr<Joint>>& devices);
template bool HardwareManager::reset_device_map(
    std::unordered_map<std::string, std::unique_ptr<Imu>>& devices);
template bool HardwareManager::reset_device_map(
    std::unordered_map<std::string, std::unique_ptr<Camera>>& devices);
template bool HardwareManager::reset_device_map(
    std::unordered_map<std::string, std::unique_ptr<Lidar>>& devices);
template bool HardwareManager::reset_device_map(
    std::unordered_map<std::string, std::unique_ptr<MobileBase>>& devices);

template std::unordered_map<std::string, JointState> HardwareManager::read_all_from(
    std::unordered_map<std::string, std::unique_ptr<Joint>>& devices);
template std::unordered_map<std::string, LidarState> HardwareManager::read_all_from(
    std::unordered_map<std::string, std::unique_ptr<Lidar>>& devices);
template std::unordered_map<std::string, MobileBaseState> HardwareManager::read_all_from(
    std::unordered_map<std::string, std::unique_ptr<MobileBase>>& devices);

}  // namespace mujoco_simulation
