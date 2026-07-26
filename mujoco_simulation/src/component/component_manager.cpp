#include "mujoco_simulation/component/component_manager.hpp"

#include <utility>
#include <vector>

#include "common/logging.hpp"

namespace mujoco_simulation {
namespace {

template <typename Component>
bool reset_components(const mjContext &context,
                      std::vector<std::unique_ptr<Component>> &components) {
  for (const auto &component : components) {
    if (component != nullptr &&
        (!component->reset(context) || !component->reset_schedule())) {
      return false;
    }
  }
  return true;
}

template <typename Component>
std::size_t component_count(const std::vector<std::unique_ptr<Component>> &v) {
  std::size_t count = 0;
  for (const auto &component : v) {
    if (component != nullptr)
      ++count;
  }
  return count;
}

template <typename Component, typename State>
bool update_components(
    const mjContext &context,
    const std::vector<std::unique_ptr<Component>> &components,
    StateSnapshots<State> &published) {
  std::vector<Component *> due;
  due.reserve(component_count(components));
  for (const auto &component : components) {
    if (component != nullptr && component->poll_update(context.data->time)) {
      due.push_back(component.get());
    }
  }
  for (Component *component : due) {
    if (!component->update(context))
      return false;
  }
  if (due.empty())
    return true;

  auto slots = std::make_shared<std::vector<StateSnapshot<State>>>(
      published == nullptr ? 0U : published->size());
  if (published != nullptr)
    *slots = *published;
  for (Component *component : due) {
    std::shared_ptr<const State> state;
    if (!component->read_state(state) || state == nullptr)
      return false;
    const ComponentId id = component->info().id;
    if (slots->size() <= id)
      slots->resize(id + 1U);
    (*slots)[id] = std::move(state);
  }
  published =
      std::static_pointer_cast<const std::vector<StateSnapshot<State>>>(slots);
  return true;
}

template <typename Component>
bool has_any(const std::vector<std::unique_ptr<Component>> &components) {
  return component_count(components) != 0U;
}

} // namespace

bool ComponentManager::init(const mjContext &context,
                            const ComponentConfigList &components,
                            ComponentId max_component_id,
                            CameraRenderer &camera_renderer) {
  if (!context.valid()) {
    LOG_ERROR << "component manager requires a valid MuJoCo context.";
    return false;
  }
  clear();
  camera_renderer_ = &camera_renderer;

  const auto add = [&](auto config, auto &slots, auto make_component) {
    const ComponentId id = config.id;
    if (id == kInvalidComponentId || id > max_component_id ||
        (slots.size() > id && slots[id] != nullptr)) {
      LOG_ERROR << "component id is invalid or duplicated.";
      return false;
    }
    auto component = make_component(std::move(config));
    if (!component->init(context))
      return false;
    if (slots.size() <= id)
      slots.resize(id + 1U);
    slots[id] = std::move(component);
    return true;
  };
  for (const ComponentConfig &entry : components) {
    if (const auto *value = std::get_if<JointInfo>(&entry);
        value != nullptr && !add(*value, joints_components_, [](JointInfo v) {
          return std::make_unique<JointComponent>(std::move(v));
        })) {
      clear();
      return false;
    }
    if (const auto *value = std::get_if<ImuInfo>(&entry);
        value != nullptr && !add(*value, imu_components_, [](ImuInfo v) {
          return std::make_unique<ImuComponent>(std::move(v));
        })) {
      clear();
      return false;
    }
    if (const auto *value = std::get_if<LidarInfo>(&entry);
        value != nullptr && !add(*value, lidar_components_, [](LidarInfo v) {
          return std::make_unique<LidarComponent>(std::move(v));
        })) {
      clear();
      return false;
    }
    if (const auto *value = std::get_if<MobileBaseInfo>(&entry);
        value != nullptr &&
        !add(*value, mobile_base_components_, [](MobileBaseInfo v) {
          return std::make_unique<MobileBaseComponent>(std::move(v));
        })) {
      clear();
      return false;
    }
    if (const auto *value = std::get_if<CameraConfig>(&entry);
        value != nullptr &&
        !add(*value, camera_components_, [](CameraConfig v) {
          return std::make_unique<CameraComponent>(std::move(v));
        })) {
      clear();
      return false;
    }
  }
  const auto warn_sparse = [](const auto &slots, const char *type) {
    const std::size_t count = component_count(slots);
    if (count == 0U)
      return;
    if (slots.front() == nullptr || count != slots.size()) {
      LOG_WARNING << type
                  << " component IDs are sparse; empty slots are reserved.";
    }
  };
  warn_sparse(joints_components_, "joint");
  warn_sparse(imu_components_, "imu");
  warn_sparse(camera_components_, "camera");
  warn_sparse(lidar_components_, "lidar");
  warn_sparse(mobile_base_components_, "mobile base");
  return true;
}

void ComponentManager::clear() {
  joints_components_.clear();
  camera_components_.clear();
  imu_components_.clear();
  lidar_components_.clear();
  mobile_base_components_.clear();
  joints_.reset();
  mobile_bases_.reset();
  imus_.reset();
  lidars_.reset();
  cameras_.reset();
  camera_renderer_ = nullptr;
}

bool ComponentManager::reset(const mjContext &context) {
  if (!context.valid())
    return false;
  if (!reset_components(context, joints_components_) ||
      !reset_components(context, camera_components_) ||
      !reset_components(context, imu_components_) ||
      !reset_components(context, lidar_components_) ||
      !reset_components(context, mobile_base_components_))
    return false;
  joints_.reset();
  mobile_bases_.reset();
  imus_.reset();
  lidars_.reset();
  cameras_.reset();
  return true;
}

bool ComponentManager::update(const mjContext &context) {
  if (!context.valid())
    return false;
  if (!update_components<JointComponent, JointState>(
          context, joints_components_, joints_) ||
      !update_components<MobileBaseComponent, MobileBaseState>(
          context, mobile_base_components_, mobile_bases_) ||
      !update_components<ImuComponent, ImuState>(context, imu_components_,
                                                 imus_) ||
      !update_components<LidarComponent, LidarState>(context, lidar_components_,
                                                     lidars_))
    return false;
  return consume_camera_results() && submit_due_cameras(context);
}

bool ComponentManager::wait_for_camera_results() {
  if (!has_cameras())
    return true;
  return camera_renderer_ != nullptr &&
         camera_renderer_->wait_for_submitted_results() &&
         consume_camera_results();
}

bool ComponentManager::has_cameras() const noexcept {
  return has_any(camera_components_);
}
void ComponentManager::clear_camera_states() noexcept {
  for (const auto &component : camera_components_)
    if (component != nullptr)
      component->clear_render_state();
  cameras_.reset();
}

bool ComponentManager::consume_camera_results() {
  if (!has_cameras())
    return true;
  if (camera_renderer_ == nullptr)
    return false;
  CameraRenderStates results;
  if (!camera_renderer_->read_results(results) || results == nullptr)
    return true;
  auto slots = std::make_shared<std::vector<StateSnapshot<CameraState>>>(
      cameras_ == nullptr ? 0U : cameras_->size());
  if (cameras_ != nullptr)
    *slots = *cameras_;
  bool changed = false;
  for (CameraId id = 0; id < camera_components_.size(); ++id) {
    CameraComponent *component = camera_components_[id].get();
    if (component == nullptr || id >= results->size() ||
        (*results)[id] == nullptr ||
        !component->apply_render_result((*results)[id]))
      continue;
    std::shared_ptr<const CameraState> state;
    if (!component->read_state(state))
      return false;
    if (slots->size() <= id)
      slots->resize(id + 1U);
    (*slots)[id] = std::move(state);
    changed = true;
  }
  if (changed)
    cameras_ =
        std::static_pointer_cast<const std::vector<StateSnapshot<CameraState>>>(
            slots);
  return true;
}

bool ComponentManager::submit_due_cameras(const mjContext &context) {
  if (!has_cameras())
    return true;
  if (camera_renderer_ == nullptr)
    return false;
  std::vector<CameraRenderTask> tasks;
  const auto timestamp =
      context.data->time > 0.0
          ? static_cast<std::uint64_t>(context.data->time * 1.0e9)
          : 0U;
  for (const auto &component : camera_components_) {
    if (component != nullptr && component->poll_update(context.data->time))
      tasks.push_back(component->make_render_task(timestamp));
  }
  if (!tasks.empty() && !camera_renderer_->submit(context, std::move(tasks)))
    LOG_WARNING << "failed to submit camera render job; keeping prior frames.";
  return true;
}

bool ComponentManager::write_command(const mjContext &context,
                                     const RobotCommand &snapshot) {
  if (!context.valid())
    return false;
  for (JointId id = 0; id < snapshot.joint_commands.size(); ++id) {
    if (!snapshot.joint_commands[id].has_value())
      continue;
    if (id >= joints_components_.size() || joints_components_[id] == nullptr) {
      LOG_ERROR << "joint command target id was not found.";
      return false;
    }
    if (!joints_components_[id]->write(context, *snapshot.joint_commands[id]))
      return false;
  }
  for (MobileBaseId id = 0; id < snapshot.mobile_base_commands.size(); ++id) {
    if (!snapshot.mobile_base_commands[id].has_value())
      continue;
    if (id >= mobile_base_components_.size() ||
        mobile_base_components_[id] == nullptr) {
      LOG_ERROR << "mobile base command target id was not found.";
      return false;
    }
    if (!mobile_base_components_[id]->write(context,
                                            *snapshot.mobile_base_commands[id]))
      return false;
  }
  return true;
}

bool ComponentManager::read_state(const mjContext &,
                                  RobotState &snapshot) const {
  snapshot.joints = joints_;
  snapshot.mobile_bases = mobile_bases_;
  snapshot.imus = imus_;
  snapshot.lidars = lidars_;
  snapshot.cameras = cameras_;
  return true;
}

} // namespace mujoco_simulation
