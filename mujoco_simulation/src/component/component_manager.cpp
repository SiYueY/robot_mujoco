#include "component/component_manager.hpp"

#include <utility>
#include <vector>

#include "common/logging.hpp"

namespace mujoco_simulation {
namespace {

constexpr ComponentId kMaximumComponentId{65535};

template <typename Component>
bool reset_components(
    const mjContext& context, std::vector<std::unique_ptr<Component>>& components) {
    for (const auto& component : components) {
        if (component != nullptr && (!component->reset(context) || !component->reset_schedule())) {
            return false;
        }
    }
    return true;
}

template <typename Component>
std::size_t component_count(const std::vector<std::unique_ptr<Component>>& v) {
    std::size_t count = 0;
    for (const auto& component : v) {
        if (component != nullptr) ++count;
    }
    return count;
}

template <typename Component, typename State>
bool update_components(
    const mjContext& context, const std::vector<std::unique_ptr<Component>>& components,
    StateSnapshots<State>& published) {
    std::vector<Component*> due;
    due.reserve(components.size());
    for (const auto& component : components) {
        if (component != nullptr && component->poll_update(context.data->time)) {
            due.push_back(component.get());
        }
    }
    for (Component* component : due) {
        if (!component->update(context)) return false;
    }
    if (due.empty()) return true;

    auto slots = std::make_shared<std::vector<StateSnapshot<State>>>(
        published == nullptr ? 0U : published->size());
    if (published != nullptr) *slots = *published;
    for (Component* component : due) {
        std::shared_ptr<const State> state;
        if (!component->read_state(state) || state == nullptr) return false;
        const ComponentId id = component->info().id;
        if (slots->size() <= id) slots->resize(id + 1U);
        (*slots)[id] = std::move(state);
    }
    published = std::static_pointer_cast<const std::vector<StateSnapshot<State>>>(slots);
    return true;
}

}  // namespace

bool ComponentManager::init(
    const mjContext& context, const ComponentConfigList& components,
    CameraRenderService& camera_render_service) {
    if (!context.valid()) {
        LOG_ERROR << "component manager requires a valid MuJoCo context.";
        return false;
    }
    clear();
    camera_render_service_ = &camera_render_service;

    const auto add = [&](auto config, auto& slots, auto make_component) {
        const ComponentId id = config.id;
        if (id == kInvalidComponentId || id > kMaximumComponentId ||
            (slots.size() > id && slots[id] != nullptr)) {
            LOG_ERROR << "component id is invalid or duplicated.";
            return false;
        }
        auto component = make_component(std::move(config));
        if (!component->init(context)) return false;
        if (slots.size() <= id) slots.resize(id + 1U);
        slots[id] = std::move(component);
        return true;
    };
    for (const ComponentConfig& entry : components) {
        if (const auto* value = std::get_if<JointInfo>(&entry);
            value != nullptr && !add(*value, joints_components_, [](JointInfo v) {
                return std::make_unique<JointComponent>(std::move(v));
            })) {
            clear();
            return false;
        }
        if (const auto* value = std::get_if<ImuInfo>(&entry);
            value != nullptr && !add(*value, imu_components_, [](ImuInfo v) {
                return std::make_unique<ImuComponent>(std::move(v));
            })) {
            clear();
            return false;
        }
        if (const auto* value = std::get_if<LidarInfo>(&entry);
            value != nullptr && !add(*value, lidar_components_, [](LidarInfo v) {
                return std::make_unique<LidarComponent>(std::move(v));
            })) {
            clear();
            return false;
        }
        if (const auto* value = std::get_if<MobileBaseInfo>(&entry);
            value != nullptr && !add(*value, mobile_base_components_, [](MobileBaseInfo v) {
                return std::make_unique<MobileBaseComponent>(std::move(v));
            })) {
            clear();
            return false;
        }
        if (const auto* value = std::get_if<CameraConfig>(&entry);
            value != nullptr && !add(*value, camera_components_, [](CameraConfig v) {
                return std::make_unique<CameraComponent>(std::move(v));
            })) {
            clear();
            return false;
        }
    }
    const auto warn_sparse = [](const auto& slots, const char* type) {
        const std::size_t count = component_count(slots);
        if (count == 0U) return;
        if (slots.front() == nullptr || count != slots.size()) {
            LOG_WARNING << type << " component IDs are sparse; empty slots are reserved.";
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
    camera_render_service_ = nullptr;
    active_camera_ticket_.reset();
    pending_camera_ticket_.reset();
    camera_request_sequence_ = 0;
    camera_generation_ = 1;
    simulation_step_ = 0;
}

bool ComponentManager::reset(const mjContext& context) {
    if (!context.valid()) return false;
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
    active_camera_ticket_.reset();
    pending_camera_ticket_.reset();
    ++camera_generation_;
    camera_request_sequence_ = 0;
    simulation_step_ = 0;
    return true;
}

bool ComponentManager::update(const mjContext& context) {
    if (!context.valid()) return false;
    ++simulation_step_;
    if (!update_components<JointComponent, JointState>(context, joints_components_, joints_) ||
        !update_components<MobileBaseComponent, MobileBaseState>(
            context, mobile_base_components_, mobile_bases_) ||
        !update_components<ImuComponent, ImuState>(context, imu_components_, imus_) ||
        !update_components<LidarComponent, LidarState>(context, lidar_components_, lidars_))
        return false;
    return consume_camera_results() && submit_due_cameras(context);
}

bool ComponentManager::wait_for_camera_results() {
    if (!has_cameras()) return true;
    if (camera_render_service_ == nullptr) return false;
    if (active_camera_ticket_.has_value()) {
        const CameraRenderWaitStatus status =
            camera_render_service_->wait(*active_camera_ticket_, std::chrono::seconds(5));
        if (status != CameraRenderWaitStatus::Completed &&
            status != CameraRenderWaitStatus::PartiallyFailed) {
            return false;
        }
    }
    if (!consume_camera_results()) return false;
    if (active_camera_ticket_.has_value()) {
        const CameraRenderWaitStatus status =
            camera_render_service_->wait(*active_camera_ticket_, std::chrono::seconds(5));
        if (status != CameraRenderWaitStatus::Completed &&
            status != CameraRenderWaitStatus::PartiallyFailed) {
            return false;
        }
    }
    return consume_camera_results();
}

bool ComponentManager::has_cameras() const noexcept { return !camera_components_.empty(); }
void ComponentManager::clear_camera_states() noexcept {
    for (const auto& component : camera_components_)
        if (component != nullptr) component->clear_render_state();
    cameras_.reset();
}

bool ComponentManager::consume_camera_results() {
    if (!has_cameras()) return true;
    if (camera_render_service_ == nullptr) return false;
    if (!active_camera_ticket_.has_value() && pending_camera_ticket_.has_value()) {
        active_camera_ticket_ = pending_camera_ticket_;
        pending_camera_ticket_.reset();
    }
    if (!active_camera_ticket_.has_value()) return true;
    const CameraRenderWaitStatus wait_status =
        camera_render_service_->query(*active_camera_ticket_);
    if (wait_status == CameraRenderWaitStatus::Timeout) return true;
    CameraRenderBatchResult result;
    if (!camera_render_service_->read_batch_result(*active_camera_ticket_, result)) {
        active_camera_ticket_.reset();
        if (pending_camera_ticket_.has_value()) {
            active_camera_ticket_ = pending_camera_ticket_;
            pending_camera_ticket_.reset();
        }
        return wait_status == CameraRenderWaitStatus::Superseded ||
               wait_status == CameraRenderWaitStatus::Stale ||
               wait_status == CameraRenderWaitStatus::Cancelled;
    }
    active_camera_ticket_.reset();
    auto slots = std::make_shared<std::vector<StateSnapshot<CameraState>>>(
        cameras_ == nullptr ? 0U : cameras_->size());
    if (cameras_ != nullptr) *slots = *cameras_;
    bool changed = false;
    for (const CameraRenderTaskResult& camera : result.cameras) {
        const CameraId id = camera.camera_id;
        if (id >= camera_components_.size()) continue;
        CameraComponent* component = camera_components_[id].get();
        if (component == nullptr || !component->apply_render_result(camera)) continue;
        std::shared_ptr<const CameraState> state;
        if (!component->read_state(state)) return false;
        if (slots->size() <= id) slots->resize(id + 1U);
        (*slots)[id] = std::move(state);
        changed = true;
    }
    if (changed)
        cameras_ = std::static_pointer_cast<const std::vector<StateSnapshot<CameraState>>>(slots);
    if (pending_camera_ticket_.has_value()) {
        active_camera_ticket_ = pending_camera_ticket_;
        pending_camera_ticket_.reset();
    }
    return true;
}

bool ComponentManager::submit_due_cameras(const mjContext& context) {
    if (!has_cameras()) return true;
    if (camera_render_service_ == nullptr) return false;
    std::vector<CameraRenderTask> tasks;
    const auto timestamp =
        context.data->time > 0.0 ? static_cast<std::uint64_t>(context.data->time * 1.0e9) : 0U;
    for (const auto& component : camera_components_) {
        if (component != nullptr && component->poll_update(context.data->time))
            tasks.push_back(component->make_render_task(timestamp));
    }
    if (!tasks.empty()) {
        CameraRenderBatchRequest request;
        request.generation = camera_generation_;
        request.sequence = ++camera_request_sequence_;
        request.simulation_step = simulation_step_;
        request.simulation_time = context.data->time;
        request.model = context.model;
        request.data = context.data;
        request.tasks = std::move(tasks);
        CameraRenderTicket ticket;
        const CameraRenderSubmitResult submit_result =
            camera_render_service_->submit(request, ticket);
        if (submit_result != CameraRenderSubmitResult::Accepted &&
            submit_result != CameraRenderSubmitResult::ReplacedPendingBatch) {
            LOG_WARNING << "failed to submit camera render job; keeping prior frames.";
        } else {
            if (!active_camera_ticket_.has_value())
                active_camera_ticket_ = ticket;
            else
                pending_camera_ticket_ = ticket;
        }
    }
    return true;
}

bool ComponentManager::write_command(const mjContext& context, const RobotCommand& command) {
    if (!context.valid()) return false;
    if (!command.joints.empty() && !write_joint_commands(context, command.joints)) return false;
    if (!command.mobile_bases.empty() && !write_mobile_base_commands(context, command.mobile_bases))
        return false;
    return true;
}

bool ComponentManager::write_joint_commands(
    const mjContext& context, const std::vector<JointCommand>& commands) {
    for (JointId id = 0; id < commands.size(); ++id) {
        if (id >= joints_components_.size() || joints_components_[id] == nullptr) {
            LOG_ERROR << "joint command target id was not found.";
            return false;
        }
        if (!joints_components_[id]->write(context, commands[id])) return false;
    }
    return true;
}

bool ComponentManager::write_mobile_base_commands(
    const mjContext& context, const std::vector<MobileBaseCommand>& commands) {
    for (MobileBaseId id = 0; id < commands.size(); ++id) {
        if (id >= mobile_base_components_.size() || mobile_base_components_[id] == nullptr) {
            LOG_ERROR << "mobile base command target id was not found.";
            return false;
        }
        if (!mobile_base_components_[id]->write(context, commands[id])) return false;
    }
    return true;
}

bool ComponentManager::read_state(const mjContext&, RobotState& snapshot) const {
    snapshot.joints = joints_;
    snapshot.mobile_bases = mobile_bases_;
    snapshot.imus = imus_;
    snapshot.lidars = lidars_;
    snapshot.cameras = cameras_;
    return true;
}

}  // namespace mujoco_simulation
