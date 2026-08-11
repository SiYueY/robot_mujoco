#include "component/mobile_base/mobile_base_component.hpp"

#include <cmath>
#include <utility>

#include "common/macro.hpp"
#include "component/mobile_base/mecanum/mecanum_mobile_base.hpp"
#include "log/logging.hpp"

namespace mujoco_simulation {
namespace {
constexpr double kHorizontalTolerance = 1.0e-10;
}

MobileBaseComponent::MobileBaseComponent(MobileBaseInfo info)
: SimulationComponent(info.mobile_base_name, info.period), info_(std::move(info)) {}

MobileBaseComponent::~MobileBaseComponent() = default;

bool MobileBaseComponent::init(const mjContext& context) {
    initialized_ = false;
    base_qpos_address_ = -1;
    base_dof_address_ = -1;
    mecanum_.reset();
    if (!configure(context) || !configure_base(context)) return false;
    if (info_.type != MobileBaseType::Mecanum) return false;
    try {
        mecanum_ = std::make_unique<MecanumMobileBase>(info_.mecanum_info);
    } catch (const std::exception&) {
        return false;
    }
    if (!mecanum_->init(context, info_.mecanum_wheels) || !reset_planar_base(context)) return false;

    working_state_ = {};
    working_state_.id = info_.id;
    working_state_.base_frame_id = info_.base_frame_id;
    working_state_.odom_frame_id = info_.odom_frame_id;
    publish_state(context);
    state_ = std::make_shared<MobileBaseState>(working_state_);
    initialized_ = true;
    return true;
}

bool MobileBaseComponent::reset(const mjContext& context) {
    if (!is_initialized() || !mecanum_) return false;
    if (!mecanum_->reset(context) || !reset_planar_base(context)) return false;
    mj_forward(context.model, context.data);
    publish_state(context);
    state_ = std::make_shared<MobileBaseState>(working_state_);
    return true;
}

bool MobileBaseComponent::advance(const mjContext& context) {
    if (!is_initialized() || !mecanum_ || !mecanum_->advance(context)) return false;
    return advance_planar_base(context);
}

bool MobileBaseComponent::update(const mjContext& context) {
    if (!is_initialized()) return false;
    publish_state(context);
    state_ = std::make_shared<MobileBaseState>(working_state_);
    return true;
}

bool MobileBaseComponent::write(const mjContext& context, const MobileBaseCommand& command) {
    UNUSED(context);
    return is_initialized() && mecanum_ != nullptr && mecanum_->write(command);
}

bool MobileBaseComponent::read_state(std::shared_ptr<const MobileBaseState>& state) const {
    if (!is_initialized()) return false;
    state = state_;
    return state != nullptr;
}

bool MobileBaseComponent::read(const mjContext& context, MobileBaseState& state) const {
    UNUSED(context);
    std::shared_ptr<const MobileBaseState> snapshot;
    if (!read_state(snapshot)) return false;
    state = *snapshot;
    return true;
}

bool MobileBaseComponent::is_initialized() const noexcept { return initialized_; }

bool MobileBaseComponent::configure_base(const mjContext& context) {
    const int body_id = mj_name2id(context.model, mjOBJ_BODY, info_.base_body_name.c_str());
    const int joint_id = mj_name2id(context.model, mjOBJ_JOINT, info_.base_joint_name.c_str());
    if (body_id < 0 || joint_id < 0 || context.model->jnt_type[joint_id] != mjJNT_FREE ||
        context.model->jnt_bodyid[joint_id] != body_id) {
        SIM_ERROR << "mobile base '" << info_.mobile_base_name
                  << "' requires a free base_joint belonging to base_body.";
        return false;
    }
    base_qpos_address_ = context.model->jnt_qposadr[joint_id];
    base_dof_address_ = context.model->jnt_dofadr[joint_id];
    return base_qpos_address_ >= 0 && base_dof_address_ >= 0;
}

bool MobileBaseComponent::reset_planar_base(const mjContext& context) {
    const mjtNum* initial_qpos = context.model->qpos0 + base_qpos_address_;
    for (int index = 0; index < 7; ++index)
        context.data->qpos[base_qpos_address_ + index] = initial_qpos[index];
    const mjtNum* qpos = context.data->qpos + base_qpos_address_;
    if (std::abs(qpos[4]) > kHorizontalTolerance || std::abs(qpos[5]) > kHorizontalTolerance) {
        SIM_ERROR << "mobile base '" << info_.mobile_base_name
                  << "' requires a horizontal initial base orientation.";
        return false;
    }
    world_odom_position_ = {qpos[0], qpos[1], qpos[2]};
    world_odom_yaw_ = wrap_angle(2.0 * std::atan2(qpos[6], qpos[3]));
    odom_pose_ = {};
    for (int index = 0; index < 6; ++index) context.data->qvel[base_dof_address_ + index] = 0.0;
    return true;
}

bool MobileBaseComponent::advance_planar_base(const mjContext& context) {
    const double dt = context.model->opt.timestep;
    if (!std::isfinite(dt) || dt <= 0.0) return false;
    const Vector3d& linear = mecanum_->base_linear();
    const Vector3d& angular = mecanum_->base_angular();
    const double yaw_next = wrap_angle(odom_pose_[2] + angular[2] * dt);
    odom_pose_[0] += (std::cos(yaw_next) * linear[0] - std::sin(yaw_next) * linear[1]) * dt;
    odom_pose_[1] += (std::sin(yaw_next) * linear[0] + std::cos(yaw_next) * linear[1]) * dt;
    odom_pose_[2] = yaw_next;

    const double world_yaw = wrap_angle(world_odom_yaw_ + odom_pose_[2]);
    const double c = std::cos(world_odom_yaw_);
    const double s = std::sin(world_odom_yaw_);
    context.data->qpos[base_qpos_address_] =
        world_odom_position_[0] + c * odom_pose_[0] - s * odom_pose_[1];
    context.data->qpos[base_qpos_address_ + 1] =
        world_odom_position_[1] + s * odom_pose_[0] + c * odom_pose_[1];
    context.data->qpos[base_qpos_address_ + 2] = world_odom_position_[2];
    context.data->qpos[base_qpos_address_ + 3] = std::cos(world_yaw * 0.5);
    context.data->qpos[base_qpos_address_ + 4] = 0.0;
    context.data->qpos[base_qpos_address_ + 5] = 0.0;
    context.data->qpos[base_qpos_address_ + 6] = std::sin(world_yaw * 0.5);
    context.data->qvel[base_dof_address_] =
        std::cos(world_yaw) * linear[0] - std::sin(world_yaw) * linear[1];
    context.data->qvel[base_dof_address_ + 1] =
        std::sin(world_yaw) * linear[0] + std::cos(world_yaw) * linear[1];
    context.data->qvel[base_dof_address_ + 2] = 0.0;
    context.data->qvel[base_dof_address_ + 3] = 0.0;
    context.data->qvel[base_dof_address_ + 4] = 0.0;
    context.data->qvel[base_dof_address_ + 5] = angular[2];
    return true;
}

void MobileBaseComponent::publish_state(const mjContext& context) {
    working_state_.pose = odom_pose_;
    working_state_.base_linear = mecanum_->base_linear();
    working_state_.base_angular = mecanum_->base_angular();
    working_state_.wheel_angular = mecanum_->wheel_angular();
    working_state_.wheel_linear = mecanum_->wheel_linear();
    working_state_.timestamp = context.data->time;
}

double MobileBaseComponent::wrap_angle(double angle) { return std::remainder(angle, 2.0 * Pi); }

}  // namespace mujoco_simulation
