#include "component/mobile_base/mecanum/kinematic_mecanum_mobile_base.hpp"
#include <cmath>
#include <unordered_set>
namespace romujoco {
KinematicMecanumMobileBase::KinematicMecanumMobileBase(MecanumMobileBaseInfo i)
: MobileBaseComponent(i.common.name, i.common.period), info_(std::move(i)), ik_(info_) {}
bool KinematicMecanumMobileBase::base(const mjContext& c) {
    int b = mj_name2id(c.model, mjOBJ_BODY, info_.common.base_body_name.c_str()), j = -1;
    if (b < 0) return false;
    for (int n = 0; n < c.model->njnt; ++n)
        if (c.model->jnt_bodyid[n] == b && c.model->jnt_type[n] == mjJNT_FREE) {
            if (j >= 0) return false;
            j = n;
        }
    if (j < 0) return false;
    q_ = c.model->jnt_qposadr[j];
    d_ = c.model->jnt_dofadr[j];
    return true;
}
bool KinematicMecanumMobileBase::wheel(const mjContext& c, const MecanumWheelInfo& i, Wheel& w) {
    w.joint = mj_name2id(c.model, mjOBJ_JOINT, i.joint_name.c_str());
    if (w.joint < 0 || c.model->jnt_type[w.joint] != mjJNT_HINGE || c.model->jnt_limited[w.joint] ||
        !std::isfinite(i.radius) || i.radius <= 0 || !std::isfinite(i.speed_response) ||
        i.speed_response < 0 || (i.direction != -1 && i.direction != 1))
        return false;
    w.qpos = c.model->jnt_qposadr[w.joint];
    w.dof = c.model->jnt_dofadr[w.joint];
    w.radius = i.radius;
    w.direction = i.direction;
    w.response = i.speed_response;
    return true;
}
bool KinematicMecanumMobileBase::init(const mjContext& c) {
    if (info_.common.execution_mode != MobileBaseExecutionMode::Kinematic || !configure(c) ||
        !base(c))
        return false;
    std::unordered_set<int> s;
    for (size_t i = 0; i < wheels_.size(); ++i)
        if (!wheel(c, info_.wheels[i], wheels_[i]) || !s.insert(wheels_[i].joint).second)
            return false;
    ready_ = true;
    return reset(c);
}
bool KinematicMecanumMobileBase::reset(const mjContext& c) {
    if (!ready_) return false;
    auto p = c.model->qpos0 + q_;
    if (std::abs(p[4]) > 1e-10 || std::abs(p[5]) > 1e-10) return false;
    for (int i = 0; i < 7; ++i) c.data->qpos[q_ + i] = p[i];
    origin_ = {p[0], p[1], p[2]};
    origin_yaw_ = 2 * std::atan2(p[6], p[3]);
    x_ = y_ = yaw_ = 0;
    linear_ = {};
    angular_ = {};
    for (auto& w : wheels_) {
        w.target = w.feedback = 0;
        w.position = c.model->qpos0[w.qpos];
        c.data->qpos[w.qpos] = w.position;
        c.data->qvel[w.dof] = 0;
    }
    publish(c);
    state_ = std::make_shared<MobileBaseState>(working_);
    return true;
}
bool KinematicMecanumMobileBase::write(const mjContext&, const MobileBaseCommand& v) {
    if (!ready_ || v.id != info_.common.id || !std::isfinite(v.velocity.linear_x) ||
        !std::isfinite(v.velocity.linear_y) || !std::isfinite(v.velocity.angular_z))
        return false;
    Vector4d t{};
    ik_.inverse(v.velocity, t);
    for (size_t i = 0; i < wheels_.size(); ++i)
        wheels_[i].target = wheels_[i].direction * t[i] / wheels_[i].radius;
    return true;
}
bool KinematicMecanumMobileBase::advance(const mjContext& c) {
    if (!ready_) return false;
    double dt = c.model->opt.timestep;
    if (!std::isfinite(dt) || dt <= 0) return false;
    Vector4d w{};
    for (size_t i = 0; i < wheels_.size(); ++i) {
        auto& a = wheels_[i];
        a.feedback = a.response == 0
                         ? a.target
                         : a.feedback + (1 - std::exp(-dt / a.response)) * (a.target - a.feedback);
        a.position += a.feedback * dt;
        c.data->qpos[a.qpos] = a.position;
        c.data->qvel[a.dof] = a.feedback;
        w[i] = a.direction * a.radius * a.feedback;
    }
    ik_.forward(w, linear_, angular_);
    yaw_ = std::remainder(yaw_ + angular_[2] * dt, 2 * Pi);
    x_ += (std::cos(yaw_) * linear_[0] - std::sin(yaw_) * linear_[1]) * dt;
    y_ += (std::sin(yaw_) * linear_[0] + std::cos(yaw_) * linear_[1]) * dt;
    double a = origin_yaw_ + yaw_, co = std::cos(origin_yaw_), si = std::sin(origin_yaw_);
    c.data->qpos[q_] = origin_[0] + co * x_ - si * y_;
    c.data->qpos[q_ + 1] = origin_[1] + si * x_ + co * y_;
    c.data->qpos[q_ + 2] = origin_[2];
    c.data->qpos[q_ + 3] = std::cos(a / 2);
    c.data->qpos[q_ + 4] = c.data->qpos[q_ + 5] = 0;
    c.data->qpos[q_ + 6] = std::sin(a / 2);
    c.data->qvel[d_] = std::cos(a) * linear_[0] - std::sin(a) * linear_[1];
    c.data->qvel[d_ + 1] = std::sin(a) * linear_[0] + std::cos(a) * linear_[1];
    c.data->qvel[d_ + 2] = c.data->qvel[d_ + 3] = c.data->qvel[d_ + 4] = 0;
    c.data->qvel[d_ + 5] = angular_[2];
    return true;
}
void KinematicMecanumMobileBase::publish(const mjContext& c) {
    double a = origin_yaw_ + yaw_;
    working_.id = info_.common.id;
    working_.timestamp = c.data->time;
    working_.pose.position = {c.data->qpos[q_], c.data->qpos[q_ + 1], c.data->qpos[q_ + 2]};
    working_.pose.orientation = {std::cos(a / 2), 0, 0, std::sin(a / 2)};
    working_.twist.linear = linear_;
    working_.twist.angular = angular_;
}
bool KinematicMecanumMobileBase::update(const mjContext& c) {
    if (!ready_) return false;
    publish(c);
    state_ = std::make_shared<MobileBaseState>(working_);
    return true;
}
bool KinematicMecanumMobileBase::read_state(std::shared_ptr<const MobileBaseState>& s) const {
    s = state_;
    return ready_ && s != nullptr;
}
}  // namespace romujoco
