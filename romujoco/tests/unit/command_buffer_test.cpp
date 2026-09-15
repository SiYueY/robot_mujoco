#include <atomic>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <thread>

#include "buffer/command_buffer.hpp"
#include "component/component_id.hpp"

namespace {

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

const romujoco::JointCommand* find_joint(
    const romujoco::RobotCommand& command, romujoco::JointId id) {
    for (const auto& joint : command.joints)
        if (joint.id == id) return &joint;
    return nullptr;
}

const romujoco::MobileBaseCommand* find_mobile_base(
    const romujoco::RobotCommand& command, romujoco::MobileBaseId id) {
    for (const auto& mobile_base : command.mobile_bases)
        if (mobile_base.id == id) return &mobile_base;
    return nullptr;
}

std::shared_ptr<const romujoco::ComponentIdResolver> make_resolver(
    std::initializer_list<std::size_t> joint_ids, std::initializer_list<std::size_t> mobile_ids,
    romujoco::ComponentConfigList& components) {
    components.clear();
    for (std::size_t id : joint_ids) {
        romujoco::JointInfo info;
        info.id = id;
        info.default_mode = romujoco::JointMode::Effort;
        info.allowed_modes = {romujoco::JointMode::Effort};
        components.push_back(std::move(info));
    }
    for (std::size_t id : mobile_ids) {
        romujoco::MobileBaseInfo info;
        info.id = id;
        components.push_back(std::move(info));
    }
    return romujoco::ComponentIdResolver::create(components);
}

}  // namespace

int main() {
    using romujoco::CommandBuffer;
    using romujoco::JointCommand;
    using romujoco::JointCommands;
    using romujoco::JointMode;
    using romujoco::MobileBaseCommand;

    JointCommand effort;
    effort.id = 0;
    effort.mode = static_cast<std::uint8_t>(JointMode::Effort);
    effort.effort = 2.5;
    JointCommand lighter = effort;
    lighter.id = 2;
    lighter.effort = 1.5;
    MobileBaseCommand twist;
    twist.id = 0;
    twist.mode = romujoco::MobileBaseControlMode::Twist;
    twist.base_linear[0] = 1.0;

    CommandBuffer buffer;
    romujoco::ComponentConfigList components;
    if (!check(!buffer.write(effort), "uninitialized buffer accepted a command") ||
        !check(
            buffer.configure(make_resolver({0, 2}, {0, 2}, components), components),
            "sparse command channels were rejected")) {
        return 1;
    }
    const auto initial = buffer.read();
    if (!check(
            initial != nullptr && initial->joints.size() == 2U && initial->joints[0].id == 0U &&
                initial->joints[1].id == 2U && initial->mobile_bases.size() == 2U &&
                initial->mobile_bases[0].id == 0U && initial->mobile_bases[1].id == 2U,
            "initial command snapshot was not compact and ID ordered")) {
        return 1;
    }

    if (!check(
            buffer.write(JointCommands{lighter, effort}), "unordered sparse batch was rejected") ||
        !check(buffer.write(twist), "single mobile base command was rejected")) {
        return 1;
    }
    const auto after_batch = buffer.read();
    if (!check(
            find_joint(*after_batch, 0U) != nullptr &&
                find_joint(*after_batch, 0U)->effort == 2.5 &&
                find_joint(*after_batch, 2U) != nullptr &&
                find_joint(*after_batch, 2U)->effort == 1.5 &&
                find_mobile_base(*after_batch, 0U) != nullptr,
            "sparse batch did not merge into the current command snapshot")) {
        return 1;
    }

    JointCommand update = lighter;
    update.effort = 4.0;
    const std::uint64_t sequence_before_update = buffer.read()->sequence;
    if (!check(buffer.write(update), "partial command update was rejected") ||
        !check(
            buffer.read()->sequence == sequence_before_update + 1U &&
                find_joint(*buffer.read(), 0U)->effort == 2.5 &&
                find_joint(*buffer.read(), 2U)->effort == 4.0,
            "partial command update did not preserve prior commands")) {
        return 1;
    }

    JointCommand duplicate = effort;
    duplicate.effort = 9.0;
    JointCommand invalid = update;
    invalid.effort = std::numeric_limits<double>::quiet_NaN();
    JointCommand unknown = update;
    unknown.id = 1;
    JointCommand out_of_range = update;
    out_of_range.id = std::numeric_limits<romujoco::JointId>::max();
    const std::uint64_t sequence_before_rejection = buffer.read()->sequence;
    if (!check(
            !buffer.write(JointCommands{effort, duplicate}),
            "duplicate command IDs were accepted") ||
        !check(!buffer.write(JointCommands{unknown}), "unconfigured command ID was accepted") ||
        !check(!buffer.write(JointCommands{invalid}), "invalid command value was accepted") ||
        !check(
            !buffer.write(JointCommands{out_of_range}), "out-of-range command ID was accepted") ||
        !check(
            buffer.read()->sequence == sequence_before_rejection &&
                find_joint(*buffer.read(), 2U)->effort == 4.0,
            "rejected command batch changed the snapshot")) {
        return 1;
    }

    romujoco::RobotCommand atomic;
    MobileBaseCommand mobile_update = twist;
    mobile_update.id = 2;
    mobile_update.base_linear[0] = 3.0;
    atomic.joints = {effort};
    atomic.mobile_bases = {mobile_update};
    if (!check(buffer.write(atomic), "atomic sparse robot command was rejected") ||
        !check(
            find_joint(*buffer.read(), 0U)->effort == 2.5 &&
                find_mobile_base(*buffer.read(), 2U)->base_linear[0] == 3.0,
            "atomic sparse robot command was not merged")) {
        return 1;
    }

    buffer.clear();
    if (!check(
            buffer.read()->joints.size() == 2U && buffer.read()->joints[0].id == 0U &&
                buffer.read()->joints[1].id == 2U && buffer.write(update),
            "clear did not retain sparse command identities")) {
        return 1;
    }
    buffer.shutdown();
    if (!check(!buffer.write(update), "shutdown buffer accepted a command")) return 1;

    if (!check(
            buffer.configure(make_resolver({0}, {0}, components), components),
            "failed to reinitialize command buffer")) {
        return 1;
    }
    std::atomic<bool> writer_done{false};
    std::atomic<bool> reader_failed{false};
    std::thread writer([&] {
        for (std::size_t iteration = 0; iteration < 1000U; ++iteration) {
            JointCommand command = effort;
            command.id = 0;
            command.effort = static_cast<double>(iteration);
            if (!buffer.write(command)) reader_failed.store(true);
        }
        writer_done.store(true);
    });
    std::thread reader([&] {
        std::uint64_t sequence = buffer.read()->sequence;
        std::shared_ptr<const romujoco::RobotCommand> snapshot;
        while (!writer_done.load()) {
            if (buffer.read(sequence, snapshot)) {
                sequence = snapshot->sequence;
                if (snapshot == nullptr || find_joint(*snapshot, 0U) == nullptr)
                    reader_failed.store(true);
            }
        }
    });
    writer.join();
    reader.join();
    return check(!reader_failed.load(), "concurrent command access produced an invalid snapshot")
               ? 0
               : 1;
}
