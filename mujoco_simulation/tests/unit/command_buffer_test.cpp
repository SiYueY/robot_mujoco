#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <thread>

#include "buffer/command_buffer.hpp"

namespace {

bool check(bool condition, const char* message) {
    if (condition) return true;
    std::cerr << message << '\n';
    return false;
}

bool has_effort_command(
    const mujoco_simulation::RobotCommand& command, std::size_t id, double effort) {
    return id < command.joints.size() &&
           command.joints[id].mode == mujoco_simulation::JointControlMode::Effort &&
           command.joints[id].effort == effort;
}

bool has_twist_command(
    const mujoco_simulation::RobotCommand& command, std::size_t id, double forward) {
    return id < command.mobile_bases.size() &&
           command.mobile_bases[id].mode == mujoco_simulation::MobileBaseControlMode::Twist &&
           command.mobile_bases[id].base_linear[0] == forward;
}

}  // namespace

int main() {
    using mujoco_simulation::CommandBuffer;
    using mujoco_simulation::JointCommand;
    using mujoco_simulation::JointCommands;
    using mujoco_simulation::JointControlMode;
    using mujoco_simulation::MobileBaseCommands;

    CommandBuffer buffer;
    JointCommand effort;
    effort.mode = JointControlMode::Effort;
    effort.effort = 2.5;

    if (!check(
            !buffer.write(JointCommands{}),
            "uninitialized buffer accepted an empty command snapshot") ||
        !check(!buffer.write(0, effort), "uninitialized buffer accepted a joint command") ||
        !check(
            buffer.configure_channels({true, true, true}, {true, true}) &&
                buffer.finalize_configuration(),
            "failed to initialize command buffer") ||
        !check(buffer.write(1, effort), "valid joint id was rejected") ||
        !check(!buffer.write(1000000000U, effort), "large joint id was accepted") ||
        !check(buffer.write(2, effort), "valid joint id was rejected") ||
        !check(
            has_effort_command(*buffer.read(), 2, effort.effort),
            "valid joint command was not retained")) {
        return 1;
    }

    std::shared_ptr<const mujoco_simulation::RobotCommand> updated;
    const std::uint64_t current_sequence = buffer.read()->sequence;
    if (!check(
            !buffer.read_if_updated(current_sequence, updated),
            "unchanged command buffer reported an update") ||
        !check(buffer.write(0, effort), "updated joint command was rejected") ||
        !check(
            buffer.read_if_updated(current_sequence, updated),
            "updated command buffer did not provide a snapshot") ||
        !check(
            has_effort_command(*updated, 0, effort.effort),
            "updated command snapshot was incomplete")) {
        return 1;
    }

    mujoco_simulation::MobileBaseCommand twist;
    twist.mode = mujoco_simulation::MobileBaseControlMode::Twist;
    twist.base_linear[0] = 1.0;
    if (!check(buffer.write(0, twist), "valid mobile base id was rejected") ||
        !check(buffer.write(1, twist), "valid mobile base id was rejected")) {
        return 1;
    }

    JointCommand invalid_effort = effort;
    invalid_effort.effort = std::numeric_limits<double>::quiet_NaN();
    if (!check(!buffer.write(2, invalid_effort), "non-finite joint command was accepted") ||
        !check(
            has_effort_command(*buffer.read(), 2, effort.effort),
            "invalid joint command replaced the previous command")) {
        return 1;
    }

    JointCommands oversized;
    oversized.resize(4);
    if (!check(!buffer.write(oversized), "oversized command snapshot was accepted") ||
        !check(
            has_effort_command(*buffer.read(), 2, effort.effort),
            "oversized snapshot replaced the previous command")) {
        return 1;
    }

    MobileBaseCommands undersized_base;
    undersized_base.resize(1);
    undersized_base[0] = twist;
    if (!check(!buffer.write(undersized_base), "undersized mobile base snapshot was accepted") ||
        !check(
            has_effort_command(*buffer.read(), 2, effort.effort),
            "undersized mobile base snapshot replaced the previous command")) {
        return 1;
    }

    JointCommands undersized_joints;
    undersized_joints.resize(2);
    undersized_joints[1] = effort;
    if (!check(!buffer.write(undersized_joints), "undersized joint snapshot was accepted") ||
        !check(
            has_effort_command(*buffer.read(), 2, effort.effort),
            "undersized joint snapshot replaced the previous command")) {
        return 1;
    }

    JointCommands invalid_value;
    invalid_value.resize(3);
    invalid_value[0] = invalid_effort;
    if (!check(
            !buffer.write(invalid_value),
            "snapshot with a non-finite joint command was accepted") ||
        !check(
            has_effort_command(*buffer.read(), 2, effort.effort),
            "invalid value snapshot replaced the previous command")) {
        return 1;
    }

    JointCommand lighter = effort;
    lighter.effort = 1.5;
    JointCommands valid;
    valid.resize(3);
    valid[0] = effort;
    valid[1] = lighter;
    valid[2] = effort;
    if (!check(buffer.write(valid), "valid command snapshot was rejected") ||
        !check(
            has_effort_command(*buffer.read(), 0, effort.effort) &&
                has_effort_command(*buffer.read(), 1, lighter.effort) &&
                has_effort_command(*buffer.read(), 2, effort.effort),
            "valid snapshot did not replace the command slots")) {
        return 1;
    }

    const std::uint64_t sequence_before_empty_batch = buffer.read()->sequence;
    if (!check(buffer.write(JointCommands{}), "empty command batch was rejected") ||
        !check(
            buffer.read()->sequence == sequence_before_empty_batch,
            "empty command batch changed the command sequence")) {
        return 1;
    }

    mujoco_simulation::RobotCommand atomic_command;
    atomic_command.joints.resize(3);
    atomic_command.joints[0] = effort;
    atomic_command.joints[0].effort = 7.0;
    atomic_command.joints[1] = effort;
    atomic_command.joints[2] = effort;
    atomic_command.mobile_bases.resize(2);
    atomic_command.mobile_bases[0] = twist;
    atomic_command.mobile_bases[1] = twist;
    atomic_command.mobile_bases[1].base_linear[0] = 3.0;
    const std::uint64_t sequence_before_atomic = buffer.read()->sequence;
    if (!check(buffer.write(atomic_command), "atomic robot command was rejected") ||
        !check(
            buffer.read()->sequence == sequence_before_atomic + 1U,
            "atomic robot command did not publish exactly one sequence") ||
        !check(
            has_effort_command(*buffer.read(), 0, 7.0) && has_twist_command(*buffer.read(), 1, 3.0),
            "atomic robot command did not publish both component types")) {
        return 1;
    }

    mujoco_simulation::RobotCommand invalid_atomic = atomic_command;
    invalid_atomic.joints.resize(2);
    invalid_atomic.joints[1] = effort;
    invalid_atomic.mobile_bases[1].base_linear[0] = 9.0;
    const std::uint64_t sequence_before_rejection = buffer.read()->sequence;
    if (!check(!buffer.write(invalid_atomic), "invalid atomic robot command was accepted") ||
        !check(
            buffer.read()->sequence == sequence_before_rejection &&
                has_effort_command(*buffer.read(), 0, 7.0) &&
                has_twist_command(*buffer.read(), 1, 3.0),
            "invalid atomic robot command partially updated a channel")) {
        return 1;
    }

    const std::uint64_t sequence_before_empty = buffer.read()->sequence;
    if (!check(
            buffer.write(mujoco_simulation::RobotCommand{}), "empty robot command was rejected") ||
        !check(
            buffer.read()->sequence == sequence_before_empty,
            "empty robot command changed the command sequence")) {
        return 1;
    }

    const std::uint64_t sequence_before_clear = buffer.read()->sequence;
    buffer.clear();
    std::shared_ptr<const mujoco_simulation::RobotCommand> cleared;
    if (!check(
            buffer.read_if_updated(sequence_before_clear, cleared),
            "clear did not update the command sequence") ||
        !check(
            cleared != nullptr && cleared->joints.at(2).mode == JointControlMode::Effort &&
                cleared->joints.at(2).effort == 0.0,
            "clear did not reset cached command values") ||
        !check(buffer.write(2, effort), "clear invalidated the configured command layout")) {
        return 1;
    }
    const std::uint64_t sequence_before_shutdown = buffer.read()->sequence;
    buffer.shutdown();
    if (!check(buffer.read() == nullptr, "shutdown did not release command slots") ||
        !check(!buffer.write(2, effort), "shutdown buffer accepted a joint command")) {
        return 1;
    }

    if (!check(
            buffer.configure_channels({true}, {true}) && buffer.finalize_configuration(),
            "failed to reinitialize command bus for concurrency test")) {
        return 1;
    }
    std::atomic<bool> writer_done{false};
    std::atomic<bool> reader_failed{false};
    std::thread writer([&] {
        for (std::size_t iteration = 0; iteration < 1000U; ++iteration) {
            JointCommand command = effort;
            command.effort = static_cast<double>(iteration);
            mujoco_simulation::MobileBaseCommand mobile_command = twist;
            mobile_command.base_linear[0] = static_cast<double>(iteration);
            mujoco_simulation::RobotCommand robot_command;
            robot_command.joints.resize(1);
            robot_command.joints[0] = command;
            robot_command.mobile_bases.resize(1);
            robot_command.mobile_bases[0] = mobile_command;
            if (!buffer.write(robot_command)) {
                reader_failed.store(true);
                break;
            }
        }
        writer_done.store(true);
    });
    std::thread reader([&] {
        std::uint64_t sequence = buffer.read()->sequence;
        std::shared_ptr<const mujoco_simulation::RobotCommand> snapshot;
        while (!writer_done.load()) {
            if (buffer.read_if_updated(sequence, snapshot)) {
                sequence = snapshot->sequence;
                if (snapshot == nullptr || snapshot->joints.empty() ||
                    snapshot->mobile_bases.empty() ||
                    !has_effort_command(*snapshot, 0, snapshot->joints.at(0).effort) ||
                    !has_twist_command(*snapshot, 0, snapshot->mobile_bases.at(0).base_linear[0]) ||
                    snapshot->joints.at(0).effort != snapshot->mobile_bases.at(0).base_linear[0])
                    reader_failed.store(true);
            }
        }
    });
    writer.join();
    reader.join();
    if (!check(
            !reader_failed.load(),
            "concurrent command bus read/write produced an invalid snapshot")) {
        return 1;
    }

    return 0;
}
