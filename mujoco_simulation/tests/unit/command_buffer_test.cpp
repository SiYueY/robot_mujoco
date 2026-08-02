#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <thread>

#include "buffer/command_buffer.hpp"

struct AuxiliaryCommand {
  double target{0.0};
};

template <> struct mujoco_simulation::CommandTraits<AuxiliaryCommand> {
  static bool validate(const AuxiliaryCommand &command) {
    return std::isfinite(command.target);
  }
};

namespace {

bool check(bool condition, const char *message) {
  if (condition)
    return true;
  std::cerr << message << '\n';
  return false;
}

bool has_effort_command(const mujoco_simulation::CommandSnapshot &snapshot,
                        std::size_t id, double effort) {
  const auto *commands = snapshot.channel<mujoco_simulation::JointCommand>();
  return commands != nullptr && id < commands->size() &&
         (*commands)[id].has_value() &&
         (*commands)[id]->mode == mujoco_simulation::JointControlMode::Effort &&
         (*commands)[id]->effort == effort;
}

} // namespace

int main() {
  using mujoco_simulation::CommandBatch;
  using mujoco_simulation::CommandBuffer;
  using mujoco_simulation::CommandSnapshot;
  using mujoco_simulation::JointCommand;
  using mujoco_simulation::JointControlMode;

  CommandBuffer buffer;
  JointCommand effort;
  effort.mode = JointControlMode::Effort;
  effort.effort = 2.5;

  if (!check(!buffer.write(CommandBatch<JointCommand>{}),
             "uninitialized buffer accepted an empty command snapshot") ||
      !check(!buffer.write(0, effort),
             "uninitialized buffer accepted a joint command") ||
      !check(buffer.configure_channel<JointCommand>({true, false, true}) &&
                 buffer.configure_channel<mujoco_simulation::MobileBaseCommand>(
                     {false, true}) &&
                 buffer.finalize_configuration(),
             "failed to initialize command buffer") ||
      !check(!buffer.write(1, effort), "sparse joint id was accepted") ||
      !check(!buffer.write(1000000000U, effort),
             "large joint id was accepted") ||
      !check(buffer.write(2, effort), "valid sparse joint id was rejected") ||
      !check(has_effort_command(buffer.read(), 2, effort.effort),
             "valid joint command was not retained")) {
    return 1;
  }

  CommandSnapshot updated;
  const std::uint64_t current_sequence = buffer.read().sequence;
  if (!check(!buffer.read_if_updated(current_sequence, updated),
             "unchanged command buffer reported an update") ||
      !check(buffer.write(0, effort), "updated joint command was rejected") ||
      !check(buffer.read_if_updated(current_sequence, updated),
             "updated command buffer did not provide a snapshot") ||
      !check(has_effort_command(updated, 0, effort.effort),
             "updated command snapshot was incomplete")) {
    return 1;
  }

  mujoco_simulation::MobileBaseCommand twist;
  twist.mode = mujoco_simulation::MobileBaseControlMode::Twist;
  twist.base_linear[0] = 1.0;
  if (!check(!buffer.write(0, twist), "sparse mobile base id was accepted") ||
      !check(buffer.write(1, twist), "valid mobile base id was rejected")) {
    return 1;
  }

  JointCommand invalid_effort = effort;
  invalid_effort.effort = std::numeric_limits<double>::quiet_NaN();
  if (!check(!buffer.write(2, invalid_effort),
             "non-finite joint command was accepted") ||
      !check(has_effort_command(buffer.read(), 2, effort.effort),
             "invalid joint command replaced the previous command")) {
    return 1;
  }

  CommandBatch<JointCommand> oversized;
  oversized.slots.resize(4);
  if (!check(!buffer.write(oversized),
             "oversized command snapshot was accepted") ||
      !check(has_effort_command(buffer.read(), 2, effort.effort),
             "oversized snapshot replaced the previous command")) {
    return 1;
  }

  CommandBatch<mujoco_simulation::MobileBaseCommand> invalid_mobile_base;
  invalid_mobile_base.slots.resize(1);
  invalid_mobile_base.slots[0] = twist;
  if (!check(!buffer.write(invalid_mobile_base),
             "snapshot command for a sparse mobile base id was accepted") ||
      !check(has_effort_command(buffer.read(), 2, effort.effort),
             "invalid mobile base snapshot replaced the previous command")) {
    return 1;
  }

  CommandBatch<JointCommand> invalid_hole;
  invalid_hole.slots.resize(2);
  invalid_hole.slots[1] = effort;
  if (!check(!buffer.write(invalid_hole),
             "snapshot command for a sparse id was accepted") ||
      !check(has_effort_command(buffer.read(), 2, effort.effort),
             "invalid sparse snapshot replaced the previous command")) {
    return 1;
  }

  CommandBatch<JointCommand> invalid_value;
  invalid_value.slots.resize(3);
  invalid_value.slots[0] = invalid_effort;
  if (!check(!buffer.write(invalid_value),
             "snapshot with a non-finite joint command was accepted") ||
      !check(has_effort_command(buffer.read(), 2, effort.effort),
             "invalid value snapshot replaced the previous command")) {
    return 1;
  }

  CommandBatch<JointCommand> valid;
  valid.slots.resize(3);
  valid.slots[0] = effort;
  if (!check(buffer.write(valid), "valid command snapshot was rejected") ||
      !check(has_effort_command(buffer.read(), 0, effort.effort) &&
                 has_effort_command(buffer.read(), 2, effort.effort),
             "valid snapshot did not preserve configured command slots")) {
    return 1;
  }

  const std::uint64_t sequence_before_clear = buffer.read().sequence;
  buffer.clear();
  CommandSnapshot cleared;
  if (!check(buffer.read_if_updated(sequence_before_clear, cleared),
             "clear did not update the command sequence") ||
      !check(!cleared.channel<JointCommand>()->at(2).has_value(),
             "clear did not remove cached command values") ||
      !check(buffer.write(2, effort),
             "clear invalidated the configured command layout")) {
    return 1;
  }
  const std::uint64_t sequence_before_shutdown = buffer.read().sequence;
  buffer.shutdown();
  if (!check(buffer.read_if_updated(sequence_before_shutdown, cleared),
             "shutdown did not update the command sequence") ||
      !check(cleared.channel<JointCommand>() == nullptr,
             "shutdown did not release command slots") ||
      !check(!buffer.write(2, effort),
             "shutdown buffer accepted a joint command")) {
    return 1;
  }

  if (!check(buffer.configure_channel<JointCommand>({true}) &&
                 buffer.configure_channel<mujoco_simulation::MobileBaseCommand>(
                     {}) &&
                 buffer.finalize_configuration(),
             "failed to reinitialize command bus for concurrency test")) {
    return 1;
  }
  std::atomic<bool> writer_done{false};
  std::atomic<bool> reader_failed{false};
  std::thread writer([&] {
    for (std::size_t iteration = 0; iteration < 1000U; ++iteration) {
      JointCommand command = effort;
      command.effort = static_cast<double>(iteration);
      if (!buffer.write(0, command)) {
        reader_failed.store(true);
        break;
      }
    }
    writer_done.store(true);
  });
  std::thread reader([&] {
    std::uint64_t sequence = buffer.read().sequence;
    CommandSnapshot snapshot;
    while (!writer_done.load()) {
      if (buffer.read_if_updated(sequence, snapshot)) {
        sequence = snapshot.sequence;
        const auto *commands = snapshot.channel<JointCommand>();
        if (commands == nullptr || !commands->at(0).has_value() ||
            !has_effort_command(snapshot, 0, commands->at(0)->effort))
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

  // Registering another controlled component type requires neither a
  // CommandBuffer member nor a CommandSnapshot field.
  buffer.shutdown();
  AuxiliaryCommand auxiliary{3.0};
  if (!check(buffer.configure_channel<AuxiliaryCommand>({true, false}) &&
                 buffer.finalize_configuration() && buffer.write(0, auxiliary),
             "custom typed command channel could not be registered") ||
      !check(buffer.read().channel<AuxiliaryCommand>()->at(0)->target == 3.0,
             "custom typed command channel was not published")) {
    return 1;
  }

  return 0;
}
