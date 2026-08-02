#pragma once
// Internal command buffering contract; not part of the installed API.

#include <cmath>
#include <memory>
#include <mutex>
#include <typeindex>
#include <unordered_map>
#include <utility>

#include "buffer/command_channel.hpp"
#include "data/command_snapshot.hpp"
#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"
#include "mujoco_simulation/data/robot_command.hpp"

namespace mujoco_simulation {

template <> struct CommandTraits<JointCommand> {
  static bool validate(const JointCommand &command) {
    switch (command.mode) {
    case JointControlMode::Position:
      return std::isfinite(command.position);
    case JointControlMode::Velocity:
      return std::isfinite(command.velocity);
    case JointControlMode::Effort:
      return std::isfinite(command.effort);
    case JointControlMode::Hybrid:
      return std::isfinite(command.position) &&
             std::isfinite(command.velocity) && std::isfinite(command.effort) &&
             std::isfinite(command.stiffness) && std::isfinite(command.damping);
    }
    return false;
  }
};

template <> struct CommandTraits<MobileBaseCommand> {
  static bool validate(const MobileBaseCommand &command) {
    switch (command.mode) {
    case MobileBaseControlMode::Twist:
      return std::isfinite(command.base_linear[0]) &&
             std::isfinite(command.base_linear[1]) &&
             std::isfinite(command.base_angular[2]);
    case MobileBaseControlMode::WheelLinear:
      for (double value : command.wheel_linear)
        if (!std::isfinite(value))
          return false;
      return true;
    case MobileBaseControlMode::WheelAngular:
      for (double value : command.wheel_angular)
        if (!std::isfinite(value))
          return false;
      return true;
    }
    return false;
  }
};

class CommandBuffer {
public:
  template <typename Command>
  bool configure_channel(CommandChannelLayout valid_ids) {
    static_assert(has_command_traits_v<Command>,
                  "Command type needs CommandTraits::validate");
    std::lock_guard<std::mutex> lock(mutex_);
    if (initialized_ || channels_.count(std::type_index(typeid(Command))) != 0)
      return false;
    auto channel = std::make_unique<ChannelHolder<Command>>();
    channel->channel.initialize(std::move(valid_ids));
    snapshot_.channels_[std::type_index(typeid(Command))] = channel->snapshot();
    channels_.emplace(std::type_index(typeid(Command)), std::move(channel));
    return true;
  }
  bool finalize_configuration();
  template <typename Command>
  bool write(ComponentId id, const Command &command) {
    static_assert(has_command_traits_v<Command>,
                  "Command type needs CommandTraits::validate");
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_)
      return false;
    auto *channel = find_channel<Command>();
    if (channel == nullptr || !channel->channel.write(id, command))
      return false;
    publish(*channel);
    return true;
  }
  template <typename Command> bool write(const CommandBatch<Command> &batch) {
    static_assert(has_command_traits_v<Command>,
                  "Command type needs CommandTraits::validate");
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_)
      return false;
    auto *channel = find_channel<Command>();
    if (channel == nullptr || !channel->channel.apply(batch.slots))
      return false;
    publish(*channel);
    return true;
  }
  bool write(const RobotCommand &command);
  CommandSnapshot read() const;
  bool read_if_updated(std::uint64_t last_sequence, CommandSnapshot &out) const;
  void clear();
  void shutdown();

private:
  class ChannelBase {
  public:
    virtual ~ChannelBase() = default;
    virtual std::shared_ptr<const void> snapshot() const = 0;
    virtual void clear() = 0;
    virtual void shutdown() = 0;
  };
  template <typename Command> class ChannelHolder final : public ChannelBase {
  public:
    std::shared_ptr<const void> snapshot() const override {
      return channel.snapshot();
    }
    void clear() override { channel.clear(); }
    void shutdown() override { channel.shutdown(); }
    CommandChannel<Command> channel;
  };
  template <typename Command> ChannelHolder<Command> *find_channel() {
    const auto found = channels_.find(std::type_index(typeid(Command)));
    return found == channels_.end()
               ? nullptr
               : static_cast<ChannelHolder<Command> *>(found->second.get());
  }
  template <typename Command> void publish(ChannelHolder<Command> &channel) {
    snapshot_.channels_[std::type_index(typeid(Command))] = channel.snapshot();
    ++snapshot_.sequence;
  }
  mutable std::mutex mutex_;
  bool initialized_{false};
  std::unordered_map<std::type_index, std::unique_ptr<ChannelBase>> channels_;
  CommandSnapshot snapshot_;
};
} // namespace mujoco_simulation
