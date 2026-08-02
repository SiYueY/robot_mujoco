#pragma once
// Internal command channel contract.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include "data/command_snapshot.hpp"
#include "mujoco_simulation/component/component_id.hpp"

namespace mujoco_simulation {

template <typename Command> struct CommandTraits;

template <typename Command, typename = void>
struct has_command_traits : std::false_type {};

template <typename Command>
struct has_command_traits<Command,
                          std::void_t<decltype(CommandTraits<Command>::validate(
                              std::declval<const Command &>()))>>
    : std::true_type {};

template <typename Command>
constexpr bool has_command_traits_v = has_command_traits<Command>::value;

// Cold-path layout bitmap. Unlike vector<bool>, this has ordinary reference
// semantics and remains easy to inspect in diagnostics and tests.
using CommandChannelLayout = std::vector<std::uint8_t>;

template <typename Command> class CommandChannel {
public:
  static_assert(has_command_traits_v<Command>,
                "CommandChannel requires CommandTraits<T>::validate(const T&)");
  using Slots = std::vector<std::optional<Command>>;

  using Snapshot = CommandChannelSnapshot<Command>;

  void initialize(CommandChannelLayout valid_ids) {
    valid_ids_ = std::move(valid_ids);
    auto snapshot = std::make_shared<Snapshot>();
    snapshot->slots.assign(valid_ids_.size(), std::optional<Command>{});
    snapshot_ = std::move(snapshot);
  }

  bool write(ComponentId id, const Command &command) {
    if (id >= valid_ids_.size() || !valid_ids_[id] ||
        !CommandTraits<Command>::validate(command))
      return false;
    auto updated = std::make_shared<Snapshot>(*snapshot_);
    updated->slots[id] = command;
    snapshot_ = std::move(updated);
    return true;
  }

  bool apply(const Slots &updates) {
    if (updates.size() > valid_ids_.size())
      return false;
    for (std::size_t id = 0; id < updates.size(); ++id) {
      if (updates[id].has_value() &&
          (!valid_ids_[id] || !CommandTraits<Command>::validate(*updates[id])))
        return false;
    }
    auto updated = std::make_shared<Snapshot>(*snapshot_);
    for (std::size_t id = 0; id < updates.size(); ++id)
      if (updates[id].has_value())
        updated->slots[id] = updates[id];
    snapshot_ = std::move(updated);
    return true;
  }

  std::shared_ptr<const Snapshot> snapshot() const noexcept {
    return snapshot_;
  }
  void clear() {
    auto cleared = std::make_shared<Snapshot>();
    cleared->slots.assign(valid_ids_.size(), std::optional<Command>{});
    snapshot_ = std::move(cleared);
  }
  void shutdown() {
    valid_ids_.clear();
    snapshot_.reset();
  }

private:
  CommandChannelLayout valid_ids_;
  std::shared_ptr<const Snapshot> snapshot_;
};

} // namespace mujoco_simulation
