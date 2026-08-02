#pragma once
// Internal command channel contract.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "component/component.hpp"
#include "data/command_snapshot.hpp"

namespace mujoco_simulation {

template <typename Command>
struct CommandTraits;

template <typename Command, typename = void>
struct has_command_traits : std::false_type {};

template <typename Command>
struct has_command_traits<
    Command,
    std::void_t<decltype(CommandTraits<Command>::validate(std::declval<const Command&>()))>>
: std::true_type {};

template <typename Command>
constexpr bool has_command_traits_v = has_command_traits<Command>::value;

// Cold-path layout bitmap. Unlike vector<bool>, this has ordinary reference
// semantics and remains easy to inspect in diagnostics and tests.
using CommandChannelLayout = std::vector<std::uint8_t>;

template <typename Command>
class CommandChannel {
public:
    static_assert(
        has_command_traits_v<Command>,
        "CommandChannel requires CommandTraits<T>::validate(const T&)");
    using Slots = std::vector<Command>;

    using Snapshot = CommandChannelSnapshot<Command>;

    // Command channels are dense: every slot in [0, size) must be a valid
    // component id, so a batch vector can be indexed directly by component id.
    bool initialize(CommandChannelLayout valid_ids) {
        if (!valid_ids.empty() &&
            std::find(valid_ids.begin(), valid_ids.end(), 0U) != valid_ids.end())
            return false;
        valid_ids_ = std::move(valid_ids);
        auto snapshot = std::make_shared<Snapshot>();
        snapshot->slots.assign(valid_ids_.size(), Command{});
        snapshot_ = std::move(snapshot);
        return true;
    }

    bool write(ComponentId id, const Command& command) {
        if (id >= valid_ids_.size() || !valid_ids_[id] ||
            !CommandTraits<Command>::validate(command))
            return false;
        auto updated = std::make_shared<Snapshot>(*snapshot_);
        updated->slots[id] = command;
        snapshot_ = std::move(updated);
        return true;
    }

    bool apply(const Slots& updates) {
        if (!validate(updates)) return false;
        snapshot_ = updated_snapshot(updates);
        return true;
    }

    bool validate(const Slots& updates) const {
        if (updates.size() != valid_ids_.size()) return false;
        for (std::size_t id = 0; id < updates.size(); ++id) {
            if (!valid_ids_[id] || !CommandTraits<Command>::validate(updates[id])) return false;
        }
        return true;
    }

    std::shared_ptr<const Snapshot> updated_snapshot(const Slots& updates) const {
        auto updated = std::make_shared<Snapshot>();
        updated->slots = updates;
        return updated;
    }

    void replace_snapshot(std::shared_ptr<const Snapshot> snapshot) noexcept {
        snapshot_ = std::move(snapshot);
    }

    std::shared_ptr<const Snapshot> snapshot() const noexcept { return snapshot_; }
    void clear() {
        auto cleared = std::make_shared<Snapshot>();
        cleared->slots.assign(valid_ids_.size(), Command{});
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

}  // namespace mujoco_simulation
