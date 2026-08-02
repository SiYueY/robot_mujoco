#pragma once
// Internal command snapshot type.

#include <cstdint>
#include <memory>
#include <optional>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace mujoco_simulation {

// A typed command batch describes caller-provided updates for one command
// channel. Empty slots leave the currently buffered command unchanged.
template <typename Command> struct CommandBatch {
  std::vector<std::optional<Command>> slots;
};

template <typename Command> struct CommandChannelSnapshot {
  std::vector<std::optional<Command>> slots;
};

// CommandSnapshot is the immutable, type-erased collection consumed by the
// physics thread. It deliberately has no knowledge of component kinds.
class CommandSnapshot {
public:
  template <typename Command>
  const std::vector<std::optional<Command>> *channel() const noexcept {
    const auto found = channels_.find(std::type_index(typeid(Command)));
    if (found == channels_.end())
      return nullptr;
    const auto typed =
        std::static_pointer_cast<const CommandChannelSnapshot<Command>>(
            found->second);
    return &typed->slots;
  }

  std::uint64_t sequence{0};

private:
  std::unordered_map<std::type_index, std::shared_ptr<const void>> channels_;

  friend class CommandBuffer;
};

} // namespace mujoco_simulation
