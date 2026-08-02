#include "buffer/command_buffer.hpp"

namespace mujoco_simulation {

bool CommandBuffer::finalize_configuration() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_ || channels_.empty())
    return false;
  initialized_ = true;
  ++snapshot_.sequence;
  return true;
}

CommandSnapshot CommandBuffer::read() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

bool CommandBuffer::read_if_updated(std::uint64_t last_sequence,
                                    CommandSnapshot &out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (snapshot_.sequence == last_sequence)
    return false;
  out = snapshot_;
  return true;
}

void CommandBuffer::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto &[_, channel] : channels_)
    channel->clear();
  for (auto &[type, channel] : channels_)
    snapshot_.channels_[type] = channel->snapshot();
  ++snapshot_.sequence;
}

void CommandBuffer::shutdown() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto &[_, channel] : channels_)
    channel->shutdown();
  channels_.clear();
  snapshot_.channels_.clear();
  initialized_ = false;
  ++snapshot_.sequence;
}

} // namespace mujoco_simulation
