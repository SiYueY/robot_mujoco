#pragma once

#include <memory>
#include <vector>

namespace romujoco {

// Reuse unpublished storage while keeping every shared snapshot immutable to
// readers. Retained readers can force the pool to grow to their high-water mark.
template <typename T>
class SnapshotPool {
public:
    std::shared_ptr<T> acquire(const T* initial = nullptr) {
        for (const auto& entry : entries_)
            if (entry.use_count() == 1) return entry;
        auto entry = initial != nullptr ? std::make_shared<T>(*initial) : std::make_shared<T>();
        entries_.push_back(entry);
        return entry;
    }

    void clear() { entries_.clear(); }

private:
    std::vector<std::shared_ptr<T>> entries_;
};

}  // namespace romujoco
