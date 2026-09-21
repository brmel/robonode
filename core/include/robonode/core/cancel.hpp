#pragma once

#include <atomic>

namespace robonode {

// A stop request the RT loop can read every cycle: one relaxed atomic load, no
// allocation, no lock. Set from any thread; cleared by the owner before a run.
class CancelToken {
public:
    void request() noexcept { stop_.store(true, std::memory_order_relaxed); }
    void clear() noexcept { stop_.store(false, std::memory_order_relaxed); }
    [[nodiscard]] bool requested() const noexcept { return stop_.load(std::memory_order_relaxed); }

private:
    std::atomic<bool> stop_{false};
};

}  // namespace robonode
