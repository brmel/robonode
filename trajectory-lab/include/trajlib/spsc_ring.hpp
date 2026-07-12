#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace trajlib {

// Single-producer single-consumer lock-free ring buffer.
//
// The pattern that lets an RT thread exchange data with a non-RT thread
// without mutexes (no priority inversion, push/pop are wait-free):
//   - producer owns head_, consumer owns tail_
//   - producer: write slot THEN release-store head_  (publishes the data)
//   - consumer: acquire-load head_ THEN read slot    (sees published data)
// One slot is sacrificed to distinguish full from empty.
//
// alignas(64) keeps head_ and tail_ on separate cache lines — without it
// the two cores ping-pong one line (false sharing) on every push/pop.
template <typename T, std::size_t Capacity>
class SpscRing {
    static_assert(Capacity >= 2, "need at least 2 slots");

public:
    // Producer side. Returns false when full (caller decides: drop or count).
    bool push(const T& item) {
        const auto head = head_.load(std::memory_order_relaxed);
        const auto next = (head + 1) % Capacity;
        if (next == tail_.load(std::memory_order_acquire)) {
            return false;  // full
        }
        slots_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // Consumer side.
    std::optional<T> pop() {
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return std::nullopt;  // empty
        }
        T item = slots_[tail];
        tail_.store((tail + 1) % Capacity, std::memory_order_release);
        return item;
    }

    [[nodiscard]] bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

private:
    std::array<T, Capacity> slots_{};
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
};

}  // namespace trajlib
