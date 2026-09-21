#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace robonode {

// Single-consumer command queue with identity. Every submission gets an id;
// `applied()` reports the last id the worker finished, so callers watch a
// number instead of polling for a side effect. Bounded (backpressure is a
// reported failure, not a memory leak) and coalescing: a pending command with
// the same key is replaced rather than queued twice.
template <class T>
class CommandBus {
public:
    struct Ack {
        std::uint64_t id{};
        bool accepted{};
        std::string reason;
        std::size_t depth{};
    };

    // `settled` runs after applied() has advanced, so anything it publishes
    // reports the command as finished.
    // The handler receives the id the submitter was given, so anything it logs
    // or records can be correlated with the acknowledgement.
    CommandBus(std::function<void(T&, std::uint64_t)> handler, std::function<void()> settled = {},
               std::size_t capacity = 64)
        : handler_{std::move(handler)}, settled_{std::move(settled)}, capacity_{capacity},
          thread_{[this] { loop(); }} {}

    ~CommandBus() {
        stop_.store(true);
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }

    CommandBus(const CommandBus&) = delete;
    CommandBus& operator=(const CommandBus&) = delete;

    Ack submit(T item, const std::string& coalesce_key = {}) {
        std::lock_guard<std::mutex> lk{mtx_};
        if (auto* pending = find(coalesce_key); pending != nullptr) {
            pending->item = std::move(item);
            cv_.notify_one();
            return {pending->id, true, "coalesced", queue_.size()};
        }
        if (queue_.size() >= capacity_) {
            return {0, false, "queue full", queue_.size()};
        }
        const auto id = ++last_id_;
        queue_.push_back({id, coalesce_key, std::move(item)});
        cv_.notify_one();
        return {id, true, {}, queue_.size()};
    }

    // Throw away everything not yet started, and report each of them applied.
    //
    // An e-stop means nothing new starts, and a queue of pending commands is
    // exactly "things about to start". Letting them run to be refused one by
    // one gives a log full of failures for a stop that worked. Dropping them
    // silently is worse: a caller waiting on `applied_id >= its id` would wait
    // for ever. So they are dropped AND accounted for — the ids advance, and
    // nothing pretends the work happened.
    std::size_t discard_pending() {
        std::lock_guard<std::mutex> lk{mtx_};
        const auto dropped = queue_.size();
        if (dropped > 0) {
            applied_.store(queue_.back().id);
            queue_.clear();
        }
        return dropped;
    }

    [[nodiscard]] std::uint64_t applied() const { return applied_.load(); }
    [[nodiscard]] std::uint64_t accepted() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return last_id_;
    }

private:
    struct Entry {
        std::uint64_t id;
        std::string key;
        T item;
    };

    Entry* find(const std::string& key) {
        if (key.empty()) return nullptr;
        for (auto& e : queue_) {
            if (e.key == key) return &e;
        }
        return nullptr;
    }

    void loop() {
        while (!stop_.load()) {
            Entry entry;
            {
                std::unique_lock<std::mutex> lk{mtx_};
                cv_.wait(lk, [this] { return stop_.load() || !queue_.empty(); });
                if (stop_.load()) return;
                entry = std::move(queue_.front());
                queue_.pop_front();
            }
            handler_(entry.item, entry.id);
            applied_.store(entry.id);
            if (settled_) settled_();
        }
    }

    std::function<void(T&, std::uint64_t)> handler_;
    std::function<void()> settled_;
    std::size_t capacity_;
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<Entry> queue_;
    std::uint64_t last_id_{0};
    std::atomic<std::uint64_t> applied_{0};
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

}  // namespace robonode
