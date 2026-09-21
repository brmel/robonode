#pragma once

#include <functional>
#include <mutex>

#include <nlohmann/json.hpp>

#include "robonode/gateway/command_router.hpp"
#include <string>

#include "robonode/core/cancel.hpp"
#include "robonode/core/status.hpp"
#include "robonode/log.hpp"

namespace robonode {

enum class CellState { kIdle, kBuilding, kPlanning, kMoving, kHeld, kFaulted };

// The cell's declared state, its stop request, and the last failure — the three
// facts every surface needs and none of them could previously see.
//
// E-stop here is a SOFTWARE stop: it cancels the running motion (on-path
// deceleration) and latches the cell so nothing new starts until resume().
// It is not, and must never be presented as, a safety function (SPEC I4) —
// that lives in hardware.
class CellSupervisor {
public:
    Status begin(CellState next) {
        std::lock_guard<std::mutex> lk{mtx_};
        if (latched_) return Status::failure("cell is latched — resume first");
        state_ = next;
        return Status::success();
    }

    // A command that succeeded clears the last failure: `last_error` describes
    // the most recent outcome, not the worst one ever seen. Leaving it sticky
    // makes every later wait report a fault that is already over.
    void end(CellState next = CellState::kIdle) {
        std::lock_guard<std::mutex> lk{mtx_};
        if (!latched_) {
            state_ = next;
            last_error_.clear();
        }
        cancel_.clear();
    }

    void fault(const std::string& why) {
        std::lock_guard<std::mutex> lk{mtx_};
        last_error_ = why;
        if (!latched_) state_ = CellState::kFaulted;
    }

    Status stop() {
        cancel_.request();
        RN_LOG_WARN("stop requested — decelerating on path");
        return Status::success();
    }

    Status estop() {
        {
            std::lock_guard<std::mutex> lk{mtx_};
            latched_ = true;
            state_ = CellState::kHeld;
        }
        cancel_.request();
        RN_LOG_ERROR("E-STOP latched (software stop — not a safety function)");
        return Status::success();
    }

    Status resume() {
        std::lock_guard<std::mutex> lk{mtx_};
        if (!latched_) return Status::failure("not latched");
        latched_ = false;
        state_ = CellState::kIdle;
        last_error_.clear();
        cancel_.clear();
        RN_LOG_INFO("resumed");
        return Status::success();
    }

    [[nodiscard]] const CancelToken& token() const { return cancel_; }

    [[nodiscard]] std::string state_name() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return name_of(state_);
    }

    [[nodiscard]] std::string last_error() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return last_error_;
    }

    [[nodiscard]] bool latched() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return latched_;
    }

    // Whether work is in flight. The supervisor owns the state machine, so it
    // owns this: the publisher used to derive it by comparing the state NAME
    // against "idle" and "faulted", which made a latched cell — stopped, on
    // purpose, waiting for a human — look like a cell that was still working.
    // Every waiter then hung until it declared the cell silent.
    [[nodiscard]] bool working() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return state_ == CellState::kBuilding || state_ == CellState::kPlanning ||
               state_ == CellState::kMoving;
    }

private:
    static const char* name_of(CellState s) {
        switch (s) {
            case CellState::kBuilding: return "building";
            case CellState::kPlanning: return "planning";
            case CellState::kMoving: return "moving";
            case CellState::kHeld: return "held";
            case CellState::kFaulted: return "faulted";
            case CellState::kIdle: break;
        }
        return "idle";
    }

    mutable std::mutex mtx_;
    CellState state_{CellState::kIdle};
    std::string last_error_;
    bool latched_{false};
    CancelToken cancel_;
};

// The verbs that drive it. stop / estop act on the CALLING thread — a queue the
// worker is draining is exactly the situation they exist for — so their work is
// done by the time the command is queued, and what queues is the
// acknowledgement. They live here because they command nothing else.
// `drop_pending` empties the command queue and reports how many went. Stop and
// e-stop are the only verbs that have it, because they are the only ones whose
// meaning is "and nothing that was about to happen, either".
template <class Router>
void register_supervisor_verbs(Router& router, CellSupervisor& supervisor,
                               std::function<std::size_t()> drop_pending = {}) {
    router.on("stop", [&supervisor](const nlohmann::json&, Command& c) {
        (void)supervisor.stop();
        c.run = [] { return Status::success(); };
        c.starts_motion = false;
        return Status::success();
    });
    router.on("estop", [&supervisor, drop_pending](const nlohmann::json&, Command& c) {
        (void)supervisor.estop();
        if (drop_pending) {
            if (const auto dropped = drop_pending(); dropped > 0) {
                RN_LOG_WARN("e-stop dropped {} queued command(s)", dropped);
            }
        }
        c.run = [] { return Status::success(); };
        c.starts_motion = false;
        return Status::success();
    });
    router.on("resume", [&supervisor](const nlohmann::json&, Command& c) {
        const auto st = supervisor.resume();
        c.run = [] { return Status::success(); };
        c.starts_motion = false;
        return st;
    });
}

}  // namespace robonode
