#pragma once

#include <cstdint>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "robonode/core/geometry.hpp"

namespace robonode {

// Owns the two JSON snapshots every read surface polls. Callers hand it
// already-computed numbers, so it has no view of the cell, kinematics or vision.
class TelemetryPublisher {
public:
    struct NodeView {
        std::string id, driver, unit, joint;  // `joint` names the link in the model
        double lo, hi;
    };

    // What a surface needs to follow progress without polling for side effects:
    // which command landed, what the cell is doing, what failed.
    struct Progress {
        std::uint64_t applied_id{};
        std::uint64_t accepted_id{};
        std::string state{"idle"};
        std::string last_error;
        bool latched{};
        bool working{};  // the supervisor's answer, not a guess from the name
        std::string step;  // "3/5 pick" while a program runs, empty otherwise
        std::string app;   // the application file those steps belong to
    };

    struct Frame {
        double t{};
        std::string family;
        std::vector<double> pos, target, err;
        std::optional<Vec3> tcp;
        std::optional<Quat> tcp_orientation;  // which way the tool points
        std::optional<Vec3> part;       // what vision currently sees
        std::optional<Vec3> workpiece;  // the item the tool carries or has placed
        bool holding{};
        std::vector<std::pair<std::string, std::string>> contacts;  // what is touching what
        Progress progress;
    };

    // `families` is which driver families this CELL declares — the UI builds its
    // selector from it rather than carrying the two names the shipped descriptor
    // happens to use. A descriptor with a third family shows three buttons
    // without anyone editing markup.
    void publish_nodes(const std::string& family, const std::vector<std::string>& families,
                       const std::vector<std::string>& available,
                       const std::vector<NodeView>& nodes) {
        nlohmann::json j{{"family", family}, {"families", families}, {"available", available}};
        auto& arr = j["nodes"] = nlohmann::json::array();
        for (const auto& n : nodes) {
            arr.push_back({{"id", n.id}, {"driver", n.driver}, {"unit", n.unit},
                           {"joint", n.joint}, {"lo", n.lo}, {"hi", n.hi},
                           {"state", "active"}});
        }
        store(nodes_snap_, j.dump());
    }

    void publish(const Frame& f) {
        {
            std::lock_guard<std::mutex> lk{mtx_};
            last_ = f;
        }
        store(telem_snap_, dump(f));
    }

    // Refresh only the progress fields. Cheap and lock-light: used the moment a
    // command is accepted, so a caller never reads a snapshot that predates it.
    void update_progress(const Progress& p) {
        {
            std::lock_guard<std::mutex> lk{mtx_};
            last_.progress = p;
            telem_snap_ = dump(last_);
            ++seq_;
        }
        changed_.notify_all();
    }

    std::string nodes_json() { return read(nodes_snap_); }
    std::string telemetry_json() { return read(telem_snap_); }

    // A reader that must react to a change — a CLI waiting for its command to
    // be applied, a test waiting for the cell to settle — waits for the next
    // publication instead of polling. One wake per real event beats fifty
    // wake-ups a second per waiter, and it reacts as fast as the cell moves.
    [[nodiscard]] std::uint64_t sequence() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return seq_;
    }

    std::uint64_t await_change(std::uint64_t seen, std::chrono::milliseconds timeout) {
        std::unique_lock<std::mutex> lk{mtx_};
        changed_.wait_for(lk, timeout, [&] { return seq_ != seen; });
        return seq_;
    }

private:
    static std::string dump(const Frame& f) {
        nlohmann::json j{{"t", f.t},
                         {"family", f.family},
                         {"pos", f.pos},
                         {"target", f.target},
                         {"err", f.err},
                         {"holding", f.holding},
                         {"applied_id", f.progress.applied_id},
                         {"accepted_id", f.progress.accepted_id},
                         {"state", f.progress.state},
                         {"latched", f.progress.latched},
                         // A faulted cell is not busy — it has STOPPED, with a
                         // reason. Reporting it as running made every waiter sit
                         // out its stall timeout before hearing the error.
                         {"running", f.progress.working}};
        if (!f.progress.last_error.empty()) j["last_error"] = f.progress.last_error;
        if (!f.progress.step.empty()) j["step"] = f.progress.step;
        if (!f.progress.app.empty()) j["app"] = f.progress.app;
        if (f.tcp) j["tcp"] = {f.tcp->x, f.tcp->y, f.tcp->z};
        if (f.tcp_orientation) {
            j["tcp_quat"] = {f.tcp_orientation->w, f.tcp_orientation->x, f.tcp_orientation->y,
                             f.tcp_orientation->z};
        }
        j["vision"] = {{"detected", f.part.has_value()}};
        if (f.part) j["vision"]["part"] = {f.part->x, f.part->y, f.part->z};
        if (f.workpiece) {
            j["workpiece"] = {{"pose", {f.workpiece->x, f.workpiece->y, f.workpiece->z}},
                              {"held", f.holding}};
        }
        auto& touching = j["contacts"] = nlohmann::json::array();
        for (const auto& [a, b] : f.contacts) touching.push_back({a, b});
        return j.dump();
    }

    void store(std::string& slot, std::string value) {
        {
            std::lock_guard<std::mutex> lk{mtx_};
            slot = std::move(value);
            ++seq_;
        }
        changed_.notify_all();
    }
    std::string read(const std::string& slot) const {
        std::lock_guard<std::mutex> lk{mtx_};
        return slot;
    }

    mutable std::mutex mtx_;
    std::condition_variable changed_;
    std::uint64_t seq_{0};
    Frame last_;
    std::string nodes_snap_{R"({"family":"physics","nodes":[]})"};
    std::string telem_snap_{
        R"({"t":0,"family":"physics","pos":[],"target":[],"err":[],"state":"idle"})"};
};

}  // namespace robonode
