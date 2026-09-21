#pragma once

#include <functional>

#include <nlohmann/json.hpp>

#include "robonode/gateway/command_router.hpp"
#include <mutex>
#include <optional>
#include <utility>

#include "robonode/celld/tool_node.hpp"
#include "robonode/core/geometry.hpp"
#include "robonode/core/status.hpp"

namespace robonode {

// What the tool is carrying, as the platform sees it. The part itself lives in
// the physics: this only decides whether a grasp is legitimate (something is
// actually at the tool) and reports what is held.
class Workpiece {
public:
    explicit Workpiece(double grasp_reach_m) : reach_{grasp_reach_m} {}

    void track_tool(Vec3 tcp) {
        std::lock_guard<std::mutex> lk{mtx_};
        tool_ = tcp;
    }

    // What vision currently reports. Kept apart from `pose_`, which is the item
    // the tool carries or has placed.
    void observe(std::optional<Vec3> detected) {
        std::lock_guard<std::mutex> lk{mtx_};
        seen_ = detected;
    }

    // Closing on nothing is not a grasp: the part must be at the tool. Without
    // this the gripper reports success in mid-air and every downstream step
    // believes it is carrying something.
    Status attach() {
        std::lock_guard<std::mutex> lk{mtx_};
        if (held_) return Status::failure("already holding a part");
        if (!tool_) return Status::failure("tool pose unknown");
        if (!seen_) return Status::failure("nothing to grasp — no part located");
        const double d = (*seen_ - *tool_).norm();
        if (d > reach_) {
            return Status::failure("nothing within grasp reach (" + std::to_string(d) + " m)");
        }
        held_ = true;
        return Status::success();
    }

    Status detach() {
        std::lock_guard<std::mutex> lk{mtx_};
        if (!held_) return Status::failure("nothing held");
        held_ = false;
        return Status::success();
    }

    // Where the part is, according to the world — never a pose we wrote down.
    [[nodiscard]] std::optional<Vec3> pose() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return seen_;
    }

    [[nodiscard]] bool held() const {
        std::lock_guard<std::mutex> lk{mtx_};
        return held_;
    }

private:
    mutable std::mutex mtx_;
    std::optional<Vec3> tool_, seen_;
    bool held_{false};
    double reach_;
};

// A simulated gripper: closing it closes a constraint in the world, so the part
// is carried by the physics and dropped by the physics. A real gripper driver
// implements the same seam by commanding real jaws.
class SimGripper final : public ToolNode {
public:
    using Hold = std::function<Status(Grip)>;

    SimGripper(Workpiece& part, Hold hold) : part_{part}, hold_{std::move(hold)} {}

    Status grasp() override {
        if (const auto st = part_.attach(); !st.ok()) return st;
        if (const auto st = close(Grip::kClosed); !st.ok()) {
            (void)part_.detach();
            return st;
        }
        return Status::success();
    }

    Status release() override {
        if (const auto st = part_.detach(); !st.ok()) return st;
        return close(Grip::kOpen);
    }

    [[nodiscard]] bool holding() const override { return part_.held(); }

private:
    Status close(Grip grip) { return hold_ ? hold_(grip) : Status::success(); }

    Workpiece& part_;
    Hold hold_;
};

// Opening and closing the tool: two verbs, no arguments, next to the tool.
template <class Router>
void register_tool_verbs(Router& router, ToolNode& tool) {
    router.on("grasp", [&tool](const nlohmann::json&, Command& c) {
        c.run = [&tool] { return tool.grasp(); };
        return Status::success();
    });
    router.on("release", [&tool](const nlohmann::json&, Command& c) {
        c.run = [&tool] { return tool.release(); };
        return Status::success();
    });
}

}  // namespace robonode
