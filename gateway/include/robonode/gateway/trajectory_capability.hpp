#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robonode/core/settings.hpp"
#include "robonode/core/status.hpp"
#include "robonode/gateway/capability.hpp"
#include "robonode/gateway/sandboxed_planner.hpp"
#include "robonode/log.hpp"
#include "robonode/motion/kinematics.hpp"
#include "robonode/motion/planner.hpp"
#include "robonode/sandbox/program.hpp"

namespace robonode {

inline CapabilityDescriptor trajectory_descriptor() {
    return {"planner",
            "Trajectory",
            "🧭",
            "inputs <code>x y z</code> (goal) → six lines = via point (xyz) then end point (xyz)",
            "x\ny\nz + 0.15\nx\ny\nz",
            /*authorable=*/true,
            {"x", "y", "z"},
            6};
}

class TrajectoryCapability final : public Capability<Planner, PlannerContext> {
public:
    TrajectoryCapability() : Capability{trajectory_descriptor(), "robonode.moveL"} {
        register_basic_planners(registry_);
        remember({"robonode.moveL", "builtin", {}});
        remember({"robonode.moveJ", "builtin", {}});
    }

    void set_kinematics(const Kinematics* kin) { kin_ = kin; }
    void set_bounds(std::vector<JointBound> bounds) { bounds_ = std::move(bounds); }
    void set_weights(std::vector<double> weights) { weights_ = std::move(weights); }
    void configure(Settings s) { cfg_ = s; }

    Status select(const std::string& version) override {
        if (!registry_.has(version)) return Status::failure("unknown version '" + version + "'");
        version_ = version;
        rebuild();
        return Status::success();
    }

    void install(const std::string& id, sandbox::Program program, Select select) override {
        register_program_planner(registry_, id, std::move(program));
        remember({id, "user", {}});
        if (select == Select::kNow) version_ = id;
        rebuild();
    }

    void remember_source(const std::string& id, const std::string& source) {
        remember({id, "user", source_hash(source)});
    }

    void rebuild() {
        PlannerContext ctx;
        ctx.kin = kin_;
        ctx.bounds = bounds_;
        ctx.weights = weights_;
        ctx.ik = cfg_.ik;
        ctx.workspace_lo = {cfg_.workspace.lo[0], cfg_.workspace.lo[1], cfg_.workspace.lo[2]};
        ctx.workspace_hi = {cfg_.workspace.hi[0], cfg_.workspace.hi[1], cfg_.workspace.hi[2]};
        if (!registry_.make(version_, ctx, planner_).ok()) {
            RN_LOG_WARN("planner version '{}' unavailable", version_);
        }
        publish();
    }

    [[nodiscard]] const std::string& active() const { return version_; }

    Status plan(const std::vector<double>& start_q, const Goal& goal,
                std::vector<std::vector<double>>& waypoints) const {
        if (!planner_) {
            return Status::failure("planner version '" + version_ + "' failed to build");
        }
        return planner_->plan(start_q, goal, waypoints);
    }

private:
    std::unique_ptr<Planner> planner_;
    const Kinematics* kin_{nullptr};
    std::vector<JointBound> bounds_;
    std::vector<double> weights_;
    Settings cfg_;
};

}  // namespace robonode
