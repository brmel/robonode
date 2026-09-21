#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robonode/core/geometry.hpp"
#include "robonode/core/status.hpp"
#include "robonode/motion/cartesian.hpp"
#include "robonode/motion/kinematics.hpp"
#include "robonode/motion/planner.hpp"
#include "robonode/sandbox/compiler.hpp"
#include "robonode/sandbox/program.hpp"

namespace robonode {

class SandboxedPlanner final : public Planner {
public:
    SandboxedPlanner(const Kinematics& kin, int steps_per_segment, sandbox::Program program,
                     sandbox::SandboxLimits limits, Vec3 bounds_lo, Vec3 bounds_hi, IkOptions ik)
        : kin_{kin}, steps_{steps_per_segment}, program_{std::move(program)}, limits_{limits},
          lo_{bounds_lo}, hi_{bounds_hi}, ik_{std::move(ik)} {}

    Status plan(const std::vector<double>& start_q, const Goal& goal,
                std::vector<std::vector<double>>& waypoints) const override {
        if (goal.kind != Goal::kCartesianPosition) {
            return Status::failure("SandboxedPlanner: non-Cartesian goal");
        }
        std::vector<double> out;
        if (const auto st = sandbox::run(program_, {goal.cartesian.x, goal.cartesian.y, goal.cartesian.z},
                                         limits_, out);
            !st.ok()) {
            return st;
        }
        if (out.size() != 6) return Status::failure("SandboxedPlanner: expected via+end (6 values)");
        for (const double v : out) {
            if (!std::isfinite(v)) return Status::failure("SandboxedPlanner: non-finite output");
        }
        const Vec3 via{out[0], out[1], out[2]};
        const Vec3 end{out[3], out[4], out[5]};
        if (!in_bounds(via) || !in_bounds(end)) {
            return Status::failure("SandboxedPlanner: output outside workspace");
        }

        std::vector<std::vector<double>> to_via;
        if (const auto st = plan_move_l(kin_, start_q, via, steps_, to_via, ik_); !st.ok()) return st;

        std::vector<double> q_via = last_column(to_via);
        std::vector<std::vector<double>> to_end;
        if (const auto st = plan_move_l(kin_, q_via, end, steps_, to_end, ik_); !st.ok()) return st;

        waypoints = std::move(to_via);
        for (std::size_t j = 0; j < waypoints.size() && j < to_end.size(); ++j) {
            waypoints[j].insert(waypoints[j].end(), to_end[j].begin(), to_end[j].end());
        }
        return Status::success();
    }

private:
    [[nodiscard]] bool in_bounds(const Vec3& q) const {
        return q.x >= lo_.x && q.x <= hi_.x && q.y >= lo_.y && q.y <= hi_.y && q.z >= lo_.z &&
               q.z <= hi_.z;
    }
    static std::vector<double> last_column(const std::vector<std::vector<double>>& wp) {
        std::vector<double> q;
        q.reserve(wp.size());
        for (const auto& j : wp) q.push_back(j.empty() ? 0.0 : j.back());
        return q;
    }

    const Kinematics& kin_;
    int steps_;
    sandbox::Program program_;
    sandbox::SandboxLimits limits_;
    Vec3 lo_, hi_;
    IkOptions ik_;
};

inline Status compile_planner_program(const std::string& source, sandbox::Program& out) {
    return sandbox::Compiler::compile(source, {"x", "y", "z"}, 6, out);
}

// Workspace bounds and step budget come from the context (settings), not from
// literals here: they are the two numbers an operator most wants to tune.
inline void register_program_planner(PlannerRegistry& reg, const std::string& version,
                                     sandbox::Program program,
                                     sandbox::SandboxLimits limits = {}) {
    reg.add(version, [program = std::move(program), limits](
                         const PlannerContext& c) -> std::unique_ptr<Planner> {
        return std::make_unique<SandboxedPlanner>(*c.kin, c.ik.sandbox_line_steps, program, limits,
                                                  c.workspace_lo, c.workspace_hi, ik_options(c));
    });
}

inline Status register_sandboxed_planner(PlannerRegistry& reg, const std::string& version,
                                         const std::string& source) {
    sandbox::Program program;
    if (const auto st = compile_planner_program(source, program); !st.ok()) return st;
    register_program_planner(reg, version, std::move(program));
    return Status::success();
}

}  // namespace robonode
