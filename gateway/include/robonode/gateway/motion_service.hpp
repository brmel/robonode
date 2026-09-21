#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robonode/celld/cell_descriptor.hpp"
#include "robonode/core/cancel.hpp"
#include "robonode/core/settings.hpp"
#include "robonode/core/geometry.hpp"
#include "robonode/core/status.hpp"
#include "robonode/gateway/cell_runtime.hpp"
#include "robonode/gateway/control_capability.hpp"
#include "robonode/gateway/trajectory_capability.hpp"
#include "robonode/log.hpp"
#include "robonode/motion/otg.hpp"
#include "robonode/motion/planner.hpp"
#include "robonode/motion/setpoint_source.hpp"
#include "robonode/motion/trajectory_validator.hpp"

namespace robonode {

// Turns intent (a Cartesian target, a named motion) into a synchronised run on
// the cell. Owns no state of its own — the cell lives in CellRuntime, the
// algorithm choice in the capabilities.
class MotionService {
public:
    MotionService(CellRuntime& runtime, TrajectoryCapability& trajectory, ControlCapability& control,
                  Settings::Motion cfg = {})
        : rt_{runtime}, traj_{trajectory}, ctl_{control}, cfg_{cfg}, validator_{cfg} {}

    using RunSink = std::function<void(const std::string&, const std::vector<std::string>&,
                                       const std::vector<std::vector<TelemetryRow>>&, double)>;

    void set_hook(CycleHook hook) { hook_ = std::move(hook); }
    void set_run_sink(RunSink sink) { sink_ = std::move(sink); }

    // How well a run went, not just whether it finished. Comparing two control
    // versions on time alone says nothing — they take the same time and track
    // the path differently, which is the entire point of swapping one.
    struct WorstAxis {
        std::string id{"-"}, unit;
        double error{0.0};
    };
    struct RunQuality {
        std::string label;
        WorstAxis worst;
        CycleStats stats;
    };
    using QualitySink = std::function<void(const RunQuality&)>;
    void set_quality_sink(QualitySink sink) { quality_ = std::move(sink); }
    void set_cancel(const CancelToken* cancel) { cancel_ = cancel; }

    Status move_l(Vec3 target, const std::string& robot = {}) {
        return move_to(target, Vec3{}, robot);
    }

    // Move to a target that is itself moving. The velocity is part of the goal,
    // so a planner can choose to meet it travelling alongside — and one that
    // ignores it still works, just less well.
    Status move_to(Vec3 target, Vec3 velocity, const std::string& robot = {}) {
        return rt_.with_robot(robot, [&](Cell& cell, const RobotNode& r) {
            std::vector<std::vector<double>> joint_wp;
            if (const auto st = plan_to(cell, r, target, velocity, joint_wp); !st.ok()) return st;

            std::vector<std::vector<double>> cell_wp;
            if (const auto st = r.compose(cell, joint_wp, cell_wp); !st.ok()) return st;

            RN_LOG_INFO("move_l -> ({:.3f}, {:.3f}, {:.3f}) m", target.x, target.y, target.z);
            return execute(cell, cell_wp, cfg_.settle_s);
        });
    }

    // Reach a full pose: the point AND the angle (#92). The planner version in
    // force must accept a pose goal — one that only reaches says so by name,
    // which is how a user learns that "moveL" is not what they wanted.
    Status move_pose(const Pose& target, const std::string& robot = {}) {
        return rt_.with_robot(robot, [&](Cell& cell, const RobotNode& r) {
            Goal goal;
            goal.kind = Goal::kCartesianPose;
            goal.pose = target;
            std::vector<std::vector<double>> joint_wp;
            if (const auto st = traj_.plan(r.joint_positions(cell), goal, joint_wp); !st.ok()) {
                RN_LOG_WARN("move_pose rejected: {}", st.message());
                return st;
            }
            std::vector<std::vector<double>> cell_wp;
            if (const auto st = r.compose(cell, joint_wp, cell_wp); !st.ok()) return st;
            RN_LOG_INFO("move_pose -> ({:.3f}, {:.3f}, {:.3f}) at ({:.3f}, {:.3f}, {:.3f}, {:.3f})",
                        target.position.x, target.position.y, target.position.z,
                        target.orientation.w, target.orientation.x, target.orientation.y,
                        target.orientation.z);
            return execute(cell, cell_wp, cfg_.settle_s);
        });
    }

    // Jog one axis to a target with the online trajectory generator: jerk-limited
    // and retargetable, the streaming primitive a teleop surface needs. Every
    // other axis holds through the same governor and safety gate.
    Status jog(const std::string& axis_id, double target) {
        return rt_.with_cell([&](Cell& cell) {
            const auto& nodes = cell.nodes();
            const auto limits = rt_.limits();
            std::vector<std::unique_ptr<SetpointSource>> owned;
            std::vector<SetpointSource*> sources;
            bool found = false;
            double horizon = 0.0;

            for (std::size_t i = 0; i < nodes.size(); ++i) {
                const double now = nodes[i].adapter->read().position;
                if (nodes[i].id == axis_id) {
                    const double goal =
                        std::clamp(target, limits[i].position_min, limits[i].position_max);
                    auto otg = std::make_unique<Otg>(cfg_.rate_hz, limits[i], now);
                    otg->retarget(goal);
                    horizon = Otg::estimate_duration(limits[i], goal - now);
                    owned.push_back(std::move(otg));
                    found = true;
                } else {
                    owned.push_back(std::make_unique<HoldSource>(now));
                }
                sources.push_back(owned.back().get());
            }
            if (!found) return Status::failure("no axis '" + axis_id + "'");

            RN_LOG_INFO("jog {} -> {:.3f}", axis_id, target);
            std::vector<std::vector<TelemetryRow>> rows;
            CycleStats stats{};
            const auto st = cell.run_sources(sources, cfg_.rate_hz, rows, stats,
                                             horizon + cfg_.settle_s, cfg_.stop_time_s, hook_,
                                             ctl_.ptrs(), cancel_);
            report(st, "jog", rows, stats);
            return st;
        });
    }

    Status run_motion(const std::string& id) {
        const auto* spec = find_motion(id);
        if (spec == nullptr) return Status::failure("no motion '" + id + "'");
        return rt_.with_cell([&](Cell& cell) {
            std::vector<std::vector<double>> wp;
            if (const auto st = compose_named(cell, *spec, wp); !st.ok()) return st;
            RN_LOG_INFO("motion '{}': {} axes", id, wp.size());
            return execute(cell, wp, spec->settle_s);
        });
    }

private:
    Status plan_to(Cell& cell, const RobotNode& r, Vec3 target, Vec3 velocity,
                   std::vector<std::vector<double>>& out) const {
        Goal goal;
        goal.kind = Goal::kCartesianPosition;
        goal.cartesian = target;
        goal.velocity = velocity;
        const auto st = traj_.plan(r.joint_positions(cell), goal, out);
        if (!st.ok()) {
            RN_LOG_WARN("move_l ({:.3f},{:.3f},{:.3f}) rejected: {}", target.x, target.y, target.z,
                        st.message());
        }
        return st;
    }

    Status execute(Cell& cell, const std::vector<std::vector<double>>& wp, double settle_s) {
        if (const auto st = validator_.check(wp, rt_.limits(), rt_.axis_ids()); !st.ok()) {
            RN_LOG_WARN("trajectory refused: {}", st.message());
            return st;
        }
        std::vector<std::vector<TelemetryRow>> rows;
        CycleStats stats{};
        const auto st = cell.run_waypoints(wp, cfg_.rate_hz, rows, stats, settle_s,
                                           cfg_.stop_time_s, hook_, ctl_.ptrs(), cancel_);
        report(st, "move", rows, stats);
        const bool asked_to_stop = cancel_ != nullptr && cancel_->requested();
        if (st.ok() && stats.stopped_early && !asked_to_stop) {
            return Status::failure("stopped before the goal");
        }
        return st;
    }

    // Every run produces real-time quality data. Reporting it is the difference
    // between "the move finished" and "the move finished, and here is how well
    // the loop held its deadline".
    void report(const Status& st, const std::string& label,
                const std::vector<std::vector<TelemetryRow>>& rows, const CycleStats& stats) const {
        const auto worst = worst_axis(rows);
        RN_LOG_INFO("{}: {} cycles · {} overrun · jitter p99 {:.0f}µs max {:.0f}µs · worst Δfollow {} {:.3f} {}{}",
                    label, stats.cycles, stats.overruns, stats.p99_jitter_us, stats.max_jitter_us,
                    worst.id, worst.error, worst.unit,
                    stats.stopped_early ? " · stopped early" : "");
        if (quality_) quality_({label, worst, stats});
        if (st.ok() && sink_) sink_(label, rt_.axis_ids(), rows, cfg_.rate_hz);
    }


    // A peak taken across axes with different units is a meaningless number
    // (130 mm of rail is not worse than 0.1 rad of wrist). Name the axis and
    // carry its unit instead, which also says *where* to look.
    WorstAxis worst_axis(const std::vector<std::vector<TelemetryRow>>& rows) const {
        const auto ids = rt_.axis_ids();
        const auto units = rt_.axis_units();
        WorstAxis worst;
        for (std::size_t i = 0; i < rows.size(); ++i) {
            double peak = 0.0;
            for (const auto& r : rows[i]) peak = std::max(peak, std::abs(r.following_error));
            const double travel = i < units.size() && units[i] == "mm" ? peak / 1000.0 : peak;
            const double best = worst.unit == "mm" ? worst.error / 1000.0 : worst.error;
            if (travel <= best) continue;  // compare in model units, report in the axis's own
            worst = {i < ids.size() ? ids[i] : std::to_string(i),
                     i < units.size() ? units[i] : "", peak};
        }
        return worst;
    }

    const MotionSpec* find_motion(const std::string& id) const {
        for (const auto& m : rt_.descriptor().motions) {
            if (m.id == id) return &m;
        }
        return nullptr;
    }

    // Axis-id-keyed waypoints → cell-ordered lists; unnamed axes hold position.
    static Status compose_named(const Cell& cell, const MotionSpec& spec,
                                std::vector<std::vector<double>>& out) {
        const auto& nodes = cell.nodes();
        std::size_t steps = 0;
        for (const auto& [_, wp] : spec.waypoints) steps = std::max(steps, wp.size());
        if (steps == 0) return Status::failure("motion '" + spec.id + "' has no waypoints");

        out.assign(nodes.size(), {});
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            const auto it = spec.waypoints.find(nodes[i].id);
            if (it == spec.waypoints.end() || it->second.size() != steps) {
                out[i].assign(steps, nodes[i].adapter->read().position);
            } else {
                out[i] = it->second;
            }
        }
        return Status::success();
    }

    CellRuntime& rt_;
    TrajectoryCapability& traj_;
    ControlCapability& ctl_;
    Settings::Motion cfg_;
    TrajectoryValidator validator_;
    CycleHook hook_;
    RunSink sink_;
    QualitySink quality_;
    const CancelToken* cancel_{nullptr};
};

}  // namespace robonode
