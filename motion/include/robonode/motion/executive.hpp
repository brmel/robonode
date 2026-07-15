#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include "robonode/core/state.hpp"
#include "robonode/core/telemetry.hpp"
#include "robonode/motion/axis_adapter.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/motion_plan.hpp"
#include "robonode/motion/setpoint_source.hpp"

namespace robonode {

// Fixed-rate executive: one cycle = source → govern → write adapter →
// step device → read state → record. Absolute deadlines (t0 + n·period) so
// timing never drifts (SPEC §3.1).
//
// The executive speaks only the SetpointSource slot — precomputed plans,
// the Ruckig OTG, and (later) Tier C plugins are interchangeable behind it.
//
// Runs on the host scheduler via sleep_until — jitter in the report is the
// host's, honestly measured. The production target is SCHED_FIFO + pinned
// core + PREEMPT_RT per NFR-1; the loop body is already RT-clean (no
// allocation after reserve, no locks, no I/O).
class Executive {
public:
    Executive(AxisAdapter& adapter, Governor& governor, double rate_hz)
        : adapter_{adapter}, governor_{governor}, period_ns_{static_cast<std::int64_t>(1e9 / rate_hz)} {}

    // Runs the source for `run_for_s`; returns timing stats, appends
    // full-rate telemetry to `rows`.
    CycleStats execute(SetpointSource& source, std::vector<TelemetryRow>& rows,
                       double run_for_s) {
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::nanoseconds{period_ns_};
        const double dt_s = static_cast<double>(period_ns_) * 1e-9;
        const auto n_cycles = static_cast<std::uint64_t>(run_for_s / dt_s) + 1;

        rows.reserve(rows.size() + n_cycles);
        jitter_us_.clear();
        jitter_us_.reserve(n_cycles);

        governor_.reset(adapter_.read().position_mm);
        // A shared world (e.g. MuJoCo) is ticked by the executive, not the
        // adapter, so stepping never depends on adapter identity (#50).
        CycleSteppable* world = adapter_.shared_world();

        CycleStats stats{};
        auto deadline = clock::now();
        for (std::uint64_t n = 0; n < n_cycles; ++n) {
            deadline += period;
            std::this_thread::sleep_until(deadline);
            const auto late_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - deadline)
                    .count();
            const double jitter_us = late_ns > 0 ? static_cast<double>(late_ns) * 1e-3 : 0.0;
            jitter_us_.push_back(jitter_us);
            if (late_ns > period_ns_) ++stats.overruns;

            const double t = static_cast<double>(n) * dt_s;

            // FR-8.2: safety state observed every cycle BEFORE commanding.
            // Non-NORMAL ⇒ hold the last governed setpoint. (Simplification:
            // instant hold; the real stack decelerates on-path via the OTG —
            // stop category 2. On return to NORMAL, catch-up is bounded by
            // the governor's rate limit.)
            const AxisState pre = adapter_.read();
            double command_mm;
            State target{};
            if (pre.safety != SafetyState::kNormal) {
                command_mm = governor_.held_position();
                target.position = command_mm;
                ++stats.safety_hold_cycles;
            } else {
                target = source.next(t, dt_s);
                command_mm = governor_.apply(target.position, dt_s).position_mm;
            }

            adapter_.write_setpoint(command_mm);
            if (world) world->tick(dt_s);
            adapter_.step(dt_s);
            const AxisState state = adapter_.read();

            rows.push_back({t, target.position, target.velocity, command_mm,
                            state.position_mm, state.velocity_mm_s,
                            command_mm - state.position_mm});
        }

        stats.cycles = n_cycles;
        finalize_jitter(stats);
        return stats;
    }

    // Convenience: run a precomputed plan to completion plus a settle window.
    CycleStats execute(const MotionPlan& plan, std::vector<TelemetryRow>& rows,
                       double settle_s = 0.05) {
        PlanSource src{plan};
        return execute(src, rows, plan.duration_s() + settle_s);
    }

private:
    void finalize_jitter(CycleStats& stats) {
        if (jitter_us_.empty()) return;
        double sum = 0.0;
        for (double j : jitter_us_) sum += j;
        stats.mean_jitter_us = sum / static_cast<double>(jitter_us_.size());
        std::sort(jitter_us_.begin(), jitter_us_.end());
        stats.max_jitter_us = jitter_us_.back();
        const auto idx = static_cast<std::size_t>(0.99 * static_cast<double>(jitter_us_.size() - 1));
        stats.p99_jitter_us = jitter_us_[idx];
    }

    AxisAdapter& adapter_;
    Governor& governor_;
    std::int64_t period_ns_;
    std::vector<double> jitter_us_;
};

}  // namespace robonode
