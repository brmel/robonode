#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "robonode/core/state.hpp"
#include "robonode/core/telemetry.hpp"
#include "robonode/motion/axis_adapter.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/sync_blend.hpp"

namespace robonode {

// Optional per-cycle observer (telemetry streaming, live viz). Called after
// each cycle's read phase with the cell clock and the adapters. Empty by
// default (one branch/cycle when unused); when set it must be cheap and is
// NOT part of the hard-RT contract — for the demo gateway, not the drive path.
using CycleHook = std::function<void(double t, const std::vector<AxisAdapter*>&)>;

// N-axis executive: one clock, one cycle, all axes commanded together —
// the motion-tree "one clock master per cell" rule (FR-2.4) in miniature.
//
// Each cycle runs in three phases across all adapters:
//   1. write — compute + govern each setpoint, write it to its adapter
//   2. step  — advance every adapter's device
//   3. read  — sample state, record telemetry
// Phasing matters when adapters share state (a MuJoCo world stepped once per
// cycle): all setpoints are in place before physics advances, so no joint
// lags a cycle behind its neighbours. For independent adapters (SimAxis) the
// result is identical to interleaving.
//
// Safety is cell-coherent: ANY axis reporting non-NORMAL holds EVERY axis at
// its last governed setpoint (a group that keeps moving while one member is
// in protective stop is how gantries rack themselves).
class SyncExecutive {
public:
    // adapters.size() must equal governors.size() and plan.axes().
    SyncExecutive(std::vector<AxisAdapter*> adapters, std::vector<Governor*> governors,
                  double rate_hz)
        : adapters_{std::move(adapters)}, governors_{std::move(governors)},
          period_ns_{static_cast<std::int64_t>(1e9 / rate_hz)} {
        if (adapters_.empty() || adapters_.size() != governors_.size()) {
            throw std::invalid_argument{"adapters/governors mismatch"};
        }
    }

    // rows[i] receives axis i's telemetry. `hook`, if set, is called each
    // cycle after read (live telemetry streaming).
    CycleStats execute(const SyncBlendPlan& plan, std::vector<std::vector<TelemetryRow>>& rows,
                       double settle_s = 0.05, const CycleHook& hook = {}) {
        if (plan.axes() != adapters_.size()) throw std::invalid_argument{"plan/axes mismatch"};
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::nanoseconds{period_ns_};
        const double dt_s = static_cast<double>(period_ns_) * 1e-9;
        const double plan_duration_s = plan.duration();
        const auto n_cycles =
            static_cast<std::uint64_t>((plan_duration_s + settle_s) / dt_s) + 1;
        const std::size_t n = adapters_.size();

        rows.assign(n, {});
        for (auto& r : rows) r.reserve(n_cycles);
        jitter_us_.clear();
        jitter_us_.reserve(n_cycles);
        cmd_.resize(n);  // per-cycle scratch, allocated once
        tgt_.resize(n);
        for (std::size_t i = 0; i < n; ++i) {
            governors_[i]->reset(adapters_[i]->read().position_mm);
        }

        CycleStats stats{};
        auto deadline = clock::now();
        for (std::uint64_t c = 0; c < n_cycles; ++c) {
            deadline += period;
            std::this_thread::sleep_until(deadline);
            const auto late_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(clock::now() - deadline)
                    .count();
            jitter_us_.push_back(late_ns > 0 ? static_cast<double>(late_ns) * 1e-3 : 0.0);
            if (late_ns > period_ns_) ++stats.overruns;

            const double t = static_cast<double>(c) * dt_s;

            // Cell-coherent safety gate (FR-8.2): any axis non-NORMAL holds all.
            bool hold = false;
            for (auto* a : adapters_) {
                if (a->read().safety != SafetyState::kNormal) {
                    hold = true;
                    break;
                }
            }
            if (hold) ++stats.safety_hold_cycles;

            // Phase 1: compute, govern, write every setpoint.
            for (std::size_t i = 0; i < n; ++i) {
                if (hold) {
                    cmd_[i] = governors_[i]->held_position();
                    tgt_[i] = State{cmd_[i], 0.0, 0.0};
                } else {
                    tgt_[i] = plan.sample(i, std::min(t, plan_duration_s));
                    cmd_[i] = governors_[i]->apply(tgt_[i].position, dt_s).position_mm;
                }
                adapters_[i]->write_setpoint(cmd_[i]);
            }
            // Phase 2: advance every device (shared worlds step once here).
            for (std::size_t i = 0; i < n; ++i) adapters_[i]->step(dt_s);
            // Phase 3: read state, record telemetry.
            for (std::size_t i = 0; i < n; ++i) {
                const AxisState st = adapters_[i]->read();
                rows[i].push_back({t, tgt_[i].position, tgt_[i].velocity, cmd_[i], st.position_mm,
                                   st.velocity_mm_s, cmd_[i] - st.position_mm});
            }
            if (hook) hook(t, adapters_);
        }

        stats.cycles = n_cycles;
        if (!jitter_us_.empty()) {
            double sum = 0.0;
            for (double j : jitter_us_) sum += j;
            stats.mean_jitter_us = sum / static_cast<double>(jitter_us_.size());
            std::sort(jitter_us_.begin(), jitter_us_.end());
            stats.max_jitter_us = jitter_us_.back();
            stats.p99_jitter_us =
                jitter_us_[static_cast<std::size_t>(0.99 * static_cast<double>(jitter_us_.size() - 1))];
        }
        return stats;
    }

private:
    std::vector<AxisAdapter*> adapters_;
    std::vector<Governor*> governors_;
    std::int64_t period_ns_;
    std::vector<double> jitter_us_;
    std::vector<double> cmd_;
    std::vector<State> tgt_;
};

}  // namespace robonode
