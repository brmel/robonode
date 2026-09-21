#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "robonode/core/cancel.hpp"
#include "robonode/core/settings.hpp"
#include "robonode/core/state.hpp"
#include "robonode/core/telemetry.hpp"
#include "robonode/motion/axis_adapter.hpp"
#include "robonode/motion/controller.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/setpoint_source.hpp"
#include "robonode/motion/sync_blend.hpp"

namespace robonode {

// Optional per-cycle observer (telemetry streaming, live viz). Called after each
// cycle's read phase. Cheap by contract, and NOT part of the hard-RT path.
using CycleHook = std::function<void(double t, const std::vector<std::vector<TelemetryRow>>&)>;

// One axis of a SyncBlendPlan seen as a SetpointSource, so the executive has a
// single way to ask for a setpoint whatever produced it.
class PlanAxisSource final : public SetpointSource {
public:
    PlanAxisSource(const SyncBlendPlan& plan, std::size_t axis) : plan_{&plan}, axis_{axis} {}

    State next(double t_s, double /*dt_s*/) noexcept override {
        return plan_->sample(axis_, std::min(t_s, plan_->duration()));
    }
    [[nodiscard]] double duration_s() const noexcept override { return plan_->duration(); }

private:
    const SyncBlendPlan* plan_;
    std::size_t axis_;
};

// N-axis executive: one clock, one cycle, all axes commanded together (FR-2.4).
//
// Each cycle runs in three phases across all adapters:
//   1. write — sample each source, control, govern, write
//   2. step  — advance every device (shared worlds ticked once)
//   3. read  — sample state, record telemetry
// Phasing matters when adapters share state (a MuJoCo world stepped once per
// cycle): all setpoints are in place before physics advances.
//
// Safety is cell-coherent: ANY axis reporting non-NORMAL holds EVERY axis.
// A cancel request ramps the path clock to zero over stop_time_s_, so axes
// decelerate ALONG the path (stop category 2) rather than stepping to a hold.
class SyncExecutive {
public:
    // stop_time_s has no default on purpose: a compiled-in one is a second
    // source of truth for a tunable the settings file owns (ADR-14), and the
    // caller that forgot to pass it looks identical to the one that meant this.
    SyncExecutive(std::vector<AxisAdapter*> adapters, std::vector<Governor*> governors,
                  double rate_hz, std::vector<Controller*> controllers, double stop_time_s)
        : adapters_{std::move(adapters)}, governors_{std::move(governors)},
          controllers_{std::move(controllers)},
          period_ns_{static_cast<std::int64_t>(1e9 / rate_hz)}, stop_time_s_{stop_time_s} {
        if (adapters_.empty() || adapters_.size() != governors_.size()) {
            throw std::invalid_argument{"adapters/governors mismatch"};
        }
        if (!controllers_.empty() && controllers_.size() != adapters_.size()) {
            throw std::invalid_argument{"controllers/adapters mismatch"};
        }
    }

    CycleStats execute(const SyncBlendPlan& plan, std::vector<std::vector<TelemetryRow>>& rows,
                       double settle_s = 0.05, const CycleHook& hook = {},
                       const CancelToken* cancel = nullptr) {
        if (plan.axes() != adapters_.size()) throw std::invalid_argument{"plan/axes mismatch"};
        plan_views_.clear();
        plan_views_.reserve(adapters_.size());
        sources_.clear();
        for (std::size_t i = 0; i < adapters_.size(); ++i) plan_views_.emplace_back(plan, i);
        for (auto& v : plan_views_) sources_.push_back(&v);
        return run(plan.duration(), rows, settle_s, hook, cancel);
    }

    // Drive arbitrary per-axis sources (an OTG, a streamed setpoint) for
    // `run_for_s`. One source per axis.
    CycleStats execute(std::vector<SetpointSource*> sources,
                       std::vector<std::vector<TelemetryRow>>& rows, double run_for_s,
                       const CycleHook& hook = {}, const CancelToken* cancel = nullptr) {
        if (sources.size() != adapters_.size()) throw std::invalid_argument{"sources/axes mismatch"};
        sources_ = std::move(sources);
        return run(run_for_s, rows, /*settle_s=*/0.0, hook, cancel);
    }

private:
    CycleStats run(double duration_s, std::vector<std::vector<TelemetryRow>>& rows, double settle_s,
                   const CycleHook& hook, const CancelToken* cancel) {
        using clock = std::chrono::steady_clock;
        const auto period = std::chrono::nanoseconds{period_ns_};
        const double dt_s = static_cast<double>(period_ns_) * 1e-9;
        const auto n_cycles = static_cast<std::uint64_t>((duration_s + settle_s) / dt_s) + 1;
        const auto max_cycles = n_cycles + static_cast<std::uint64_t>(stop_time_s_ / dt_s) + 1;
        const std::size_t n = adapters_.size();

        prepare(n, max_cycles, rows);

        CycleStats stats{};
        auto deadline = clock::now();
        double path_s = 0.0;
        double path_rate = 1.0;
        auto settle_left = static_cast<std::uint64_t>(settle_s / dt_s) + 1;

        for (std::uint64_t c = 0; c < max_cycles; ++c) {
            deadline += period;
            std::this_thread::sleep_until(deadline);
            record_jitter(clock::now() - deadline, stats);

            if (cancel != nullptr && cancel->requested()) {
                path_rate = std::max(0.0, path_rate - dt_s / stop_time_s_);
                ++stats.cancel_cycles;
            }
            if (any_unsafe()) {
                ++stats.safety_hold_cycles;
                hold_all();
            } else {
                follow_all(path_s, dt_s);
                path_s = std::min(path_s + path_rate * dt_s, duration_s);
            }
            step_all(dt_s);
            read_all(static_cast<double>(c) * dt_s, rows);
            if (hook) hook(static_cast<double>(c) * dt_s, rows);

            const bool finished = path_rate == 0.0 || path_s >= duration_s;
            if (finished && settle_left-- == 0) {
                stats.cycles = c + 1;
                stats.stopped_early = path_rate == 0.0 && path_s < duration_s;
                finalize_jitter(stats);
                return stats;
            }
        }
        stats.cycles = max_cycles;
        finalize_jitter(stats);
        return stats;
    }

    void prepare(std::size_t n, std::uint64_t max_cycles,
                 std::vector<std::vector<TelemetryRow>>& rows) {
        rows.assign(n, {});
        for (auto& r : rows) r.reserve(max_cycles);
        jitter_us_.clear();
        jitter_us_.reserve(max_cycles);
        cmd_.resize(n);
        tgt_.resize(n);
        collect_worlds();
        for (std::size_t i = 0; i < n; ++i) {
            const double p = adapters_[i]->read().position;
            governors_[i]->reset(p);
            if (!controllers_.empty()) controllers_[i]->reset(p);
        }
    }

    // Distinct shared worlds, ticked once per cycle: physics advances whichever
    // adapters exist, so a live node swap stays safe (#50).
    void collect_worlds() {
        worlds_.clear();
        for (auto* a : adapters_) {
            auto* w = a->shared_world();
            if (w != nullptr && std::find(worlds_.begin(), worlds_.end(), w) == worlds_.end()) {
                worlds_.push_back(w);
            }
        }
    }

    [[nodiscard]] bool any_unsafe() const {
        for (auto* a : adapters_) {
            if (a->read().safety != SafetyState::kNormal) return true;
        }
        return false;
    }

    // Two ways to command the axes on a cycle. They were one function behind a
    // bool, which is a mode wearing a flag's clothes: the call site said
    // `command_all(..., hold)` and told you nothing about what the axes do.
    void follow_all(double path_s, double dt_s) {
        for (std::size_t i = 0; i < adapters_.size(); ++i) {
            tgt_[i] = sources_[i]->next(path_s, dt_s);
            const double ref =
                controllers_.empty()
                    ? tgt_[i].position
                    : controllers_[i]->command(tgt_[i], adapters_[i]->read(), dt_s);
            cmd_[i] = governors_[i]->apply(ref, dt_s).position;
            adapters_[i]->write_setpoint(cmd_[i]);
        }
    }

    // An axis reported something other than normal: hold where the governor
    // last had it, and do not advance the path.
    void hold_all() {
        for (std::size_t i = 0; i < adapters_.size(); ++i) {
            cmd_[i] = governors_[i]->held_position();
            tgt_[i] = State{cmd_[i], 0.0, 0.0};
            adapters_[i]->write_setpoint(cmd_[i]);
        }
    }

    void step_all(double dt_s) {
        for (auto* w : worlds_) w->tick(dt_s);
        for (auto* a : adapters_) a->step(dt_s);
    }

    void read_all(double t, std::vector<std::vector<TelemetryRow>>& rows) {
        for (std::size_t i = 0; i < adapters_.size(); ++i) {
            const AxisState st = adapters_[i]->read();
            rows[i].push_back({t, tgt_[i].position, tgt_[i].velocity, cmd_[i], st.position,
                               st.velocity, cmd_[i] - st.position});
        }
    }

    void record_jitter(std::chrono::steady_clock::duration late, CycleStats& stats) {
        const auto late_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(late).count();
        jitter_us_.push_back(late_ns > 0 ? static_cast<double>(late_ns) * 1e-3 : 0.0);
        if (late_ns > period_ns_) ++stats.overruns;
    }

    void finalize_jitter(CycleStats& stats) {
        if (jitter_us_.empty()) return;
        double sum = 0.0;
        for (const double j : jitter_us_) sum += j;
        stats.mean_jitter_us = sum / static_cast<double>(jitter_us_.size());
        std::sort(jitter_us_.begin(), jitter_us_.end());
        stats.max_jitter_us = jitter_us_.back();
        stats.p99_jitter_us =
            jitter_us_[static_cast<std::size_t>(0.99 * static_cast<double>(jitter_us_.size() - 1))];
    }

    std::vector<AxisAdapter*> adapters_;
    std::vector<Governor*> governors_;
    std::vector<Controller*> controllers_;
    std::int64_t period_ns_;
    double stop_time_s_;
    std::vector<double> jitter_us_;
    std::vector<double> cmd_;
    std::vector<State> tgt_;
    std::vector<CycleSteppable*> worlds_;
    std::vector<PlanAxisSource> plan_views_;
    std::vector<SetpointSource*> sources_;
};

}  // namespace robonode
