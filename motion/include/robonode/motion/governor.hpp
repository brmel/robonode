#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robonode/core/limits.hpp"

namespace robonode {

// Platform-owned limit envelope BELOW the plugin boundary (SPEC invariant
// I3, FR-8.4). Whatever a plan, stream, or Tier C plugin emits, the governor
// guarantees the setpoint leaving the RT cycle respects the descriptor
// limits. User code can degrade motion quality, never exceed the envelope.
//
// RT contract: apply() is allocation-free and branch-cheap.
class Governor {
public:
    explicit Governor(AxisLimits limits) : limits_{limits} {}

    struct Result {
        double position;
        bool clamped;
    };

    // Governs one absolute setpoint given the previous governed setpoint.
    // Position: hard clamp to [min, max]. Velocity: rate-limit |Δp|/dt.
    // (Acceleration/jerk envelopes join when the OTG slot lands — the OTG
    // already enforces them upstream for planned motion.)
    //
    // Non-finite setpoints and non-positive dt are rejected by holding the
    // previous governed position — FR-3.1: NaN from any upstream (plan bug,
    // Tier C plugin) must never reach a drive.
    Result apply(double setpoint, double dt_s) noexcept {
        if (!std::isfinite(setpoint) || !(dt_s > 0.0)) {
            ++rejected_setpoints_;
            return {prev_, true};
        }
        double p = std::clamp(setpoint, limits_.position_min, limits_.position_max);
        bool clamped = p != setpoint;
        if (clamped) ++position_clamps_;

        // Tolerance: a plan cruising exactly at v_max produces dp == dp_max in
        // exact arithmetic; float rounding lands a few ulps above and must not
        // count as a violation.
        const double dp_max = limits_.velocity_max * dt_s * (1.0 + 1e-9);
        const double dp = p - prev_;
        if (std::abs(dp) > dp_max) {
            p = prev_ + std::copysign(dp_max, dp);
            ++velocity_clamps_;
            clamped = true;
        }
        prev_ = p;
        return {p, clamped};
    }

    // Seed the rate limiter with the axis's real position at activation so
    // the first cycle doesn't see a phantom jump.
    void reset(double position) noexcept {
        prev_ = position;
        position_clamps_ = 0;
        velocity_clamps_ = 0;
        rejected_setpoints_ = 0;
    }

    // Last governed setpoint — the hold value during safety stops.
    [[nodiscard]] double held_position() const noexcept { return prev_; }

    [[nodiscard]] std::uint64_t position_clamps() const noexcept { return position_clamps_; }
    [[nodiscard]] std::uint64_t velocity_clamps() const noexcept { return velocity_clamps_; }
    [[nodiscard]] std::uint64_t rejected_setpoints() const noexcept { return rejected_setpoints_; }
    [[nodiscard]] const AxisLimits& limits() const noexcept { return limits_; }

private:
    AxisLimits limits_;
    double prev_{};
    std::uint64_t position_clamps_{};
    std::uint64_t velocity_clamps_{};
    std::uint64_t rejected_setpoints_{};
};

}  // namespace robonode
