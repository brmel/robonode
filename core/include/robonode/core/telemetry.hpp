#pragma once

#include <cstdint>

namespace robonode {

// One RT cycle's worth of axis telemetry — the cross-module currency:
// produced by executives (motion), consumed by recorders and, later, the
// data plane. Mirrors robonode.v0.AxisTelemetry field-for-field.
struct TelemetryRow {
    double t_s;
    double target_position;
    double target_velocity;
    double governed_position;
    double actual_position;
    double actual_velocity;
    double following_error;
};

struct CycleStats {
    std::uint64_t cycles{};
    std::uint64_t overruns{};            // wake-ups later than one full period
    std::uint64_t safety_hold_cycles{};  // cycles spent holding on non-NORMAL safety
    std::uint64_t cancel_cycles{};       // cycles spent decelerating on a stop request
    bool stopped_early{};                // the run ended on a stop request, not at the goal
    double max_jitter_us{};
    double p99_jitter_us{};
    double mean_jitter_us{};
};

}  // namespace robonode
