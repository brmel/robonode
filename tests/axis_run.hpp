#pragma once

#include <utility>
#include <vector>

#include "robonode/core/telemetry.hpp"
#include "robonode/motion/setpoint_source.hpp"
#include "robonode/motion/sync_executive.hpp"

// Single-axis convenience over the one executive: the platform has exactly one
// RT loop, so tests drive it with a one-element cell.
namespace testing {

inline robonode::CycleStats run_source(robonode::AxisAdapter& axis, robonode::Governor& gov,
                                       double rate_hz, robonode::SetpointSource& source,
                                       std::vector<robonode::TelemetryRow>& rows,
                                       double run_for_s) {
    robonode::SyncExecutive exec{{&axis}, {&gov}, rate_hz, {}, 0.3};
    std::vector<std::vector<robonode::TelemetryRow>> all;
    const auto stats = exec.execute({&source}, all, run_for_s);
    rows = std::move(all.front());
    return stats;
}

inline robonode::CycleStats run_plan(robonode::AxisAdapter& axis, robonode::Governor& gov,
                                     double rate_hz, const robonode::MotionPlan& plan,
                                     std::vector<robonode::TelemetryRow>& rows, double settle_s) {
    robonode::PlanSource source{plan};
    return run_source(axis, gov, rate_hz, source, rows, plan.duration_s() + settle_s);
}

}  // namespace testing
