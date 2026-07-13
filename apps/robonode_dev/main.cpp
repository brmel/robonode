// robonode_dev — the `robonode dev` experience slice (FR-3.4): boot
// simulated nodes from descriptor data, run governed moves at 1 kHz, drop
// telemetry (CSV + MCAP), report timing.
//
// Demo 1: jerk-limited S-curve inside the envelope.
// Demo 2: target beyond the position limit — governor envelope holds
//         (SPEC invariant I3).
// Demo 3: two axes, blended waypoints, one clock (FR-2.3/2.4).

#include <cstdio>
#include <vector>

#include "robonode/motion/executive.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/motion_plan.hpp"
#include "robonode/motion/sim_axis.hpp"
#include "robonode/motion/sync_blend.hpp"
#include "robonode/motion/sync_executive.hpp"
#include "robonode/recorder/mcap_recorder.hpp"

namespace {

// Mirrors robonode-idl/examples/rail-x.descriptor.json. Limits are data
// (FR-1.3); a real celld parses the descriptor — the dev app inlines the
// same values.
constexpr robonode::AxisLimits kRailX{
    .position_min_mm = 0.0,
    .position_max_mm = 1450.0,
    .velocity_max_mm_s = 1200.0,
    .acceleration_max_mm_s2 = 8000.0,
    .jerk_max_mm_s3 = 120000.0,
};
constexpr double kRateHz = 1000.0;

void write_csv(const char* path, const std::vector<robonode::TelemetryRow>& rows) {
    std::FILE* f = std::fopen(path, "w");
    if (f == nullptr) {
        std::fprintf(stderr, "warning: could not open %s — telemetry not saved\n", path);
        return;
    }
    std::fputs("t_s,target_pos_mm,target_vel_mm_s,governed_pos_mm,actual_pos_mm,actual_vel_mm_s,following_err_mm\n", f);
    for (const auto& r : rows) {
        std::fprintf(f, "%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f\n", r.t_s, r.target_position_mm,
                     r.target_velocity_mm_s, r.governed_position_mm, r.actual_position_mm,
                     r.actual_velocity_mm_s, r.following_error_mm);
    }
    std::fclose(f);
}

void report(const char* title, const robonode::CycleStats& s,
            const std::vector<robonode::TelemetryRow>& rows, const robonode::Governor& gov) {
    const auto& last = rows.back();
    std::printf("\n== %s ==\n", title);
    std::printf("cycles          : %llu @ %.0f Hz\n", static_cast<unsigned long long>(s.cycles), kRateHz);
    std::printf("final target    : %.3f mm\n", last.governed_position_mm);
    std::printf("final actual    : %.3f mm (err %.4f mm)\n", last.actual_position_mm,
                last.governed_position_mm - last.actual_position_mm);
    std::printf("governor        : pos clamps=%llu vel clamps=%llu rejected=%llu\n",
                static_cast<unsigned long long>(gov.position_clamps()),
                static_cast<unsigned long long>(gov.velocity_clamps()),
                static_cast<unsigned long long>(gov.rejected_setpoints()));
    std::printf("safety holds    : %llu cycles\n",
                static_cast<unsigned long long>(s.safety_hold_cycles));
    std::printf("host jitter     : mean %.1f us | p99 %.1f us | max %.1f us | overruns %llu\n",
                s.mean_jitter_us, s.p99_jitter_us, s.max_jitter_us,
                static_cast<unsigned long long>(s.overruns));
}

}  // namespace

int main() {
    std::printf("robonode dev — twin cell: node rail-x [MotionAxis@1, sim]\n");
    std::printf("limits: pos [%.0f, %.0f] mm | vel %.0f mm/s | acc %.0f mm/s^2 | jerk %.0f mm/s^3\n",
                kRailX.position_min_mm, kRailX.position_max_mm, kRailX.velocity_max_mm_s,
                kRailX.acceleration_max_mm_s2, kRailX.jerk_max_mm_s3);

    robonode::SimAxis axis{"rail-x", 0.0, /*tau_s=*/0.005};
    robonode::Governor governor{kRailX};
    robonode::Executive exec{axis, governor, kRateHz};

    // Demo 1: jerk-limited move inside the envelope.
    {
        std::vector<robonode::TelemetryRow> rows;
        const auto plan = robonode::MotionPlan::move(
            0.0, 500.0,
            {kRailX.velocity_max_mm_s, kRailX.acceleration_max_mm_s2, kRailX.jerk_max_mm_s3});
        std::printf("\nmove_to 500 mm, S-curve, planned duration %.3f s\n", plan.duration_s());
        const auto stats = exec.execute(plan, rows);
        write_csv("telemetry-move.csv", rows);
        report("demo 1: S-curve move 0 -> 500 mm", stats, rows, governor);
    }

    // Demo 2: target beyond position limit — governor must hold 1450 mm.
    {
        std::vector<robonode::TelemetryRow> rows;
        const auto plan = robonode::MotionPlan::move(
            500.0, 1600.0, {kRailX.velocity_max_mm_s, kRailX.acceleration_max_mm_s2, 0.0});
        std::printf("\nmove_to 1600 mm (beyond 1450 limit), trapezoid\n");
        const auto stats = exec.execute(plan, rows);
        write_csv("telemetry-governed.csv", rows);
        report("demo 2: governor envelope (target 1600 mm)", stats, rows, governor);
    }

    // Demo 3: two axes, blended waypoints, one clock — the coordinated-motion
    // shape (FR-2.3/2.4). Interior waypoints are passed through at speed.
    {
        robonode::SimAxis turret{"turret-a", 0.0, 0.005};
        constexpr robonode::AxisLimits kTurret{
            .position_min_mm = -10.0,
            .position_max_mm = 100.0,
            .velocity_max_mm_s = 300.0,
            .acceleration_max_mm_s2 = 2000.0,
            .jerk_max_mm_s3 = 0.0,
        };
        robonode::SimAxis rail2{"rail-x", 0.0, 0.005};
        robonode::Governor g0{kRailX}, g1{kTurret};
        robonode::SyncExecutive sync{{&rail2, &turret}, {&g0, &g1}, kRateHz};

        const auto plan = robonode::SyncBlendPlan::plan(
            {{0.0, 500.0, 300.0}, {0.0, 90.0, 45.0}}, {kRailX, kTurret});
        std::printf("\nsync move: rail 0->500->300 mm + turret 0->90->45, duration %.3f s\n",
                    plan.duration());
        std::vector<std::vector<robonode::TelemetryRow>> rows;
        const auto stats = sync.execute(plan, rows);
        std::printf("\n== demo 3: synchronized blended 2-axis sequence ==\n");
        std::printf("both axes on one clock: rail end %.3f mm, turret end %.3f mm (same %llu cycles)\n",
                    rows[0].back().actual_position_mm, rows[1].back().actual_position_mm,
                    static_cast<unsigned long long>(stats.cycles));
        std::printf("governor clamps : rail pos=%llu vel=%llu | turret pos=%llu vel=%llu\n",
                    static_cast<unsigned long long>(g0.position_clamps()),
                    static_cast<unsigned long long>(g0.velocity_clamps()),
                    static_cast<unsigned long long>(g1.position_clamps()),
                    static_cast<unsigned long long>(g1.velocity_clamps()));
        std::printf("host jitter     : mean %.1f us | p99 %.1f us | max %.1f us\n",
                    stats.mean_jitter_us, stats.p99_jitter_us, stats.max_jitter_us);
        write_csv("telemetry-sync-rail.csv", rows[0]);
        write_csv("telemetry-sync-turret.csv", rows[1]);

        // Flight-recorder v0: same run as MCAP — open robonode-dev.mcap in
        // Foxglove (FR-6.3).
        const auto rec = robonode::McapRecorder::write(
            "robonode-dev.mcap",
            {"rn/dev-cell/rail-x/MotionAxis/telemetry",
             "rn/dev-cell/turret-a/MotionAxis/telemetry"},
            rows, kRateHz);
        std::printf("mcap recording  : %s\n",
                    rec.ok() ? "robonode-dev.mcap" : rec.message().c_str());
    }

    std::printf("\ntelemetry: telemetry-*.csv + robonode-dev.mcap (Foxglove-openable)\n");
    return 0;
}
