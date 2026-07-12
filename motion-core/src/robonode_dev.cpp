// robonode_dev — M0 slice of the `robonode dev` experience (FR-3.4):
// boot a simulated axis node from its descriptor, run a jerk-limited move
// through governor + adapter at 1 kHz, drop telemetry, report timing.
//
// Demo 2 deliberately commands a target beyond the descriptor's position
// limit to show the governor envelope holding (SPEC invariant I3).

#include <cstdio>
#include <vector>

#include "robonode/executive.hpp"
#include "robonode/governor.hpp"
#include "robonode/motion_plan.hpp"
#include "robonode/sim_axis.hpp"

namespace {

// Mirrors robonode-idl/examples/rail-x.descriptor.json. Limits are data
// (FR-1.3); a real celld parses the descriptor — M0 inlines the same values.
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
    if (f == nullptr) return;
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
    std::printf("governor clamps : pos=%llu vel=%llu\n",
                static_cast<unsigned long long>(gov.position_clamps()),
                static_cast<unsigned long long>(gov.velocity_clamps()));
    std::printf("host jitter     : mean %.1f us | p99 %.1f us | max %.1f us | overruns %llu\n",
                s.mean_jitter_us, s.p99_jitter_us, s.max_jitter_us,
                static_cast<unsigned long long>(s.overruns));
}

}  // namespace

int main() {
    std::printf("robonode dev — M0 twin cell: node rail-x [MotionAxis@1, sim]\n");
    std::printf("limits: pos [%.0f, %.0f] mm | vel %.0f mm/s | acc %.0f mm/s^2 | jerk %.0f mm/s^3\n",
                kRailX.position_min_mm, kRailX.position_max_mm, kRailX.velocity_max_mm_s,
                kRailX.acceleration_max_mm_s2, kRailX.jerk_max_mm_s3);

    robonode::SimAxis axis{"rail-x", 0.0, /*tau_s=*/0.005};
    robonode::Governor governor{kRailX};
    robonode::Executive exec{axis, governor, kRateHz};

    // Demo 1: jerk-limited move inside the envelope.
    {
        std::vector<robonode::TelemetryRow> rows;
        const auto plan = robonode::MotionPlan::scurve(
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
        const auto plan = robonode::MotionPlan::trapezoid(
            500.0, 1600.0, {kRailX.velocity_max_mm_s, kRailX.acceleration_max_mm_s2});
        std::printf("\nmove_to 1600 mm (beyond 1450 limit), trapezoid\n");
        const auto stats = exec.execute(plan, rows);
        write_csv("telemetry-governed.csv", rows);
        report("demo 2: governor envelope (target 1600 mm)", stats, rows, governor);
    }

    std::printf("\ntelemetry: telemetry-move.csv, telemetry-governed.csv\n");
    return 0;
}
