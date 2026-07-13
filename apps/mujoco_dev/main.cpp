// mujoco_dev — the motion stack against real physics: the same
// Executive/Governor/MotionPlan that drives SimAxis, now moving a MuJoCo
// carriage through the robonode.mujoco-axis driver. Following error is now
// physical (inertia + servo dynamics), not a filter constant.
//
//   ./mujoco_dev
//
// Writes mujoco-rail.mcap (open in Foxglove).

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "robonode/motion/driver_registry.hpp"
#include "robonode/motion/executive.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/motion_plan.hpp"
#include "robonode/recorder/mcap_recorder.hpp"
#include "robonode/sim_mujoco/mujoco_driver.hpp"

#ifndef ROBONODE_WORLDS
#error "ROBONODE_WORLDS must point at the sim-mujoco worlds dir"
#endif

int main() {
    // Same descriptor limits as robonode-idl/examples/rail-x (mm).
    const robonode::AxisLimits rail_limits{
        .position_min_mm = 0.0,
        .position_max_mm = 1450.0,
        .velocity_max_mm_s = 1200.0,
        .acceleration_max_mm_s2 = 8000.0,
        .jerk_max_mm_s3 = 120000.0,
    };

    robonode::DriverRegistry registry;
    robonode::register_mujoco_axis(registry);

    std::unique_ptr<robonode::AxisAdapter> rail;
    const robonode::DriverContext ctx{
        "rail-x",
        rail_limits,
        {{"world", std::string{ROBONODE_WORLDS} + "/rail.xml"},
         {"joint", "rail"},
         {"actuator", "rail_servo"},
         {"units_per_m", "1000"}}};
    if (const auto st = registry.make("robonode.mujoco-axis", ctx, rail); !st.ok()) {
        std::fprintf(stderr, "make failed: %s\n", st.message().c_str());
        return 1;
    }
    if (const auto st = rail->configure(); !st.ok()) {
        std::fprintf(stderr, "configure failed: %s\n", st.message().c_str());
        return 1;
    }
    (void)rail->activate();

    robonode::Governor governor{rail_limits};
    robonode::Executive exec{*rail, governor, 1000.0};
    const auto plan = robonode::MotionPlan::move(
        0.0, 500.0,
        {rail_limits.velocity_max_mm_s, rail_limits.acceleration_max_mm_s2,
         rail_limits.jerk_max_mm_s3});
    std::printf("mujoco rail: S-curve 0 -> 500 mm, planned %.3f s @1 kHz\n", plan.duration_s());

    std::vector<robonode::TelemetryRow> rows;
    const auto stats = exec.execute(plan, rows, /*settle_s=*/0.3);

    double max_follow = 0.0;
    for (const auto& r : rows) {
        if (std::abs(r.following_error_mm) > max_follow) max_follow = std::abs(r.following_error_mm);
    }
    const auto& last = rows.back();
    std::printf("final actual %.3f mm (target 500) | peak following error %.3f mm (physical)\n",
                last.actual_position_mm, max_follow);
    std::printf("governor: pos=%llu vel=%llu | jitter max %.0f us | %llu cycles\n",
                static_cast<unsigned long long>(governor.position_clamps()),
                static_cast<unsigned long long>(governor.velocity_clamps()), stats.max_jitter_us,
                static_cast<unsigned long long>(stats.cycles));

    const auto rec = robonode::McapRecorder::write(
        "mujoco-rail.mcap", {"rn/dev-cell/rail-x/MotionAxis/telemetry"}, {rows}, 1000.0);
    std::printf("recording: %s\n", rec.ok() ? "mujoco-rail.mcap" : rec.message().c_str());
    return 0;
}
