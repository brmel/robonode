// M1 slice 2 — the platform stack against a real (simulated) robot, now
// through the driver-factory seam:
//
//   DriverRegistry("robonode.ur-wrist") → UrWristAdapter → URSim
//   MotionPlan (S-curve) → Executive @500 Hz → Governor → adapter
//
// Identical motion code that drives SimAxis; only the registered driver
// differs. The adapter owns its connection (opened in configure()), so this
// app never constructs a UrDriver — exactly how celld builds a UR node.
//
//   ./ur_governed_move [robot_ip=127.0.0.1]
//
// Requires URSim powered on with brakes released (scripts/ursim.sh up).
// Writes ur-move.mcap.

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "robonode/adapter_ur/ur_wrist_adapter.hpp"
#include "robonode/motion/driver_registry.hpp"
#include "robonode/motion/executive.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/motion_plan.hpp"
#include "robonode/recorder/mcap_recorder.hpp"

#ifndef URCL_RESOURCES
#error "URCL_RESOURCES must point at the ur_client_library source dir"
#endif

int main(int argc, char** argv) {
    const std::string robot_ip = argc > 1 ? argv[1] : "127.0.0.1";

    // Descriptor-style limits (units: rad, rad/s, rad/s^2, rad/s^3) — data,
    // not runtime-derived. Wrist range generous; the move stays well inside.
    const robonode::AxisLimits wrist_limits{
        .position_min = -6.283,
        .position_max = 6.283,
        .velocity_max = 0.5,
        .acceleration_max = 2.0,
        .jerk_max = 20.0,
    };

    robonode::DriverRegistry registry;
    robonode::register_ur_wrist(registry, URCL_RESOURCES);

    std::unique_ptr<robonode::AxisAdapter> wrist;
    const robonode::DriverContext ctx{"ur10e-wrist3", wrist_limits, {{"robot_ip", robot_ip}}};
    if (const auto st = registry.make("robonode.ur-wrist", ctx, wrist); !st.ok()) {
        std::fprintf(stderr, "make failed: %s\n", st.message().c_str());
        return 1;
    }
    if (const auto st = wrist->configure(); !st.ok()) {
        std::fprintf(stderr, "configure failed: %s\n", st.message().c_str());
        return 1;
    }
    if (const auto st = wrist->activate(); !st.ok()) {
        std::fprintf(stderr, "activate failed: %s\n", st.message().c_str());
        return 1;
    }

    const double q5 = wrist->read().position;  // rad
    std::printf("wrist3 at %.4f rad, safety %s\n", q5,
                wrist->read().safety == robonode::SafetyState::kNormal ? "NORMAL" : "NOT-NORMAL");
    if (wrist->read().safety != robonode::SafetyState::kNormal &&
        wrist->read().safety != robonode::SafetyState::kReduced) {
        std::fprintf(stderr, "robot not ready (power on + brake release first)\n");
        return 1;
    }

    robonode::Governor governor{wrist_limits};
    robonode::Executive exec{*wrist, governor, /*rate_hz=*/500.0};

    const auto plan = robonode::MotionPlan::move(
        q5, q5 + 0.3,
        {wrist_limits.velocity_max, wrist_limits.acceleration_max,
         wrist_limits.jerk_max});
    std::printf("streaming S-curve %.4f -> %.4f rad, duration %.3f s @500 Hz\n", q5, q5 + 0.3,
                plan.duration_s());

    std::vector<robonode::TelemetryRow> rows;
    const auto stats = exec.execute(plan, rows, /*settle_s=*/0.3);
    (void)wrist->deactivate();

    const auto& last = rows.back();
    std::printf("done: target %.4f, actual %.4f rad (err %.5f)\n", last.governed_position,
                last.actual_position, last.governed_position - last.actual_position);
    std::printf("governor: pos=%llu vel=%llu rejected=%llu | safety holds %llu | jitter max %.0f us\n",
                static_cast<unsigned long long>(governor.position_clamps()),
                static_cast<unsigned long long>(governor.velocity_clamps()),
                static_cast<unsigned long long>(governor.rejected_setpoints()),
                static_cast<unsigned long long>(stats.safety_hold_cycles), stats.max_jitter_us);

    const auto rec = robonode::McapRecorder::write(
        "ur-move.mcap", {"rn/dev-cell/ur10e-wrist3/MotionAxis/telemetry"}, {rows}, 500.0);
    std::printf("recording: %s\n", rec.ok() ? "ur-move.mcap" : rec.message().c_str());
    return 0;
}
