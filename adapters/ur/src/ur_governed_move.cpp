// M1 slice 2 — the platform stack against a real (simulated) robot:
//
//   MotionPlan (S-curve) → Executive @500 Hz → Governor → UrWristAdapter → URSim
//
// Identical motion code that drives SimAxis; only the adapter differs.
// That is the AxisAdapter seam doing its job.
//
//   ./ur_governed_move [robot_ip=127.0.0.1]
//
// Requires URSim powered on with brakes released (scripts/ursim.sh up).
// Writes ur-move.mcap.

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <ur_client_library/ur/ur_driver.h>

#include "robonode/adapter_ur/ur_wrist_adapter.hpp"
#include "robonode/motion/executive.hpp"
#include "robonode/motion/governor.hpp"
#include "robonode/motion/motion_plan.hpp"
#include "robonode/recorder/mcap_recorder.hpp"

#ifndef URCL_RESOURCES
#error "URCL_RESOURCES must point at the ur_client_library source dir"
#endif

int main(int argc, char** argv) {
    const std::string robot_ip = argc > 1 ? argv[1] : "127.0.0.1";
    const std::string res = URCL_RESOURCES;

    std::unique_ptr<urcl::UrDriver> driver;
    try {
        driver = std::make_unique<urcl::UrDriver>(
            robot_ip, res + "/resources/external_control.urscript",
            res + "/examples/resources/rtde_output_recipe.txt",
            res + "/examples/resources/rtde_input_recipe.txt",
            [](bool running) { std::printf("external-control program %s\n", running ? "running" : "stopped"); },
            /*headless=*/true);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "driver init failed: %s\n", e.what());
        return 1;
    }
    driver->startRTDECommunication();

    robonode::UrWristAdapter wrist{*driver, "ur10e-wrist3"};
    if (const auto st = wrist.configure(); !st.ok()) {
        std::fprintf(stderr, "configure failed: %s\n", st.message().c_str());
        return 1;
    }
    if (const auto st = wrist.activate(); !st.ok()) {
        std::fprintf(stderr, "activate failed: %s\n", st.message().c_str());
        return 1;
    }

    const double q5 = wrist.read().position_mm;  // rad
    std::printf("wrist3 at %.4f rad, safety %s\n", q5,
                wrist.read().safety == robonode::SafetyState::kNormal ? "NORMAL" : "NOT-NORMAL");
    if (wrist.read().safety != robonode::SafetyState::kNormal &&
        wrist.read().safety != robonode::SafetyState::kReduced) {
        std::fprintf(stderr, "robot not ready (power on + brake release first)\n");
        return 1;
    }

    // Limits play the descriptor role (units: rad, rad/s, rad/s^2, rad/s^3).
    const robonode::AxisLimits wrist_limits{
        .position_min_mm = q5 - 0.6,
        .position_max_mm = q5 + 0.6,
        .velocity_max_mm_s = 0.5,
        .acceleration_max_mm_s2 = 2.0,
        .jerk_max_mm_s3 = 20.0,
    };
    robonode::Governor governor{wrist_limits};
    robonode::Executive exec{wrist, governor, /*rate_hz=*/500.0};

    const auto plan = robonode::MotionPlan::move(
        q5, q5 + 0.3,
        {wrist_limits.velocity_max_mm_s, wrist_limits.acceleration_max_mm_s2,
         wrist_limits.jerk_max_mm_s3});
    std::printf("streaming S-curve %.4f -> %.4f rad, duration %.3f s @500 Hz\n", q5, q5 + 0.3,
                plan.duration_s());

    std::vector<robonode::TelemetryRow> rows;
    const auto stats = exec.execute(plan, rows, /*settle_s=*/0.3);
    (void)wrist.deactivate();
    driver->stopControl();

    const auto& last = rows.back();
    std::printf("done: target %.4f, actual %.4f rad (err %.5f)\n", last.governed_position_mm,
                last.actual_position_mm, last.governed_position_mm - last.actual_position_mm);
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
